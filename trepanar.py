# /// script
# dependencies = ["numpy", "scipy", "opencv-python-headless", "pyserial", "pygame", "zarr>=3", "fsspec", "aiohttp", "requests", "compresso"]
# requires-python = ">=3.11,<3.14"
# ///
"""Corte oblicuo de un volumen HOA orientado por una IMU (ESP32 por USB).

Uso:
  uv run trepanar.py                       # /dev/ttyACM0 a 115200
  uv run trepanar.py --puerto /dev/ttyUSB0 --baud 115200
  uv run trepanar.py --sin-imu             # solo teclado
  uv run trepanar.py --completa --monitor 1 # pantalla completa en el segundo monitor

La IMU debe mandar líneas "roll,pitch,yaw[,...]" en grados (lo demás se ignora).

Teclas:
  p        calibrar 5 puntos guiado por voz (frente, Cz, temp izq, temp der, occipital)
           -> modo puntos: el corte se ubica bajo la punta del lápiz; tocar un imán corrige deriva
           -> girar el lápiz sobre su eje (destornillador) = más/menos hondo (1 vuelta = 3 cm)
  n        modo puntos on/off      x  invertir izquierda/derecha del volumen
  z        invertir sentido del giro      0  volver a la superficie
  e        mostrar/ocultar las regiones (colores y nombre); parte oculto
  Golpecito con el dedo = 5 mm más hondo; doble golpecito (< 0,5 s) = 5 mm hacia la superficie.
  Sin teclado (lápiz quieto sobre un imán): toque breve = corrige deriva,
  Espacio arma la superficie; al apoyar el lápiz quieto en el imán de la frente, la vista vuelve
  al polo rostral, en la superficie (espacio otra vez cancela; se desarma sola a los 20 s)
  t        modo inclinación->profundidad on/off (1/2 = máximo adelante/atrás)
  c        fijar orientación actual como cero
  g        recalibrar giroscopio (sensor quieto ~1 s)
  m        configurar la sonda: montaje del sensor (3 posiciones; guía en pantalla)
  b        ventana de imanes: intensidad y polaridad (Norte/Sur hacia la sonda)
  w / s    avanzar / retroceder el plano de corte
  i/k j/l u/o  rotar a mano (pitch, roll, yaw) — se suma a la IMU
  r        reset de rotación manual y profundidad
  q / Esc  salir
"""
import argparse
import collections
import glob
import json
import os
import subprocess
import sys
import textwrap
import threading
import time
from pathlib import Path

import cv2
import numpy as np
from scipy.ndimage import map_coordinates
from scipy.spatial.transform import Rotation

import instalador
import paquetes
import vista3d

AQUI = Path(__file__).parent
CONGELADO = getattr(sys, "frozen", False)  # binario de PyInstaller
RECURSOS = Path(getattr(sys, "_MEIPASS", AQUI))  # datos empaquetados (solo lectura en el binario)
# calibraciones: junto al código en desarrollo; en ~/.local/share/trepanar en producción
USUARIO = (Path(os.environ.get("XDG_DATA_HOME", Path.home() / ".local" / "share")) / "trepanar"
           if CONGELADO else AQUI / "datos")
_VIEJO = USUARIO.parent / "hoa-imu-demo"  # nombre anterior: conservar calibración y paquetes bajados
if CONGELADO and not USUARIO.exists() and _VIEJO.is_dir():
    _VIEJO.rename(USUARIO)
USUARIO.mkdir(parents=True, exist_ok=True)
CONFIG = USUARIO / "config.json"  # paquete elegido
TECLA_PAQUETES = 1002  # F2: volver a la pantalla de paquetes


def _slerp(a, b, t):
    """Interpolación sobre la esfera entre los vectores unitarios a y b."""
    t = float(np.clip(t, 0, 1))
    ang = np.arccos(np.clip(a @ b, -1, 1))
    if ang < 1e-6:
        return a
    v = (np.sin((1 - t) * ang) * a + np.sin(t * ang) * b) / np.sin(ang)
    return v / np.linalg.norm(v)


class Velocidad:
    """Velocidad angular (°/s) robusta al temblor del sensor: ángulo girado en la última ventana
    (no entre lecturas consecutivas, que con 60 cuadros/s convierte 0,3° de ruido en 20-30 °/s)."""

    def __init__(self, ventana=0.5):
        self.ventana = ventana
        self.hist = collections.deque()

    def __call__(self, t, r):
        self.hist.append((t, r))
        while len(self.hist) > 2 and t - self.hist[1][0] >= self.ventana:
            self.hist.popleft()
        t0, r0 = self.hist[0]
        if t - t0 < 0.15:
            return 0.0
        return float(np.degrees((r0.inv() * r).magnitude()) / (t - t0))


class DetectorToques:
    """Golpecito con el dedo en el lápiz, en la dirección que le acomode a cada uno.

    La calibración aprende de 5 golpes: el eje en que se golpea, la fuerza mínima y que antes
    del golpe el lápiz estaba quieto (apoyarlo en el cráneo llega con el lápiz en movimiento).
    Sin calibrar: cualquier dirección, fuerza >= 2 g y lápiz quieto.
    """

    REFRACTARIO = 0.15  # s: el rebote de un golpe llega a ~0,1 s (medido); un doble golpe, a 0,15-0,5 s
    FABRICA = {"umbral": 2.5, "eje": None, "cos_min": 0.0, "giro_max": 60.0}

    def __init__(self):
        self.p = dict(self.FABRICA)
        self.t_ultimo = 0.0

    def configurar(self, params):
        self.p = dict(self.FABRICA, **(params or {}))

    def alimentar(self, t, hp, giro_previo):
        """hp: vector de impacto (g); giro_previo: velocidad angular justo antes (°/s)."""
        m = float(np.linalg.norm(hp))
        if m < self.p["umbral"] or t - self.t_ultimo < self.REFRACTARIO:
            return False
        if giro_previo > self.p["giro_max"] and t - self.t_ultimo > 0.6:  # tras un golpe no exigir quietud
            return False
        if self.p["eje"] is not None and abs(np.dot(hp, self.p["eje"])) / m < self.p["cos_min"]:
            return False
        self.t_ultimo = t
        return True

    @staticmethod
    def _picos(filas, minimo, sep=0.2):
        """Picos de |impacto| de mayor a menor, separados por `sep` s. filas: (t, hx, hy, hz, giro)."""
        picos = []
        for f in sorted(filas, key=lambda f: -np.linalg.norm(f[1:4])):
            if np.linalg.norm(f[1:4]) < minimo:
                break
            if all(abs(f[0] - g[0]) > sep for g in picos):
                picos.append(f)
        return picos

    @classmethod
    def calibrar(cls, registro, t_ini, t_fin):
        """registro: [(t, hx, hy, hz, giro_previo)]; [t_ini, t_fin] = tramo en que se pidió golpear.
        Devuelve (parámetros o None, texto para decir)."""
        r = np.array(registro) if registro else np.zeros((0, 5))
        en = (r[:, 0] >= t_ini) & (r[:, 0] <= t_fin)
        golpes = cls._picos(r[en], 0.8)
        if golpes:  # los golpes de verdad: al menos la mitad de fuerza que el más fuerte
            fuerte = np.linalg.norm(golpes[0][1:4])
            golpes = [g for g in golpes if np.linalg.norm(g[1:4]) >= 0.5 * fuerte]
        otros = cls._picos(r[~en], 0.4)
        if len(golpes) < 3:
            return None, "No detecté bien los golpes. Uso el valor de fábrica."
        G = np.array([g[1:4] for g in golpes])
        fuerza = np.linalg.norm(G, axis=1)
        u = G / fuerza[:, None]
        # eje de golpe (sin signo: el impacto rebota): vector propio principal
        eje = np.linalg.eigh(u.T @ u)[1][:, -1]
        cos = np.abs(u @ eje)
        cos_min = float(max(0.0, cos.min() - 0.25))
        giro_max = float(max(40.0, 1.5 * max(g[4] for g in golpes)))

        def pasa(f):  # un no-golpe que el detector confundiría (salvo por la fuerza)
            m = np.linalg.norm(f[1:4])
            return abs(np.dot(f[1:4], eje)) / m >= cos_min and f[4] <= giro_max

        confundibles = [np.linalg.norm(f[1:4]) for f in otros if pasa(f)]
        no_golpe = max(confundibles, default=0.4)
        debil = float(fuerza.min())
        if debil > 1.2 * no_golpe:
            umbral, texto = float(np.sqrt(debil * max(no_golpe, 0.3))), "Golpes calibrados."
        else:
            umbral = 0.85 * debil
            texto = "Golpes calibrados, pero golpea un poco más fuerte para que no se confundan."
        params = {"umbral": float(np.clip(umbral, 0.6, 5.0)), "eje": eje.tolist(),
                  "cos_min": cos_min, "giro_max": giro_max}
        return params, texto


class LectorIMU(threading.Thread):
    """Lee el puerto serie en segundo plano y guarda el último (roll, pitch, yaw)."""

    def __init__(self, puerto, baud):
        super().__init__(daemon=True)
        import serial
        self.serial = serial
        self.puerto_fijo = puerto
        self.baud = baud
        self.ser = None  # se abre en run(): si el lápiz no está, lo espera
        self.angulos = np.zeros(3)
        self.quat = None
        self.campo = 0.0
        self.toques = 0  # contador del firmware (umbral fijo)
        self.crudo, self.ultima_cruda = None, 0.0  # acel, giro, mag (modo crudo)
        self.mag = None  # vector del magnetómetro (µT; 9999 = saturado)
        self.cara = np.array([0, 0, 1.0])  # cara detectora en el sensor (la fija el montaje)
        self.polaridad, self.t_polaridad = None, 0.0  # "N"/"S" del último imán leído sin saturar
        self.signo_iman = 0
        self.detector = DetectorToques()  # detección propia con umbral calibrado
        self.toques_propios = 0
        self.tiene_impacto = False  # firmware nuevo: manda aceleración lateral/axial
        self.registro = None  # lista [(t, hx, hy, hz, giro_previo)] mientras se calibra
        self._vel = collections.deque(maxlen=25)  # velocidad angular de las últimas muestras (°/s)
        self._q_ant, self._t_ant = None, None
        self.ultima = 0.0
        self.lineas_malas = 0
        self.ultimo_texto = ""

    def reconectar(self):
        """La placa puede volver como otro ttyACM tras un corte de USB."""
        try:
            if self.ser is not None:
                self.ser.close()
        except Exception:
            pass
        while True:
            for p in [self.puerto_fijo] + sorted(glob.glob("/dev/ttyACM*") + glob.glob("/dev/ttyUSB*")):
                try:
                    self.ser = self.serial.Serial(p, self.baud, timeout=1)
                    self.ultimo_texto = f"conectado en {p}"
                    return
                except Exception:
                    pass
            self.ultimo_texto = "lapiz desconectado: enchufalo por USB"
            time.sleep(0.5)

    def run(self):
        self.reconectar()
        while True:
            try:
                crudo = self.ser.readline().decode(errors="ignore").strip()
            except Exception:
                self.reconectar()
                continue
            if not crudo:
                continue
            if crudo.startswith("R,") and crudo.count(",") == 9:  # modo crudo (ventana de imanes)
                try:
                    self.crudo = np.array([float(x) for x in crudo[2:].split(",")])
                    self.ultima_cruda = time.time()
                except ValueError:
                    pass
                continue
            try:
                vals = [float(x) for x in crudo.replace(";", ",").split(",")]
            except ValueError:
                self.lineas_malas += 1
                self.ultimo_texto = crudo[:70]
                continue
            if len(vals) >= 3:
                self.angulos = np.array(vals[:3])
                # firmware hoa_imu: roll,pitch,yaw,qw,qx,qy,qz[,campo_uT]
                self.quat = np.array(vals[3:7]) if len(vals) >= 7 else None
                self.campo = vals[7] if len(vals) >= 8 else 0.0
                self.toques = int(vals[8]) if len(vals) >= 9 else 0
                self.ultima = time.time()
                if len(vals) >= 15:  # polaridad: se lee al acercarse, antes de que el imán sature
                    self.mag = np.array(vals[12:15])
                    b = float(np.linalg.norm(self.mag))
                    if np.all(self.mag < 9999) and b > 150:
                        comp = float(self.mag @ self.cara) / b
                        if abs(comp) > 0.5:  # Sur hacia la sonda: el campo entra al imán (va hacia la cara)
                            self.polaridad, self.t_polaridad = ("S" if comp > 0 else "N"), self.ultima
                            self.signo_iman = 1 if comp > 0 else -1  # signo crudo (no depende de la convención N/S)
                if len(vals) >= 12:  # cada muestra (100 Hz): aquí no se pierden picos de 10 ms
                    self.tiene_impacto = True
                    q = self.quat / np.linalg.norm(self.quat)
                    if self._q_ant is not None:
                        dt = max(self.ultima - self._t_ant, 1e-3)
                        ang = 2 * np.degrees(np.arccos(min(1.0, abs(float(q @ self._q_ant)))))
                        self._vel.append(ang / dt)
                    self._q_ant, self._t_ant = q, self.ultima
                    # giro justo ANTES del impacto (el golpe mismo sacude el lápiz): 50-200 ms atrás
                    previas = list(self._vel)[:-5]
                    giro_previo = float(np.median(previas)) if previas else 0.0
                    hp = np.array(vals[9:12])
                    if self.detector.alimentar(self.ultima, hp, giro_previo):
                        self.toques_propios += 1
                    if self.registro is not None:
                        self.registro.append((self.ultima, *hp, giro_previo))

    def signo_reciente(self, segundos=1.5):
        """+1/-1 según el campo del último imán leído (sin saturar) hace poco; 0 si no hay."""
        return self.signo_iman if time.time() - self.t_polaridad < segundos else 0

    def polaridad_reciente(self, segundos=1.5):
        """Polaridad del imán bajo la sonda si se leyó hace poco (al tocarlo ya satura)."""
        return self.polaridad if time.time() - self.t_polaridad < segundos else None

    def conteo_toques(self):
        return self.toques_propios if self.tiene_impacto else self.toques

    def rotacion(self):
        if self.quat is not None:
            w, x, y, z = self.quat
            return Rotation.from_quat([x, y, z, w])
        return Rotation.from_euler("ZYX", self.angulos[::-1], degrees=True)  # yaw, pitch, roll


class Cerebro:
    """Segmenta el cerebro (componente brillante más grande) para medir su superficie real,
    no la del frasco de la muestra."""

    def __init__(self, vol, cache=None, seg=None, factor_seg=1):
        if seg is not None:  # con regiones: la máscara del cerebro es exacta
            zyx = np.argwhere(seg > 0)[:, :3] * float(factor_seg)
            if len(zyx) > 20_000:
                zyx = zyx[np.random.default_rng(0).choice(len(zyx), 20_000, replace=False)]
            self.pts = zyx[:, ::-1]
        elif cache is not None and cache.exists():
            self.pts = np.load(cache)
        else:
            from scipy import ndimage
            sub = vol[::2, ::2, ::2]  # segmentar a mitad de resolución: 8x más rápido
            umbral = np.percentile(sub, 99) * 0.6
            etiquetas, n = ndimage.label(ndimage.binary_opening(sub > umbral, iterations=1))
            tam = np.bincount(etiquetas.ravel())[1:]
            zyx = np.argwhere(etiquetas == (np.argmax(tam) + 1)) * 2.0
            if len(zyx) > 20_000:  # suficiente para medir la superficie, y rápido
                zyx = zyx[np.random.default_rng(0).choice(len(zyx), 20_000, replace=False)]
            self.pts = zyx[:, ::-1]  # (x, y, z)
            if cache is not None:
                try:
                    np.save(cache, self.pts)
                except OSError:
                    pass  # carpeta de solo lectura (binario): se recalcula al abrir
        self.centro = self.pts.mean(0)

    def radio(self, d):
        """Distancia del centro a la superficie del cerebro en la dirección d (vóxeles)."""
        return float(np.percentile((self.pts - self.centro) @ d, 99.5))


def corte(vol, rot, profundidad, lado, centro=None, px=300, seg=None, factor_seg=1):
    """Muestrea un plano de lado×lado vóxeles (en px×px muestras), rotado por `rot`.
    Con `seg` devuelve también las etiquetas de región en el mismo plano."""
    if centro is None:
        centro = (np.array(vol.shape[::-1]) - 1) / 2  # (x, y, z)
    u, v, n = rot.apply(np.eye(3))  # ejes del plano en coordenadas (x, y, z)
    t = np.linspace(-lado / 2, lado / 2, min(px, int(lado)))
    s, r = np.meshgrid(t, t)
    pts = centro + profundidad * n + s[..., None] * u + r[..., None] * v
    coords = pts[..., ::-1].transpose(2, 0, 1)  # -> (z, y, x)
    img = map_coordinates(vol, coords, order=1, cval=0).astype(np.uint8)
    if seg is None:
        return img
    return img, map_coordinates(seg, coords / factor_seg, order=0, cval=0)


def pintar_regiones(img_bgr, etq, lut, alfa=0.35):
    """Colorea las regiones (no la sustancia blanca ni el líquido) y marca sus bordes."""
    col = lut[etq]
    pintar = col.any(axis=2)
    img_bgr[pintar] = ((1 - alfa) * img_bgr[pintar] + alfa * col[pintar]).astype(np.uint8)
    borde = cv2.morphologyEx((etq % 251).astype(np.uint8), cv2.MORPH_GRADIENT, np.ones((2, 2), np.uint8)) > 0
    img_bgr[borde & pintar] = (0.5 * img_bgr[borde & pintar] + 0.5 * col[borde & pintar]).astype(np.uint8)
    return img_bgr


def lut_regiones(etiquetas):
    """Tabla etiqueta -> color BGR; sin color para sustancia blanca, líquido y fondo."""
    sin_color = ("White-Matter", "WM-", "CSF", "Unknown")
    lut = np.zeros((max([int(k) for k in etiquetas] + [0]) + 1, 3), np.uint8)
    for k, e in etiquetas.items():
        if not any(t in e["en"] for t in sin_color):
            h = e["color"].lstrip("#")
            lut[int(k)] = (int(h[4:6], 16), int(h[2:4], 16), int(h[0:2], 16))
    return lut


class InclinacionProfundidad:
    """Convierte la inclinación adelante/atrás en profundidad del plano de corte.

    Se calibra con dos posiciones (máximo atrás y máximo adelante). El eje de giro entre
    ambas se calcula solo, así que no importa cómo esté montado el sensor. El giro sobre
    ese eje se quita de la orientación (el plano queda en la inclinación media) y se usa
    como fracción 0..1 del recorrido.
    """

    ARCHIVO = USUARIO / "calib_inclinacion.json"

    def __init__(self):
        self.atras = self.adelante = None
        self.activo = False
        self.eje = None
        self.angulo = 0.0
        if self.ARCHIVO.exists():
            d = json.loads(self.ARCHIVO.read_text())
            self.atras = Rotation.from_quat(d["atras"])
            self.adelante = Rotation.from_quat(d["adelante"])
            self._preparar()

    def fijar(self, cual, rot):
        setattr(self, cual, rot)
        if self.atras is not None and self.adelante is not None:
            self._preparar()
            if self.activo:
                self.ARCHIVO.write_text(json.dumps({
                    "atras": self.atras.as_quat().tolist(),
                    "adelante": self.adelante.as_quat().tolist(),
                    "recorrido_grados": float(np.degrees(self.angulo)),
                }, indent=2))

    def _preparar(self):
        v = (self.atras.inv() * self.adelante).as_rotvec()
        self.angulo = float(np.linalg.norm(v))
        self.activo = self.angulo > np.radians(5)  # menos de 5° no sirve
        if not self.activo:
            self.eje = None
            return
        self.eje = v / self.angulo  # eje de inclinación, en el marco del sensor
        # Vector del sensor que queda horizontal en "atrás" y sube al inclinar: su
        # elevación respecto de la gravedad mide la inclinación sin depender del yaw.
        arriba = self.atras.inv().apply([0, 0, 1])
        b = np.cross(self.eje, arriba)
        self.b = b / np.linalg.norm(b)
        self.e0 = self._elevacion(self.atras)
        self.e1 = self._elevacion(self.adelante)

    def _elevacion(self, r):
        return float(np.arcsin(np.clip(r.apply(self.b)[2], -1, 1)))

    def aplicar(self, r_imu):
        """Devuelve (rotación sin la inclinación, fracción 0..1 del recorrido)."""
        f = (self._elevacion(r_imu) - self.e0) / (self.e1 - self.e0)
        # Quitar el giro sobre el eje propio y dejar el plano en la inclinación media.
        quitar = Rotation.from_rotvec(-(f - 0.5) * self.angulo * self.eje)
        return r_imu * quitar, float(np.clip(f, 0, 1))


MENSAJE = ["", 0.0]  # mensaje en pantalla (texto, hasta cuándo); reemplaza a la voz


def decir(texto, segundos=3.0):
    """Muestra el mensaje en grande en la pantalla (antes era voz; era lenta)."""
    MENSAJE[0], MENSAJE[1] = texto, time.time() + segundos


class ModoPuntos:
    """Ubica la punta del lápiz sobre el cráneo y controla la profundidad girando el lápiz.

    Calibración guiada por voz, sin suponer cómo va montado el sensor dentro del lápiz:
      1. Cz (vértice) con el lápiz derecho: la gravedad da el eje del lápiz dentro del sensor
      2. frente, 3. deslizar de la frente a la nuca por encima (plano medio: adelante/atrás y
         el eje izquierda-derecha), 4. nuca, 5-6. temporales (lados), 7. golpecitos.
    Con eso se ajusta por mínimos cuadrados un marco de la cabeza. En uso:
      - posición: dirección del eje del lápiz en ese marco (no depende de girarlo en los dedos)
      - profundidad: girar el lápiz sobre su eje, como destornillador
      - imán tocado con el lápiz quieto: corrige la deriva del rumbo (giro en el eje vertical)
    """

    ARCHIVO = USUARIO / "calib_puntos.json"
    # (clave, nombre hablado, tiene imán, dirección anatómica, normal aproximada en la cabeza)
    # marco de la cabeza: x = derecha, y = anterior, z = superior
    PUNTOS = [
        ("frente", "la frente", True, "anterior", (0, 0.8, 0.6)),  # normal física de la frente
        ("cz", "Cz, arriba", True, "superior", (0, 0, 1)),
        ("temp_izq", "el temporal izquierdo", True, "izquierda", (-0.97, 0, 0.25)),
        ("temp_der", "el temporal derecho", True, "derecha", (0.97, 0, 0.25)),
        ("occipital", "el occipital", True, "posterior", (0, -1, 0.1)),
    ]
    # pasos de la calibración: (clave, orden hablada, tipo)
    PASOS = [
        ("cz", "Apoya la sonda plana sobre Cz, quieta.", "punto"),
        ("frente", "Ahora plana en la frente, quieta.", "punto"),
        ("barrido", "Desliza la sonda plana de la frente a la nuca, por arriba.", "barrido"),
        ("occipital", "Quieta en la nuca.", "punto"),
        ("temp_izq", "Ahora plana en el temporal izquierdo, quieta.", "punto"),
        ("temp_der", "Ahora plana en el temporal derecho, quieta.", "punto"),
    ]
    UMBRAL_IMAN = 150.0  # µT sin calibrar; Tierra ~45; imanes grandes saturan (>4800) al apoyar
    VERSION = 5  # se descartan las anteriores (en Cz se pedía "lápiz vertical": tomaba el mango como eje)
    MM_POR_VUELTA = 30.0  # profundidad por vuelta completa del lápiz

    def __init__(self, anatomia):
        self.anatomia = {k: np.array(v, float) for k, v in anatomia.items()}
        self.rots = {}
        self.activo = False
        self.correccion = Rotation.identity()
        self.calibrando = None  # (índice, t0, muestras)
        self.ultimo_iman = ""
        self.signo_x = 1.0
        self.signo_giro = 1.0
        self.toques_params = None
        self.imanes = {}  # calibración: {"tierra": µT, "campo": {punto: µT}, "umbral": {punto: µT}}
        self.en_frente = False  # lápiz apoyado en el imán de la frente
        self.imu = None
        self.eje = np.array([0, 0, 1.0])
        self.eje_medido = None  # eje del lápiz en el sensor, medido con la gravedad en la coronilla
        self.eje_montaje = None  # perpendicular hacia afuera de la cara de la sonda, medida sobre la mesa
        montaje = USUARIO / "montaje.json"
        if montaje.exists():
            cara = np.array(json.loads(montaje.read_text())["cara"], float)
            self.eje_montaje = -cara / np.linalg.norm(cara)  # la cara mira al cráneo: afuera es lo opuesto
        self.barrido = []  # orientaciones del recorrido frente -> nuca
        self.r_cabeza = Rotation.identity()
        self.residuo = 0.0
        if self.ARCHIVO.exists():
            d = json.loads(self.ARCHIVO.read_text())
            self.rots = {k: Rotation.from_quat(q) for k, q in d["puntos"].items()}
            self.signo_x = d.get("signo_x", 1.0)
            self.signo_giro = d.get("signo_giro", 1.0)
            self.toques_params = d.get("toques")
            self.imanes = d.get("imanes", {})
            self.eje_medido = np.array(d["eje"]) if d.get("eje") is not None else None
            self.barrido = [Rotation.from_quat(q) for q in d.get("barrido", [])]
            # calibraciones antiguas (sin el eje medido con la gravedad) no sirven si el sensor
            # cambió de posición dentro del lápiz: se pide calibrar de nuevo
            self.activo = (len(self.rots) == len(self.PUNTOS)
                           and (self.eje_medido is not None or self.eje_montaje is not None)
                           and d.get("version", 1) >= self.VERSION)
            if self.activo:
                self._ajustar()

    def usar_frente(self, h):
        """La frente física se muestra como el polo rostral h de este cerebro, sin tocar el ajuste:
        cerca de la frente la dirección se gira hacia h; cerca de los otros puntos no cambia."""
        cab = np.array(self.PUNTOS[[k for k, *_ in self.PUNTOS].index("frente")][4], float)
        cab /= np.linalg.norm(cab)
        h = np.asarray(h, float) / np.linalg.norm(h)
        self._giro_frente = Rotation.align_vectors([h], [cab])[0]
        self._normales_modelo = np.array([np.array(hh, float) / np.linalg.norm(hh) for *_, hh in self.PUNTOS])
        self._i_frente = [k for k, *_ in self.PUNTOS].index("frente")

    def mapear(self, h):
        """Dirección en la cabeza (física) -> dirección en el cerebro (ver usar_frente)."""
        if not hasattr(self, "_giro_frente"):
            return h
        ang = np.arccos(np.clip(self._normales_modelo @ h, -1, 1))
        w = 1 / (ang + 1e-3) ** 2
        w /= w.sum()
        d = (1 - w[self._i_frente]) * h + w[self._i_frente] * self._giro_frente.apply(h)
        return d / np.linalg.norm(d)

    def _ajustar(self):
        """Ajusta la rotación cabeza -> mundo con la perpendicular de la sonda en cada punto (la sonda
        se apoya plana, como una lupa) y el plano del recorrido frente -> nuca. El eje es el medido
        con la gravedad en Cz; el refinamiento con los demás puntos no puede alejarlo más de 15°."""
        cabeza = np.array([np.array(h, float) / np.linalg.norm(h) for *_, h in self.PUNTOS])
        ks = [k for k, *_ in self.PUNTOS]

        def ajustar_con(eje):
            mundo = np.array([self.rots[k].apply(eje) for k in ks])
            vm, vc, pesos = list(mundo), list(cabeza), [1.0] * len(mundo)
            lateral = self._lateral_del_barrido(eje, mundo)
            if lateral is not None:  # el recorrido frente -> nuca gira alrededor del eje izquierda-derecha
                vm.append(lateral)
                vc.append(np.array([1.0, 0, 0]))
                pesos.append(3.0)
            r, _ = Rotation.align_vectors(np.array(vm), np.array(vc), weights=pesos)
            err = np.degrees(np.arccos(np.clip(np.sum(r.apply(cabeza) * mundo, 1), -1, 1)))
            return float(err.mean()), r

        if self.eje_montaje is not None or self.eje_medido is not None:
            # el montaje medido sobre la mesa manda; si no hay, el medido en Cz con la gravedad
            medido = np.asarray(self.eje_montaje if self.eje_montaje is not None else self.eje_medido, float)
            eje = medido
            err, r = ajustar_con(eje)
            for _ in range(6):  # refinar con todos los puntos, con peso fuerte del medido
                esperado = r.apply(cabeza)
                en_sensor = [self.rots[k].inv().apply(v) for k, v in zip(ks, esperado)]
                nuevo = np.sum(en_sensor, axis=0) + 5 * medido
                nuevo /= np.linalg.norm(nuevo)
                if np.degrees(np.arccos(np.clip(nuevo @ medido, -1, 1))) > 15:
                    break
                e2, r2 = ajustar_con(nuevo)
                if e2 >= err:
                    break
                eje, err, r = nuevo, e2, r2
        else:  # calibraciones antiguas: probar los ejes del sensor
            err, r, eje = min(((*ajustar_con(v), v) for v in np.vstack([np.eye(3), -np.eye(3)])),
                              key=lambda t: t[0])
        self.residuo, self.eje, self.r_cabeza = err, np.asarray(eje, float), r
        self.p_vec = self.eje
        self.normales = {k: self.rots[k].apply(self.eje) for k in ks}

    def _lateral_del_barrido(self, eje, mundo):
        """Normal del plano que barre el lápiz de la frente a la nuca (= eje derecha), o None."""
        if len(self.barrido) < 10:
            return None
        N = np.array([r.apply(eje) for r in self.barrido])
        if np.degrees(np.arccos(np.clip(N[0] @ N[-1], -1, 1))) < 60:  # recorrido muy corto
            return None
        lateral = np.linalg.svd(N)[2][-1]  # los puntos del recorrido están en un plano que pasa por el centro
        der = mundo[[k for k, *_ in self.PUNTOS].index("temp_der")]
        return lateral if lateral @ der > 0 else -lateral

    def direccion(self, nombre):
        """Vector (x, y, z) del volumen para una dirección anatómica."""
        a = self.anatomia
        izq = a["izquierda"] * self.signo_x
        return {"anterior": a["anterior"], "posterior": -a["anterior"], "superior": a["superior"],
                "izquierda": izq, "derecha": -izq}[nombre]

    def guardar(self):
        self.ARCHIVO.write_text(json.dumps({
            "version": self.VERSION,
            "puntos": {k: r.as_quat().tolist() for k, r in self.rots.items()},
            "signo_x": self.signo_x, "signo_giro": self.signo_giro,
            "toques": self.toques_params, "imanes": self.imanes,
            "eje": None if self.eje_medido is None else self.eje_medido.tolist(),
            "barrido": [r.as_quat().tolist() for r in self.barrido[::3]]}, indent=2))

    T_TOQUES, T_TOQUES_VOZ = 9.0, 3.0  # tiempo para golpear; lo que tarda la orden hablada

    def iniciar_calibracion(self, imu=None):
        self.correccion = Rotation.identity()
        self.imu = imu
        if imu is not None and imu.tiene_impacto:
            imu.registro = []  # graba impactos durante toda la calibración (apoyos = no-golpes)
        self._campos, self._tierra = {}, []
        self._signos = {}  # punto -> signos leídos al acercarse a su imán
        self._t_registro = {}
        self.eje_medido, self._barrido_nuevo = None, []
        self._ref = None  # orientación del último punto registrado
        self._velocidad = Velocidad()
        self._vel = 0.0
        self._hechos = []
        self.ui = {}
        self._empezar_paso(0, f"Calibración. {self.PASOS[0][1]}")

    @staticmethod
    def _dur_voz(texto):
        return 0.5  # sin voz: medio segundo para leer la instrucción antes de aceptar el punto

    def _empezar_paso(self, i, texto):
        ahora = time.time()
        self.calibrando = (i, ahora, [])
        self._fin_voz = ahora + self._dur_voz(texto)
        self._quieto_desde, self._movio, self._ultimo_aviso = None, False, ahora
        decir(texto)

    def paso_calibracion(self, r_imu):
        """Avanza la calibración; devuelve texto de estado.
        Cada punto se registra cuando el lápiz llegó: terminó la orden hablada, se movió respecto del
        punto anterior y quedó quieto. El recorrido se graba desde que empieza a moverse hasta que para."""
        i, t0, muestras = self.calibrando
        ahora = time.time()
        campo = self.imu.campo if self.imu is not None else 0.0
        self._vel = self._velocidad(ahora, r_imu)
        self.ui = {"paso": i, "hechos": list(self._hechos), "fase": "voz", "progreso": 0.0, "campo": campo}
        if i == "toques":
            dt = ahora - t0
            self.ui.update(fase="toques", progreso=min(dt / self.T_TOQUES, 1.0),
                           toques=(self.imu.conteo_toques() - getattr(self, "_toques_inicio", 0)) % 2**32)
            if dt < self.T_TOQUES:
                return f"CALIBRANDO: golpea el lápiz 5 veces, como te acomode ({self.T_TOQUES - dt:.0f} s)"
            params, texto = DetectorToques.calibrar(self.imu.registro, t0 + self.T_TOQUES_VOZ - 1, t0 + self.T_TOQUES)
            self.imu.registro = None
            if params is not None:
                self.toques_params = params
                self.imu.detector.configurar(params)
            self._terminar_calibracion(texto)
            return "CALIBRANDO..."
        clave, orden, tipo = self.PASOS[i]
        if tipo == "punto" and self.imu is not None and self.imu.signo_reciente(0.3):
            self._signos.setdefault(clave, []).append((ahora, self.imu.signo_reciente(0.3)))
        if ahora < self._fin_voz:
            return f"CALIBRANDO: {orden}"
        if self._vel > 15:  # se está moviendo: no repetir la orden
            self._ultimo_aviso = ahora
        quieto = self._vel < 8
        self.ui["fase"] = "ir"
        if tipo == "barrido":
            if not self._movio:
                if self._vel > 15:
                    self._movio = True
                self.ui["fase"] = "empieza"
                return f"CALIBRANDO: {orden}"
            self._barrido_nuevo.append(r_imu)
            n = len(self._barrido_nuevo)
            if n > 20 and n % 4 == 0:
                self._tierra.append(campo)
            recorrido = np.degrees((self._barrido_nuevo[0].inv() * r_imu).magnitude())
            self.ui.update(fase="deslizando", progreso=min(recorrido / 140, 1.0))
            if quieto and recorrido > 60:
                self._quieto_desde = self._quieto_desde or ahora
                if ahora - self._quieto_desde > 0.8:
                    self._hechos.append("barrido")
                    return self._siguiente(i)
            else:
                self._quieto_desde = None
            return f"CALIBRANDO: desliza hasta la nuca ({recorrido:.0f}°)"
        # punto
        lejos = self._ref is None or np.degrees((self._ref.inv() * r_imu).magnitude()) > 25
        if self._ref is not None and self.PASOS[i - 1][2] == "barrido":
            lejos = True  # la nuca: ya llegó deslizando
        if not (quieto and lejos):
            self._quieto_desde = None
            muestras.clear()
            self._campos.pop(clave, None)
            if not lejos:
                return f"CALIBRANDO: {orden}"
            return "CALIBRANDO: llegando…"
        self._quieto_desde = self._quieto_desde or ahora
        self.ui.update(fase="quieto", progreso=min((ahora - self._quieto_desde) / 1.4, 1.0))
        if ahora - self._quieto_desde < 0.4:
            return "CALIBRANDO: quieto…"
        muestras.append(r_imu.as_quat())
        self._campos.setdefault(clave, []).append(campo)  # campo con la punta sobre el imán
        if ahora - self._quieto_desde < 1.4:
            return "CALIBRANDO: quieto…"
        rots = Rotation.from_quat(np.array(muestras))
        self.rots[clave] = rots.mean()
        self._ref = self.rots[clave]
        self._hechos.append(clave)
        if not hasattr(self, "_t_registro"):
            self._t_registro = {}
        self._t_registro[clave] = ahora
        if clave == "cz":  # sonda plana sobre Cz: "arriba" visto desde el sensor = perpendicular a su cara
            arriba = rots.inv().apply([0, 0, 1.0]).mean(0)
            self.eje_medido = arriba / np.linalg.norm(arriba)
        return self._siguiente(i)

    def _siguiente(self, i):
        if i + 1 < len(self.PASOS):
            self._empezar_paso(i + 1, self.PASOS[i + 1][1])
            return "CALIBRANDO..."
        self.barrido = self._barrido_nuevo
        self._ajustar()
        self._calibrar_imanes()
        if self.imu is not None and self.imu.registro is not None:
            self.calibrando = ("toques", time.time(), [])
            self._toques_inicio = self.imu.conteo_toques()
            decir("Golpea la sonda cinco veces.")
        else:
            self._terminar_calibracion("")
        return "CALIBRANDO..."

    def errores_por_punto(self):
        """Ángulo (°) entre la perpendicular medida de cada punto y la del modelo de cabeza ajustado."""
        return {k: float(np.degrees(np.arccos(np.clip(
            self.normales[k] @ self.r_cabeza.apply(np.array(h, float) / np.linalg.norm(h)), -1, 1))))
            for k, *_, h in self.PUNTOS}

    def puntos_parecidos(self):
        """Pares de puntos cuya orientación medida quedó casi igual (< 20°): la calibración no
        los distingue bien (probablemente se registró uno en el lugar del otro)."""
        ks = list(self.normales)
        return [(a, b) for i, a in enumerate(ks) for b in ks[i + 1:]
                if np.degrees(np.arccos(np.clip(self.normales[a] @ self.normales[b], -1, 1))) < 20]

    def _calibrar_imanes(self):
        """Umbral por imán: entre el campo de la Tierra (medido en el camino) y lo que marca
        la punta sobre cada imán. Así un imán débil también se detecta."""
        # campo de la Tierra: lo más bajo del trayecto (con imanes grandes, al pasar cerca satura)
        tierra = float(min(np.percentile(self._tierra, 10), 120.0)) if self._tierra else 50.0
        signos = {}
        for k, lista in getattr(self, "_signos", {}).items():
            fin = self._t_registro.get(k, 1e18) if hasattr(self, "_t_registro") else 1e18
            vals = [sg for t, sg in lista if fin - 3 < t <= fin + 0.1]  # los últimos 3 s antes de registrar
            if len(vals) >= 5 and abs(np.mean(vals)) > 0.6:
                signos[k] = int(np.sign(np.mean(vals)))
        campo = {k: float(np.median(v)) for k, v in self._campos.items() if v}
        umbral = {}
        for k, c in campo.items():
            if c > 2.0 * tierra:  # media geométrica (el campo cae muy rápido), con margen sobre la Tierra
                umbral[k] = float(np.sqrt(max(1.3 * tierra, 40.0) * c))
        self.imanes = {"tierra": tierra, "campo": campo, "umbral": umbral, "signo": signos}
        nombres = {k: n for k, n, *_ in self.PUNTOS}
        self._sin_iman = [nombres[k] for k, *_ in self.PUNTOS if k not in umbral]

    def umbral_superficie(self):
        """Campo que indica "lápiz apoyado en un imán", para completar la superficie."""
        tierra = self.imanes.get("tierra", 45.0)
        return max(300.0, 4 * tierra)

    def umbral_iman(self, punto):
        return self.imanes.get("umbral", {}).get(punto, self.UMBRAL_IMAN)

    def hay_iman(self, campo, punto):
        u = self.imanes.get("umbral", {})
        if u and punto not in u:  # calibrado: ese punto no tiene imán detectable
            return False
        return campo > self.umbral_iman(punto)

    def _terminar_calibracion(self, texto_toques):
        self.calibrando = None
        self.activo = True
        self.guardar()
        sin = getattr(self, "_sin_iman", [])
        texto_imanes = (f"No detecté el imán en {', '.join(sin)}." if sin else "Todos los imanes detectados.")
        nombres = {"cz": "Cz", "frente": "frente", "temp_izq": "temporal izq.", "temp_der": "temporal der.",
                   "occipital": "nuca"}
        malos = [nombres[k] for k, e in self.errores_por_punto().items() if e > 25]
        texto_malos = (f"Ojo: {', '.join(malos)} no calzó; repite con P (sonda plana)." if malos else "")
        decir(f"Calibración lista. Error {self.residuo:.0f} grados. {texto_malos} {texto_imanes} {texto_toques}", 8)

    def volver_a_punto(self, r_imu, punto="frente"):
        """Superficie: corrige la orientación para que la sonda quede exactamente como se midió en
        ese punto al calibrar (la vista queda en su ancla: la frente -> polo rostral)."""
        n = (self.correccion * r_imu).apply(self.p_vec)
        objetivo = self.normales[punto]
        eje_giro = np.cross(n, objetivo)
        seno, coseno = np.linalg.norm(eje_giro), float(np.clip(n @ objetivo, -1, 1))
        if seno > 1e-6:
            self.correccion = Rotation.from_rotvec(eje_giro / seno * np.arctan2(seno, coseno)) * self.correccion
        self.ultimo_iman = punto

    def giro_lapiz(self, r_antes, r_ahora):
        """Grados girados sobre el eje del lápiz entre dos lecturas (positivo = más hondo)."""
        return float(np.degrees((r_antes.inv() * r_ahora).as_rotvec() @ self.eje)) * self.signo_giro

    def ubicar(self, r_imu, campo, quieto, signo=0):
        """Devuelve (dirección unitaria en el volumen, punto más cercano)."""
        n = (self.correccion * r_imu).apply(self.p_vec)
        ang = {k: np.arccos(np.clip(n @ self.normales[k], -1, 1)) for k in self.normales}
        cercano = min(ang, key=ang.get)
        # imán bajo la punta: el punto calibrado más cercano que tenga imán y lo esté marcando.
        # Con el lápiz quieto, la orientación se corrige exactamente a ese punto (corrige la deriva).
        con_iman = [k for k in self.normales if self.hay_iman(campo, k) and ang[k] < np.radians(45)]
        aprendidos = self.imanes.get("signo", {})  # signo de cada imán aprendido al calibrar
        if signo and aprendidos:
            iguales = [k for k, sg in aprendidos.items() if sg == signo and k in self.normales]
            if len(iguales) == 1 and campo > self.umbral_superficie():
                con_iman = iguales  # polaridad única (p. ej. Cz al revés): ese punto, aunque la orientación esté corrida
            else:
                con_iman = [k for k in con_iman if aprendidos.get(k, signo) == signo]
        punto = min(con_iman, key=ang.get) if con_iman else None
        if punto is not None and quieto:
            objetivo = self.normales[punto]
            eje_giro = np.cross(n, objetivo)
            seno, coseno = np.linalg.norm(eje_giro), float(np.clip(n @ objetivo, -1, 1))
            if seno > 1e-6:
                self.correccion = Rotation.from_rotvec(eje_giro / seno * np.arctan2(seno, coseno)) * self.correccion
                n = (self.correccion * r_imu).apply(self.p_vec)
            cercano = self.ultimo_iman = punto
        self.en_frente = punto == "frente"  # la superficie (espacio) se completa aquí
        h = self.mapear(self.r_cabeza.inv().apply(n))  # x = derecha, y = anterior, z = superior
        d = h[0] * self.direccion("derecha") + h[1] * self.direccion("anterior") + h[2] * self.direccion("superior")
        return d / np.linalg.norm(d), cercano


def plano_desde(d, anatomia):
    """Rotación cuyo eje z (normal del corte) es d, orientada para un modelo que mira al público.

    'Arriba' en la imagen:
      - frente, temporales y occipital (lápiz bajo ~30° de elevación): lo superior
      - vértice (Cz): la nuca (la cara queda hacia el que mira, abajo en la pantalla)
    Se construye llevando la referencia del vértice por el meridiano hasta la punta (sin
    singularidades salvo bajo el cuello) y girándola de a poco hacia 'superior arriba' según
    baja la elevación. Así pasar por encima de la cabeza no invierte adelante/atrás. El único
    salto posible queda arriba-atrás de la cabeza, al cruzar la línea media posterior.
    """
    n = d / np.linalg.norm(d)
    sup = np.array(anatomia["superior"], float)
    post = -np.array(anatomia["anterior"], float)
    e = float(np.clip(n @ sup, -1, 1))  # seno de la elevación
    # 1) referencia del vértice (nuca arriba) transportada por el arco más corto vértice -> n
    eje = np.cross(sup, n)
    if np.linalg.norm(eje) < 1e-9:
        arriba = post if e > 0 else -post
    else:
        arriba = Rotation.from_rotvec(eje / np.linalg.norm(eje) * np.arccos(e)).apply(post)
    # 2) girarla hacia 'superior arriba' a medida que baja la elevación
    w = float(np.clip((np.sin(np.radians(70)) - e) / (np.sin(np.radians(70)) - 0.5), 0, 1))
    w = w * w * (3 - 2 * w)  # transición suave: 0 sobre ~70°, 1 bajo 30°
    if w > 0:
        s_proj = sup - e * n
        s_proj /= np.linalg.norm(s_proj)
        psi = np.arctan2(n @ np.cross(arriba, s_proj), arriba @ s_proj)
        arriba = Rotation.from_rotvec(n * w * psi).apply(arriba)
    v = -(arriba - np.dot(arriba, n) * n)  # filas de la imagen crecen hacia abajo
    v /= np.linalg.norm(v)
    u = np.cross(v, n)  # (si el volumen está en espejo, la imagen se voltea al dibujarla)
    return Rotation.from_matrix(np.column_stack([u, v, n]))


def alcance(vol, rot):
    """Mitad del espesor del volumen medido a lo largo de la normal del plano (en vóxeles)."""
    mitad = (np.array(vol.shape[::-1]) - 1) / 2  # (x, y, z)
    return float(np.abs(rot.apply([0, 0, 1])) @ mitad)


def dibujar_barra(img, frac, arriba="adelante", abajo="atras", invertir=False, y0=40):
    """Barra vertical a la derecha. invertir=True: se llena desde arriba (profundidad)."""
    h, w = img.shape[:2]
    x0, y1 = w - 30, h - 70
    y0 = max(y0, 150) if invertir else y0  # deja lugar al dibujo de la cabeza
    cv2.rectangle(img, (x0, y0), (x0 + 12, y1), (90, 90, 90), 1)
    y = int(y0 + frac * (y1 - y0)) if invertir else int(y1 - frac * (y1 - y0))
    cv2.rectangle(img, (x0, y0) if invertir else (x0, y), (x0 + 12, y) if invertir else (x0 + 12, y1),
                  (0, 200, 255), -1)
    cv2.putText(img, arriba, (x0 - 70, y0 - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 200, 255), 1, cv2.LINE_AA)
    cv2.putText(img, abajo, (x0 - 40, y1 + 16), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 200, 255), 1, cv2.LINE_AA)


def dibujar_cabeza(img, puntos, d, cercano, campo, r=55):
    """Vista superior de la cabeza: frente arriba, puntos calibrados y posición de la punta."""
    h, w = img.shape[:2]
    c = np.array([w - r - 15, r + 15])
    cv2.circle(img, tuple(c), r, (200, 200, 200), 1, cv2.LINE_AA)
    cv2.line(img, (c[0] - 6, c[1] - r), (c[0], c[1] - r - 10), (200, 200, 200), 1)  # nariz
    cv2.line(img, (c[0] + 6, c[1] - r), (c[0], c[1] - r - 10), (200, 200, 200), 1)
    ant, der = puntos.direccion("anterior"), puntos.direccion("derecha")

    def en_pantalla(v):  # vista superior: anterior arriba, derecha del paciente a la derecha
        return tuple((c + np.array([v @ der, -(v @ ant)]) * r * 0.85).astype(int))

    for clave, _, iman, nombre, _ in puntos.PUNTOS:
        color = (0, 0, 255) if clave == cercano else (160, 160, 160)
        cv2.circle(img, en_pantalla(puntos.direccion(nombre)), 6 if iman else 4, color,
                   -1 if iman else 1, cv2.LINE_AA)
    p = en_pantalla(d)
    cv2.circle(img, p, 5, (0, 255, 255), -1, cv2.LINE_AA)
    if getattr(puntos, "en_frente", False) and campo > 0:
        cv2.putText(img, "IMAN", (c[0] - 18, c[1] + r + 18), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)


def dibujar_ejes(img, rot, origen=(60, 60), largo=40):
    for eje, color in zip(np.eye(3), [(0, 0, 255), (0, 255, 0), (255, 0, 0)]):
        d = rot.apply(eje)
        p = (int(origen[0] + d[0] * largo), int(origen[1] + d[1] * largo))
        cv2.line(img, origen, p, color, 2)


class Pantalla:
    """Ventana con pygame (SDL): pantalla completa fiable en X11/Wayland y elección de monitor.
    OpenCV solo se usa para dibujar sobre las imágenes.

    En pantallas anchas el corte (cuadrado) queda al centro y sobran franjas a los lados:
    ahí van paneles (vistas 3D a la izquierda, información a la derecha).
    El mouse sirve de puntero (nombre de la estructura bajo el cursor, clic = decirlo) y,
    arrastrando con el botón apretado, para recorrer la cabeza cuando no hay lápiz."""

    def __init__(self, titulo, completa, lado, monitor):
        os.environ.setdefault("PYGAME_HIDE_SUPPORT_PROMPT", "1")
        import pygame
        self.pg = pygame
        pygame.display.init()
        self.prueba = os.environ.get("HOA_PRUEBA")  # "1920x1080;960,500;captura.png" (sin ventana)
        if self.prueba:
            tam, pos, self.archivo_prueba = self.prueba.split(";")
            self.ancho, self.alto = (int(v) for v in tam.split("x"))
            self.sup = pygame.Surface((self.ancho, self.alto))
            self.mouse_prueba = tuple(int(v) for v in pos.split(",")) if pos else None
            self.cuadros = 0
        elif completa:
            tamanos = pygame.display.get_desktop_sizes()
            monitor = min(monitor, len(tamanos) - 1)
            self.ancho, self.alto = tamanos[monitor]
            self.sup = pygame.display.set_mode((self.ancho, self.alto), pygame.FULLSCREEN, display=monitor)
            pygame.mouse.set_visible(False)
        else:
            self.ancho = self.alto = lado
            self.sup = pygame.display.set_mode((lado, lado))
        if not self.prueba:
            pygame.display.set_caption(titulo)
        self.lado = min(self.ancho, self.alto)
        self.panel = (self.ancho - self.lado) // 2 if self.ancho >= 1.25 * self.alto else 0
        self.reloj = pygame.time.Clock()
        self.mouse_px = (self.ancho // 2, self.alto // 2)
        self.t_mouse = 0.0  # último movimiento del mouse
        self._rueda, self._arrastre, self._click = 0, [0, 0], None
        self._apretado, self._movio = None, False
        self._mouse_visible = not completa
        self.rect_img = (0, 0, self.lado)
        pygame.font.init()
        self._fuentes = {}

    @property
    def ancha(self):
        return self.panel >= 200

    def mostrar(self, img_bgr, textos=(), panel_izq=None, textos_der=()):
        """Dibuja la imagen cuadrada centrada, ampliada al alto de la pantalla.
        textos: [(texto, x, y, tamaño, rgb)] en coordenadas de la imagen base;
        panel_izq: imagen BGR para la franja izquierda; textos_der: en píxeles de la franja derecha.
        pygame dibuja tildes y ñ."""
        base = img_bgr.shape[0]
        if img_bgr.shape[0] != self.lado:
            img_bgr = cv2.resize(img_bgr, (self.lado, self.lado), interpolation=cv2.INTER_LINEAR)
        self.sup.fill((0, 0, 0))
        x0, y0 = (self.ancho - self.lado) // 2, (self.alto - self.lado) // 2
        self.rect_img = (x0, y0, self.lado)
        self.sup.blit(self._superficie(img_bgr), (x0, y0))
        if panel_izq is not None and self.ancha:
            self.sup.blit(self._superficie(panel_izq), (0, 0))
        esc = self.lado / base
        for texto, x, y, tam, color in textos:
            self._texto(texto, x0 + x * esc, y0 + y * esc, int(tam * esc), color)
        xd = x0 + self.lado + 24
        for texto, x, y, tam, color in textos_der:
            self._texto(texto, xd + x, y, tam, color)
        if self.prueba:
            self.cuadros += 1
            if self.cuadros == 3:
                self.pg.image.save(self.sup, self.archivo_prueba)
            return
        self.pg.display.flip()

    def _superficie(self, img_bgr):
        return self.pg.surfarray.make_surface(np.ascontiguousarray(img_bgr[..., ::-1].swapaxes(0, 1)))

    def _texto(self, texto, x, y, tam, color):
        if not texto:
            return
        fuente = self._fuente(tam)
        for dx, dy in ((-2, 0), (2, 0), (0, -2), (0, 2)):  # contorno negro para leer sobre la imagen
            self.sup.blit(fuente.render(texto, True, (0, 0, 0)), (x + dx, y + dy))
        self.sup.blit(fuente.render(texto, True, color), (x, y))

    def _fuente(self, tam):
        if tam not in self._fuentes:
            self._fuentes[tam] = self.pg.font.Font(None, tam)
        return self._fuentes[tam]

    def tecla(self):
        """Código de la última tecla (como cv2.waitKey) o -1; 27 si se cierra la ventana."""
        self.reloj.tick(60)
        k = -1
        pg = self.pg
        if self.prueba:
            if self.mouse_prueba:
                self.mouse_px, self.t_mouse = self.mouse_prueba, time.time()
                if self.cuadros == 1:
                    self._click = self.mouse_prueba
            return 27 if self.cuadros >= 3 else (ord("e") if self.cuadros == 0 and "regiones" in self.archivo_prueba else -1)
        for ev in pg.event.get():
            if ev.type == pg.QUIT:
                return 27
            if ev.type == pg.MOUSEMOTION:
                self.mouse_px, self.t_mouse = ev.pos, time.time()
                if self._apretado is not None:
                    self._arrastre[0] += ev.rel[0]
                    self._arrastre[1] += ev.rel[1]
                    if abs(ev.pos[0] - self._apretado[0]) + abs(ev.pos[1] - self._apretado[1]) > 6:
                        self._movio = True
            if ev.type == pg.MOUSEBUTTONDOWN and ev.button == 1:
                self._apretado, self._movio, self.t_mouse = ev.pos, False, time.time()
            if ev.type == pg.MOUSEBUTTONUP and ev.button == 1:
                if self._apretado is not None and not self._movio:
                    self._click = ev.pos
                self._apretado = None
            if ev.type == pg.MOUSEWHEEL:
                self._rueda += ev.y
                self.t_mouse = time.time()
            if ev.type == pg.KEYDOWN:
                flechas = {pg.K_UP: ord("s"), pg.K_DOWN: ord("w")}  # arriba = hacia la superficie
                if ev.key == pg.K_ESCAPE:
                    k = 27
                elif ev.key == pg.K_F2:
                    k = TECLA_PAQUETES
                elif ev.key in flechas:
                    k = flechas[ev.key]
                else:
                    k = ord(ev.unicode) if ev.unicode else -1
        return k

    def rueda(self):
        """Pasos de la rueda desde la última lectura (positivo = hacia adentro)."""
        r, self._rueda = self._rueda, 0
        return -r

    def arrastre(self):
        """Píxeles arrastrados con el botón apretado desde la última lectura."""
        a, self._arrastre = tuple(self._arrastre), [0, 0]
        return a

    def click(self):
        """Posición (u, v) 0-1 dentro del corte del último clic sin arrastre, o None."""
        c, self._click = self._click, None
        return self.en_imagen(c) if c is not None else None

    def en_imagen(self, pos=None):
        """Posición (u, v) 0-1 dentro del corte, o None si está fuera."""
        x, y = pos if pos is not None else self.mouse_px
        x0, y0, lado = self.rect_img
        u, v = (x - x0) / lado, (y - y0) / lado
        return (u, v) if 0 <= u < 1 and 0 <= v < 1 else None

    def mostrar_mouse(self, visible):
        if self.prueba:
            return
        if visible != self._mouse_visible:
            self.pg.mouse.set_visible(visible)
            self._mouse_visible = visible

    def cerrar(self):
        if not self.prueba:
            self.pg.display.quit()


class ConfigSonda:
    """Configuración de la sonda (montaje del sensor): 3 posiciones quietas sobre la mesa / en la
    mano. En cada una se mide hacia dónde queda "arriba" visto desde el sensor (gravedad):
      cara abajo -> la cara detectora es lo opuesto a "arriba"; mango arriba -> el mango;
      cara arriba -> confirma la cara. Se guarda en montaje.json."""

    PASOS = [
        ("cara_abajo", "Deja la sonda en la mesa con la cara detectora hacia ABAJO."),
        ("mango_arriba", "Sostén la sonda con el mango hacia ARRIBA y la cara mirándote."),
        ("cara_arriba", "Deja la sonda en la mesa con la cara detectora hacia ARRIBA."),
    ]
    T_QUIETO = 2.0  # s quieta para medir

    def __init__(self):
        self.i, self.medidas, self.muestras = 0, {}, []
        self._vel, self._quieto_desde, self._ref = Velocidad(), None, None
        self.ui = {"paso": 0, "fase": "mover", "progreso": 0.0, "hechos": []}
        self.terminado, self.resultado = False, None

    def paso(self, r_imu):
        ahora = time.time()
        vel = self._vel(ahora, r_imu)
        movio = self._ref is None or np.degrees((self._ref.inv() * r_imu).magnitude()) > 45
        if vel < 3 and movio:
            self._quieto_desde = self._quieto_desde or ahora
            dt = ahora - self._quieto_desde
            if dt > 0.5:
                self.muestras.append(r_imu.inv().apply([0, 0, 1.0]))  # "arriba" visto desde el sensor
            self.ui.update(fase="quieto", progreso=min(dt / self.T_QUIETO, 1.0))
            if dt >= self.T_QUIETO:
                m = np.mean(self.muestras, axis=0)
                self.medidas[self.PASOS[self.i][0]] = m / np.linalg.norm(m)
                self.ui["hechos"].append(self.PASOS[self.i][0])
                self._ref, self._quieto_desde, self.muestras = r_imu, None, []
                self.i += 1
                if self.i == len(self.PASOS):
                    self._calcular()
                else:
                    self.ui.update(paso=self.i, fase="mover", progreso=0.0)
        else:
            self._quieto_desde, self.muestras = None, []
            self.ui.update(fase="mover" if movio else "cambia", progreso=0.0)

    def _calcular(self):
        ab, ar, mango = -self.medidas["cara_abajo"], self.medidas["cara_arriba"], self.medidas["mango_arriba"]
        dif = np.degrees(np.arccos(np.clip(ab @ ar, -1, 1)))
        cara = (ab + ar) / np.linalg.norm(ab + ar)
        m = mango - (mango @ cara) * cara
        ang_cm = np.degrees(np.arccos(np.clip(mango @ cara, -1, 1)))
        self.terminado = True
        if dif > 20 or not 60 < ang_cm < 120 or np.linalg.norm(m) < 1e-3:
            self.resultado = (False, f"No calzó (cara abajo vs arriba {dif:.0f}°, cara-mango {ang_cm:.0f}°). Repite con M.")
            return
        m /= np.linalg.norm(m)
        (USUARIO / "montaje.json").write_text(json.dumps({
            "cara": cara.round(4).tolist(), "mango": m.round(4).tolist(),
            "nota": "ejes del sensor (x,y,z): cara = hacia el cráneo; mango. Medido con la configuración de la sonda."},
            indent=2))
        self.resultado = (True, f"Sonda configurada (diferencia {dif:.0f}°). Ahora calibra con P.")


def pantalla_config_sonda(cfg, lado=720):
    """Guía visual de la configuración de la sonda: las 3 posiciones dibujadas, la actual resaltada."""
    img = np.full((lado, lado, 3), (32, 24, 20), np.uint8)
    e = lado / 720
    ui = cfg.ui
    GRIS, VERDE, AMARILLO, ROJO = (110, 110, 110), (90, 200, 90), (40, 210, 255), (60, 60, 255)
    textos = [("Configurar la sonda · posición "
               f"{min(ui['paso'] + 1, 3)} de 3", 24, 18, 28, (200, 200, 200))]
    if not cfg.terminado:
        textos.append((cfg.PASOS[ui["paso"]][1], 24, 56, 30, (255, 255, 255)))
    pulso = 0.5 + 0.5 * np.sin(time.time() * 6)

    def lupa_lado(cx, cy, cara_abajo, color):
        # vista de costado sobre la mesa: cabeza (rectángulo) + mango; la cara detectora en amarillo
        cv2.line(img, (cx - 90, cy + 22), (cx + 110, cy + 22), (90, 90, 90), 2)  # mesa
        y0 = cy + 6 if cara_abajo else cy - 2
        cv2.rectangle(img, (cx - 60, y0 - 10), (cx + 20, y0 + 6), color, 2)
        ycara = y0 + 6 if cara_abajo else y0 - 10
        cv2.line(img, (cx - 58, ycara), (cx + 18, ycara), AMARILLO, 4)
        cv2.line(img, (cx + 20, y0 - 2), (cx + 100, y0 - 2), color, 5)  # mango

    def lupa_frente(cx, cy, color):
        cv2.circle(img, (cx, cy + 20), 32, AMARILLO, -1)  # la cara mirando hacia quien mira
        cv2.circle(img, (cx, cy + 20), 32, color, 2)
        cv2.line(img, (cx, cy - 12), (cx, cy - 80), color, 6)  # mango hacia arriba

    cajas = [(140, 330), (360, 330), (580, 330)]
    nombres = ["Cara abajo", "Mango arriba", "Cara arriba"]
    for j, (cx, cy) in enumerate(cajas):
        clave = cfg.PASOS[j][0]
        hecho, actual = clave in ui["hechos"], j == ui["paso"] and not cfg.terminado
        col = VERDE if hecho else ((255, 255, 255) if actual else GRIS)
        cx, cy = int(cx * e), int(cy * e)
        if j == 1:
            lupa_frente(cx, cy, col)
        else:
            lupa_lado(cx, cy, j == 0, col)
        if actual:
            cv2.rectangle(img, (cx - 105, cy - 110), (cx + 115, cy + 60), ROJO, int(2 + 2 * pulso))
        textos.append((nombres[j] + (" (listo)" if hecho else ""), cx / e - 60, 410, 24,
                       (120, 255, 140) if hecho else ((255, 255, 255) if actual else (160, 160, 160))))
    textos.append(("Amarillo = cara detectora (la que mira al cráneo)", 24, 455, 22, (230, 210, 120)))
    if cfg.terminado:
        ok, texto = cfg.resultado
        textos.append((texto, 24, 505, 30, (120, 255, 140) if ok else (255, 140, 120)))
    else:
        fase, prog = ui["fase"], ui["progreso"]
        estado = {"mover": ("Pon la sonda como en el dibujo y suéltala / quédate quieto", (255, 220, 60)),
                  "cambia": ("Cambia a la posición del dibujo", (255, 160, 80)),
                  "quieto": ("Midiendo… no la muevas", (120, 255, 140))}[fase]
        textos.append((estado[0], 24, 505, 30, estado[1]))
        if fase == "quieto":
            x0, y0, w, h = int(24 * e), int(548 * e), int(672 * e), int(16 * e)
            cv2.rectangle(img, (x0, y0), (x0 + w, y0 + h), (80, 80, 80), 1)
            cv2.rectangle(img, (x0, y0), (x0 + int(w * prog), y0 + h), VERDE, -1)
    textos.append(("M: repetir · Esc: salir", 24, 640, 22, (150, 150, 150)))
    return img, textos


ZONAS_IMAN = [(20, 150, (120, 120, 120), "lejos", "acércala", (200, 200, 200)),
              (150, 2500, (90, 200, 90), "bien", "distancia ideal", (120, 255, 140)),
              (2500, 4800, (40, 210, 255), "cerca", "no te acerques más", (255, 220, 60)),
              (4800, 6000, (60, 60, 255), "demasiado cerca", "aléjala 1-2 cm", (255, 110, 110))]


def dibujar_distancia(img, campo, x, y, w, h, escala_txt=1.0):
    """Semáforo de distancia al imán (gris lejos / verde bien / amarillo cerca / rojo saturado), con
    marca en el valor actual. Dibuja en img (coordenadas de la imagen) y devuelve el texto para
    pygame [(texto, x, y, tamaño, color)] en esas mismas coordenadas."""
    log = lambda v: (np.log10(v) - np.log10(20)) / (np.log10(6000) - np.log10(20))
    for lo, hi, col, *_ in ZONAS_IMAN:
        cv2.rectangle(img, (x + int(w * log(lo)), y), (x + int(w * log(hi)), y + h), col, -1)
    v = min(max(campo, 20), 5999)
    pos = x + int(w * np.clip(log(v), 0, 1))
    cv2.fillPoly(img, [np.array([[pos, y - 3], [pos - 9, y - 16], [pos + 9, y - 16]], np.int32)], (255, 255, 255))
    cv2.line(img, (pos, y - 3), (pos, y + h + 3), (255, 255, 255), 3)
    zona = next((z for z in ZONAS_IMAN if z[0] <= v < z[1]), ZONAS_IMAN[0])
    valor = "saturado" if campo >= 9999 else f"{campo:.0f} µT"
    return [(f"Imán: {zona[3]} — {zona[4]} ({valor})", x, y + h + 4, int(24 * escala_txt), zona[5])]


def pantalla_imanes(imu, cara, lado=720, hist=None):
    """Ventana de imanes: intensidad del campo y polaridad respecto de la cara detectora.
    Si el campo sale del imán hacia la sonda, el polo Norte mira hacia afuera."""
    img = np.full((lado, lado, 3), (32, 24, 20), np.uint8)
    e = lado / 720
    textos = [("Imanes: polaridad", 24, 18, 30, (200, 200, 200)),
              ("Pon la cara detectora mirando al imán, a 1-2 cm, sin tocarlo.", 24, 56, 26, (255, 255, 255))]
    if imu is None or imu.mag is None or time.time() - imu.ultima > 1:
        textos.append(("Esperando datos de la sonda…", 24, 320, 34, (255, 200, 120)))
        return img, textos
    m = imu.mag
    saturado = bool(np.any(m >= 9999))
    b = float(np.linalg.norm(m)) if not saturado else 9999.0
    x0, y0 = int(24 * e), int(120 * e)
    textos += [(t, tx / e, ty / e, tam, c) for t, tx, ty, tam, c in
               dibujar_distancia(img, b, x0, y0, int(672 * e), int(34 * e), 1.2)]
    if saturado:
        textos.append(("Muy cerca: aleja la sonda hasta volver al verde.", 24, 330, 32, (255, 140, 120)))
        return img, textos
    if b < 150:
        textos.append(("Acerca la sonda a un imán (sin tocarlo).", 24, 330, 32, (255, 220, 120)))
        return img, textos
    componente = float(m @ cara) / b  # +1: el campo entra a la sonda por la cara (sale del imán... ver abajo)
    # el campo sale del imán por su polo Norte y viaja hacia la sonda, que está frente a la cara:
    # en la sonda apunta desde el imán hacia afuera = opuesto a la cara (que mira al imán)
    norte = componente < 0
    if hist is not None:
        hist.append(componente)
        del hist[:-30]
        componente = float(np.median(hist))
        norte = componente < 0
    col = (90, 160, 255) if norte else (60, 150, 255)
    col_txt = (130, 200, 255) if norte else (255, 170, 90)
    textos.append(("NORTE hacia la sonda" if norte else "SUR hacia la sonda", 24, 250, 64, col_txt))
    seguridad = abs(componente)
    textos.append((f"Claridad: {seguridad * 100:.0f} %  (más de 60 % es confiable)", 24, 330, 24,
                   (200, 200, 200) if seguridad > 0.6 else (255, 200, 120)))
    # indicador Sur <-> Norte
    cx, cy, ancho = int(360 * e), int(430 * e), int(300 * e)
    cv2.line(img, (cx - ancho, cy), (cx + ancho, cy), (120, 120, 120), 3)
    pos = int(cx + (-componente) * ancho * 0.85)
    cv2.circle(img, (pos, cy), int(16 * e), (255, 255, 255), -1, cv2.LINE_AA)
    cv2.circle(img, (pos, cy), int(16 * e), col, 3, cv2.LINE_AA)
    textos.append(("SUR", 24, 418, 26, (255, 170, 90)))
    textos.append(("NORTE", 610, 418, 26, (130, 200, 255)))
    textos.append(("B: tecla para salir (o Esc)", 24, 640, 22, (150, 150, 150)))
    return img, textos


def pantalla_calibracion(puntos, lado=720):
    """Guía visual de la calibración: cabeza de perfil y desde arriba con el punto a tocar,
    estado en vivo (escuchar / ir / quieto / deslizar / golpes) y pasos hechos.
    Devuelve (imagen BGR, textos para pygame)."""
    ui = puntos.ui
    img = np.full((lado, lado, 3), (32, 24, 20), np.uint8)
    textos = []
    if not ui:
        return img, textos
    e = lado / 720
    paso = ui["paso"]
    pasos = [k for k, *_ in puntos.PASOS] + ["toques"]
    actual = "toques" if paso == "toques" else pasos[paso]
    hechos = set(ui["hechos"])
    pulso = 0.5 + 0.5 * np.sin(time.time() * 6)
    GRIS, VERDE, AMARILLO, ROJO = (110, 110, 110), (90, 200, 90), (40, 210, 255), (60, 60, 255)

    def color(k):
        return VERDE if k in hechos else (AMARILLO if k == actual else GRIS)

    def punto(c, k):
        cv2.circle(img, c, int(11 * e), color(k), -1 if k in hechos else 2, cv2.LINE_AA)
        if k == actual:
            cv2.circle(img, c, int((18 + 10 * pulso) * e), ROJO, 3, cv2.LINE_AA)

    # perfil (la cara mira a la derecha)
    cp, r = (int(190 * e), int(330 * e)), int(120 * e)
    cv2.circle(img, cp, r, (200, 200, 200), 2, cv2.LINE_AA)
    nariz = np.array([[cp[0] + r - 4, cp[1] - 10], [cp[0] + r + 22, cp[1] + 18], [cp[0] + r - 4, cp[1] + 30]], np.int32)
    cv2.polylines(img, [nariz], False, (200, 200, 200), 2, cv2.LINE_AA)
    cv2.ellipse(img, (cp[0] - int(10 * e), cp[1] + int(10 * e)), (int(10 * e), int(18 * e)), 0, 0, 360, (150, 150, 150), 2)
    perfil = {"cz": (0, -1), "frente": (0.94, -0.34), "occipital": (-1, -0.05)}
    if actual == "barrido" or "barrido" in hechos:  # arco frente -> por arriba -> nuca
        prog = 1.0 if "barrido" in hechos else ui.get("progreso", 0.0)
        a0, a1 = -20, -180
        cv2.ellipse(img, cp, (r, r), 0, a0, a1, GRIS, 4, cv2.LINE_AA)
        if prog > 0:
            cv2.ellipse(img, cp, (r, r), 0, a0, a0 + (a1 - a0) * prog, VERDE, 6, cv2.LINE_AA)
        fin = (cp[0] - r, cp[1] + 6)
        cv2.arrowedLine(img, (cp[0] - r, cp[1] - 14), fin, AMARILLO if actual == "barrido" else VERDE, 3, tipLength=0.6)
    for k, (x, y) in perfil.items():
        punto((int(cp[0] + x * r), int(cp[1] + y * r)), k)
    textos.append(("De lado", 150, 465, 22, (200, 200, 200)))
    # desde arriba (la nariz hacia arriba; la derecha del modelo a la derecha)
    ct = (int(530 * e), int(330 * e))
    cv2.circle(img, ct, r, (200, 200, 200), 2, cv2.LINE_AA)
    cv2.polylines(img, [np.array([[ct[0] - 12, ct[1] - r + 2], [ct[0], ct[1] - r - 20], [ct[0] + 12, ct[1] - r + 2]], np.int32)],
                  False, (200, 200, 200), 2, cv2.LINE_AA)
    arriba = {"cz": (0, 0), "frente": (0, -0.85), "occipital": (0, 0.85), "temp_izq": (-0.88, 0), "temp_der": (0.88, 0)}
    if actual == "barrido":
        cv2.arrowedLine(img, (ct[0], int(ct[1] - 0.7 * r)), (ct[0], int(ct[1] + 0.7 * r)), AMARILLO, 3, tipLength=0.08)
    for k, (x, y) in arriba.items():
        punto((int(ct[0] + x * r), int(ct[1] + y * r)), k)
    textos.append(("Desde arriba", 478, 465, 22, (200, 200, 200)))
    textos.append(("izq.", 362, 340, 20, (170, 170, 170)))
    textos.append(("der.", 664, 340, 20, (170, 170, 170)))

    # textos: paso, instrucción y estado
    n_paso = len(pasos) if paso == "toques" else paso + 1
    textos.append((f"Calibración · paso {n_paso} de {len(pasos)}", 24, 18, 28, (200, 200, 200)))
    orden = "Golpea la sonda 5 veces, como te acomode" if paso == "toques" else puntos.PASOS[paso][1]
    textos.append((orden, 24, 56, 34, (255, 255, 255)))
    fase, prog = ui.get("fase"), ui.get("progreso", 0.0)
    estados = {"voz": ("Lee la instrucción…", (180, 180, 180)), "ir": ("Ve al punto marcado", (255, 220, 60)),
               "quieto": ("Quieto…", (120, 255, 140)), "empieza": ("Empieza a deslizar", (255, 220, 60)),
               "deslizando": ("Deslizando… sigue hasta la nuca", (120, 255, 140)),
               "toques": (f"Golpes detectados: {ui.get('toques', 0)}", (120, 255, 140))}
    texto, col = estados.get(fase, ("", (255, 255, 255)))
    textos.append((texto, 24, 505, 36, col))
    if fase in ("quieto", "deslizando", "toques"):
        x0, y0, w, h = int(24 * e), int(552 * e), int(672 * e), int(16 * e)
        cv2.rectangle(img, (x0, y0), (x0 + w, y0 + h), (80, 80, 80), 1)
        cv2.rectangle(img, (x0, y0), (x0 + int(w * prog), y0 + h), VERDE, -1)
    if paso != "toques" and ui.get("campo") is not None:  # semáforo: acercarse sin pasarse
        textos += [(t, tx / e, ty / e, tam, c) for t, tx, ty, tam, c in
                   dibujar_distancia(img, ui["campo"], int(24 * e), int(592 * e), int(672 * e), int(14 * e))]
    # pasos
    nombres = {"cz": "Cz", "frente": "Frente", "barrido": "Recorrido", "occipital": "Nuca",
               "temp_izq": "T. izq.", "temp_der": "T. der.", "toques": "Golpes"}
    ancho = 672 / len(pasos)
    for j, k in enumerate(pasos):
        cx = int((24 + ancho * (j + 0.5)) * e)
        cv2.circle(img, (cx, int(668 * e)), int(9 * e), color(k), -1 if k in hechos or k == actual else 2, cv2.LINE_AA)
        textos.append((nombres[k], 24 + ancho * j + ancho / 2 - 28, 684, 20, (230, 230, 230) if k == actual else (160, 160, 160)))
    return img, textos


def polo_rostral(seg, factor_seg, centro, anatomia):
    """Dirección (derecha, anterior, superior) del polo rostral visto desde el centro del cerebro:
    el extremo más anterior de los giros frontales, en la línea media."""
    frontales = [1028, 2028, 1027, 2027, 1014, 2014, 1012, 2012, 1032, 2032, 1019, 2019]
    pts = np.argwhere(np.isin(seg[::2, ::2, ::2], frontales))[:, ::-1] * 2.0 * factor_seg - centro  # (x, y, z)
    izq = np.array(anatomia["izquierda"], float)
    ant, sup = np.array(anatomia["anterior"], float), np.array(anatomia["superior"], float)
    h = np.stack([pts @ -izq, pts @ ant, pts @ sup], 1)
    punta = h[h[:, 1] >= np.percentile(h[:, 1], 99.5)].mean(0)
    punta[0] = 0.0  # en la línea media (espacio interhemisférico)
    return punta / np.linalg.norm(punta)


def etiqueta_en(seg, factor_seg, rot, prof_total, lado, centro, forma_vol, uv, espejo):
    """Etiqueta de la región en el punto (u, v) 0-1 del corte mostrado."""
    u, v = uv
    if espejo:
        u = 1 - u
    if centro is None:
        centro = (np.array(forma_vol[::-1]) - 1) / 2
    eu, ev, en = rot.apply(np.eye(3))
    p = centro + prof_total * en + (u - 0.5) * lado * eu + (v - 0.5) * lado * ev  # (x, y, z) del volumen
    idx = np.round(p[::-1] / factor_seg).astype(int)
    if np.any(idx < 0) or np.any(idx >= seg.shape):
        return 0
    return int(seg[tuple(idx)])


def paneles(pantalla, vistas, info_puntos, prof_total, nombre, zona, bajo_mouse, hondo_mm, estado,
            sin_lapiz, mostrar_regiones, hay_regiones):
    """Franjas laterales de una pantalla ancha: vistas 3D a la izquierda, información a la derecha."""
    W, H = pantalla.panel, pantalla.alto
    izq = np.zeros((H, W, 3), np.uint8)
    textos_der = []
    if info_puntos is not None and vistas:
        alto_v = vistas[0][1].lado
        hueco = (H - len(vistas) * (alto_v + 40)) // (len(vistas) + 1)
        y = hueco
        for titulo, v in vistas:
            x = (W - v.lado) // 2
            cv2.putText(izq, titulo, (x, y + 22), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (200, 200, 200), 1, cv2.LINE_AA)
            izq[y + 34:y + 34 + v.lado, x:x + v.lado] = v.dibujar(info_puntos[0], prof_total)
            y += v.lado + 40 + hueco
    blanco, tenue, amarillo = (255, 255, 255), (170, 178, 195), (255, 230, 120)
    ancho_max = W - 48

    def agregar(texto, y, tam, color, espacio=1.15):
        """Agrega texto partido en líneas que quepan en el panel; devuelve la y siguiente."""
        fuente, linea = pantalla._fuente(tam), ""
        for palabra in texto.split():
            prueba = f"{linea} {palabra}".strip()
            if linea and fuente.size(prueba)[0] > ancho_max:
                textos_der.append((linea, 0, y, tam, color))
                y += int(tam * espacio)
                linea = palabra
            else:
                linea = prueba
        if linea:
            textos_der.append((linea, 0, y, tam, color))
            y += int(tam * espacio)
        return y

    y = 40
    if nombre:
        y = agregar(nombre, y, 44, blanco)
        y = agregar(zona.capitalize(), y + 4, 30, (255, 220, 190)) + 30
    if bajo_mouse:
        y = agregar("Bajo el puntero:", y, 26, tenue)
        y = agregar(bajo_mouse["es"], y + 2, 34, amarillo) + 30
    if hondo_mm is not None:
        y = agregar(f"Profundidad: {hondo_mm:.0f} mm", y, 34, blanco) + 30
    ayuda = (["Sin lápiz: arrastra con el mouse para recorrer la cabeza.",
              "Rueda o flechas: más o menos hondo."] if sin_lapiz else
             ["Gira el lápiz, o golpecito = más hondo / doble golpecito = hacia la superficie.",
              "Superficie: espacio y luego apoya la sonda en el imán de la frente."])
    ayuda += ["M: configurar la sonda (montaje del sensor).  B: ver imanes (polaridad)." if not sin_lapiz else ""]
    ayuda += ["P: calibrar el lápiz." if not sin_lapiz
              else "P: calibrar el lápiz (conéctalo primero)."]
    ayuda += ["Pasa el mouse sobre el corte para ver el nombre; clic para resaltarlo."]
    if hay_regiones:
        ayuda.append(f"E: {'ocultar' if mostrar_regiones else 'mostrar'} las regiones.")
    ayuda += ["F2: elegir cerebro.  Q: salir."]
    # la ayuda va abajo: medirla primero
    marca = len(textos_der)
    yb = agregar(" ", 0, 24, tenue)
    for t in ayuda:
        yb = agregar(t, yb + 6, 24, tenue)
    alto_ayuda = yb
    del textos_der[marca:]
    yb = max(y + 20, H - 50 - alto_ayuda)
    for t in ayuda:
        yb = agregar(t, yb + 6, 24, tenue)
    textos_der.append((estado if not sin_lapiz else "Lápiz desconectado", 0, H - 34, 22, (120, 128, 145)))
    return izq, textos_der


def instalar_en_terminal(pid):
    """Descarga un paquete sin ventana (para preparar equipos). Devuelve el código de salida."""
    paq = paquetes.por_id(pid)
    if paq is None:
        print(f"Paquete desconocido: {pid}. Disponibles: {', '.join(q['id'] for q in paquetes.CATALOGO)}")
        return 2
    d = paquetes.Descarga(USUARIO, paq)
    d.start()
    while d.is_alive():
        print(f"\r{d.progreso * 100:5.1f} %  {d.estado:<25}", end="", flush=True)
        time.sleep(0.5)
    print(f"\r{d.progreso * 100:5.1f} %  {d.estado:<25}")
    if d.error:
        print(d.error)
        return 1
    CONFIG.write_text(json.dumps({"paquete": pid}))
    return 0


def registrar_superficie(resultado, campo, vel, signo, segundos):
    """Anota cada intento de superficie (para revisar después qué falló)."""
    try:
        with open(USUARIO / "registro_superficie.log", "a") as f:
            f.write(f"{time.strftime('%Y-%m-%d %H:%M:%S')}  {resultado:18}  campo={campo:7.0f} µT  "
                    f"vel={vel:5.1f} °/s  signo={signo:+d}  en_iman={segundos:.1f} s\n")
    except OSError:
        pass


def elegir_paquete_instalado():
    """El paquete guardado en la configuración, o el primero instalado del catálogo."""
    try:
        pid = json.loads(CONFIG.read_text()).get("paquete")
        if pid and paquetes.instalado(USUARIO, pid):
            return pid
    except (OSError, ValueError):
        pass
    return next((p["id"] for p in paquetes.CATALOGO if paquetes.instalado(USUARIO, p["id"])), None)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--vol", default=None, help="volumen .npy suelto (desarrollo); por defecto, el paquete elegido")
    ap.add_argument("--paquete", default=None, help="id del paquete a usar (ver paquetes.py)")
    ap.add_argument("--paquetes", action="store_true", help="abrir la pantalla de paquetes al inicio")
    ap.add_argument("--instalar", metavar="ID", help="descargar un paquete desde la terminal y salir "
                    f"({', '.join(q['id'] for q in paquetes.CATALOGO)})")
    puertos = sorted(glob.glob("/dev/ttyACM*") + glob.glob("/dev/ttyUSB*"))
    ap.add_argument("--puerto", default=puertos[0] if puertos else "/dev/ttyACM0")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--sin-imu", action="store_true")
    ap.add_argument("--ventana", type=int, default=720, help="tamaño de la imagen en píxeles (modo ventana)")
    ap.add_argument("--completa", action=argparse.BooleanOptionalAction, default=CONGELADO,
                    help="pantalla completa (por defecto en el binario)")
    ap.add_argument("--monitor", type=int, default=0, help="monitor para la pantalla completa (0, 1, ...)")
    a = ap.parse_args()

    if a.instalar:
        sys.exit(instalar_en_terminal(a.instalar))

    # una sola instancia a la vez: dos abiertas se pelean la sonda y el procesador (todo se pone lento)
    import fcntl
    candado = open(USUARIO / "demo.lock", "w")
    try:
        fcntl.flock(candado, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except OSError:
        sys.exit("TrepanAr ya está abierto (solo puede haber una a la vez).")

    pantalla = Pantalla("Explora un cerebro real", a.completa, a.ventana, a.monitor)
    seg, etiquetas, factor_seg = None, {}, 1
    if a.vol:
        vol = np.load(a.vol)
        meta_path = Path(a.vol).with_suffix(".json")
        meta = json.loads(meta_path.read_text()) if meta_path.exists() else {}
        cache = Path(a.vol).with_suffix(".cerebro.npy")
    else:
        pid = a.paquete or elegir_paquete_instalado()
        if a.paquetes or pid is None:  # primera vez (o pedido): elegir/descargar un cerebro
            pid = instalador.PantallaPaquetes(pantalla, USUARIO, CONFIG).ejecutar()
            if pid is None:
                pantalla.cerrar()
                return
        instalador.pantalla_cargando(pantalla, f"Cargando {paquetes.por_id(pid)['nombre'].lower()}…")
        dir_paq = paquetes.carpeta(USUARIO, pid)
        vol = np.load(dir_paq / "vol.npy", mmap_mode="r")
        vol = np.ascontiguousarray(vol)
        meta = json.loads((dir_paq / "meta.json").read_text())
        if (dir_paq / "seg.npy").exists():
            seg = np.load(dir_paq / "seg.npy")
            etiquetas, factor_seg = meta.get("etiquetas", {}), meta.get("factor_seg", 1)
        cache = dir_paq / "cerebro.npy"
    voxel = meta.get("voxel_um", 0)
    anatomia = meta.get("anatomia", {"anterior": [0, -1, 0], "superior": [0, 0, -1], "izquierda": [-1, 0, 0]})
    espejo = anatomia.get("quiralidad", 1.0) < 0
    lut = lut_regiones(etiquetas) if etiquetas else None
    cerebro = Cerebro(vol, cache, seg, factor_seg)
    h_rostral = polo_rostral(seg, factor_seg, cerebro.centro, anatomia) if seg is not None else np.array([0, 1.0, 0.1])
    mini, vistas = None, []  # inserto chico (pantalla angosta) o [(título, vista)] para el panel izquierdo
    if seg is not None:
        if pantalla.ancha:
            tam = int(min(pantalla.panel - 40, (pantalla.alto - 140) / 2))
            pedidas = [("De frente", 30, 20), ("De lado (izquierdo)", 90, 10)]
        else:
            tam, pedidas = 200, [("De frente", 30, 20)]
        for titulo, az_v, el_v in pedidas:
            cache3d = None if a.vol else dir_paq / f"vista3d_{az_v}_{tam}.npz"
            if cache3d is None or not cache3d.exists():
                instalador.pantalla_cargando(pantalla, "Preparando las vistas 3D (solo la primera vez)…")
            vistas.append((titulo, vista3d.Vista3D(seg, factor_seg, anatomia, cerebro.centro, etiquetas, lado=tam,
                                                   azimut=az_v, elevacion=el_v, cache=cache3d)))
        if not pantalla.ancha:
            mini = vistas[0][1]
    # el plano cubre el cerebro con margen (más chico que la diagonal del volumen = más rápido)
    lado = int(2.3 * np.max(np.linalg.norm(cerebro.pts - cerebro.centro, axis=1)))

    imu = None
    if not a.sin_imu:
        try:
            imu = LectorIMU(a.puerto, a.baud)
            imu.start()
        except Exception as e:
            print(f"sin IMU ({e}); sigo solo con teclado")

    cero = Rotation.identity()
    manual = np.zeros(3)  # roll, pitch, yaw en grados
    profundidad = 0.0
    paso = 3.0
    incl = InclinacionProfundidad()
    puntos = ModoPuntos(anatomia)
    puntos.usar_frente(h_rostral)  # el punto "frente" del modelo = polo rostral de este cerebro
    if imu and puntos.eje_montaje is not None:
        imu.cara = -puntos.eje_montaje
    if imu:
        imu.detector.configurar(puntos.toques_params)
    HONDO_BASE = 20  # vóxeles bajo la superficie al partir, en modo puntos
    giro_acum, hondo, frac_hondo = 0.0, 0.0, 0.0
    armado_hasta, quieto_iman_desde = 0.0, None  # espacio arma la superficie; se completa con imán + quieto
    iman_desde, estado_armado = None, (False, False, False)
    config_sonda, config_fin = None, 0.0  # configuración de la sonda (tecla m); se muestra el resultado 4 s
    ventana_imanes, hist_polaridad = False, []  # tecla b: polaridad de los imanes
    az_teclas, el_teclas = 0.0, 25.0  # sin lápiz: azimut (rad) y elevación (°); mouse arrastrando o j/l i/k
    mostrar_regiones = False  # tecla e: colores y nombre de las regiones (parte apagado)
    resaltado = None  # (etiqueta, hasta) estructura elegida con clic

    MM_POR_TOQUE = 5.0  # golpecito = 5 mm más hondo; doble golpecito = 5 mm hacia la superficie
    T_DOBLE = 0.5  # s máximos entre los dos golpes de un doble golpe
    toque_pendiente = None  # momento del último golpe, esperando ver si llega el segundo
    toques_vistos, giros_recientes, sin_giro_hasta = None, [], 0.0
    primera_vez_t0 = None
    r_prev, t_prev, vel = Rotation.identity(), time.time(), 0.0
    velocidad = Velocidad()
    aviso, aviso_hasta = "", 0.0
    a.ventana = 720  # se dibuja a 720 px (texto legible) y la pantalla amplía al final

    while True:
        ang_imu = imu.angulos.copy() if imu else np.zeros(3)
        r_imu = imu.rotacion() if imu else Rotation.identity()
        ahora = time.time()
        vel = velocidad(ahora, r_imu)
        if puntos.activo and imu:
            g = puntos.giro_lapiz(r_prev, r_imu)
            if abs(g) > 0.1 and ahora > sin_giro_hasta:  # zona muerta: ignora deriva lenta
                giro_acum += g
                giros_recientes.append((ahora, g))
            giros_recientes = [(tt, gg) for tt, gg in giros_recientes if ahora - tt < 0.3]
            if toques_vistos is None:
                toques_vistos = imu.conteo_toques()
            if imu.conteo_toques() != toques_vistos:
                n_toques = (imu.conteo_toques() - toques_vistos) % 2**32
                toques_vistos = imu.conteo_toques()
                # el golpe sacude el lápiz: deshacer el giro de los últimos 0.3 s y pausar un momento
                giro_acum -= sum(gg for _, gg in giros_recientes)
                giros_recientes, sin_giro_hasta = [], ahora + 0.2
                for _ in range(n_toques):  # doble golpe (dos en < 0,5 s) = hacia la superficie
                    if toque_pendiente is not None and ahora - toque_pendiente < T_DOBLE:
                        profundidad -= MM_POR_TOQUE / (voxel / 1000)
                        toque_pendiente = None
                    else:
                        toque_pendiente = ahora
            if toque_pendiente is not None and ahora - toque_pendiente >= T_DOBLE:  # golpe simple = más hondo
                profundidad += MM_POR_TOQUE / (voxel / 1000)
                toque_pendiente = None
        r_prev, t_prev = r_imu, ahora
        quieto = vel < 15  # °/s
        frac = None
        info_puntos = None
        r_ef = r_imu
        centro = None
        if (imu and puntos.eje_montaje is None and config_sonda is None and puntos.calibrando is None
                and time.time() - imu.ultima < 0.5 and imu.quat is not None):  # sonda nueva: configurarla primero
            config_sonda = ConfigSonda()
        if ventana_imanes:
            cara_sensor = -puntos.eje_montaje if puntos.eje_montaje is not None else np.array([0, 0, 1.0])
            img, textos = pantalla_imanes(imu, cara_sensor, a.ventana, hist_polaridad)
            pantalla.mostrar(img, textos)
            k = pantalla.tecla()
            if k in (ord("b"), ord("q"), 27):
                ventana_imanes = False
            continue
        if config_sonda is not None:
            if not config_sonda.terminado and imu and time.time() - imu.ultima < 0.5:
                config_sonda.paso(r_imu)
                if config_sonda.terminado:
                    config_fin = time.time() + 4
                    if config_sonda.resultado[0]:  # aplicar el montaje nuevo
                        cara = np.array(json.loads((USUARIO / "montaje.json").read_text())["cara"])
                        puntos.eje_montaje = -cara / np.linalg.norm(cara)
                        imu.cara = cara / np.linalg.norm(cara)
                        if puntos.activo:
                            puntos._ajustar()
            img, textos = pantalla_config_sonda(config_sonda, a.ventana)
            pantalla.mostrar(img, textos)
            k = pantalla.tecla()
            if k in (ord("q"), 27) or (config_sonda.terminado and time.time() > config_fin):
                config_sonda = None
            elif k == ord("m"):
                config_sonda = ConfigSonda()
            continue
        if (imu and not puntos.activo and puntos.calibrando is None
                and time.time() - imu.ultima < 0.5 and imu.quat is not None):
            if primera_vez_t0 is None:
                primera_vez_t0 = ahora
            elif ahora - primera_vez_t0 > 3:  # sin calibración: empezar sola (no hay teclado)
                puntos.iniciar_calibracion(imu)
        if puntos.calibrando is not None and imu:
            aviso, aviso_hasta = puntos.paso_calibracion(r_imu), ahora + 0.5
        sin_lapiz = imu is None or time.time() - imu.ultima > 1.0
        pantalla.mostrar_mouse(sin_lapiz or time.time() - pantalla.t_mouse < 3)
        if sin_lapiz:  # arrastrar con el mouse: horizontal = alrededor de la cabeza, vertical = subir/bajar
            dx, dy = pantalla.arrastre()
            az_teclas += np.radians(dx * 0.4)
            el_teclas = float(np.clip(el_teclas - dy * 0.3, -20, 90))
            az = az_teclas
            el = np.radians(el_teclas)
            h = np.array([np.sin(az) * np.cos(el), np.cos(az) * np.cos(el), np.sin(el)])  # der, ant, sup
            d = h[0] * puntos.direccion("derecha") + h[1] * puntos.direccion("anterior") + h[2] * puntos.direccion("superior")
            d /= np.linalg.norm(d)
            profundidad += pantalla.rueda() * 2.0 / (voxel / 1000)  # 2 mm por paso de la rueda
            cercano = max(puntos.PUNTOS, key=lambda q: h @ np.array(q[4], float) / np.linalg.norm(q[4]))[0]
            rot = plano_desde(d, anatomia)
            radio = cerebro.radio(d)
            hondo = float(np.clip(HONDO_BASE + profundidad, 0, 2 * radio))
            profundidad = hondo - HONDO_BASE
            prof_total, frac_hondo, centro = radio - hondo, hondo / (2 * radio), cerebro.centro
            info_puntos = (d, cercano)
        elif puntos.activo and imu and puntos.calibrando is None:
            d, cercano = puntos.ubicar(r_imu, imu.campo, quieto, imu.signo_reciente())
            if armado_hasta > ahora:  # superficie armada con espacio: imán + (quieta o 1 s en el imán)
                sg, aprendidos = imu.signo_reciente(), puntos.imanes.get("signo", {})
                es_cz = bool(sg and aprendidos.get("cz") == sg and aprendidos.get("frente", -sg) != sg)
                hay_iman = imu.campo > puntos.umbral_superficie() and not es_cz
                quieta = vel < 20
                estado_armado = (hay_iman, quieta, es_cz)
                if hay_iman:
                    iman_desde = iman_desde or ahora
                    quieto_iman_desde = (quieto_iman_desde or ahora) if quieta else None
                    listo = ((quieto_iman_desde and ahora - quieto_iman_desde > 0.2) or ahora - iman_desde > 1.0)
                    if listo:
                        puntos.volver_a_punto(r_imu, "frente")
                        giro_acum, profundidad, armado_hasta = 0.0, 0.0, 0.0
                        d, cercano = puntos.ubicar(r_imu, imu.campo, quieto, imu.signo_reciente())
                        decir("superficie")
                        registrar_superficie("lograda", imu.campo, vel, sg, ahora - iman_desde)
                else:
                    quieto_iman_desde, iman_desde = None, None
                if armado_hasta and ahora > armado_hasta - 0.05:  # se acaba: anotar una vez y desarmar
                    registrar_superficie("se acabó el tiempo", imu.campo, vel, sg, 0)
                    armado_hasta = 0.0
            rot = plano_desde(d, anatomia)
            radio = cerebro.radio(d)
            vox_por_grado = puntos.MM_POR_VUELTA / 360 / (voxel / 1000)
            hondo = HONDO_BASE + profundidad + giro_acum * vox_por_grado
            if not 0 <= hondo <= 2 * radio:  # tope: superficie y lado opuesto
                hondo = float(np.clip(hondo, 0, 2 * radio))
                giro_acum = (hondo - HONDO_BASE - profundidad) / vox_por_grado
            prof_total = radio - hondo
            frac_hondo = hondo / (2 * radio)
            centro = cerebro.centro
            info_puntos = (d, cercano)
        else:
            if incl.activo and imu:
                r_ef, frac = incl.aplicar(r_imu)
            r_man = Rotation.from_euler("ZYX", manual[::-1], degrees=True)
            rot = r_man * cero.inv() * r_ef
            prof_total = profundidad
            if frac is not None:
                prof_total += (2 * frac - 1) * alcance(vol, rot)

        region, textos = None, []
        resaltar = resaltado[0] if resaltado and time.time() < resaltado[1] else None
        if seg is not None and (mostrar_regiones or resaltar):
            img, etq = corte(vol, rot, prof_total, lado, centro, seg=seg, factor_seg=factor_seg)
        else:
            img, etq = corte(vol, rot, prof_total, lado, centro), None
        if espejo:
            img = img[:, ::-1]
            etq = etq[:, ::-1] if etq is not None else None
        img = cv2.resize(np.ascontiguousarray(img), (a.ventana, a.ventana), interpolation=cv2.INTER_LINEAR)
        img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
        if info_puntos is not None:
            c = a.ventana // 2  # mira en el punto bajo la punta
            cv2.circle(img, (c, c), 7, (255, 255, 255), 1, cv2.LINE_AA)
            cv2.line(img, (c - 14, c), (c - 8, c), (255, 255, 255), 1)
            cv2.line(img, (c + 8, c), (c + 14, c), (255, 255, 255), 1)
            cv2.line(img, (c, c - 14), (c, c - 8), (255, 255, 255), 1)
            cv2.line(img, (c, c + 8), (c, c + 14), (255, 255, 255), 1)
        if etq is not None:
            etq = cv2.resize(np.ascontiguousarray(etq).astype(np.uint16), (a.ventana, a.ventana),
                             interpolation=cv2.INTER_NEAREST)
        if etq is not None and info_puntos is not None and mostrar_regiones:
            img = pintar_regiones(img, etq, lut)
            region = etiquetas.get(str(int(etq[a.ventana // 2, a.ventana // 2])))
        if etq is not None and resaltar:  # estructura elegida con clic: contorno blanco grueso
            m = (etq == resaltar).astype(np.uint8)
            borde = cv2.morphologyEx(m, cv2.MORPH_GRADIENT, np.ones((3, 3), np.uint8)) > 0
            img[m > 0] = (0.6 * img[m > 0] + 0.4 * 255).astype(np.uint8)
            img[borde] = (255, 255, 255)
        # puntero: nombre de la estructura bajo el mouse; clic = decirlo en voz alta y resaltarla
        bajo_mouse = None
        uv = pantalla.en_imagen() if seg is not None and time.time() - pantalla.t_mouse < 3 else None
        if uv is not None:
            bajo_mouse = etiqueta_en(seg, factor_seg, rot, prof_total, lado, centro, vol.shape, uv, espejo)
        clic = pantalla.click()
        if clic is not None and seg is not None:
            sid = etiqueta_en(seg, factor_seg, rot, prof_total, lado, centro, vol.shape, clic, espejo)
            if sid and str(sid) in etiquetas:
                resaltado = (sid, time.time() + 4)
                decir(etiquetas[str(sid)]["es"])
        if region is not None or (mostrar_regiones and info_puntos is not None):
            nombre, zona = (region["es"], region["zona"]) if region else ("Fuera del cerebro", "")
        else:
            nombre, zona = "", ""
        if bajo_mouse and str(bajo_mouse) in etiquetas and not pantalla.ancha:
            e_m = etiquetas[str(bajo_mouse)]
            ux, uy = uv
            tx = min(ux * a.ventana + 14, a.ventana - 260)
            textos.append((e_m["es"], tx, uy * a.ventana + 10, 22, (255, 255, 160)))
        if not pantalla.ancha:
            textos += [(nombre, 14, 14, 34, (255, 255, 255)), (zona.capitalize(), 14, 46, 24, (255, 230, 200))]
        if info_puntos is not None and mini is not None and not pantalla.ancha:  # mini vista 3D abajo a la izquierda
            m = mini.dibujar(info_puntos[0], prof_total)
            y0, x0 = img.shape[0] - 48 - mini.lado, 8
            zona = img[y0:y0 + mini.lado, x0:x0 + mini.lado]
            pinta = m.any(axis=2)
            zona[pinta] = m[pinta]
            cv2.rectangle(img, (x0 - 1, y0 - 1), (x0 + mini.lado, y0 + mini.lado), (90, 90, 90), 1)
            textos.append(("Vista 3D (de frente)", x0 + 4, y0 + 4, 18, (200, 200, 200)))
        if info_puntos is not None:
            dibujar_cabeza(img, puntos, *info_puntos, imu.campo if imu else 0.0)
            dibujar_barra(img, frac_hondo, "superficie", "fondo", invertir=True)
        else:
            dibujar_ejes(img, rot)
        if frac is not None:
            dibujar_barra(img, frac)
        if imu and not sin_lapiz and imu.campo > 100:  # imán cerca: avisar si acercarse o alejarse
            textos += dibujar_distancia(img, imu.campo, 200, 130, 320, 14, 0.9)
        if time.time() < MENSAJE[1]:  # mensajes (antes por voz): grandes, arriba
            for j, linea in enumerate(textwrap.wrap(MENSAJE[0], 40)):
                textos.append((linea, 24, 96 + 38 * j, 34, (255, 240, 120)))
        if armado_hasta > time.time():
            hay, quieta, es_cz = estado_armado
            aviso = ("ARMADO · imán: " + ("sí" if hay else "no") + " · quieta: " + ("sí" if quieta else "no")
                     + (" · ese imán es Cz" if es_cz else ""))
            aviso_hasta = armado_hasta
        if time.time() < aviso_hasta:  # mensajes de calibración (pygame: con tildes)
            textos.append((aviso, 14, 84, 26, (120, 255, 140)))

        if imu is None:
            estado = "IMU: no conectada"
        elif time.time() - imu.ultima > 1:
            estado = f"IMU: sin datos validos (ultimo: {imu.ultimo_texto})"
        elif not ang_imu.any():
            estado = "IMU: llegan ceros (revisar sensor/I2C)"
        else:
            estado = "IMU r/p/y: " + " ".join(f"{x:7.1f}" for x in ang_imu)
        if info_puntos is not None and sin_lapiz:
            estado = "SIN LAPIZ: arrastra con el mouse para recorrer la cabeza; clic = resaltar estructura"
            modo = ""
            linea2 = (f"hondo: {hondo * voxel / 1000:.1f} mm   rueda o flechas = profundidad | espacio = superficie | "
                      "e = regiones | F2 = paquetes | q = salir")
        elif info_puntos is not None:
            modo = f"PUNTOS: cerca de {info_puntos[1]}  campo {imu.campo:.0f} uT"
            linea2 = (f"hondo: {hondo * voxel / 1000:.1f} mm   girar lapiz | golpe = +5 mm, doble = -5 mm | espacio + iman frente quieto = superficie | "
                      f"p = calibrar | e = regiones")
        else:
            modo = f"incl->prof ON ({np.degrees(incl.angulo):.0f} deg)" if incl.activo else "incl->prof OFF (1=adelante 2=atras)"
            linea2 = (f"prof: {prof_total * voxel / 1000:+.1f} mm   p=calibrar puntos c=cero g=giro m=mag "
                      "1/2=max adel/atras t=on/off w/s=prof r=reset q=salir")
        txt = [estado + "   " + modo, linea2]
        panel_izq, textos_der = None, []
        if puntos.calibrando is not None and imu:  # guía visual de la calibración
            img, textos = pantalla_calibracion(puntos, a.ventana)
            pantalla.mostrar(img, textos)
            if pantalla.tecla() in (ord("q"), 27):  # atender la ventana; q/Esc sale
                break
            continue
        if pantalla.ancha:
            panel_izq, textos_der = paneles(pantalla, vistas, info_puntos, prof_total, nombre, zona,
                                            etiquetas.get(str(bajo_mouse)) if bajo_mouse else None,
                                            hondo * voxel / 1000 if info_puntos is not None else None,
                                            estado, sin_lapiz, mostrar_regiones, seg is not None)
        else:
            for i, t in enumerate(txt):
                cv2.putText(img, t, (10, img.shape[0] - 40 + 22 * i), cv2.FONT_HERSHEY_SIMPLEX,
                            0.45, (0, 255, 255), 1, cv2.LINE_AA)
        pantalla.mostrar(img, textos, panel_izq, textos_der)

        k = pantalla.tecla()
        if k in (ord("q"), 27):
            break
        elif k == ord(" "):  # armar la superficie: se completa con el lápiz quieto en el imán de la frente
            if sin_lapiz:
                az_teclas, el_teclas = 0.0, float(np.degrees(np.arcsin(np.clip(h_rostral[2], -1, 1))))
                profundidad = 0.0
            elif puntos.activo:
                armado_hasta = 0.0 if armado_hasta > time.time() else time.time() + 20  # espacio otra vez: cancela
                quieto_iman_desde, iman_desde = None, None
                decir("Listo. Apoya el lápiz quieto en el imán de la frente." if armado_hasta else "Cancelado.")
        elif k == ord("e") and seg is not None:
            mostrar_regiones = not mostrar_regiones
        elif k == TECLA_PAQUETES:
            pantalla.cerrar()
            argv = [x for x in sys.argv[1:] if x not in ("--paquetes",) and not x.startswith("--paquete=")]
            inicio = [sys.executable] if CONGELADO else [sys.executable, sys.argv[0]]
            os.execv(sys.executable, inicio + argv + ["--paquetes"])
        elif k == ord("c"):
            cero = r_ef
        elif k in (ord("1"), ord("2")) and imu:
            cual = "adelante" if k == ord("1") else "atras"
            incl.fijar(cual, r_imu)
            if incl.activo:
                cero = incl.aplicar(r_imu)[0]  # plano recto en la posición media
                aviso = f"listo: recorrido {np.degrees(incl.angulo):.0f} deg (guardado)"
            elif incl.atras is not None and incl.adelante is not None:
                aviso = "recorrido < 5 deg: inclina mas y repite 1 y 2"
            else:
                aviso = f"max {cual} guardado; ahora marca el otro extremo"
            aviso_hasta = time.time() + 3
        elif k == ord("t") and incl.eje is not None:
            incl.activo = not incl.activo
        elif k == ord("p") and imu:
            if sin_lapiz:
                aviso, aviso_hasta = "Conecta el lapiz para calibrar", time.time() + 3
                decir("Conecta el lápiz para calibrar.")
            else:
                puntos.iniciar_calibracion(imu)
        elif k == ord("n") and len(puntos.rots) == len(puntos.PUNTOS):
            puntos.activo = not puntos.activo
        elif k == ord("z"):
            puntos.signo_giro *= -1
            if puntos.rots:
                puntos.guardar()
        elif k == ord("0"):
            giro_acum, profundidad = 0.0, 0.0
        elif k == ord("x"):
            puntos.signo_x *= -1
            if puntos.rots:
                puntos.guardar()
        elif k == ord("b") and imu and not sin_lapiz:  # ventana de imanes (polaridad)
            ventana_imanes, hist_polaridad = True, []
        elif k == ord("m") and imu:  # configurar la sonda (montaje del sensor)
            config_sonda = ConfigSonda()
        elif k == ord("g") and imu:
            try:
                imu.ser.write(bytes([k]))  # g: recalibrar giro
            except Exception:
                pass  # puerto caído; el lector se reconecta solo
        elif k == ord("w"):
            profundidad += paso
        elif k == ord("s"):
            profundidad -= paso
        elif k == ord("r"):
            manual[:] = 0
            profundidad = 0
        elif sin_lapiz and k in (ord("j"), ord("l"), ord("i"), ord("k")):
            az_teclas += {ord("j"): -0.1, ord("l"): 0.1}.get(k, 0.0)
            el_teclas = float(np.clip(el_teclas + {ord("i"): 5.0, ord("k"): -5.0}.get(k, 0.0), -20, 90))
        else:
            teclas = {"j": (0, -5), "l": (0, 5), "i": (1, -5), "k": (1, 5), "u": (2, -5), "o": (2, 5)}
            for tecla, (eje, delta) in teclas.items():
                if k == ord(tecla):
                    manual[eje] += delta
    pantalla.cerrar()


if __name__ == "__main__":
    main()
