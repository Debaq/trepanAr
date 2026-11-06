"""Mini vista 3D: el cerebro visto desde adelante (como el público ve el cráneo), coloreado por
lóbulo, con el corte actual marcado y la sonda.

Como la cámara es fija, la superficie se calcula una vez: se lanzan rayos desde la cámara a
través de la segmentación y se guarda, por píxel, el primer punto del cerebro que tocan
(profundidad, lóbulo y sombreado). En cada cuadro solo se pinta:
  - la línea amarilla donde el plano de corte toca la superficie
  - más oscuro, el trozo que queda entre la sonda y el plano (lo "cortado")
  - la sonda (punto y raya roja)
"""
from pathlib import Path

import cv2
import numpy as np
from scipy.ndimage import map_coordinates

AMARILLO = (40, 230, 255)  # BGR
ROJO = (60, 60, 255)
# colores por lóbulo (BGR, suaves)
COLORES = {
    "frontal": (230, 160, 90),
    "parietal": (120, 200, 120),
    "temporal": (80, 170, 240),
    "occipital": (200, 120, 190),
    "límbico": (150, 140, 240),
    "insular": (90, 220, 230),
    "cerebelo": (110, 140, 170),
    "tronco": (170, 170, 170),
    "otro": (200, 200, 200),
}


def lobulo(etiqueta):
    zona, en = etiqueta.get("zona", ""), etiqueta.get("en", "")
    if "Cerebellum" in en:
        return "cerebelo"
    if "Brain-Stem" in en:
        return "tronco"
    for k in ("frontal", "parietal", "temporal", "occipital", "límbico", "insular"):
        if k in zona:
            return k
    return "otro"


class Vista3D:
    def __init__(self, seg, factor_seg, anatomia, centro, etiquetas, lado=200, azimut=30, elevacion=20,
                 cache=None):
        self.lado = lado
        self.factor = float(factor_seg)
        izq = np.array(anatomia["izquierda"], float)
        self.ejes = np.stack([-izq, np.array(anatomia["anterior"], float),
                              np.array(anatomia["superior"], float)])  # filas: derecha, anterior, superior
        self.centro = np.asarray(centro, float)

        # cámara delante del modelo, girada `azimut` hacia su izquierda y `elevacion` hacia arriba
        az, el = np.radians(azimut), np.radians(elevacion)
        pos = np.array([-np.sin(az) * np.cos(el), np.cos(az) * np.cos(el), np.sin(el)])
        adelante = -pos
        derecha = np.cross(adelante, [0, 0, 1.0])
        derecha /= np.linalg.norm(derecha)
        arriba = np.cross(derecha, adelante)
        self.cam = np.stack([derecha, arriba, adelante])  # filas, en coordenadas de cabeza

        clave = f"{lado}_{azimut}_{elevacion}"
        z = np.load(cache) if cache is not None and Path(cache).exists() else None
        if z is not None and str(z["clave"]) == clave:
            self.P, self.color, self.dentro, self.radio = z["P"], z["color"], z["dentro"], float(z["radio"])
        else:
            self._calcular(seg, etiquetas)
            if cache is not None:
                try:
                    np.savez_compressed(cache, P=self.P, color=self.color, dentro=self.dentro,
                                        radio=self.radio, clave=clave)
                except OSError:
                    pass

    def _calcular(self, seg, etiquetas):
        """Rayos ortográficos desde la cámara a través de la segmentación (submuestreada)."""
        mascara_pts = np.argwhere(seg[::4, ::4, ::4] > 0)[:, ::-1] * 4 * self.factor  # (x, y, z) volumen
        radio = np.max(np.linalg.norm((mascara_pts - self.centro) @ self.ejes.T, axis=1)) * 1.05
        self.radio = float(radio)
        n = self.lado
        t = np.linspace(-radio, radio, n)
        prof = np.linspace(-radio, radio, int(n * 1.2))
        # puntos de muestreo en coordenadas de cabeza -> volumen -> índice de la segmentación (z, y, x)
        X, Y = np.meshgrid(t, -t)  # filas de la imagen hacia abajo
        cab = (X[..., None, None] * self.cam[0] + Y[..., None, None] * self.cam[1]
               + prof[None, None, :, None] * self.cam[2])  # (n, n, k, 3)
        vol = cab @ self.ejes + self.centro  # (x, y, z) del volumen
        idx = (vol[..., ::-1] / self.factor).transpose(3, 0, 1, 2)  # (z, y, x) de la segmentación
        etq = map_coordinates(seg, idx.reshape(3, -1), order=0, cval=0).reshape(n, n, len(prof))
        toca = etq > 0
        dentro = toca.any(-1)
        k = np.argmax(toca, -1)  # primer vóxel del cerebro a lo largo del rayo
        D = np.where(dentro, prof[k], radio)
        lab = np.take_along_axis(etq, k[..., None], -1)[..., 0]
        self.P = (X[..., None] * self.cam[0] + Y[..., None] * self.cam[1] + D[..., None] * self.cam[2])
        # sombreado desde la profundidad (normal ~ gradiente del mapa de profundidad)
        Ds = cv2.GaussianBlur(D.astype(np.float32), (5, 5), 1.2)
        gy, gx = np.gradient(Ds, t[1] - t[0])
        nz = 1 / np.sqrt(1 + gx ** 2 + gy ** 2)
        luz = np.clip(0.25 + 0.75 * nz, 0, 1)
        tabla = {int(kk): COLORES[lobulo(e)] for kk, e in etiquetas.items()}
        base = np.array([tabla.get(int(v), COLORES["otro"]) for v in lab.ravel()], np.float32).reshape(n, n, 3)
        self.color = np.where(dentro[..., None], base * luz[..., None], 0).astype(np.uint8)
        self.dentro = dentro

    def _proyectar(self, h):
        cam = h @ self.cam.T
        r = self.radio
        return np.round((cam[:, :2] * [1, -1] + r) / (2 * r) * (self.lado - 1)).astype(int)

    def dibujar(self, d_vol, prof_total):
        """Imagen BGR lado×lado. d_vol: dirección de la sonda (coordenadas del volumen); el plano de
        corte es normal a d_vol y pasa por centro + prof_total·d_vol."""
        n = self.ejes @ d_vol
        n /= np.linalg.norm(n)
        dist = self.P @ n - prof_total  # >0: entre el plano y la sonda (lo "cortado")
        img = self.color.copy()
        cortado = self.dentro & (dist > 0)
        img[cortado] = (img[cortado] * 0.35).astype(np.uint8)
        grosor = 1.2 * self.factor + 1.0
        img[self.dentro & (np.abs(dist) < grosor)] = AMARILLO
        # sonda: punto sobre la superficie del cerebro y una raya hacia afuera
        radio = float(np.max(self.P[self.dentro] @ n))
        a = self._proyectar(np.array([n * radio, n * radio * 1.3]))
        cv2.line(img, tuple(a[0]), tuple(a[1]), ROJO, 2, cv2.LINE_AA)
        cv2.circle(img, tuple(a[0]), 4, ROJO, -1, cv2.LINE_AA)
        return img
