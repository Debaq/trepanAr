# /// script
# requires-python = ">=3.11"
# dependencies = ["pyserial"]
# ///
"""Graba los sensores crudos del MPU9250 dando órdenes por voz (espeak-ng) y notificación.

Uso:
  uv run grabar_guiado.py                 # guarda en datos/grabaciones/guiado_<hora>.csv
  uv run grabar_guiado.py --nombre iman_1
  uv run grabar_guiado.py --guion puntos     # 5 imanes: frente, Cz, temp izq, temp der, occipital

Fases: quieto en la frente -> mover lento al occipital -> quieto en el occipital.
La columna `fase` del CSV marca cada tramo para el análisis.
"""
import argparse
import glob
import subprocess
import time
from pathlib import Path

import serial

AQUI = Path(__file__).parent

# (segundo, fase, orden hablada)
RECORRIDO = [
    (0, "preparar", "Pon el sensor en la frente. Quieto."),
    (4, "frente", "Quieto en la frente."),
    (9, "frente", "Tres. Dos. Uno."),
    (12, "movimiento", "Ahora. Mueve lento hacia el occipital."),
    (19, "movimiento", "Sigue lento. Vas a la mitad."),
    (26, "movimiento", "Casi llegas."),
    (32, "occipital", "Para. Quieto en el occipital."),
    (38, "fin", "Listo. Terminado."),
]

# Imanes: 1 frente, 2 Cz, 3 temporal izquierdo, 4 temporal derecho, 5 occipital
PUNTOS = [
    (0, "preparar", "Pon el lápiz en la frente, sobre el imán. Quieto."),
    (4, "frente", "Quieto en la frente."),
    (9, "mov", "Ahora ve a Cz, arriba."),
    (14, "cz", "Quieto en Cz."),
    (19, "mov", "Ahora ve al temporal izquierdo."),
    (24, "temp_izq", "Quieto en el temporal izquierdo."),
    (29, "mov", "Ahora ve al temporal derecho."),
    (35, "temp_der", "Quieto en el temporal derecho."),
    (40, "mov", "Ahora ve al occipital."),
    (45, "occipital", "Quieto en el occipital."),
    (50, "fin", "Listo. Terminado."),
]

# Golpecitos con el dedo vs apoyar el lápiz en el cráneo vs girarlo
TOQUES = [
    (0, "quieto", "Toma el lápiz y déjalo quieto."),
    (4, "golpes", "Golpea el costado del lápiz con el dedo, cinco veces, lento."),
    (14, "apoyos", "Ahora apoya la punta en el cráneo y levanta, cinco veces."),
    (24, "giros", "Ahora gira el lápiz sobre su eje, ida y vuelta."),
    (32, "fin", "Listo. Terminado."),
]

GUIONES = {"recorrido": RECORRIDO, "puntos": PUNTOS, "toques": TOQUES}


def decir(texto):
    subprocess.Popen(["espeak-ng", "-v", "es-419", "-s", "150", texto],
                     stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    subprocess.Popen(["notify-send", "-t", "4000", "IMU", texto],
                     stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--nombre", default=time.strftime("guiado_%H%M%S"))
    ap.add_argument("--puerto", default=None)
    ap.add_argument("--guion", choices=GUIONES, default="recorrido")
    a = ap.parse_args()
    guion = GUIONES[a.guion]

    puerto = a.puerto or sorted(glob.glob("/dev/ttyACM*") + glob.glob("/dev/ttyUSB*"))[0]
    out = AQUI / "datos" / "grabaciones" / f"{a.nombre}.csv"
    out.parent.mkdir(parents=True, exist_ok=True)

    s = serial.Serial(puerto, 115200, timeout=0.1)
    s.reset_input_buffer()
    s.write(b"r")  # modo crudo

    fin = guion[-1][0]
    paso, fase, n, buf = 0, guion[0][1], 0, b""
    t0 = time.time()
    with open(out, "w") as f:
        f.write("t,fase,ax,ay,az,gx,gy,gz,mx,my,mz\n")
        while (t := time.time() - t0) < fin + 0.5:
            if paso < len(guion) and t >= guion[paso][0]:
                _, fase, texto = guion[paso]
                decir(texto)
                print(f"[{t:5.1f}s] {texto}")
                paso += 1
            buf += s.read(4096)
            *lineas, buf = buf.split(b"\n")
            for l in lineas:
                l = l.decode(errors="ignore").strip()
                if l.startswith("R,") and l.count(",") == 9:
                    f.write(f"{t:.3f},{fase},{l[2:]}\n")
                    n += 1
    s.write(b"r")  # volver a modo normal
    print(f"{out} ({n} muestras)")


if __name__ == "__main__":
    main()
