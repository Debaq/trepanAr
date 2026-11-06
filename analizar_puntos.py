# /// script
# requires-python = ">=3.11"
# dependencies = ["numpy", "scipy"]
# ///
"""Analiza una grabación guiada de puntos con imanes (grabar_guiado.py --guion puntos).

Uso: uv run analizar_puntos.py datos/grabaciones/puntos_1.csv

Por punto: intensidad |B|, saturación, dirección del campo (en el marco del sensor),
dirección de la gravedad y orientación integrada con el giroscopio. Luego compara
pares de puntos para ver qué señal los distingue.
"""
import sys

import numpy as np
from scipy.spatial.transform import Rotation

# (fase, ¿tiene imán?)
PUNTOS = [("frente", True), ("cz", False), ("temp_izq", True), ("temp_der", True), ("occipital", False)]
DESCARTE = 1.5  # s al inicio de cada fase quieta (todavía llegando)


def angulo(u, v):
    return np.degrees(np.arccos(np.clip(np.dot(u, v) / np.linalg.norm(u) / np.linalg.norm(v), -1, 1)))


def main():
    D = np.genfromtxt(sys.argv[1], delimiter=",", names=True, dtype=None, encoding="utf-8")
    t, fase = D["t"], D["fase"]
    acc = np.c_[D["ax"], D["ay"], D["az"]].astype(float)
    gyr = np.radians(np.c_[D["gx"], D["gy"], D["gz"]].astype(float))
    mag = np.c_[D["mx"], D["my"], D["mz"]].astype(float)
    sat = mag[:, 0] >= 9999
    modulo = np.where(sat, np.nan, np.linalg.norm(mag, axis=1))

    # Orientación por integración del giroscopio (marco del cuerpo, desde el inicio)
    rot = [Rotation.identity()]
    for i in range(1, len(t)):
        rot.append(rot[-1] * Rotation.from_rotvec(gyr[i] * (t[i] - t[i - 1])))

    # Campo terrestre de referencia: tramos de traslado lejos de imanes (|B| bajo)
    lejos = (fase == "mov") & (modulo < 70)
    print(f"|B| lejos de imanes (traslados): mediana {np.nanmedian(modulo[lejos]):.0f} µT\n")

    res = {}
    print(f"{'punto':10} {'iman':>6} {'|B| µT':>8} {'(min-max)':>13} {'satur.':>6}  dir. campo (sensor)   gravedad (sensor)")
    for nombre, n in PUNTOS:
        f = np.where(fase == nombre)[0]
        if not len(f):
            continue
        k = f[t[f] >= t[f[0]] + DESCARTE]
        B = np.nanmedian(modulo[k])
        lo, hi = np.nanpercentile(modulo[k], [5, 95])
        mdir = np.nanmedian(mag[k][~sat[k]], axis=0)
        g = acc[k].mean(0)
        res[nombre] = dict(B=B, mdir=mdir, g=g, rot=rot[k[len(k) // 2]], n=n)
        print(f"{nombre:10} {'si' if n else 'no':>6} {B:8.0f} {lo:6.0f}-{hi:<6.0f} {sat[k].mean() * 100:5.0f}%  "
              f"{np.round(mdir / np.linalg.norm(mdir), 2)}   {np.round(g / np.linalg.norm(g), 2)}")

    print("\nPares: ángulo entre orientaciones del lápiz (giroscopio) | entre gravedades | entre direcciones del campo")
    nombres = list(res)
    for i in range(len(nombres)):
        for j in range(i + 1, len(nombres)):
            a, b = res[nombres[i]], res[nombres[j]]
            ang_rot = np.degrees((a["rot"].inv() * b["rot"]).magnitude())
            print(f"  {nombres[i]:9} - {nombres[j]:9}: {ang_rot:6.0f}° | {angulo(a['g'], b['g']):5.0f}° | "
                  f"{angulo(a['mdir'], b['mdir']):5.0f}°   |B| {a['B']:.0f} vs {b['B']:.0f}")

    tierra = np.nanmedian(modulo[lejos])
    print("\nDetección de imán (umbral 2x campo terrestre):")
    for r, d in res.items():
        detectado = d["B"] > 2 * tierra
        ok = "OK" if detectado == d["n"] else "FALLA"
        print(f"  {r:10} imán={'si' if d['n'] else 'no'}  detectado={'si' if detectado else 'no'}  {ok}")


if __name__ == "__main__":
    main()
