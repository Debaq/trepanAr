# /// script
# requires-python = ">=3.11"
# dependencies = ["numpy", "scipy"]
# ///
"""Compara canales de una grabación guiada: ¿cuál distingue frente de occipital?

Uso: uv run analizar_fases.py datos/grabaciones/guiado_1.csv

Por canal muestra:
  separación  = |media occipital - media frente| / ruido en reposo  (más alto = mejor)
  monotonía   = correlación de Spearman con el tiempo durante el movimiento (±1 = ideal)
"""
import sys

import numpy as np
from scipy.stats import spearmanr


def main():
    D = np.genfromtxt(sys.argv[1], delimiter=",", names=True, dtype=None, encoding="utf-8")
    t = D["t"]
    fase = D["fase"]
    X = {k: D[k].astype(float) for k in ["ax", "ay", "az", "gx", "gy", "gz", "mx", "my", "mz"]}
    X["|m|"] = np.sqrt(X["mx"] ** 2 + X["my"] ** 2 + X["mz"] ** 2)
    dt = np.gradient(t)
    for k in "xyz":
        # giro acumulado desde que empieza el movimiento (grados)
        g = np.where((fase == "movimiento") | (fase == "occipital"), X["g" + k], 0)
        X["giro_" + k] = np.cumsum(g * dt)

    fr, mov, oc = fase == "frente", fase == "movimiento", fase == "occipital"
    print(f"muestras: frente {fr.sum()}  movimiento {mov.sum()}  occipital {oc.sum()}\n")
    print(f"{'canal':8} {'frente':>9} {'occipital':>10} {'ruido':>7} {'separacion':>11} {'monotonia':>10}")
    filas = []
    for k, v in X.items():
        a, b = v[fr].mean(), v[oc].mean()
        ruido = max(v[fr].std(), v[oc].std(), 1e-6)
        sep = abs(b - a) / ruido
        rho = spearmanr(t[mov], v[mov])[0] if v[mov].std() > 0 else 0
        filas.append((k, a, b, ruido, sep, rho))
        print(f"{k:8} {a:9.2f} {b:10.2f} {ruido:7.2f} {sep:11.1f} {rho:10.2f}")

    print("\nmedias cada 2 s durante el movimiento:")
    cols = ["ax", "ay", "az", "|m|", "mx", "my", "mz", "giro_x", "giro_y", "giro_z"]
    print("   t  " + " ".join(f"{c:>7}" for c in cols))
    for s0 in np.arange(t[fr].min(), t[oc].max(), 2):
        m = (t >= s0) & (t < s0 + 2)
        if m.any():
            print(f"{s0:5.0f} " + " ".join(f"{X[c][m].mean():7.2f}" for c in cols) + f"  {fase[m][0]}")

    mejor = max(filas, key=lambda f: f[4] * abs(f[5]))
    print(f"\nmejor candidato: {mejor[0]} (separación {mejor[4]:.1f}, monotonía {mejor[5]:+.2f})")


if __name__ == "__main__":
    main()
