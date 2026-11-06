# /// script
# requires-python = ">=3.11"
# dependencies = ["zarr>=3", "fsspec", "aiohttp", "requests", "numpy", "tifffile"]
# ///
"""Baja una versión liviana (nivel multiescala) de un volumen del Human Organ Atlas.

Uso:
  uv run hoa_bajar.py --nivel 4 --salida i58_242um
  uv run hoa_bajar.py --zarr I58/brain-hemi/6.21um_VOI-01_bm18.ome.zarr --nivel 5

Genera:
  <salida>.npy        volumen uint8 en orden (z, y, x)
  <salida>.tif        mismo volumen como pila TIFF (abre en Fiji / 3D Slicer)
  <salida>.raw + .json  binario crudo + metadatos (para Unity/Godot/Three.js)
"""
import argparse
import json

import numpy as np
import tifffile
import zarr

BUCKET = "https://storage.googleapis.com/ucl-hip-ct-35a68e99feaae8932b1d44da0358940b"
DEFAULT_ZARR = "I58/brain-hemi/15.13um_complete-sample_bm18.ome.zarr"
# Ventana de contraste que usa el visor oficial (neuroglancer.json) para I58
DEFAULT_RANGO = (23045, 24788)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--zarr", default=DEFAULT_ZARR, help="ruta dentro del bucket")
    ap.add_argument("--nivel", type=int, default=4, help="0=full ... 7=mínimo")
    ap.add_argument("--salida", default=None)
    ap.add_argument("--min", type=float, default=DEFAULT_RANGO[0])
    ap.add_argument("--max", type=float, default=DEFAULT_RANGO[1])
    ap.add_argument("--uint16", action="store_true", help="no convertir a uint8")
    a = ap.parse_args()

    url = f"{BUCKET}/{a.zarr}"
    grupo = zarr.open_group(url, mode="r")
    attrs = grupo.attrs
    # OME-Zarr 0.5 guarda todo bajo "ome"; 0.4 lo deja en la raíz
    ms = (attrs["ome"] if "ome" in attrs else attrs)["multiscales"][0]
    ds = ms["datasets"][a.nivel]
    escala_um = ds["coordinateTransformations"][0]["scale"]
    ejes = [e["name"] if isinstance(e, dict) else e for e in ms.get("axes", ["x", "y", "z"])]
    arr = grupo[ds["path"]]
    print(f"nivel {a.nivel}: shape {dict(zip(ejes, arr.shape))} {arr.dtype}, voxel={escala_um[0]} µm")

    vol = np.ascontiguousarray(arr[:].transpose([ejes.index(e) for e in "zyx"]))  # -> (z, y, x)
    escala_um = [escala_um[ejes.index(e)] for e in "zyx"]

    if not a.uint16:
        v = (vol.astype(np.float32) - a.min) / (a.max - a.min)
        vol = (np.clip(v, 0, 1) * 255).astype(np.uint8)

    salida = a.salida or f"{a.zarr.split('/')[0]}_{escala_um[0]:.0f}um_n{a.nivel}"
    np.save(f"{salida}.npy", vol)
    tifffile.imwrite(f"{salida}.tif", vol, imagej=True,
                     resolution=(1 / escala_um[1], 1 / escala_um[2]),
                     metadata={"spacing": escala_um[0], "unit": "um", "axes": "ZYX"})
    vol.tofile(f"{salida}.raw")
    json.dump({"shape_zyx": list(vol.shape), "dtype": str(vol.dtype),
               "voxel_um": escala_um[0], "origen": url, "nivel": a.nivel},
              open(f"{salida}.json", "w"), indent=2)
    print(f"listo: {salida}.npy/.tif/.raw ({vol.nbytes / 1e6:.1f} MB)")


if __name__ == "__main__":
    main()
