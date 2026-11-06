#!/usr/bin/env bash
# Construye el binario de un solo archivo y el paquete de distribución:
#   dist/trepanar-linux/            binario + firmware + guías
#   dist/trepanar-linux.tar.gz
# Los cerebros NO van dentro: la demo los descarga la primera vez (pantalla de paquetes).
# Requiere uv (https://docs.astral.sh/uv/). Uso: ./construir_binario.sh
set -euo pipefail
cd "$(dirname "$0")"
SALIDA=dist/trepanar-linux

rm -rf build_pyi dist/construyendo
uv run --no-project --python 3.12 --with-requirements requirements.txt --with pyinstaller \
  pyinstaller --noconfirm --onefile --name trepanar \
  --distpath dist/construyendo --workpath build_pyi --specpath build_pyi \
  --hidden-import serial --hidden-import compresso --hidden-import aiohttp \
  --collect-submodules fsspec --collect-all zarr --collect-all numcodecs \
  --copy-metadata zarr --copy-metadata numcodecs --copy-metadata fsspec \
  --exclude-module tkinter --exclude-module matplotlib --exclude-module PIL \
  trepanar.py

mkdir -p "$SALIDA/firmware"
mv -f dist/construyendo/trepanar "$SALIDA/"
rmdir dist/construyendo
cp README.md GUIA-VISITANTES.md LEEME-TECNICO.md "$SALIDA/"
if [ -f firmware/build/hoa_imu.ino.merged.bin ]; then
  cp firmware/build/hoa_imu.ino.merged.bin "$SALIDA/firmware/hoa_imu_esp32c3.bin"
fi
cp firmware/flashear_lapiz.sh "$SALIDA/firmware/" 2>/dev/null || true
(cd dist && rm -f trepanar-linux.tar.gz && tar czf trepanar-linux.tar.gz trepanar-linux)
ls -la dist/trepanar-linux.tar.gz "$SALIDA"
