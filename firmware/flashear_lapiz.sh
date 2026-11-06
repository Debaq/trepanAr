#!/usr/bin/env bash
# Graba el firmware del lápiz (ESP32-C3 Super Mini + MPU9250) por USB.
# Uso: ./flashear_lapiz.sh [/dev/ttyACM0]
set -euo pipefail
cd "$(dirname "$0")"
PUERTO="${1:-$(ls /dev/ttyACM* /dev/ttyUSB* 2>/dev/null | head -1)}"
[ -n "$PUERTO" ] || { echo "No encuentro el lápiz: conéctalo por USB."; exit 1; }
if command -v esptool >/dev/null; then ESPTOOL=(esptool)
elif command -v esptool.py >/dev/null; then ESPTOOL=(esptool.py)
elif command -v uvx >/dev/null; then ESPTOOL=(uvx esptool)
elif command -v pipx >/dev/null; then ESPTOOL=(pipx run esptool)
else echo "Falta esptool: instala con 'pipx install esptool' (o pip install esptool)."; exit 1; fi
echo "Grabando en $PUERTO ..."
"${ESPTOOL[@]}" --chip esp32c3 --port "$PUERTO" -b 921600 write-flash 0x0 hoa_imu_esp32c3.bin
echo "Listo. Desconecta y vuelve a conectar el lápiz."
