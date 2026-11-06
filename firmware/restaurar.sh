#!/usr/bin/env bash
# Vuelve a grabar el firmware original (respaldo completo de 4 MB, 2026-10-22).
set -euo pipefail
cd "$(dirname "$0")"
uvx esptool --port "${1:-/dev/ttyACM0}" -b 921600 write-flash 0 respaldo/esp32c3_original_4MB.bin
