#!/usr/bin/env bash
# Compila y sube firmware/hoa_imu a la ESP32-C3 Super Mini.
set -euo pipefail
cd "$(dirname "$0")"
PUERTO="${1:-/dev/ttyACM0}"
FQBN="espressif:esp32:nologo_esp32c3_super_mini:USBMode=hwcdc,CDCOnBoot=default"
arduino-cli compile -b "$FQBN" --output-dir build hoa_imu
arduino-cli upload -b "$FQBN" -p "$PUERTO" --input-dir build hoa_imu
