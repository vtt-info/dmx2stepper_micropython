#!/usr/bin/env bash
set -euo pipefail

DEVICE="${1:-/dev/ttyACM0}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

cd "${SCRIPT_DIR}"
mpremote connect "${DEVICE}" fs cp config.py dmx_receiver.py pio_stepper.py tmc2209_uart.py tmc2209.py motion_axis.py main.py :
echo "Firmware uploaded to ${DEVICE}"
