#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DEVICE="/dev/ttyACM0"
UPLOAD=0
RUNTIME_MS=0
DEBUG=0

usage() {
  cat <<'EOF'
Usage: ./run_firmware.sh [--device /dev/ttyACM0] [--upload] [--runtime-ms N] [--debug]

Starts the current Pico firmware in the verified startup path:
1. UART-only homing
2. move to center
3. one-axis DMX runtime

Options:
  --device PATH     MicroPython serial device. Default: /dev/ttyACM0
  --upload          Upload the current firmware/ files before starting
  --runtime-ms N    Exit after N milliseconds. Default: 0 (run until interrupted)
  --debug           Enable RP2040 console logging for bring-up
  --help            Show this help

While this script is attached, firmware logs stay visible in the terminal.
Press Ctrl-C to stop the session.
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --device)
      DEVICE="${2:?missing device path}"
      shift 2
      ;;
    --upload)
      UPLOAD=1
      shift
      ;;
    --runtime-ms)
      RUNTIME_MS="${2:?missing runtime duration}"
      shift 2
      ;;
    --debug)
      DEBUG=1
      shift
      ;;
    --help|-h)
      usage
      exit 0
      ;;
    *)
      echo "Unknown argument: $1" >&2
      usage >&2
      exit 1
      ;;
  esac
done

if ! command -v mpremote >/dev/null 2>&1; then
  echo "mpremote is required but was not found in PATH" >&2
  exit 1
fi

if [[ "${UPLOAD}" -eq 1 ]]; then
  "${ROOT_DIR}/firmware/deploy.sh" "${DEVICE}"
fi

echo "Preparing ${DEVICE}"
mpremote connect "${DEVICE}" fs rm homing_result.json >/dev/null 2>&1 || true
mpremote connect "${DEVICE}" fs rm controller_status.json >/dev/null 2>&1 || true

EXEC_CODE="import sys; sys.modules.pop('main', None); sys.modules.pop('config', None); import config; config.RUN_RUNTIME_AFTER_HOMING=True; config.RUNTIME_EXIT_AFTER_MS=${RUNTIME_MS}; config.DEBUG_LOGGING=${DEBUG}; import main; main.main()"

echo "Starting firmware on ${DEVICE}"
echo "  upload=${UPLOAD} runtime_ms=${RUNTIME_MS} debug=${DEBUG}"
echo "  homing -> center -> DMX runtime"
echo

exec mpremote connect "${DEVICE}" exec "${EXEC_CODE}"
