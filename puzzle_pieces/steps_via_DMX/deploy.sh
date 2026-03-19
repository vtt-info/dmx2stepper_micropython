#!/usr/bin/env bash
set -euo pipefail

BOARD_PORT="${1:-/dev/ttyACM0}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

MPREMOTE="$PROJECT_ROOT/.venv/bin/python -m mpremote"

if [[ ! -x "$PROJECT_ROOT/.venv/bin/python" ]]; then
  echo "Error: venv python not found at $PROJECT_ROOT/.venv/bin/python"
  echo "Create it first (example): python3 -m venv .venv && .venv/bin/pip install mpremote"
  exit 1
fi

echo "Deploying to $BOARD_PORT ..."
cd "$SCRIPT_DIR"

$MPREMOTE connect "$BOARD_PORT" fs cp config.py dmx_receiver.py main.py :

echo "Running main.py on board ..."
$MPREMOTE connect "$BOARD_PORT" run main.py
