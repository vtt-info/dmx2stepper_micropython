#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

exec python "${ROOT_DIR}/hil/verify_smooth_dmx_ramp.py" "$@"
