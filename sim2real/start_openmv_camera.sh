#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
if ! /usr/bin/python3 -c 'import serial' 2>/dev/null; then
  echo "Missing pyserial. Install: sudo apt update && sudo apt install python3-serial"
  exit 1
fi
exec /usr/bin/python3 "$ROOT/openmv_usb_sender.py" "$@"
