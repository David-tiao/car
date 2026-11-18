#!/usr/bin/env bash
# Compatible with existing 'car' shell shortcut.
set -euo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
exec python3 "$ROOT/launcher.py"
