#!/usr/bin/env bash
# 唯一对外管理入口：start | stop | restart | status | doctor | logs
set -Eeuo pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd -- "$SCRIPT_DIR/.." && pwd)"
exec python3 "$ROOT/oneclick_stage/ai_manager.py" "$@"
