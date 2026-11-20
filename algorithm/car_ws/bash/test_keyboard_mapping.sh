#!/usr/bin/env bash
# Fifth-stage keyboard mapping software acceptance: no ROS / STM32 required.
set -Eeuo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../../.." && pwd -P)"
export PYTHONDONTWRITEBYTECODE=1
cd "$ROOT"
python3 -m unittest discover -s sim2real/tests -p 'test_keyboard_mapping.py' -v
python3 -m unittest discover -s sim2real/tests -p 'test_four_modes.py' -v
python3 - "$ROOT" <<'PY'
import ast
from pathlib import Path
import sys
root=Path(sys.argv[1])
for file in ("sim2real/keyboard_mapping_core.py",
             "sim2real/keyboard_ros2_pc.py",
             "sim2real/keyboard_ros2_pi.py",
             "sim2real/web_mode_gateway.py"):
    ast.parse((root/file).read_text(encoding="utf-8"), filename=file)
    print("SYNTAX OK:", file)
PY
if command -v node >/dev/null 2>&1; then
  node --check sim2real/web/keyboard_mapping_panel.js
  node --check sim2real/web/mode_router.js
  node tests/test_mode_router.js 2>/dev/null || node sim2real/tests/test_mode_router.js
fi
echo '[PASS] KEYBOARD MAP: no motor command, REAL-only error and ROS2 candidate protocol'
