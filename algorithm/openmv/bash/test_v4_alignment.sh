#!/usr/bin/env bash
# Host-only V4 OpenMV firmware contract checks. No serial device / ROS / motor.
set -Eeuo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
export PYTHONDONTWRITEBYTECODE=1
python3 -m unittest discover -s "$ROOT/tests" -p 'test_v4_electrical_contract.py' -v
python3 - "$ROOT" <<'PY'
import ast
import sys
from pathlib import Path
root = Path(sys.argv[1])
for path in sorted((root / 'openmv_v4').glob('*.py')):
    ast.parse(path.read_text(encoding='utf-8'), filename=str(path))
    print('SYNTAX OK', path.name)
for path in sorted((root.parent.parent / 'STM32F4' / 'vision_openmv_stm32_v4_redownload' / 'OpenMV').glob('*.py')):
    ast.parse(path.read_text(encoding='utf-8'), filename=str(path))
    print('MIRROR SYNTAX OK', path.name)
PY
echo '[PASS] Host-only V4 UART and QR/session contract regression'
