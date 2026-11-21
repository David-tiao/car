#!/usr/bin/env bash
# Offline acceptance for fifth-stage TEST-page dual-engine STM32 workbench.
# Runs real anonymous PTY simulator + genuine RaceTaskPlan C source via GCC.
set -Eeuo pipefail
HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
ROOT="$(cd -- "$HERE/../../.." && pwd -P)"
MISSION="$ROOT/algorithm/car_ws/src/car_rule_mission"
STM="$ROOT/STM32F4"
export PYTHONDONTWRITEBYTECODE=1

echo '[PY-TEST] Existing PTY simulation, TEST-page dual-engine FSM and REAL isolation'
cd "$MISSION"
python3 -m unittest -v \
  test_mission_execution test_mission_wire_sim test_mission_ros_session \
  test_mission_ros_web_store test_stm32_fsm_realview

echo '[C-TEST] RaceTaskPlan + independent C feedback gate'
command -v gcc >/dev/null || { echo 'gcc required' >&2; exit 2; }
tmp="$(mktemp -d)"
trap 'rm -rf -- "$tmp"' EXIT
cd "$STM"
gcc -std=c99 -Wall -Wextra -Werror -IApp \
  tests/race/test_task_gate.c App/race_task_plan.c App/race_task_gate.c \
  -o "$tmp/test_task_gate"
"$tmp/test_task_gate"

python3 - "$ROOT" <<'PY'
import ast
from pathlib import Path
import sys
root = Path(sys.argv[1])
for path in ('sim2real/labs/test_stm32_fsm_lab.py',
             'sim2real/web_mode_gateway.py'):
    target = root / path
    ast.parse(target.read_text(encoding='utf-8'), filename=str(target))
    print('SYNTAX OK:', path)
PY
if command -v node >/dev/null 2>&1; then
  node --check "$ROOT/sim2real/web/test_stm32_fsm_workbench.js"
  node --check "$ROOT/sim2real/web/field_ui_hierarchy.js"
fi
echo '[PASS] Preserved mock STM32 PTY, TEST dual-engine browser API and C task-plan feedback gate'
