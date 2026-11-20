#!/usr/bin/env bash
# One-command V4 QR contract acceptance. No STM32, no OpenMV, no ROS, no motors.
set -Eeuo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../../.." && pwd -P)"
STM="$ROOT/STM32F4"
export PYTHONDONTWRITEBYTECODE=1
python3 -m unittest discover -s "$ROOT/algorithm/openmv/tests" -p 'test_v4_electrical_contract.py' -v
python3 -m unittest discover -s "$ROOT/algorithm/openmv/tests" -p 'test_v4_qr_lab.py' -v
command -v gcc >/dev/null || { echo '[ERROR] gcc required for actual STM32 C tests' >&2; exit 2; }
out="$(mktemp -d)"
trap 'rm -rf -- "$out"' EXIT
cd "$STM"
common=(-std=c99 -Wall -Wextra -Werror
  -Itests/race/stubs -IConfig -IControl -IApp -IHardware
  -include tests/race/stubs/board.h)
compile_run() {
  local name="$1"; shift
  echo "[C-TEST] Compile $name against real STM32 source"
  gcc "${common[@]}" "$@" -lm -o "$out/$name"
  "$out/$name"
}
compile_run test_race tests/race/test_race.c App/race_fsm.c Hardware/vision.c App/race_task_plan.c
compile_run test_uart tests/race/test_uart.c App/race_fsm.c Hardware/vision.c Hardware/bsp_uart_router.c
compile_run test_ring tests/race/test_ring.c App/race_fsm.c Hardware/vision.c App/app_ring_align.c
compile_run test_app -include tests/race/stubs/board.h -include tests/race/stubs/config.h \
  tests/race/test_app.c App/race_fsm.c App/app_race.c App/race_ui.c \
  App/app_pick_fsm.c App/app_pick_motion.c App/race_task_plan.c
echo '[PASS] STM32 C parser + OpenMV V4 + TEST web lab contract'
