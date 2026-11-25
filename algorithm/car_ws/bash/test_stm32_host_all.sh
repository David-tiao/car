#!/usr/bin/env bash
# Host-only STM32F4 C firmware regression matrix; from Windows run.ps1.
# Runs compiled C with HAL stubs, NEVER opens /dev/tty or powers a motor.
set -Eeuo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../../.." && pwd -P)"
cd "$ROOT/STM32F4"
command -v gcc >/dev/null || { echo "gcc required" >&2; exit 2; }
tmp="$(mktemp -d -t car_stm32_offline_XXXXXXXX)"
trap 'rm -rf -- "$tmp"' EXIT
common=(-std=c99 -Wall -Wextra -Werror
  -Itests/race/stubs -IConfig -IControl -IApp -IHardware
  -include tests/race/stubs/board.h)
compile_run(){
  local name="$1";shift
  echo "[C/HOST] $name"
  gcc "${common[@]}" "$@" -lm -o "$tmp/$name"
  "$tmp/$name"
}
compile_run test_race tests/race/test_race.c \
  App/race_fsm.c Hardware/vision.c App/race_task_plan.c
compile_run test_uart tests/race/test_uart.c \
  App/race_fsm.c Hardware/vision.c Hardware/bsp_uart_router.c
compile_run test_ring tests/race/test_ring.c \
  App/race_fsm.c Hardware/vision.c App/app_ring_align.c
compile_run test_app \
  -include tests/race/stubs/board.h \
  -include tests/race/stubs/config.h \
  tests/race/test_app.c App/race_fsm.c App/app_race.c App/race_ui.c \
  App/app_pick_fsm.c App/app_pick_motion.c App/race_task_plan.c

compile_run test_task_mission -DTEST_TASK_MISSION -include tests/race/stubs/config.h \
  tests/race/test_app.c App/race_fsm.c App/app_race.c App/race_ui.c \
  App/app_pick_fsm.c App/app_pick_motion.c App/race_task_plan.c
TEST_DROP_CONFIRMATION=1 "$tmp/test_task_mission"
TEST_STOP_TRANSFER=1 "$tmp/test_task_mission"
compile_run test_task_gate tests/race/test_task_gate.c App/race_task_gate.c App/race_task_plan.c

# The ULog host logger also has a deterministic test suite with firmware
# modes 0 and 3. Cover no-logging build and direct/manual flow branches.
ulog_common=(-DULOG_TEST -std=c99 -Wall -Wextra -Werror
  -Itests/ulog -Itests/race/stubs -IConfig -IControl -IApp -IHardware
  -include tests/race/stubs/board.h)
for mode in 0 3; do
  echo "[C/HOST] ULog APP_RUN_MODE=$mode"
  gcc "${ulog_common[@]}" "-DAPP_RUN_MODE=$mode" \
    -Dsnprintf=ulog_test_snprintf tests/ulog/test_ulog.c \
    Hardware/bsp_ulog.c App/race_task_plan.c -lm -o "$tmp/ulog_$mode"
  for case in normal early abort timeout tx_error format_error nonfinite manual null init_error; do
    "$tmp/ulog_$mode" "$case"
  done
done
gcc "${ulog_common[@]}" -DULOG_ENABLE=0 tests/ulog/test_ulog.c \
  Hardware/bsp_ulog.c -lm -o "$tmp/ulog_disabled"
"$tmp/ulog_disabled"
for direct in 0 1; do
  echo "[C/HOST] ULog APP_RACE flow direct=$direct"
  args=()
  if [[ "$direct" == 1 ]]; then args+=(-DTEST_DIRECT=1);fi
  gcc "${ulog_common[@]}" -include tests/ulog/config.h "${args[@]}" \
    tests/ulog/test_flow.c App/app_race.c App/race_ui.c App/race_fsm.c \
    App/app_pick_fsm.c App/app_pick_motion.c App/race_task_plan.c -lm \
    -o "$tmp/ulog_flow_$direct"
  "$tmp/ulog_flow_$direct"
done
echo "[PASS] STM32F4 C host race/QR/UART/ring/app/ULog regression — NO HARDWARE"
