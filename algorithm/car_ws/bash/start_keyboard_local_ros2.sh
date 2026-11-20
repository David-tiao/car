#!/usr/bin/env bash
# ONE PC, NO Pi/STM32 REQUIRED. Runs real ROS2 pub+sub for /keyboard local loopback.
# Prerequisites: website already on localhost; ROS2 Humble installed/sourced.
set -Eeuo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../../.." && pwd -P)"
SERVER="${1:-http://127.0.0.1:8090}"
set +u
source /opt/ros/humble/setup.bash
if [[ -f "$ROOT/algorithm/car_ws/install/setup.bash" ]]; then
  source "$ROOT/algorithm/car_ws/install/setup.bash"
fi
set -u
pc=""
feedback=""
cleanup(){
  if [[ -n "$pc" ]];then kill "$pc" 2>/dev/null || true;fi
  if [[ -n "$feedback" ]];then kill "$feedback" 2>/dev/null || true;fi
  wait 2>/dev/null || true
}
trap cleanup EXIT INT TERM
/usr/bin/python3 "$ROOT/sim2real/keyboard_ros2_pc.py" --server "$SERVER" &
pc=$!
/usr/bin/python3 "$ROOT/sim2real/keyboard_ros2_web_feedback.py" --server "$SERVER" &
feedback=$!
echo "[KEYBOARD] LOCAL ROS2 pub+sub running (no Pi needed). Open $SERVER/keyboard"
echo "[KEYBOARD] Default mode is local ROS2; STM32 UART TX remains disabled."
wait -n "$pc" "$feedback"
