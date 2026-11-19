#!/usr/bin/env bash
# Stage 3: isolated TEST-only ROS2 mission replay and localhost web relay.
# Does NOT launch real hardware bridge, /cmd_vel publisher, or /odom faker.
set -Eeuo pipefail
HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
WS="$(cd -- "$HERE/.." && pwd -P)"
ROOT="$(cd -- "$WS/../.." && pwd -P)"
[[ -f /opt/ros/humble/setup.bash ]] || { echo 'ERROR: ROS2 Humble not installed' >&2; exit 2; }
set +u
source /opt/ros/humble/setup.bash
[[ ! -f "$WS/install/setup.bash" ]] || source "$WS/install/setup.bash"
set -u
export ROS_DOMAIN_ID="${CAR_TEST_ROS_DOMAIN_ID:-73}"
export PYTHONUNBUFFERED=1
SERVER="${CAR_TEST_WEB_ORIGIN:-http://127.0.0.1:8080}"
case "$SERVER" in
  http://127.0.0.1:*) ;;
  *) echo 'ERROR: server must be 127.0.0.1 localhost TEST website' >&2; exit 2 ;;
esac
echo "[TEST ONLY] ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
echo '[TEST ONLY] synthetic Dijkstra -> PTY -> ROS2 /car/test/mission/state -> web'
echo '[TEST ONLY] no /cmd_vel, no real UART, no synthetic /odom publication'
python3 "$ROOT/sim2real/test_mission_relay.py" --server "$SERVER" &
RELAY_PID=$!
cleanup() {
  kill "$RELAY_PID" 2>/dev/null || true
  wait "$RELAY_PID" 2>/dev/null || true
}
trap cleanup EXIT
exec_cleanup() { :; }
python3 "$WS/src/car_rule_mission/mission_ros_test.py" "$@"
