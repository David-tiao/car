#!/usr/bin/env bash
# Regression test for original ULog43 CSV -> ROS2 -> car_debug.
set -Eeuo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/lib/common.sh"
car_check_workspace
car_source_ros
work="$(mktemp -d -t car_ulog_smoke_XXXXXXXX)"
pid=''
cleanup() { if [[ -n "$pid" ]];then kill -INT -- "-$pid" 2>/dev/null || true; sleep 1; kill -TERM -- "-$pid" 2>/dev/null || true; wait "$pid" 2>/dev/null || true;fi; }
trap cleanup EXIT
setsid ros2 launch car_bringup offline_debug.launch.xml replay_interval_ms:=150 >"$work/launch.log" 2>&1 &
pid=$!
echo "ULog43 offline test started: PID=$pid, log=$work/launch.log"
sleep 3
if ! kill -0 "$pid" 2>/dev/null;then echo 'FAIL: launch stopped';tail -70 "$work/launch.log";exit 1;fi
check() {
  local topic="$1" type="$2" i
  printf 'Checking %s ... ' "$topic"
  for i in 1 2 3 4 5;do
    if timeout 4 ros2 topic echo "$topic" "$type" --once >"$work/last-topic.txt" 2>&1;then echo PASS;return 0;fi
    sleep 1
  done
  echo 'FAIL'; cat "$work/last-topic.txt";tail -70 "$work/launch.log";return 1
}
check /odom nav_msgs/msg/Odometry
check /car/debug/trajectory nav_msgs/msg/Path
check /car/debug/diagnostics diagnostic_msgs/msg/DiagnosticArray
printf '\nPASS: synthetic CSV -> C++ ULog43 bridge -> ROS2 -> diagnostics\nLog: %s/launch.log\n' "$work"
