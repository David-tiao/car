#!/usr/bin/env bash
# End-to-end offline CAR-LINK test. All temporary artifacts live in /tmp.
set -Eeuo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/lib/common.sh"
car_check_workspace
car_source_ros
work="$(mktemp -d -t car_serial_smoke_XXXXXXXX)"
pid=''
cleanup() {
  if [[ -n "$pid" ]]; then kill -INT -- "-$pid" 2>/dev/null || true; sleep 1; kill -TERM -- "-$pid" 2>/dev/null || true; wait "$pid" 2>/dev/null || true; fi
}
trap cleanup EXIT
# New process group makes it possible to stop launch and child processes together.
setsid ros2 launch car_serial_link offline_serial_demo.launch.xml with_debug:=true >"$work/launch.log" 2>&1 &
pid=$!
echo "CAR-LINK offline test started: PID=$pid, log=$work/launch.log"
sleep 3
if ! kill -0 "$pid" 2>/dev/null;then echo 'FAIL: launch stopped'; tail -70 "$work/launch.log";exit 1;fi
check() {
  local topic="$1" type="$2" i
  printf 'Checking %s ... ' "$topic"
  for i in 1 2 3 4 5 6;do
    if timeout 4 ros2 topic echo "$topic" "$type" --once >"$work/last-topic.txt" 2>&1; then echo PASS;return 0;fi
    sleep 1
  done
  echo 'FAIL';cat "$work/last-topic.txt";tail -70 "$work/launch.log";return 1
}
check /odom nav_msgs/msg/Odometry
check /car/chassis/wheels car_interfaces/msg/WheelTelemetry
check /imu/data sensor_msgs/msg/Imu
check /car/vision/event car_interfaces/msg/VisionEvent
check /car/mechanism/state car_interfaces/msg/MechanismState
check /car/power car_interfaces/msg/PowerTelemetry
check /car/stm32/link_status car_interfaces/msg/LinkStatus
check /car/stm32/command_feedback car_interfaces/msg/CommandFeedback
check /car/debug/trajectory nav_msgs/msg/Path
printf '\nPASS: mock binary UART -> CAR-LINK C++ -> ROS2 topics -> trajectory\nLog: %s/launch.log\n' "$work"
