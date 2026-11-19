#!/usr/bin/env bash
# Usage: check_topics.sh [serial|ulog|list]
# Run while the corresponding launch is still running in another terminal.
set -Eeuo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/lib/common.sh"
car_check_workspace
car_source_ros
mode="${1:-list}"
case "$mode" in
  list)
    echo '=== Nodes ==='; ros2 node list
    echo '=== Topics ==='; ros2 topic list -t
    ;;
  serial|ulog)
    echo '=== Nodes ==='; ros2 node list
    echo '=== Topics ==='; ros2 topic list -t
    if [[ "$mode" == serial ]]; then
      topics=(/odom /car/chassis/wheels /imu/data /car/vision/event /car/mechanism/state /car/stm32/link_status)
      types=(nav_msgs/msg/Odometry car_interfaces/msg/WheelTelemetry sensor_msgs/msg/Imu car_interfaces/msg/VisionEvent car_interfaces/msg/MechanismState car_interfaces/msg/LinkStatus)
    else
      topics=(/odom /car/chassis/wheels /car/mission/state /car/debug/diagnostics)
      types=(nav_msgs/msg/Odometry car_interfaces/msg/WheelTelemetry car_interfaces/msg/MissionState diagnostic_msgs/msg/DiagnosticArray)
    fi
    failed=0
    for i in "${!topics[@]}"; do
      printf 'Checking %s ... ' "${topics[i]}"
      if timeout 8 ros2 topic echo "${topics[i]}" "${types[i]}" --once >/dev/null 2>&1;then
        echo PASS
      else
        echo 'NOT RECEIVED (launch stopped, wrong type, or no data)'
        failed=1
      fi
    done
    exit "$failed"
    ;;
  *) echo 'Usage: bash/check_topics.sh [serial|ulog|list]' >&2;exit 2 ;;
esac
