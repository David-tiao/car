#!/usr/bin/env bash
# Run while start_phase4_demo.sh is still active in another terminal.
set -Eeuo pipefail
WS="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
set +u
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"
set -u
ros2 node list
ros2 topic list -t
names=(/odom /car/vision/event /car/vision/status /car/mechanism/state /car/mechanism/health /car/mission/state /car/mission/summary /car/stm32/link_status /car/stm32/command_feedback)
types=(nav_msgs/msg/Odometry car_interfaces/msg/VisionEvent car_interfaces/msg/VisionStatus car_interfaces/msg/MechanismState car_interfaces/msg/MechanismHealth car_interfaces/msg/MissionState car_interfaces/msg/MissionSummary car_interfaces/msg/LinkStatus car_interfaces/msg/CommandFeedback)
fail=0
for i in "${!names[@]}";do
  printf '%s ... ' "${names[i]}"
  if timeout 10 ros2 topic echo "${names[i]}" "${types[i]}" --once >/dev/null 2>&1;then echo PASS
  else echo FAIL;fail=1;fi
done
exit "$fail"
