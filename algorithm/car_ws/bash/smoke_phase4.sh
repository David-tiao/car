#!/usr/bin/env bash
# Starts the entire offline pipeline, verifies messages, and kills its process group.
set -Eeuo pipefail
WS="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
set +u
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"
set -u
LOG="$(mktemp -d -t car_phase4_smoke_XXXXXXXX)/launch.log"
pid=''
cleanup(){
  if [[ -n "$pid" ]];then
    kill -INT -- "-$pid" 2>/dev/null || true
    sleep 1
    kill -TERM -- "-$pid" 2>/dev/null || true
    wait "$pid" 2>/dev/null || true
  fi
}
trap cleanup EXIT
setsid ros2 launch car_mission offline_mission.launch.xml >"$LOG" 2>&1 &
pid=$!
echo "Offline ROS2 phase4 test started; log=$LOG"
sleep 3
if ! kill -0 "$pid" 2>/dev/null;then tail -80 "$LOG";exit 1;fi
names=(/odom /car/vision/event /car/vision/status /car/mechanism/health /car/mission/summary /car/stm32/command_feedback)
types=(nav_msgs/msg/Odometry car_interfaces/msg/VisionEvent car_interfaces/msg/VisionStatus car_interfaces/msg/MechanismHealth car_interfaces/msg/MissionSummary car_interfaces/msg/CommandFeedback)
for i in "${!names[@]}";do
  printf 'Checking %s ... ' "${names[i]}"
  if timeout 12 ros2 topic echo "${names[i]}" "${types[i]}" --once >/dev/null 2>&1;then echo PASS
  else echo 'FAIL';tail -100 "$LOG";exit 1;fi
done
echo 'PASS: simulated binary serial -> ROS2 -> vision/mechanism/mission monitors'
echo "Log: $LOG"
