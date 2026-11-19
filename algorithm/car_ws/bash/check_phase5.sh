#!/usr/bin/env bash
# Check online topic presence and data; run in another terminal during phase 5 launch.
set -Eeuo pipefail
WS="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
set +u
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"
set -u
failed=0
for entry in \
  '/map nav_msgs/msg/OccupancyGrid' \
  '/car/navigation/keepout_mask nav_msgs/msg/OccupancyGrid' \
  '/scan sensor_msgs/msg/LaserScan' \
  '/car/navigation/diagnostics diagnostic_msgs/msg/DiagnosticArray' \
  '/car/navigation/plan_status std_msgs/msg/String';do
  read -r name kind <<< "$entry"
  printf '%s ... ' "$name"
  if timeout 12 ros2 topic echo "$name" "$kind" --once >/dev/null 2>&1;then echo PASS
  else echo FAIL;failed=1;fi
done
printf '\nPreview path needs manual start+goal. Run bash/phase5_demo_goal.sh first.\n'
if [[ "${1:-}" == path ]];then
  printf '/car/navigation/preview_path ... '
  if timeout 12 ros2 topic echo /car/navigation/preview_path nav_msgs/msg/Path --once >/dev/null 2>&1;then echo PASS
  else echo FAIL;failed=1;fi
fi
exit "$failed"
