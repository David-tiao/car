#!/usr/bin/env bash
# Launch RViz preview (no radar or planner processes are started by this script).
set -Eeuo pipefail
WS="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
set +u
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"
set -u
RVIZ="$(ros2 pkg prefix car_navigation)/share/car_navigation/rviz/phase5_preview.rviz"
[[ -f "$RVIZ" ]] || { echo "ERROR: RViz config missing: $RVIZ" >&2;exit 2; }
exec rviz2 -d "$RVIZ"
