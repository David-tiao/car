#!/usr/bin/env bash
set -Eeuo pipefail
WS="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
[[ -f /opt/ros/humble/setup.bash ]] || { echo 'ERROR: ROS2 Humble not found' >&2;exit 2; }
set +u
source /opt/ros/humble/setup.bash
set -u
cd "$WS"
colcon build --packages-up-to car_navigation "$@"
printf '\nPASS build finished. source %s/install/setup.bash\n' "$WS"
