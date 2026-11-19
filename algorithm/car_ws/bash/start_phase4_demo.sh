#!/usr/bin/env bash
set -Eeuo pipefail
WS="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
set +u
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"
set -u
exec ros2 launch car_mission offline_mission.launch.xml "$@"
