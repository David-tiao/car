#!/usr/bin/env bash
set -Eeuo pipefail
WS="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
[[ -f /opt/ros/humble/setup.bash ]] || { echo 'ERROR: ROS2 Humble missing' >&2;exit 2; }
set +u
source /opt/ros/humble/setup.bash
set -u
cd "$WS"
colcon build --packages-up-to car_serial_link car_debug car_vision car_mechanism car_mission "$@"
echo "PASS: phase4 built; run: bash $WS/bash/start_phase4_demo.sh"
