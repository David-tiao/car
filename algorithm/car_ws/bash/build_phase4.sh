#!/usr/bin/env bash
set -Eeuo pipefail
source "$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)/lib/common.sh"
car_check_workspace
WS="$CAR_WS"
car_source_ros_base
cd "$WS"
colcon build --packages-up-to car_serial_link car_debug car_vision car_mechanism car_mission "$@"
echo "PASS: phase4 built; run: bash $WS/bash/start_phase4_demo.sh"
