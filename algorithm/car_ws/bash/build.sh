#!/usr/bin/env bash
# build.sh [core|serial|all] [additional colcon build options]
set -Eeuo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/lib/common.sh"
car_check_workspace
car_source_ros_base
mode="${1:-core}"
if (($#)); then shift; fi
cd "$CAR_WS"
case "$mode" in
  core)
    echo 'Building C++ monitoring packages (dependencies included).'
    colcon build --packages-up-to car_stm32_bridge car_debug car_bringup car_serial_link "$@"
    ;;
  serial)
    echo 'Building CAR-LINK bridge and interfaces.'
    colcon build --packages-up-to car_serial_link "$@"
    ;;
  all)
    echo 'Building all discoverable ROS2 packages. Older SLAM packages may need dependencies.'
    colcon build "$@"
    ;;
  *)
    echo 'Usage: bash/build.sh [core|serial|all] [colcon build options]' >&2
    exit 2
    ;;
esac
printf '\nBuild finished. Load environment with: source %s/bash/env.sh\n' "$CAR_WS"
