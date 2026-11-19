#!/usr/bin/env bash
# Run ROS2 C++ unit tests for binary CAR-LINK codec.
set -Eeuo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/lib/common.sh"
car_check_workspace
car_source_ros
cd "$CAR_WS"
colcon test --packages-select car_serial_link --event-handlers console_direct+ "$@"
colcon test-result --verbose
