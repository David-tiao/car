#!/usr/bin/env bash
set -Eeuo pipefail
source "$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)/lib/common.sh"
car_check_workspace
WS="$CAR_WS"
car_source_ros_base
cd "$WS"
colcon build --packages-up-to car_navigation "$@"
printf '\nPASS build finished. source %s/install/setup.bash\n' "$WS"
