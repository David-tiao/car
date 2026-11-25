#!/usr/bin/env bash
# PC Ubuntu ROS2: website keyboard -> ROS2 candidate.
set -Eeuo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../../.." && pwd -P)"
source "$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)/lib/common.sh"
car_check_workspace
car_source_ros_base
[[ ! -f "$CAR_WS/install/setup.bash" ]] || car_source_setup "$CAR_WS/install/setup.bash"
exec /usr/bin/python3 "$ROOT/sim2real/keyboard/keyboard_ros2_pc.py" "$@"
