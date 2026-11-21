#!/usr/bin/env bash
# PC Ubuntu ROS2: website keyboard -> ROS2 candidate.
set -Eeuo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../../.." && pwd -P)"
set +u
source /opt/ros/humble/setup.bash
if [[ -f "$ROOT/algorithm/car_ws/install/setup.bash" ]]; then
  source "$ROOT/algorithm/car_ws/install/setup.bash"
fi
set -u
exec /usr/bin/python3 "$ROOT/sim2real/keyboard/keyboard_ros2_pc.py" "$@"
