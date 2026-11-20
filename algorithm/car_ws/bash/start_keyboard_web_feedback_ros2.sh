#!/usr/bin/env bash
# PC ROS2 Humble: mirror local/Pi candidate TwistStamped topics into the LOCAL website.
set -Eeuo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../../.." && pwd -P)"
set +u
source /opt/ros/humble/setup.bash
if [[ -f "$ROOT/algorithm/car_ws/install/setup.bash" ]]; then
  source "$ROOT/algorithm/car_ws/install/setup.bash"
fi
set -u
exec /usr/bin/python3 "$ROOT/sim2real/keyboard_ros2_web_feedback.py" "$@"
