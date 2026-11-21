#!/usr/bin/env bash
# Ubuntu second or third terminal: connect existing ROS2 navigation topics to the V12 website.
set -euo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ROS_SETUP=/opt/ros/humble/setup.bash
if [[ ! -f "$ROS_SETUP" ]]; then
  echo 'ROS2 Humble not installed at /opt/ros/humble. This bridge runs on the Ubuntu ROS2 PC.' >&2
  exit 1
fi
# Preserve the user's existing ROS_DOMAIN_ID/RMW_IMPLEMENTATION for Pi5 connectivity.
source "$ROS_SETUP"
exec /usr/bin/python3 "$ROOT/bridges/ros2_visualization_bridge.py" "$@"
