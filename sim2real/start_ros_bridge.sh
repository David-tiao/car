#!/usr/bin/env bash
# Second terminal: ROS2 /odom -> running car HTTP server.
set -euo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ROS_SETUP=/opt/ros/humble/setup.bash
if [[ ! -f "$ROS_SETUP" ]]; then
  echo "ROS2 Humble not found at $ROS_SETUP. Run the bridge on your ROS2 Ubuntu computer."
  exit 1
fi
source "$ROS_SETUP"
# For custom robot nodes, source your workspace FIRST in the normal way if needed.
exec /usr/bin/python3 "$ROOT/bridges/ros2_odom_bridge.py" "$@"
