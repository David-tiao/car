#!/usr/bin/env bash
# Second terminal: ROS2 /odom -> running car HTTP server.
set -euo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ROS_SETUP="${CAR_ROS_SETUP:-/opt/ros/humble/setup.bash}"
if [[ ! -f "$ROS_SETUP" ]]; then
  echo "ROS2 Humble not found at $ROS_SETUP. Run the bridge on your ROS2 Ubuntu computer."
  exit 1
fi
if [[ -n "${CONDA_PREFIX:-}" || -n "${VIRTUAL_ENV:-}" ]]; then
  echo 'ERROR: use a clean ROS terminal (no active Conda/venv)' >&2
  exit 2
fi
set +u
source "$ROS_SETUP"
set -u
[[ "${ROS_DISTRO:-}" == humble ]] || { echo 'ERROR: ROS2 Humble required' >&2; exit 2; }
# For custom robot nodes, source your workspace FIRST in the normal way if needed.
exec /usr/bin/python3 "$ROOT/bridges/ros2_odom_bridge.py" "$@"
