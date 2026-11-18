#!/usr/bin/env bash
# Optional original desktop RViz2 for features not replicated by the web 2D viewer.
set -euo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo 'ROS2 Humble not found. Start on the Ubuntu ROS2 machine.' >&2
  exit 1
fi
source /opt/ros/humble/setup.bash
exec rviz2 -d "$ROOT/rviz_profiles/competition_stage6.rviz" "$@"
