#!/usr/bin/env bash
# Optional original desktop RViz2 for features not replicated by the web 2D viewer.
set -euo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ROS_SETUP="${CAR_ROS_SETUP:-/opt/ros/humble/setup.bash}"
if [[ ! -f "$ROS_SETUP" ]]; then
  echo 'ROS2 Humble not found. Start on the Ubuntu ROS2 machine.' >&2
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
exec rviz2 -d "$ROOT/rviz_profiles/competition_stage6.rviz" "$@"
