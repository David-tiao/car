#!/usr/bin/env bash
# Shared path discovery. Source from car_ws/bash scripts.
CAR_WS="${CAR_WS:-$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd -P)}"
CAR_ROS_SETUP="${CAR_ROS_SETUP:-/opt/ros/humble/setup.bash}"

car_check_workspace() {
  if [[ ! -d "$CAR_WS/src" ]]; then
    echo "ERROR: not a ROS2 workspace (missing src/): $CAR_WS" >&2
    return 2
  fi
  CAR_WS="$(cd -- "$CAR_WS" && pwd -P)" || return 2
}

car_source_setup() {
  local setup="$1" had_nounset=0 rc=0
  [[ -r "$setup" ]] || { echo "ERROR: ROS setup missing: $setup" >&2; return 2; }
  case "$-" in *u*) had_nounset=1 ;; esac
  set +u
  source "$setup" || rc=$?
  if ((had_nounset)); then set -u; fi
  if ((rc)); then echo "ERROR: sourcing $setup failed ($rc)" >&2; fi
  return "$rc"
}

car_source_ros_base() {
  if [[ -n "${CONDA_PREFIX:-}" || -n "${VIRTUAL_ENV:-}" ]]; then
    echo 'ERROR: use a clean ROS terminal (no active Conda/venv)' >&2
    return 2
  fi
  car_source_setup "$CAR_ROS_SETUP" || return $?
  [[ "${ROS_DISTRO:-}" == humble ]] || {
    echo 'ERROR: ROS2 Humble required; check CAR_ROS_SETUP' >&2; return 2;
  }
}

car_source_ros() {
  car_source_ros_base || return $?
  car_source_setup "$CAR_WS/install/setup.bash"
}
