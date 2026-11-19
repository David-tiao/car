#!/usr/bin/env bash
# Shared helpers. This file must be sourced by scripts in car_ws/bash/.
CAR_WS="${CAR_WS:-$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd -P)}"

car_check_workspace() {
  if [[ ! -d "$CAR_WS/src" ]]; then
    echo "ERROR: not a ROS2 workspace (missing src/): $CAR_WS" >&2
    return 2
  fi
}

car_source_ros() {
  local had_nounset=0 rc=0
  case "$-" in *u*) had_nounset=1 ;; esac
  if [[ ! -f /opt/ros/humble/setup.bash ]]; then
    echo 'ERROR: /opt/ros/humble/setup.bash missing' >&2
    return 2
  fi
  if [[ ! -f "$CAR_WS/install/setup.bash" ]]; then
    echo "ERROR: $CAR_WS/install/setup.bash missing; run bash/build.sh first" >&2
    return 2
  fi
  # ROS2 Humble setup uses variables that might be unset under 'set -u'.
  set +u
  source /opt/ros/humble/setup.bash || rc=$?
  if ((rc == 0)); then source "$CAR_WS/install/setup.bash" || rc=$?; fi
  if ((had_nounset == 1)); then set -u; fi
  if ((rc != 0)); then
    echo "ERROR: sourcing ROS2 environment failed ($rc)" >&2
    return "$rc"
  fi
}

car_source_ros_base() {
  local had_nounset=0 rc=0
  case "$-" in *u*) had_nounset=1 ;; esac
  [[ -f /opt/ros/humble/setup.bash ]] || { echo 'ERROR: ROS2 Humble not installed' >&2; return 2; }
  set +u
  source /opt/ros/humble/setup.bash || rc=$?
  if ((had_nounset == 1)); then set -u; fi
  ((rc == 0)) || return "$rc"
}
