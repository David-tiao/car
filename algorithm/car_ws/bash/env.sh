#!/usr/bin/env bash
# Usage: source ~/Desktop/car/algorithm/car_ws/bash/env.sh
if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  echo 'This file must be sourced, e.g.: source ~/Desktop/car/algorithm/car_ws/bash/env.sh' >&2
  exit 2
fi
source "$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)/lib/common.sh"
car_check_workspace || return $?
car_source_ros || return $?
printf 'ROS2 Humble environment loaded: %s\n' "$CAR_WS"
