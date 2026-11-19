#!/usr/bin/env bash
# Synthetic CSV -> ULog43 C++ bridge -> ROS2; no hardware.
set -Eeuo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/lib/common.sh"
car_check_workspace
car_source_ros
echo 'OFFLINE ONLY: ULog43 CSV replay; do not run with serial demo (duplicate /odom).'
exec ros2 launch car_bringup offline_debug.launch.xml replay_interval_ms:=150 "$@"
