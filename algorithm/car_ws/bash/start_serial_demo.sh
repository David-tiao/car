#!/usr/bin/env bash
# Virtual STM32 -> PTY binary stream -> ROS2; no hardware.
set -Eeuo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/lib/common.sh"
car_check_workspace
car_source_ros
echo 'OFFLINE ONLY: virtual STM32, CRC16 + ROS2 bridge + monitor; no motor commands.'
exec ros2 launch car_serial_link offline_serial_demo.launch.xml with_debug:=true "$@"
