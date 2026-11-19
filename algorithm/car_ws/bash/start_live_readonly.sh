#!/usr/bin/env bash
# Future real STM32 UART4: receive only. No commands can be sent.
set -Eeuo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/lib/common.sh"
car_check_workspace
port="${1:-/dev/ttyUSB0}"
if (($#)); then shift; fi
if [[ ! -e "$port" ]]; then
  echo "ERROR: serial device not found: $port (you are not connected to the car yet)" >&2
  exit 2
fi
car_source_ros
echo "READ ONLY: car_serial_bridge on $port; STM32 CAR-LINK firmware must be implemented first."
exec ros2 launch car_serial_link live_readonly.launch.xml "port:=$port" "$@"
