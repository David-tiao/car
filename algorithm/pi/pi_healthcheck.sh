#!/usr/bin/env bash
# Pi5 read-only inventory. No sudo, no package install, no network changes,
# no device access, no ROS node launches, no motor or actuator commands.
set -uo pipefail

heading() { printf '\n== %s ==\n' "$1"; }
available() { command -v "$1" >/dev/null 2>&1; }
show_file() { if [[ -r "$1" ]]; then cat "$1"; else printf 'not readable: %s\n' "$1"; fi; }
safe() { "$@" 2>&1 || printf '[unavailable] %s\n' "$*"; }

heading "System"
safe uname -a
show_file /etc/os-release
if [[ -r /proc/device-tree/model ]]; then
  printf 'Device model: '; tr -d '\000' < /proc/device-tree/model; printf '\n'
fi
printf 'Architecture: '; safe uname -m

heading "Resources"
safe free -h
safe df -h
if available vcgencmd; then safe vcgencmd measure_temp; fi

heading "Networking (read only)"
if available ip; then safe ip -brief address; fi
if available nmcli; then safe nmcli device status; fi
if available systemctl; then
  printf 'SSH service: '; systemctl is-active ssh 2>/dev/null || printf 'not running or not installed\n'
fi

heading "Software presence"
for binary in ssh scp docker ros2 colcon python3 rpicam-hello rpicam-still; do
  if available "$binary"; then
    printf '%-16s %s\n' "$binary" "$(command -v "$binary")"
  else
    printf '%-16s %s\n' "$binary" 'not in PATH'
  fi
done
if [[ -f /opt/ros/humble/setup.bash ]]; then
  printf 'Native ROS2 Humble setup: present\n'
else
  printf 'Native ROS2 Humble setup: absent (a container is not checked here)\n'
fi

heading "Camera and UART device names (listing only)"
shopt -s nullglob
devices=(/dev/video* /dev/ttyUSB* /dev/ttyACM* /dev/serial/by-id/*)
if (("${#devices[@]}" == 0)); then
  printf 'No matching video/serial devices found\n'
else
  for device in "${devices[@]}"; do
    if [[ -L "$device" ]]; then
      printf '%s -> %s\n' "$device" "$(readlink "$device")"
    else
      printf '%s\n' "$device"
    fi
  done
fi
printf '\nThis is inventory only: it does not validate ROS2, camera capture, radar scans, motor safety, or STM32 connectivity.\n'
