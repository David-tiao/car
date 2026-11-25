#!/usr/bin/env bash
# Read-only host diagnosis; do not require root and never configure devices.
set -Eeuo pipefail

HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
ROOT="$(cd -- "$HERE/../../.." && pwd -P)"
echo '== Raspberry Pi / ROS 2 Humble deployment doctor (READ ONLY) =='
echo "Repo: $ROOT"
echo "Kernel: $(uname -sr)"
echo "Machine architecture: $(uname -m)"
if [[ -r /etc/os-release ]]; then
  # Only show distro name/version; never expose user configuration.
  . /etc/os-release
  echo "OS: ${PRETTY_NAME:-unknown}"
fi
if [[ -d "$ROOT/algorithm/car_ws/src/car_stm32_bridge" ]]; then
  echo '[OK] Existing C++ read-only STM32 ULog43 bridge found'
else
  echo '[FAIL] algorithm/car_ws/src/car_stm32_bridge missing' >&2
fi
if [[ "$(uname -m)" != "aarch64" ]]; then
  echo '[NOTE] Target is Raspberry Pi 5 / aarch64; this host is for software tests only'
fi
if command -v docker >/dev/null 2>&1; then
  echo "Docker CLI: $(docker --version)"
  if docker info --format '{{.ServerVersion}}' 2>/dev/null; then
    echo '[OK] Docker daemon reachable by current user'
  else
    echo '[WARN] Docker daemon unavailable or current user lacks Docker access'
    echo '       Do not run internet install scripts blindly; follow distro Docker instructions'
  fi
else
  echo '[WARN] Docker not installed'
fi
if [[ -d /dev/serial/by-id ]]; then
  echo 'Serial aliases (inspect before choosing the STM32 telemetry port):'
  ls -l /dev/serial/by-id || true
else
  echo 'No /dev/serial/by-id directory (USB-UART may be disconnected)'
fi
for glob in /dev/ttyUSB* /dev/ttyACM*; do
  if [[ -c "$glob" ]]; then
    ls -l "$glob"
  fi
done
echo 'No serial device was opened. No motor/GPIO/ROS command was sent.'
