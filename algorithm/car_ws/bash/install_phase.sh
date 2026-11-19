#!/usr/bin/env bash
# Install a phase bundle from Downloads without maintaining scripts in Downloads.
# Existing installed packages are never overwritten by this helper.
# Usage: install_phase.sh phase1|phase2|serial [zip_path]
set -Eeuo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/lib/common.sh"
car_check_workspace
phase="${1:-}"
if (($#));then shift;fi
case "$phase" in
  phase1)
    zip_path="${1:-$HOME/Downloads/car_ros2_cpp_phase1.zip}"
    bundle='car_ros2_cpp_phase1/install.sh'
    packages=(car_interfaces car_stm32_bridge)
    ;;
  phase2)
    zip_path="${1:-$HOME/Downloads/car_ros2_cpp_phase2.zip}"
    bundle='car_cpp_phase2/install.sh'
    packages=(car_debug car_bringup)
    ;;
  serial)
    zip_path="${1:-$HOME/Downloads/car_ros2_car_link_v1.zip}"
    bundle='car_link_ros2_v1/install.sh'
    packages=(car_serial_link)
    ;;
  *)
    echo 'Usage: bash/install_phase.sh phase1|phase2|serial [zip_file]' >&2
    exit 2
    ;;
esac
installed=0
for pkg in "${packages[@]}";do
  if [[ -e "$CAR_WS/src/$pkg" ]];then
    echo "Refusing to overwrite existing source: $CAR_WS/src/$pkg"
    installed=1
  fi
done
if ((installed));then
  echo 'Already installed (partially or fully). No changes made. Use build.sh to rebuild.'
  exit 0
fi
if [[ ! -f "$zip_path" ]];then echo "ERROR: archive missing: $zip_path" >&2;exit 2;fi
tmp="$(mktemp -d -t car_phase_bundle_XXXXXXXX)"
trap 'rm -rf -- "$tmp"' EXIT
python3 -m zipfile -e "$zip_path" "$tmp"
if [[ ! -f "$tmp/$bundle" ]];then echo "ERROR: missing $bundle in $zip_path" >&2;exit 2;fi
bash "$tmp/$bundle" "$CAR_WS"
printf '\nFinished installing %s. Next: bash %s/bash/build.sh core\n' "$phase" "$CAR_WS"
