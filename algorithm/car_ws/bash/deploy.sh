#!/usr/bin/env bash
# Native deployment profiles; no installations, hardware starts or automatic cleanup.
set -Eeuo pipefail
source "$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)/lib/common.sh"
mode="${1:-plan}"
arch="$(uname -m)"
case "$arch" in
  x86_64) default_profile=pc; elf_machine=62 ;;
  aarch64) default_profile=pi5; elf_machine=183 ;;
  *) echo "ERROR: unsupported native architecture: $arch (64-bit Linux required)" >&2; exit 2 ;;
esac
profile="${2:-$default_profile}"
[[ $# -le 2 ]] || { echo "Usage: deploy.sh plan|paths|doctor|build|verify|test [pc|pi5|sim]" >&2; exit 2; }
case "$profile" in
  pc|pi5)
    packages=(car_interfaces car_stm32_bridge car_debug car_bringup car_serial_link
      car_vision car_mechanism car_mission car_navigation car_web_sim car_web_real car_web_test)
    ;;
  sim) packages=(car_navigation car_gazebo) ;;
  *) echo "ERROR: unknown profile: $profile" >&2; exit 2 ;;
esac
case "$mode" in
  plan|paths|doctor|build|verify|test) ;;
  *) echo "Usage: deploy.sh plan|paths|doctor|build|verify|test [pc|pi5|sim]" >&2; exit 2 ;;
esac
if [[ ( "$profile" == pi5 && "$arch" != aarch64 ) || ( "$profile" == pc && "$arch" != x86_64 ) ]]; then
  if [[ "$mode" != plan && "$mode" != paths ]]; then
    echo "ERROR: profile $profile does not match native architecture $arch; use target machine" >&2
    exit 2
  fi
fi
car_check_workspace
paths=()
for pkg in "${packages[@]}"; do
  path="$CAR_WS/src/$pkg"
  [[ -r "$path/package.xml" && -r "$path/CMakeLists.txt" && ! -e "$path/COLCON_IGNORE" ]] ||
    { echo "ERROR: missing or ignored package: $path" >&2; exit 2; }
  paths+=("$path")
done
if [[ "$mode" == paths ]]; then printf '%s\n' "${paths[@]}"; exit 0; fi
printf 'PROFILE=%s ARCH=%s WORKSPACE=%s\n' "$profile" "$arch" "$CAR_WS"
printf 'PACKAGES: %s\n' "${packages[*]}"
printf 'ROS SETUP: %s\n' "$CAR_ROS_SETUP"
echo "No hardware startup. Pi5/PC core profiles exclude Gazebo, RViz2 and the separately built YDLIDAR driver."
if [[ "$mode" == plan ]]; then exit 0; fi
car_source_ros_base
for tool in ros2 colcon cmake gcc g++ /usr/bin/python3; do
  command -v "$tool" >/dev/null || { echo "ERROR: missing tool: $tool" >&2; exit 2; }
done
/usr/bin/python3 -c 'import rclpy; print("ROS system Python ready")'
if [[ "$profile" == sim ]]; then
  backend="$(/usr/bin/python3 "$CAR_WS/src/car_gazebo/scripts/gazebo_backend.py" --backend "${CAR_GAZEBO_BACKEND:-auto}")"
  echo "SIM backend: $backend (matching bridge ABI)"
fi
# Check declared ROS dependencies, not all packages in the source tree.
# External non-ROS dependencies still belong to rosdep / platform provisioning.
dep_text="$(/usr/bin/python3 - "${paths[@]}" <<'PY'
import sys
import pathlib
import xml.etree.ElementTree as ET
roots = [ET.parse(pathlib.Path(p) / "package.xml").getroot() for p in sys.argv[1:]]
local = {r.findtext("name") for r in roots}
tags = {"depend", "buildtool_depend", "build_depend", "build_export_depend", "exec_depend"}
deps = {e.text.strip() for r in roots for e in r if e.tag in tags and e.text}
print("\n".join(sorted(deps - local)))
PY
)"
mapfile -t deps <<< "$dep_text"
for dep in "${deps[@]}"; do
  [[ -n "$dep" ]] || continue
  ros2 pkg prefix "$dep" >/dev/null || { echo "ERROR: missing declared ROS package: $dep" >&2; exit 2; }
done
echo 'PASS ENV: tools and declared ROS packages found; no physical device checked.'
if [[ "$mode" == doctor ]]; then exit 0; fi
cd "$CAR_WS"
# Reject old CMake caches from a moved workspace; do not delete or rewrite them.
# A symlink to the SAME physical workspace is fine.
if [[ "$mode" == build || "$mode" == test ]]; then
  /usr/bin/python3 - "$CAR_WS" "$elf_machine" "${packages[@]}" <<'PY'
from pathlib import Path
import sys
ws = Path(sys.argv[1]).resolve()
machine = int(sys.argv[2])
for pkg in sys.argv[3:]:
    # CMake compiler ABI files survive relocation/copy and reveal a stale CPU target.
    for abi in (ws / "build" / pkg / "CMakeFiles").glob("*/CMakeDetermineCompilerABI_*.bin"):
        with abi.open("rb") as stream:
            header = stream.read(20)
        if len(header) == 20 and header[:4] == b"\x7fELF":
            order = {1: "little", 2: "big"}.get(header[5])
            if order is None or header[4] != 2 or int.from_bytes(header[18:20], order) != machine:
                raise SystemExit(f"ERROR: stale compiler architecture in {abi}; preserve old artifacts and rebuild in a clean workspace")
    cache = ws / "build" / pkg / "CMakeCache.txt"
    if not cache.is_file():
        continue
    for line in cache.read_text(errors="replace").splitlines():
        if line.startswith("CMAKE_HOME_DIRECTORY:INTERNAL="):
            value = Path(line.split("=", 1)[1])
            if value.resolve() != (ws / "src" / pkg).resolve():
                raise SystemExit(f"ERROR: stale CMake source path in {cache}; preserve old artifacts and rebuild in a clean workspace")
        if line.startswith("CMAKE_CACHEFILE_DIR:INTERNAL="):
            value = Path(line.split("=", 1)[1])
            if value.resolve() != cache.parent.resolve():
                raise SystemExit(f"ERROR: stale CMake build path in {cache}; preserve old artifacts and rebuild in a clean workspace")
PY
fi
if [[ "$mode" == build ]]; then
  colcon build --base-paths "${paths[@]}" --packages-select "${packages[@]}" --event-handlers console_direct+
  echo 'PASS BUILD: selected packages built; run deploy.sh verify next.'
  exit 0
fi
car_source_setup "$CAR_WS/install/setup.bash"
for pkg in "${packages[@]}"; do
  prefix="$(ros2 pkg prefix "$pkg")"
  [[ -d "$CAR_WS/install/$pkg" && "$(cd -- "$prefix" && pwd -P)" == "$(cd -- "$CAR_WS/install/$pkg" && pwd -P)" ]] ||
    { echo "ERROR: $pkg resolves outside this workspace's isolated install: $prefix" >&2; exit 2; }
done
# Read ELF headers, not execute serial or navigation nodes.
binaries=(car_navigation/car_map_probe car_navigation/car_navigation_monitor)
if [[ "$profile" != sim ]]; then
  binaries+=(car_stm32_bridge/car_ulog43_bridge car_serial_link/car_serial_bridge
    car_mission/car_mission_monitor car_vision/car_vision_monitor car_mechanism/car_mechanism_monitor)
  ros2 interface show car_interfaces/msg/ChassisTelemetry >/dev/null
fi
/usr/bin/python3 - "$CAR_WS/install" "$elf_machine" "${binaries[@]}" <<'PY'
import os
from pathlib import Path
import sys
root, machine = Path(sys.argv[1]), int(sys.argv[2])
for item in sys.argv[3:]:
    pkg, name = item.split("/")
    path = root / pkg / "lib" / pkg / name
    if not os.access(path, os.X_OK):
        raise SystemExit(f"ERROR: missing executable {path}")
    with path.open("rb") as stream:
        header = stream.read(20)
    if (len(header) != 20 or header[:4] != b"\x7fELF" or header[4] != 2
            or header[5] not in (1, 2)
            or int.from_bytes(header[18:20], "little" if header[5] == 1 else "big") != machine):
        raise SystemExit(f"ERROR: executable is not native 64-bit ELF: {path}")
    print(f"PASS ELF: {item}")
PY
echo 'PASS VERIFY: local package prefixes and native executable headers.'
if [[ "$mode" == verify ]]; then exit 0; fi
# Software-only tests; no real serial, laser, TF publication or motor node.
colcon test --base-paths "${paths[@]}" --packages-select "${packages[@]}" --event-handlers console_direct+
colcon test-result --test-result-base "$CAR_WS/build" --verbose
map="$CAR_WS/maps/competition_v1"
"$CAR_WS/install/car_navigation/lib/car_navigation/car_map_probe" \
  "$map/competition_navigation_map.yaml" "$map/competition_keepout_mask.yaml" 0.17
echo 'PASS TEST: native package tests and static map probe; hardware NOT_RUN.'
