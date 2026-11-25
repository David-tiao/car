#!/usr/bin/env bash
# Explicit Docker launcher for the EXISTING C++ ROS2/STM32 bridge.
# No automatic service, motor control, serial writes, privileged containers,
# host package modifications, or workspace edits.
set -Eeuo pipefail

HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
REPO_ROOT="$(cd -- "$HERE/../../.." && pwd -P)"
ROS_SRC="$REPO_ROOT/algorithm/car_ws/src"
IMAGE="${PI_ROS_IMAGE:-car-pi5-ros2-humble:local}"
VOLUME="${PI_ROS_VOLUME:-car_pi5_ros2_humble_out}"
DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-1}"

usage() {
  cat <<'EOF'
Usage:
  bash algorithm/pi/ros2_humble/run.sh doctor
  bash algorithm/pi/ros2_humble/run.sh image
  bash algorithm/pi/ros2_humble/run.sh build
  bash algorithm/pi/ros2_humble/run.sh test
  bash algorithm/pi/ros2_humble/run.sh replay
  bash algorithm/pi/ros2_humble/run.sh monitor /dev/serial/by-id/<USB-UART>
  bash algorithm/pi/ros2_humble/run.sh echo-odom
  bash algorithm/pi/ros2_humble/run.sh topics
  bash algorithm/pi/ros2_humble/run.sh shell

Commands:
  doctor     Read-only host OS, device, Docker and workspace checks.
  image      Build an Ubuntu Jammy ROS2 Humble image; no hardware access.
  build      Build existing car_interfaces, car_stm32_bridge, car_debug,
             car_bringup into persistent Docker volume (no source writes).
  test       Run colcon tests for those packages, must build first.
  replay     Start built-in synthetic ULog43 replay and diagnostic node;
             offline only, no serial device, Ctrl-C to stop.
  monitor    OPEN READ ONLY: map precisely one USB-UART receiver into Docker.
  echo-odom  Inspect a single /odom message (run monitor or replay elsewhere).
  topics     List ROS2 topics in local domain.
  shell      Start a diagnostic shell with the ROS2 overlay sourced.

Docker privileges:
  --network host and --ipc host enable same-host ROS2 discovery.
  Never uses --privileged. monitor maps only the selected character
  device with Docker ':r' permission; ROS bridge opens it O_RDONLY.
  Source tree is mounted read-only; build/install use a named volume.

Defaults: ROS_DOMAIN_ID=42 ROS_LOCALHOST_ONLY=1
WARNING: Docker daemon/group access is root-equivalent. Use only trusted users.
EOF
}

command="${1:-}"
if [[ "$command" == "doctor" ]]; then
  exec bash "$HERE/doctor.sh"
fi
if [[ "$command" == "-h" || "$command" == "--help" || -z "$command" ]]; then
  usage
  [[ -n "$command" ]] && exit 0
  exit 2
fi

[[ -d "$ROS_SRC/car_stm32_bridge" ]] || {
  echo "ERROR: ROS2 packages missing: $ROS_SRC" >&2
  exit 2
}
command -v docker >/dev/null 2>&1 || {
  echo 'ERROR: Docker CLI missing; see README.md for manual installation' >&2
  exit 2
}
[[ "$DOMAIN_ID" =~ ^[0-9]+$ ]] && (( 10#$DOMAIN_ID <= 232 )) || {
  echo 'ERROR: ROS_DOMAIN_ID must be an integer 0..232' >&2; exit 2;
}
[[ "$LOCALHOST_ONLY" == 0 || "$LOCALHOST_ONLY" == 1 ]] || {
  echo 'ERROR: ROS_LOCALHOST_ONLY must be 0 or 1' >&2; exit 2;
}
[[ "$VOLUME" =~ ^[a-zA-Z0-9][a-zA-Z0-9_.-]*$ ]] || {
  echo 'ERROR: PI_ROS_VOLUME has unsupported characters' >&2; exit 2;
}

if [[ "$command" == "image" ]]; then
  [[ "$#" == 1 ]] || { usage >&2; exit 2; }
  exec docker build --tag "$IMAGE" "$HERE"
fi

docker image inspect "$IMAGE" >/dev/null 2>&1 || {
  echo "ERROR: image not built. First: bash $HERE/run.sh image" >&2
  exit 2
}

# Use one named persistent Docker volume for build artifacts; never
# overwrite ./algorithm/car_ws (which is tracked in the main repository).
run_args=(
  run --rm
  --network host --ipc host
  --env "ROS_DOMAIN_ID=$DOMAIN_ID"
  --env "ROS_LOCALHOST_ONLY=$LOCALHOST_ONLY"
  --mount "type=bind,source=$ROS_SRC,target=/workspace/src,readonly"
  --mount "type=volume,source=$VOLUME,target=/workspace_state"
  --workdir /workspace
)

case "$command" in
  build)
    [[ "$#" == 1 ]] || { usage >&2; exit 2; }
    exec docker "${run_args[@]}" "$IMAGE" bash -lc '
      set -e
      source /opt/ros/humble/setup.bash
      colcon build --base-paths /workspace/src \
        --build-base /workspace_state/build \
        --install-base /workspace_state/install \
        --log-base /workspace_state/log \
        --packages-up-to car_bringup \
        --cmake-args -DCMAKE_BUILD_TYPE=Release
    '
    ;;
  test)
    [[ "$#" == 1 ]] || { usage >&2; exit 2; }
    exec docker "${run_args[@]}" "$IMAGE" bash -lc '
      set -e
      source /opt/ros/humble/setup.bash
      test -f /workspace_state/install/setup.bash || {
        echo "Build first: run.sh build" >&2; exit 2;
      }
      source /workspace_state/install/setup.bash
      colcon test --base-paths /workspace/src \
        --build-base /workspace_state/build \
        --test-result-base /workspace_state/build \
        --packages-select car_interfaces car_stm32_bridge car_debug car_bringup \
        --return-code-on-test-failure
      colcon test-result --test-result-base /workspace_state/build --verbose
    '
    ;;
  replay)
    [[ "$#" == 1 ]] || { usage >&2; exit 2; }
    echo 'OFFLINE SYNTHETIC ULog43 REPLAY. No real serial device is available.'
    exec docker "${run_args[@]}" "$IMAGE" bash -lc '
      set -e
      source /opt/ros/humble/setup.bash
      source /workspace_state/install/setup.bash
      exec ros2 launch car_bringup offline_debug.launch.xml with_rviz:=false
    '
    ;;
  monitor)
    [[ "$#" == 2 ]] || { usage >&2; exit 2; }
    device="$2"
    [[ -c "$device" ]] || {
      echo "ERROR: $device is not a character device (USB-UART not attached?)" >&2
      exit 2
    }
    # Resolve stable udev symlinks; do not inadvertently mount a directory.
    device="$(readlink -f -- "$device")"
    [[ -c "$device" ]] || { echo 'ERROR: invalid serial target' >&2; exit 2; }
    echo "MONITOR ONLY / NO TX: $device → /dev/car_ulog"
    echo 'Verify this adapter RX is wired to STM32 USART2 PD5 TX, common ground.'
    run_args+=( --device "$device:/dev/car_ulog:r" )
    exec docker "${run_args[@]}" "$IMAGE" bash -lc '
      set -e
      source /opt/ros/humble/setup.bash
      source /workspace_state/install/setup.bash
      exec ros2 launch car_stm32_bridge monitor.launch.xml port:=/dev/car_ulog
    '
    ;;
  echo-odom)
    [[ "$#" == 1 ]] || { usage >&2; exit 2; }
    exec docker "${run_args[@]}" "$IMAGE" bash -lc '
      set -e
      source /opt/ros/humble/setup.bash
      source /workspace_state/install/setup.bash
      exec timeout 12 ros2 topic echo /odom nav_msgs/msg/Odometry --once
    '
    ;;
  topics)
    [[ "$#" == 1 ]] || { usage >&2; exit 2; }
    exec docker "${run_args[@]}" "$IMAGE" bash -lc '
      set -e
      source /opt/ros/humble/setup.bash
      source /workspace_state/install/setup.bash
      exec ros2 topic list -t
    '
    ;;
  shell)
    [[ "$#" == 1 ]] || { usage >&2; exit 2; }
    # Do not automatically enable real-device access in the shell.
    exec docker "${run_args[@]}" -it "$IMAGE" bash -lc '
      source /opt/ros/humble/setup.bash
      if [[ -f /workspace_state/install/setup.bash ]]; then
        source /workspace_state/install/setup.bash
      fi
      exec bash --noprofile --norc -i
    '
    ;;
  *)
    usage >&2
    exit 2
    ;;
esac
