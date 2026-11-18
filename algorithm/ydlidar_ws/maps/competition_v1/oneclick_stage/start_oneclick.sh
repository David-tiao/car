#!/usr/bin/env bash
# Compatibility wrapper: existing desktop shortcut still works, but uses lifecycle manager.
set -Eeuo pipefail
DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd -- "$DIR/.." && pwd)"
if [[ "${AI_SUPERVISOR_CHILD:-}" != "1" ]]; then
    if [[ "${1:-}" == "--check" ]]; then
        exec bash "$ROOT/scripts/ai.sh" doctor --offline
    fi
    if [[ -t 0 ]] && [[ $# -eq 0 ]]; then
        exec bash "$DIR/desktop_start.sh"
    fi
    exec bash "$ROOT/scripts/ai.sh" start "$@"
fi
# Only an owned child of ai_manager may invoke the real pipeline.
# ROS setup.bash may access undefined AMENT_* values; disable nounset only while sourcing.
WS="$(cd -- "$DIR/../../../" && pwd)"
set +u
if [[ -r /opt/ros/humble/setup.bash ]]; then
    # shellcheck disable=SC1091
    source /opt/ros/humble/setup.bash
fi
# Source the current workspace overlay first. The old install_x3 overlay may
# contain symlinks into the previous /home/tao/Desktop/ydlidar_ws path.
if [[ -r "$WS/install/setup.bash" ]]; then
    # shellcheck disable=SC1091
    source "$WS/install/setup.bash"
fi
# The standardized launcher builds the driver into this relocatable overlay.
if [[ -r "$WS/install_oneclick_lidar/setup.bash" ]]; then
    # shellcheck disable=SC1091
    source "$WS/install_oneclick_lidar/setup.bash"
else
    LEGACY_NODE="$WS/install_x3/ydlidar_ros2_driver/lib/ydlidar_ros2_driver/ydlidar_ros2_driver_node"
    if [[ -r "$WS/install_x3/setup.bash" && -x "$(readlink -e "$LEGACY_NODE" 2>/dev/null || true)" ]]; then
        # shellcheck disable=SC1091
        source "$WS/install_x3/setup.bash"
    fi
fi
set -u
exec python3 "$DIR/main.py"
