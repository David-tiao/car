#!/usr/bin/env bash
# Phase 5 map + real-scan diagnostics; start radar separately: bash car_ws/bash/lidar.sh start
# NEVER run at the same time as the existing V6 viewer: /map publisher conflict.
set -Eeuo pipefail
WS="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
MAP_DIR="${CAR_PHASE5_MAP_DIR:-$WS/maps/competition_v1}"
MAP="$MAP_DIR/competition_navigation_map.yaml"
MASK="$MAP_DIR/competition_keepout_mask.yaml"
[[ -f "$MAP" && -f "$MASK" ]] || { echo "ERROR: missing map/mask in $MAP_DIR" >&2;exit 2; }
set +u
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"
set -u
existing="$(ros2 topic info /map 2>/dev/null | sed -n 's/^Publisher count: //p' | head -1 || true)"
if [[ "$existing" =~ ^[1-9][0-9]*$ ]];then
 echo "ERROR: /map already has $existing publisher(s). Stop existing viewer/map server." >&2;exit 3
fi
echo '[INFO] Read-only navigation monitor. Does NOT start lidar; run car_ws/bash/lidar.sh in another terminal.'
echo '[INFO] No mock scan, no TF assumptions, no motor commands.'
exec ros2 launch car_navigation live_readonly.launch.xml \
  map_yaml:="$MAP" mask_yaml:="$MASK" clearance_m:="${CAR_PHASE5_CLEARANCE_M:-0.17}" "$@"
