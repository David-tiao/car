#!/usr/bin/env bash
# Phase 5 read-only navigation preview: real existing competition maps + SYNTHETIC scan.
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
# Do not overwrite /map from the existing V6 viewer or other map servers.
existing="$(ros2 topic info /map 2>/dev/null | sed -n 's/^Publisher count: //p' | head -1 || true)"
if [[ "$existing" =~ ^[1-9][0-9]*$ ]];then
 echo "ERROR: /map already has $existing publisher(s). Stop V6 viewer/other map server first." >&2;exit 3
fi
echo '[INFO] Phase 5 map + mask + synthetic /scan + A* visual preview (NO motor commands).'
echo "[INFO] map=$MAP"
echo "[INFO] mask=$MASK"
exec ros2 launch car_navigation offline_preview.launch.xml \
  map_yaml:="$MAP" mask_yaml:="$MASK" clearance_m:="${CAR_PHASE5_CLEARANCE_M:-0.17}" "$@"
