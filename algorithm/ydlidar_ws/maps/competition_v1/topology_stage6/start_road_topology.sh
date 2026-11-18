#!/usr/bin/env bash
set -Eeo pipefail
source /opt/ros/humble/setup.bash
STAGE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
exec python3 "$STAGE_DIR/road_topology.py" "$@"
