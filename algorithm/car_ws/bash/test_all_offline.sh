#!/usr/bin/env bash
# One-command, zero-hardware, all-stage offline software acceptance.
set -Eeuo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../../.." && pwd -P)"
export PYTHONDONTWRITEBYTECODE=1
python3 "$ROOT/algorithm/car_ws/maps/competition_v1/lidar_stage5/scan_regions.py" --self-test
exec python3 "$ROOT/sim2real/labs/offline_acceptance_suite.py" "$@"
