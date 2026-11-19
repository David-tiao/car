#!/usr/bin/env bash
# Pure g++ test, works without ROS2.
set -Eeuo pipefail
WS="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
TMP="$(mktemp -d -t car_phase4_gpp_XXXXXXXX)"
trap 'rm -rf -- "$TMP"' EXIT
g++ -std=c++17 -Wall -Wextra -Werror -I"$WS/src/car_mission/include" \
  "$WS/src/car_mission/tests/test_core.cpp" -o "$TMP/test_core"
"$TMP/test_core"
