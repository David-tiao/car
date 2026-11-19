#!/usr/bin/env bash
# Pure C++17 test. Does not require ROS2, hardware, or running nodes.
set -Eeuo pipefail
HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
WS="$(cd -- "$HERE/.." && pwd -P)"
PKG="$WS/src/car_navigation"
[[ -f "$PKG/tests/test_navigation_core.cpp" ]] || { echo "ERROR: missing $PKG" >&2;exit 2; }
TMP="$(mktemp -d)"
trap 'rm -rf "$TMP"' EXIT
g++ -std=c++17 -O2 -Wall -Wextra -pedantic -I"$PKG/include" \
  "$PKG/src/navigation_core.cpp" "$PKG/tests/test_navigation_core.cpp" -o "$TMP/navigation_test"
"$TMP/navigation_test"
