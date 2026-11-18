#!/usr/bin/env bash
set -Eeuo pipefail
HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
TMP="$(mktemp -d)"
trap 'rm -rf "$TMP"' EXIT
c++ -std=c++17 -Wall -Wextra -Wpedantic -Werror \
  -I"$HERE/../include" "$HERE/test_health_core.cpp" -o "$TMP/test_health_core"
"$TMP/test_health_core"
echo 'PASS: core offline watchdog tests'
