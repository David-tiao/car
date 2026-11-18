#!/usr/bin/env bash
set -Eeuo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUT="$(mktemp -d)"
trap 'rm -rf "$OUT"' EXIT
c++ -std=c++17 -Wall -Wextra -Werror -I"$ROOT/include" \
  "$ROOT/src/ulog43_parser.cpp" "$ROOT/tests/test_parser.cpp" -o "$OUT/test_parser"
"$OUT/test_parser"
