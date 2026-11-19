#!/usr/bin/env bash
# List existing shell scripts. Old scripts in maps/ may depend on relative paths.
set -Eeuo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/lib/common.sh"
car_check_workspace
printf '=== Central scripts: %s/bash ===\n' "$CAR_WS"
find "$CAR_WS/bash" -type f -name '*.sh' | sort
printf '\n=== Legacy shell scripts outside bash/ (not moved automatically) ===\n'
find "$CAR_WS" \( -name build -o -name install -o -name log -o -name .git \) -prune -o \
   -type f -name '*.sh' ! -path "$CAR_WS/bash/*" -print | sort
printf '\nCAUTION: do not move existing map stage scripts without fixing their relative paths.\n'
