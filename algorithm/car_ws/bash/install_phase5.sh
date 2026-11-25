#!/usr/bin/env bash
# One-time installer: new package and scripts only. NEVER overwrite existing edits.
set -Eeuo pipefail
HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
ROOT="$(cd -- "$HERE/.." && pwd -P)"
WS="${1:-${CAR_WS:-$ROOT}}"
[[ -d "$WS/src" && -d "$WS/maps/competition_v1" ]] || {
 echo "ERROR: wrong workspace / missing competition map: $WS" >&2;exit 2;
}
[[ -d "$ROOT/src/car_navigation" ]] || { echo 'ERROR: missing package in ZIP' >&2;exit 2; }
mkdir -p "$WS/bash"
if [[ -e "$WS/src/car_navigation" ]] && ! diff -qr "$ROOT/src/car_navigation" "$WS/src/car_navigation" >/dev/null;then
 echo 'ERROR: car_navigation already exists and differs; not overwriting' >&2;exit 3;
fi
for script in "$ROOT"/bash/*.sh;do
 name="${script##*/}"
 if [[ -e "$WS/bash/$name" ]] && ! cmp -s "$script" "$WS/bash/$name";then
   echo "ERROR: existing bash/$name differs; not overwriting" >&2;exit 3;
 fi
done
if [[ ! -e "$WS/src/car_navigation" ]];then cp -a "$ROOT/src/car_navigation" "$WS/src/car_navigation";fi
for script in "$ROOT"/bash/*.sh;do
 name="${script##*/}"
 [[ -e "$WS/bash/$name" ]] || cp "$script" "$WS/bash/$name"
done
chmod +x "$WS/bash/"*phase5*.sh
printf 'PASS installed car_navigation + bash/*phase5* into %s\n' "$WS"
printf 'Next: bash %s/bash/build_phase5.sh\n' "$WS"
