#!/usr/bin/env bash
# Bootstraps into car_ws/bash/; no deletes or Git operations.
set -Eeuo pipefail
HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
ROOT="$(cd -- "$HERE/.." && pwd -P)"
WS="${1:-${CAR_WS:-$ROOT}}"
[[ -d "$WS/src/car_interfaces/msg" && -f "$WS/src/car_interfaces/CMakeLists.txt" ]] || { echo "ERROR: car_interfaces not found in $WS" >&2; exit 2; }
[[ -d "$WS/src/car_serial_link" ]] || { echo 'ERROR: CAR-LINK phase 3 must already be installed.' >&2; exit 2; }
[[ -d "$WS/bash" ]] || mkdir -p -- "$WS/bash"
for name in car_vision car_mechanism car_mission;do
  if [[ -e "$WS/src/$name" ]] && ! diff -qr "$ROOT/src/$name" "$WS/src/$name" >/dev/null;then
    echo "ERROR: $WS/src/$name already exists and differs. Not overwriting." >&2;exit 3
  fi
done
for msg in VisionStatus MechanismHealth MissionSummary;do
  if [[ -e "$WS/src/car_interfaces/msg/$msg.msg" ]] && ! cmp -s "$ROOT/messages/$msg.msg" "$WS/src/car_interfaces/msg/$msg.msg";then
    echo "ERROR: Existing $msg.msg differs. Not overwriting." >&2;exit 3
  fi
done
for script in "$ROOT"/bash/*.sh;do
  name="${script##*/}"
  if [[ -e "$WS/bash/$name" ]] && ! cmp -s "$script" "$WS/bash/$name";then
    echo "ERROR: Existing bash/$name differs. Not overwriting." >&2;exit 3
  fi
done
python3 - "$WS/src/car_interfaces/CMakeLists.txt" <<'PY'
import pathlib,sys
p=pathlib.Path(sys.argv[1]);s=p.read_text()
anchor='"msg/MechanismState.msg"'
if anchor not in s:
    raise SystemExit('ERROR: cannot find ROSIDL anchor in car_interfaces/CMakeLists.txt; no files copied')
for x in ('VisionStatus','MechanismHealth','MissionSummary'):
    if f'"msg/{x}.msg"' in s:continue
    s=s.replace(anchor,anchor+f'\n  "msg/{x}.msg"',1)
p.write_text(s)
print('Registered phase-4 message definitions')
PY
for name in car_vision car_mechanism car_mission;do
  if [[ ! -e "$WS/src/$name" ]];then cp -a "$ROOT/src/$name" "$WS/src/$name";fi
done
for msg in VisionStatus MechanismHealth MissionSummary;do
  if [[ ! -e "$WS/src/car_interfaces/msg/$msg.msg" ]];then cp "$ROOT/messages/$msg.msg" "$WS/src/car_interfaces/msg/$msg.msg";fi
done
for script in "$ROOT"/bash/*.sh;do
  name="${script##*/}"
  [[ -e "$WS/bash/$name" ]] || cp "$script" "$WS/bash/$name"
done
chmod +x "$WS/bash/"*phase4*.sh
printf 'PASS: phase4 installed in %s (only new C++ packages/messages and bash scripts)\n' "$WS"
printf 'Next: bash %s/bash/build_phase4.sh\n' "$WS"
