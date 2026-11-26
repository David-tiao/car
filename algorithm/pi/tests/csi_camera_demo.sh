#!/usr/bin/env bash
# Independent CSI hardware test. No installation, ROS, STM32 or motor access.
set -euo pipefail
export LC_ALL=C
usage() {
    cat <<'HELP'
Usage: csi_camera_demo.sh list | capture [index] | preview [index [seconds]]
  list                  enumerate CSI cameras
  capture [index]       save one JPEG to a new temporary directory
  preview [index [sec]] show live camera preview (default 10s, 1..300s)
If exactly one camera is detected, index may be omitted.
Exit 0: command completed; 1: test failed; 2: bad arguments; 124: timeout.
Preview completion alone does not prove the screen displayed a picture.
HELP
}
fail() { echo "[FAIL] $*" >&2; exit 1; }
usage_error() { usage >&2; exit 2; }
mode=${1:-list}
case "$mode" in
    -h|--help) usage; exit 0 ;;
    list) (( $# <= 1 )) || usage_error ;;
    capture) (( $# <= 2 )) || usage_error ;;
    preview) (( $# <= 3 )) || usage_error ;;
    *) usage_error ;;
esac
index=${2:-}
seconds=${3:-10}
[[ -z "$index" || "$index" =~ ^(0|[1-9][0-9]{0,3})$ ]] || usage_error
[[ "$seconds" =~ ^[1-9][0-9]{0,2}$ ]] && (( seconds <= 300 )) || usage_error
for tool in timeout awk; do command -v "$tool" >/dev/null || fail "Missing $tool"; done
if command -v rpicam-hello >/dev/null; then
    hello=rpicam-hello; still=rpicam-still
elif command -v libcamera-hello >/dev/null; then
    hello=libcamera-hello; still=libcamera-still
else
    fail 'No rpicam-hello or libcamera-hello. No dependencies were installed.'
fi
run_bounded() {
    local limit=$1 rc=0; shift
    timeout --kill-after=3s "${limit}s" "$@" || rc=$?
    if (( rc != 0 )); then
        echo "[FAIL] $1 exit=$rc (124=timeout; 137=forced termination)" >&2
        exit "$rc"
    fi
}
# Capture both streams: older releases may print the list to stderr.
rc=0
listing=$(timeout --kill-after=3s 10s "$hello" --list-cameras 2>&1) || rc=$?
printf '%s\n' "$listing"
if (( rc != 0 )); then echo "[FAIL] Camera enumeration exit=$rc" >&2; exit "$rc"; fi
mapfile -t cameras < <(printf '%s\n' "$listing" | awk '/^[[:space:]]*[0-9]+[[:space:]]*:[[:space:]]*/ { print $1+0 }')
(( ${#cameras[@]} > 0 )) || fail 'No camera enumerated; check CSI cable, sensor and OS configuration.'
if [[ "$mode" == list ]]; then echo "[PASS] Enumerated ${#cameras[@]} camera(s); capture not tested."; exit 0; fi
if [[ -z "$index" ]]; then
    (( ${#cameras[@]} == 1 )) || fail 'Multiple cameras: specify index from list.'
    index=${cameras[0]}
fi
found=false
for candidate in "${cameras[@]}"; do [[ "$candidate" != "$index" ]] || found=true; done
[[ "$found" == true ]] || fail "Camera index $index is not enumerated."
echo "[INFO] Selected camera index=$index; stop other camera applications first."
if [[ "$mode" == preview ]]; then
    run_bounded "$((seconds + 15))" "$hello" --camera "$index" --timeout "$((seconds * 1000))"
    echo '[DONE] Preview command completed. Visually confirm live image on the local display.'
else
    command -v "$still" >/dev/null || fail "Missing $still"
    command -v python3 >/dev/null || fail 'Missing python3 (JPEG boundary check)'
    out=$(mktemp -d "${TMPDIR:-/tmp}/car-csi-demo.XXXXXXXX")
    echo "[INFO] Capture directory: $out"
    run_bounded 20 "$still" --camera "$index" --nopreview --timeout 1000 --width 640 --height 480 --encoding jpg --output "$out/frame.jpg"
    python3 - "$out/frame.jpg" <<'PY' || fail 'JPEG missing or incomplete'
import pathlib, sys
p = pathlib.Path(sys.argv[1])
data = p.read_bytes()
if len(data) <= 4 or not data.startswith(b'\xff\xd8') or not data.endswith(b'\xff\xd9'):
    sys.exit(1)
print(f'[PASS] JPEG boundary check: {len(data)} bytes; {p}')
PY
    echo '[DONE] Still capture completed; inspect the image to validate content.'
fi
