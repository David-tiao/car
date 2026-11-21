#!/usr/bin/env bash
# ONE PC, NO Pi/STM32 REQUIRED. Runs real ROS2 pub+sub for /keyboard local loopback.
# Prerequisites: website already on localhost; ROS2 Humble installed/sourced.
set -Eeuo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../../.." && pwd -P)"
SERVER="${1:-http://127.0.0.1:8090}"
# Run this BEFORE starting either ROS2 process: a refused HTTP connection is
# a missing website, not a ROS2/DDS/STM32 failure.  No motor topic is opened.
if ! /usr/bin/python3 - "$SERVER" <<'PY'
import json
import sys
from urllib.parse import urlsplit
from urllib.request import urlopen

url = sys.argv[1].rstrip("/")
parsed = urlsplit(url)
if parsed.scheme != "http" or parsed.hostname not in ("127.0.0.1", "localhost", "::1"):
    print("[ERROR] Keyboard website must be local HTTP.", file=sys.stderr)
    raise SystemExit(2)
try:
    with urlopen(url + "/api/keyboard/status", timeout=1.5) as response:
        status = json.loads(response.read(8192))
    if (status.get("schema") != "car_keyboard_mapping_v1" or
        status.get("hardware_authorized") is not False or
        status.get("stm32_tx_enabled") is not False):
        raise ValueError("Unexpected website endpoint or unsafe status")
except (OSError, ValueError, json.JSONDecodeError) as exc:
    print("[ERROR] Web keyboard endpoint unavailable: " + str(exc), file=sys.stderr)
    raise SystemExit(2)
print("[KEYBOARD] Web API preflight OK:", url + "/api/keyboard/status")
PY
then
  echo "[KEYBOARD] ROS2 nodes were NOT started. Start the website in another terminal:" >&2
  echo "  cd $ROOT && python3 sim2real/server.py --host 127.0.0.1 --port 8090" >&2
  echo "[KEYBOARD] Then retry: bash algorithm/car_ws/bash/start_keyboard_local_ros2.sh $SERVER" >&2
  exit 2
fi
set +u
source /opt/ros/humble/setup.bash
if [[ -f "$ROOT/algorithm/car_ws/install/setup.bash" ]]; then
  source "$ROOT/algorithm/car_ws/install/setup.bash"
fi
set -u
pc=""
feedback=""
cleanup(){
  if [[ -n "$pc" ]];then kill "$pc" 2>/dev/null || true;fi
  if [[ -n "$feedback" ]];then kill "$feedback" 2>/dev/null || true;fi
  wait 2>/dev/null || true
}
trap cleanup EXIT INT TERM
/usr/bin/python3 "$ROOT/sim2real/keyboard/keyboard_ros2_pc.py" --server "$SERVER" &
pc=$!
/usr/bin/python3 "$ROOT/sim2real/keyboard/keyboard_ros2_web_feedback.py" --server "$SERVER" &
feedback=$!
echo "[KEYBOARD] LOCAL ROS2 pub+sub running (no Pi needed). Open $SERVER/keyboard"
echo "[KEYBOARD] Default mode is local ROS2; STM32 UART TX remains disabled."
wait -n "$pc" "$feedback"
