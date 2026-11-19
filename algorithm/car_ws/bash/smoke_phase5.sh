#!/usr/bin/env bash
# Automated read-only Phase5 smoke: no hardware, no STM32. Closes only its own launch.
set -Eeuo pipefail
WS="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
[[ -f "$WS/install/setup.bash" ]] || { echo 'ERROR: build first' >&2;exit 2; }
set +u
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"
set -u
LOGDIR="$(mktemp -d)"
PID=''
cleanup(){
 if [[ -n "$PID" ]];then kill -INT "$PID" >/dev/null 2>&1 || true;wait "$PID" >/dev/null 2>&1 || true;fi
 echo "[INFO] Smoke log: $LOGDIR/launch.log"
}
trap cleanup EXIT
"$WS/bash/start_phase5_offline.sh" >"$LOGDIR/launch.log" 2>&1 &
PID=$!
for _ in $(seq 1 30);do
 if ! kill -0 "$PID" 2>/dev/null;then echo 'FAIL launch stopped early';cat "$LOGDIR/launch.log";exit 1;fi
 if timeout 3 ros2 topic echo /map nav_msgs/msg/OccupancyGrid --once >/dev/null 2>&1;then break;fi
 sleep 0.25
done
bash "$WS/bash/check_phase5.sh"
bash "$WS/bash/phase5_demo_goal.sh"
printf 'Checking preview status ... '
STATUS="$(timeout 12 ros2 topic echo /car/navigation/plan_status std_msgs/msg/String --once)"
if grep -q 'OK:' <<< "$STATUS";then echo PASS
else echo FAIL;echo "$STATUS";cat "$LOGDIR/launch.log";exit 1;fi
printf 'Checking preview path ... '
PATH_OUTPUT="$(timeout 12 ros2 topic echo /car/navigation/preview_path nav_msgs/msg/Path --once)" || {
 echo FAIL;cat "$LOGDIR/launch.log";exit 1;
}
if grep -q 'frame_id: map' <<< "$PATH_OUTPUT";then echo PASS
else echo FAIL;cat "$LOGDIR/launch.log";exit 1;fi
echo '[PASS] Phase5: map + keepout + synthetic scan + A* preview. Not real localization/navigation.'
