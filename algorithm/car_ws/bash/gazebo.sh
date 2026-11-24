#!/usr/bin/env bash
# Independent Gazebo SIM only. No hardware startup or broad process cleanup.
set -eo pipefail
source "$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)/lib/common.sh"
car_check_workspace
WS="$CAR_WS"
cd "$WS"
usage() {
  echo "Usage: bash bash/gazebo.sh doctor | start [goal] | test [goal] | status"
  echo "start: GUI + RViz, readiness only; test: server only + explicit SIM route."
  echo "Fixed test fixture starts at C3; default goal A1 (start) or C2 (test)."
}
mode="${1:-}"
if [[ "$mode" == "-h" || "$mode" == "--help" ]]; then usage; exit 0; fi
if [[ "$mode" != doctor && "$mode" != start && "$mode" != test && "$mode" != status ]]; then
  usage >&2; exit 2
fi
if [[ $# -gt 2 || ( "$mode" != start && "$mode" != test && $# -ne 1 ) ]]; then
  usage >&2; exit 2
fi
goal=A1
if [[ "$mode" == test ]]; then goal=C2; fi
goal="${2:-$goal}"
if [[ ! "$goal" =~ ^[ABC][123]$ || "$goal" == C3 ]]; then
  echo "Invalid distinct goal node: $goal" >&2; exit 2
fi
car_source_ros
export ROS_DOMAIN_ID=84 ROS_LOCALHOST_ONLY=1
check_environment() {
  local p
  if [[ "${ROS_DISTRO:-}" != humble ]]; then echo "ENV FAIL: ROS_DISTRO must be humble"; return 1; fi
  if ! command -v ign >/dev/null || ! ign gazebo --versions >/dev/null 2>&1; then
    echo "ENV FAIL: Gazebo Fortress (ign gazebo) required; Harmonic/Classic alone are not supported by this backend." >&2
    return 1
  fi
  for p in ros2 python3 flock setsid; do
    command -v "$p" >/dev/null || { echo "ENV FAIL: missing $p"; return 1; }
  done
  for p in car_gazebo car_navigation ros_gz_bridge robot_state_publisher nav2_controller nav2_lifecycle_manager dwb_core dwb_critics rviz2; do
    ros2 pkg prefix "$p" >/dev/null || { echo "ENV FAIL: package $p unavailable"; return 1; }
  done
  python3 -c 'import rclpy; from nav2_msgs.action import FollowPath; from ros_gz_interfaces.srv import SetEntityPose' || return 1
  echo "PASS ENV: Humble packages found, reserved local domain 84. Runtime not yet checked."
}
check_environment
if [[ "$mode" == doctor ]]; then exit 0; fi
if [[ "$mode" == status ]]; then
  echo "SIM graph (read-only):"
  ros2 node list --no-daemon --spin-time 2
  exit 0
fi
umask 077
root="${CAR_GAZEBO_RUN_DIR:-$HOME/.cache/car_gazebo}"
mkdir -p "$root"
if [[ -L "$root" || ! -O "$root" ]]; then
  echo "ENV FAIL: run directory must be owned by current user and not a symlink" >&2; exit 1
fi
root="$(cd "$root" && pwd -P)"
# Global per-user lock independent of log destination; never kills foreign nodes.
lockdir="${XDG_RUNTIME_DIR:-$HOME/.cache}"
mkdir -p "$lockdir"
exec 9>"$lockdir/car_gazebo_${UID}.lock"
flock -n 9 || { echo "START FAIL: another managed SIM session holds the lock" >&2; exit 1; }
occupied="$(ros2 node list --no-daemon --spin-time 2)"
if [[ -n "$occupied" ]]; then
  echo "START FAIL: domain 84 is occupied; stop its owner first. No foreign processes were terminated." >&2
  printf '%s\n' "$occupied" >&2
  exit 1
fi
MAP="$WS/maps/competition_v1"
OUT="$(mktemp -d "$root/run.XXXXXXXX")"
echo "Run files and logs: $OUT"
ros2 run car_gazebo generate_scene.py --zones "$MAP/zones.json" --output "$OUT" \
  --mode sim --start 2.05 2.05 --obstacle 1.2 1.5 --obstacle 1.5 2.05
server=false
rviz=true
extra=()
if [[ "$mode" == test ]]; then
  server=true
  rviz=false
  extra+=(--execute-route)
fi
pid=""
cleanup() {
  local result=$?
  trap - EXIT INT TERM
  if [[ -n "$pid" ]]; then
    # setsid launches only this run in a separate process group.
    if kill -0 -- "-$pid" 2>/dev/null; then
      kill -INT -- "-$pid" 2>/dev/null || true
      for _ in {1..30}; do
        kill -0 -- "-$pid" 2>/dev/null || break
        sleep .2
      done
      if kill -0 -- "-$pid" 2>/dev/null; then
        kill -TERM -- "-$pid" 2>/dev/null || true
        sleep 1
      fi
      if kill -0 -- "-$pid" 2>/dev/null; then kill -KILL -- "-$pid" 2>/dev/null || true; fi
    fi
    wait "$pid" 2>/dev/null || true
  fi
  echo "SIM session ended (code $result); logs retained: $OUT"
  exit "$result"
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
setsid ros2 launch car_gazebo sim_full.launch.py \
  world:="$OUT/car_sim.sdf" map_dir:="$MAP" partition:="car_sim_$(basename "$OUT" | tr . _)" \
  start:=C3 goal:="$goal" server_only:="$server" with_rviz:="$rviz" >"$OUT/launch.log" 2>&1 &
pid=$!
echo "START: checking live clock, lidar, odometry, TF, lock and Nav2; see $OUT/launch.log"
if ! ros2 run car_gazebo check_sim.py --topology "$MAP/topology_stage6/road_topology.py" \
  --start C3 --goal "$goal" "${extra[@]}" 2>&1 | tee "$OUT/acceptance.log"; then
  echo "ACCEPTANCE FAIL: inspect $OUT/launch.log and acceptance.log" >&2
  tail -n 60 "$OUT/launch.log" >&2 || true
  exit 1
fi
if ! kill -0 "$pid" 2>/dev/null; then
  echo "START FAIL: launch ended before acceptance" >&2
  exit 1
fi
if [[ "$mode" == test ]]; then
  echo "PASS: SIM route acceptance completed; shutting down only this run."
  exit 0
fi
echo "READY: Gazebo and RViz2 running; no automatic motion."
echo "In another sourced terminal export ROS_DOMAIN_ID=84 ROS_LOCALHOST_ONLY=1"
echo "Then explicitly request /car_gz/follow_locked_route with std_srvs/srv/Trigger."
echo "Ctrl+C here ends only this run."
wait "$pid"
