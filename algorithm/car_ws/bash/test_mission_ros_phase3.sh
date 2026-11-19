#!/usr/bin/env bash
# Phase 3 software regression. No ROS runtime, camera, lidar, or physical UART required.
set -Eeuo pipefail
WS="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
cd "$WS/src/car_rule_mission"
python3 -m unittest -v \
  test_rule_mission.py \
  test_mission_execution.py \
  test_mission_wire_sim.py \
  test_mission_ros_session.py \
  test_mission_ros_web_store.py
bash -n "$WS/bash/run_test_mission_ros.sh"
python3 -m py_compile mission_test_session.py mission_ros_test.py \
  "$WS/../../sim2real/test_mission_store.py" \
  "$WS/../../sim2real/test_mission_relay.py"
python3 mission_wire_sim.py --block B3-C3 >/dev/null
printf '\nPASS: stage3 offline regression and script syntax\n'
