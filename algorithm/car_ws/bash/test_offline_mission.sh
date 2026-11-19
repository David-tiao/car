#!/usr/bin/env bash
# Offline-only: no ros2, serial port, lidar or hardware.
set -Eeuo pipefail
WS="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
cd "$WS/src/car_rule_mission"
python3 test_mission_execution.py
python3 test_rule_mission.py
printf '\n[SIM] Dijkstra 8 legs + task-code state-machine smoke test\n'
python3 mission_sim_demo.py --start-zone upper_right --block B3-C3 > /dev/null
printf '[PASS] normal simulation\n'
if python3 mission_sim_demo.py --start-zone upper_right --block B3-C3 --block C2-C3 > /dev/null; then
  echo '[FAIL] unreachable route incorrectly succeeded' >&2; exit 1
else
  echo '[PASS] unreachable route correctly rejected'
fi
if python3 mission_sim_demo.py --fail-at 3 > /dev/null; then
  echo '[FAIL] failed command incorrectly succeeded' >&2; exit 1
else
  echo '[PASS] failure correctly stopped mission'
fi
printf '\nPASS: offline mission core and fail-closed checks\n'
