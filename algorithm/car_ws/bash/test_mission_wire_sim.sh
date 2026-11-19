#!/usr/bin/env bash
# TEST ONLY: Python stdlib + anonymous Linux PTY; never opens a physical serial port.
set -Eeuo pipefail
WS="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
cd "$WS/src/car_rule_mission"
python3 -m unittest -v test_mission_execution.py test_mission_wire_sim.py
printf '\n[PTY] Run normal Dijkstra -> CAR-LINK wire simulation\n'
python3 mission_wire_sim.py --start-zone upper_right --task-code '156+123+516+231' --block B3-C3
printf '\n[PTY] Fault injection: failure and missing ACK must stop\n'
if python3 mission_wire_sim.py --fail-at 3 >/dev/null; then
  echo 'ERROR: FAILED command was accepted as success' >&2; exit 1
fi
if python3 mission_wire_sim.py --drop-ack-at 2 >/dev/null; then
  echo 'ERROR: missing ACK was accepted as success' >&2; exit 1
fi
printf 'PASS: TEST-only CAR-LINK PTY loopback, error paths and Dijkstra integration\n'
