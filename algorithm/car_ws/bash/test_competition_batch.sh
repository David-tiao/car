#!/usr/bin/env bash
# Competition phase 4, deterministic SIM_ONLY acceptance. Never connects hardware.
set -Eeuo pipefail
HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
WS="$(cd -- "$HERE/.." && pwd -P)"
export PYTHONDONTWRITEBYTECODE=1
cd "$WS/src/car_rule_mission"
python3 -m unittest -v \
    test_rule_mission.py test_mission_execution.py test_mission_wire_sim.py \
    test_mission_ros_session.py test_mission_ros_web_store.py \
    test_competition_batch_acceptance.py
echo '[QA] Executing deterministic competition batch tests'
python3 competition_batch_acceptance.py "$@"
