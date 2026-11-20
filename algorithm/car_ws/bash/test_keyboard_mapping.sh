#!/usr/bin/env bash
# Fifth-stage keyboard mapping software acceptance: no ROS / STM32 required.
set -Eeuo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../../.." && pwd -P)"
export PYTHONDONTWRITEBYTECODE=1
cd "$ROOT"
python3 -m unittest discover -s sim2real/tests -p 'test_keyboard_mapping.py' -v
python3 -m unittest discover -s sim2real/tests -p 'test_keyboard_dryrun.py' -v
python3 -m unittest discover -s sim2real/tests -p 'test_keyboard_offline_analysis.py' -v
python3 -m unittest discover -s sim2real/tests -p 'test_keyboard_ros2_node_startup.py' -v
python3 -m unittest discover -s sim2real/tests -p 'test_keyboard_link_health.py' -v
python3 -m unittest discover -s sim2real/tests -p 'test_keyboard_local_smoke.py' -v
python3 -m unittest discover -s sim2real/tests -p 'test_four_modes.py' -v
python3 sim2real/keyboard_fault_lab.py --out-dir /tmp/car_keyboard_fault_lab
python3 sim2real/keyboard_offline_analysis.py --out-dir /tmp/car_keyboard_offline_analysis
python3 sim2real/keyboard_offline_analysis.py \
  --input /tmp/car_keyboard_offline_analysis/synthetic_events.jsonl \
  --out-dir /tmp/car_keyboard_offline_replay
python3 - <<'PY'
import json
from pathlib import Path
original=json.loads(Path("/tmp/car_keyboard_offline_analysis/report.json").read_text())
replayed=json.loads(Path("/tmp/car_keyboard_offline_replay/report.json").read_text())
assert original==replayed, "Offline recording/replay must be byte-equivalent in statistics"
assert all(p["source_type"]=="ROS2_SOFTWARE_DIAGNOSTIC_ONLY" for p in original["packets"])
assert original["stm32_tx_enabled"] is False
print("PASS: deterministic offline ROS2 diagnostic replay and no physical UART TX")
PY
python3 - "$ROOT" <<'PY'
import ast
from pathlib import Path
import sys
root=Path(sys.argv[1])
for file in ("sim2real/keyboard_mapping_core.py",
             "sim2real/keyboard_dryrun_chassis.py",
             "sim2real/keyboard_offline_analysis.py",
             "sim2real/keyboard_local_smoke.py",
             "sim2real/keyboard_fault_lab.py",
             "sim2real/keyboard_ros2_pc.py",
             "sim2real/keyboard_ros2_pi.py",
             "sim2real/keyboard_ros2_web_feedback.py",
             "sim2real/web_mode_gateway.py"):
    ast.parse((root/file).read_text(encoding="utf-8"), filename=file)
    print("SYNTAX OK:", file)
PY
if command -v node >/dev/null 2>&1; then
  node --check sim2real/web/keyboard_mapping_panel.js
  node --check sim2real/web/mode_router.js
  node sim2real/tests/test_mode_router.js
fi
bash -n algorithm/car_ws/bash/start_keyboard_local_ros2.sh
bash -n algorithm/car_ws/bash/start_keyboard_pc_ros2.sh
bash -n algorithm/car_ws/bash/start_keyboard_pi_ros2.sh
bash -n algorithm/car_ws/bash/start_keyboard_web_feedback_ros2.sh
echo '[PASS] KEYBOARD MAP: PC local ROS2 echo + Pi candidate + REAL odom error, no motor control'
