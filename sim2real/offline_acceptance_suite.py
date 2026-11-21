#!/usr/bin/env python3
"""Unified hardware-free acceptance: mission, STM32 C, OpenMV, ROS2 contracts, UI.

Pure source/stub/PTY/software checks. Never opens physical UART or launches
real ROS2 nodes. Each check is run as an independent subprocess; failures are
recorded, not hidden by later passing checks.
"""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import shutil
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "algorithm/car_ws/bash"
# Import the actual host-only OpenMV tests with an explicit start/top
# directory. The tests/__init__.py marker makes discovery Python 3.10-safe.
SCHEMA = "car_hardware_free_acceptance_v1"
BASICS = [
    ("KEYBOARD_AND_SOFTWARE_LOOPBACK", ["bash", str(SCRIPT/"test_keyboard_mapping.sh")], 140),
    ("MISSION_DIJKSTRA_AND_COMMAND_FAULTS", ["bash", str(SCRIPT/"test_offline_mission.sh")], 90),
    ("CARLINK_ANONYMOUS_PTY", ["bash", str(SCRIPT/"test_mission_wire_sim.sh")], 90),
    ("STMT32_C_GCC_QR_TASK_PTY", ["bash", str(SCRIPT/"test_stm32_fsm_realview.sh")], 120),
    ("STMT32_C_HAL_STUB_MATRIX", ["bash", str(SCRIPT/"test_stm32_host_all.sh")], 120),
    ("MISSION_CPP_CORE", ["bash", str(SCRIPT/"test_phase4_core.sh")], 120),
    ("NAVIGATION_CPP_CORE", ["bash", str(SCRIPT/"test_phase5_core.sh")], 120),
    ("OPENMV_V4_CAMERA_PROTOCOL", [sys.executable, "-m", "unittest",
                                    "discover", "-s", str(ROOT / "algorithm/openmv/tests"),
                                    "-t", str(ROOT / "algorithm/openmv/tests"),
                                    "-p", "test_v4_electrical_contract.py", "-v"], 90),
    ("OPENMV_V4_QR_SESSION_HTTP", [sys.executable, "-m", "unittest",
                                    "discover", "-s", str(ROOT / "algorithm/openmv/tests"),
                                    "-t", str(ROOT / "algorithm/openmv/tests"),
                                    "-p", "test_v4_qr_contract_http.py", "-v"], 90),
    ("OPENMV_REFERENCE_FRAME_CODEC", [sys.executable,
                                      "algorithm/car_ws/bash/check_vision_protocol.py",
                                      "--self-test"], 50),
    ("OPENMV_V4_STM32_STATIC_ALIGNMENT", [sys.executable,
                                          "algorithm/car_ws/bash/check_vision_protocol.py",
                                          "--variant", "v4"], 60),
    ("WEB_TEST_API_ORIGIN_SECURITY", [sys.executable, "-m", "unittest",
                                      "discover", "-s", "sim2real/tests",
                                      "-p", "test_test_suite.py", "-v"], 90),
]


def cases(full):
    batch = ["bash", str(SCRIPT / "test_competition_batch.sh"),
             "--cases", "300" if full else "60",
             "--seed", "20271120", "--wire-samples", "10" if full else "3"]
    return BASICS + [
        ("COMPETITION_8_LEG_QR_BATCH", batch, 120),
        ("WEB_JAVASCRIPT_SYNTAX", ["bash", "-c",
             "set -euo pipefail; "
             "for file in sim2real/web/*.js; do node --check \"$file\"; done; "
             "node sim2real/tests/test_mode_router.js"], 90),
    ]


def run_case(name, argv, timeout):
    started = time.monotonic()
    try:
        result = subprocess.run(
            argv, cwd=ROOT, capture_output=True, text=True,
            timeout=timeout, check=False, errors="replace")
        status = "PASS" if result.returncode == 0 else "FAIL"
        exit_code = result.returncode
        log = (result.stdout + "\n" + result.stderr)[-9000:]
    except subprocess.TimeoutExpired as exc:
        status, exit_code = "TIMEOUT", None
        log = f"Exceeded {timeout}s: {exc}"
    except OSError as exc:
        status, exit_code = "ERROR", None
        log = f"Could not launch test: {exc}"
    return {"name": name, "status": status, "exit_code": exit_code,
            "seconds": round(time.monotonic()-started, 3), "log_tail": log,
            "cmd": argv}


def readiness():
    return {
        "offline_verified_scope": [
            "Python keyboard mapping/ROS2 message schemas",
            "PC-only test ROS2 candidate subscriber contracts; not ROS2 DDS runtime",
            "finite deadman/watchdog, stale stamps, ROS2 message diagnostic replay",
            "kinematic MOCK chassis, collision checks and model fault matrix",
            "Dijkstra competition mission planning, 300 deterministic scenarios",
            "CAR-LINK anonymous PTY/CRC state machine",
            "STM32 C RaceFsm/TaskGate/UART/ring/app/ULog host stubs",
            "OpenMV V4 serial format, QR/ACK/session contract and static mirror",
            "C++ planning/navigation cores, TEST web API and JS syntax",
        ],
        "must_wait_for_hardware": [
            "Pi5 network DDS discovery and PC↔Pi time synchronization",
            "actual UART pinout, baud, electrical isolation and STM32 TX authorization",
            "physical hard E-stop, arming, watchdog, motor power stage and control arbitration",
            "OpenMV H7 Plus camera image thresholds, lighting, lens and grab pixel calibration",
            "LiDAR extrinsics, blocked-road coverage, localization and map frame alignment",
            "wheel direction/encoder counts, actual wheelbase and mecanum slip",
            "real IMU/odometry and vehicle speed/step-response PID tuning",
            "robot footprint/turning geometry collision safety and actual competition runs",
        ],
        "known_static_gap": (
            "Current V5 OpenMV extended OBS/USE_OK/RESULT_OK semantics are not a "
            "drop-in STM32 V4 protocol; V4 host contract is the tested baseline."
        ),
    }


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--quick", action="store_true",
                        help="60 rather than 300 seeded mission scenarios")
    parser.add_argument("--out-dir", default="/tmp/car_offline_full_acceptance")
    args = parser.parse_args(argv)
    out = Path(args.out_dir)
    out.mkdir(parents=True, exist_ok=True)
    checks = []
    for name, cmd, timeout in cases(not args.quick):
        print(f"[OFFLINE] {name}", flush=True)
        row = run_case(name, cmd, timeout)
        checks.append(row)
        print(f"[{row['status']}] {name} ({row['seconds']:.2f}s)", flush=True)
        if row["status"] != "PASS":
            print("----- failing test output -----", flush=True)
            print(row["log_tail"][-6500:], flush=True)
            print("----- end failure -----", flush=True)
    passed = all(check["status"] == "PASS" for check in checks)
    report = {
        "schema": SCHEMA,
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "mode": "QUICK" if args.quick else "FULL",
        "source_type": "NO_HARDWARE_STUBS_SIMULATORS_AND_STATIC_CODE",
        "hardware_authorized": False, "stm32_tx_enabled": False,
        "passed": passed,
        "summary": {"PASS": sum(c["status"] == "PASS" for c in checks),
                    "FAIL": sum(c["status"] != "PASS" for c in checks)},
        "checks": checks,
        "readiness": readiness(),
    }
    (out / "acceptance.json").write_text(
        json.dumps(report, ensure_ascii=False, indent=2)+"\n", encoding="utf-8")
    md = [
        "# 工创小车：无实车全链路软件验收",
        "",
        f"- 结果：{'PASS' if passed else 'FAIL'}；"
        f"已通过 {report['summary']['PASS']} / {len(checks)}",
        "- 数据性质：本机单元测试、PTY 模拟、HAL 桩代码和静态源码核对",
        "- 实体控制授权：false；未验证实车运动、安全和识别",
        "",
        "| 软件验收项 | 状态 | 耗时 (s) |",
        "| --- | --- | ---: |",
    ]
    for c in checks:
        md.append(f"| {c['name']} | {c['status']} | {c['seconds']:.2f} |")
    md += ["", "## 必须等待硬件才能进行的验收"]
    md += ["- " + text for text in report["readiness"]["must_wait_for_hardware"]]
    md += ["", "## 现有静态协议缺口",
           report["readiness"]["known_static_gap"], "",
           "详细失败原因及命令：查看 acceptance.json 的 checks[].log_tail。", ""]
    (out / "acceptance.md").write_text("\n".join(md), encoding="utf-8")
    print(json.dumps({"passed": passed, "summary": report["summary"],
                      "json": str(out/"acceptance.json"),
                      "markdown": str(out/"acceptance.md")}, ensure_ascii=False))
    return 0 if passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
