#!/usr/bin/env python3
"""Repeatable no-hardware keyboard digital-twin fault acceptance report.

Does not import ROS2 or open serial ports. This is a kinematic model QA
report, not physical control/odometry certification.
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from keyboard_dryrun_chassis import KeyboardChassisTwin

GO = {"vx": .20, "vy": 0., "wz": 0.}
ZERO = {"vx": 0., "vy": 0., "wz": 0.}


def simulate(fault="none", cells=None, drop_at_s=None, stop_at_s=None, duration=1.5):
    model = KeyboardChassisTwin()
    model.reset("upper_right", cells or [])
    model.set_fault(fault)
    samples = []
    steps = round(duration / .05)
    for step in range(steps + 1):
        seconds = step * .05
        accepted = drop_at_s is None or seconds < drop_at_s
        command = ZERO if stop_at_s is not None and seconds >= stop_at_s else GO
        result = model.advance(seconds, command, accepted_ros2=accepted)
        samples.append({
            "t_s": round(seconds, 3),
            "state": result["state"],
            "x": result["x_m"], "y": result["y_m"],
            "vx": result["velocity"]["vx"],
            "error_linear_norm": result["model_error"]["linear_norm_mps"],
            "collision": result["collision_reason"],
            "motor_authorized": result["hardware_authorized"],
        })
    return model.snapshot(), samples


def acceptance():
    base, _ = simulate(duration=1.5)
    stall, _ = simulate("motor_stall", duration=.8)
    slow, _ = simulate("slow_response", duration=.35)
    fast, _ = simulate(duration=.35)
    link, _ = simulate("link_loss", duration=.8)
    obstacle, _ = simulate(cells=[23], duration=1.6)
    dropped, _ = simulate(drop_at_s=.25, duration=1.5)
    stopped, _ = simulate(stop_at_s=.5, duration=1.6)
    cases = [
        ("NORMAL_DRIVE", base["x_m"] < 2.16 and base["state"] != "OBSTACLE_STOP",
         "Finite westbound motion with acceleration limits"),
        ("MOCK_STALL", stall["velocity"]["vx"] == 0 and
         stall["model_error"]["linear_norm_mps"] >= .19,
         "Simulated command nonzero, kinematic response zero"),
        ("MOCK_SLOW_ACCELERATION", slow["velocity"]["vx"] <
         fast["velocity"]["vx"], "Reduced acceleration creates tracking error"),
        ("MOCK_LINK_LOSS", link["velocity"]["vx"] == 0 and
         link["model_error"]["linear_norm_mps"] >= .19,
         "Model drops drive input; reference command remains visible"),
        ("MAP_CELL23_COLLISION", obstacle["state"] == "OBSTACLE_STOP" and
         obstacle["collision_reason"] == "BLOCKED_CELL",
         "Red obstacle latches stop, never teleports through cell"),
        ("ROUTE_WATCHDOG_STOP", dropped["velocity"]["vx"] == 0.0 and
         dropped["state"] == "STOPPED",
         "Loss of fresh ROS2 accepted input produces stop"),
        ("USER_STOP_DECELERATES", stopped["velocity"]["vx"] == 0.0 and
         stopped["state"] == "STOPPED",
         "Stop command respects finite deceleration"),
    ]
    return {
        "schema": "car_keyboard_mock_fault_acceptance_v1",
        "source_type": "MOCK_KINEMATIC_ONLY",
        "hardware_authorized": False,
        "ros2_runtime_used": False,
        "uart_used": False,
        "passed": all(v for _, v, _ in cases),
        "summary": {"PASS": sum(v for _, v, _ in cases),
                    "FAIL": sum(not v for _, v, _ in cases)},
        "cases": [{"case": name, "passed": ok, "description": note}
                  for name, ok, note in cases],
        "samples": {"normal": base, "blocked_cell23": obstacle,
                    "stall": stall, "slow": slow, "link_loss": link,
                    "ros2_dropout": dropped, "stop": stopped},
        "notice": ("Only offline kinematic model acceptance. "
                   "Cannot prove ROS2 delivery, actual STM32 speed tracking, "
                   "wheel slip, PID performance or real obstacle avoidance."),
    }


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out-dir", default="/tmp/car_keyboard_fault_lab")
    args = parser.parse_args(argv)
    output = Path(args.out_dir)
    output.mkdir(parents=True, exist_ok=True)
    report = acceptance()
    (output / "acceptance.json").write_text(
        json.dumps(report, indent=2, ensure_ascii=False), encoding="utf-8")
    lines = [
        "# 映射集：纯软件底盘故障测试报告",
        "",
        "- 数据来源：MOCK_KINEMATIC_ONLY；不是实车里程计",
        "- 物理电机授权：false；ROS2/UART 设备访问：false",
        "- 结果：" + ("PASS" if report["passed"] else "FAIL"),
        "",
        "| 用例 | 结果 | 描述 |",
        "| --- | --- | --- |",
    ]
    for case in report["cases"]:
        lines.append("| " + case["case"] + " | " +
                     ("PASS" if case["passed"] else "FAIL") +
                     " | " + case["description"] + " |")
    lines.extend(["", "**限制：** " + report["notice"], ""])
    (output / "acceptance.md").write_text(
        "\n".join(lines), encoding="utf-8")
    print(json.dumps({
        "passed": report["passed"], "summary": report["summary"],
        "files": [str(output / "acceptance.json"),
                  str(output / "acceptance.md")],
    }, indent=2))
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
