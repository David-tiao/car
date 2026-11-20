#!/usr/bin/env python3
"""No-hardware ROS2 teleop diagnostics: packet gaps, latency, replay and steps.

These are *software transport/kinematics* measurements, not a physical motor
or STM32 /odom test. Caller must never substitute mock samples for REAL.
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from statistics import mean

SCHEMA = "car_keyboard_offline_diagnostic_v1"
ALLOWED_CHANNELS = frozenset(("pc_local_ros2", "pi_candidate"))
MAX_RECORDS = 20000


def numeric(value):
    return type(value) in (int, float) and math.isfinite(value)


def percentile(numbers, fraction):
    """Nearest-rank interpolation, deterministic for offline reports."""
    if not numbers:
        return None
    ordered = sorted(numbers)
    at = (len(ordered) - 1) * fraction
    lo = math.floor(at)
    hi = math.ceil(at)
    return ordered[lo] * (hi-at) + ordered[hi] * (at-lo) if hi != lo else ordered[lo]


def valid_packet(event):
    if not isinstance(event, dict) or set(event) != {
            "kind", "channel", "session", "seq", "sent_s", "received_s"}:
        raise ValueError("Invalid diagnostic packet fields")
    if event["kind"] != "packet" or event["channel"] not in ALLOWED_CHANNELS:
        raise ValueError("Unexpected diagnostic packet origin")
    if type(event["session"]) is not str or not (1 <= len(event["session"]) <= 64):
        raise ValueError("Invalid session ID")
    if type(event["seq"]) is not int or not 0 <= event["seq"] <= 2**32-1:
        raise ValueError("Invalid diagnostic seq")
    if not all(numeric(event[k]) for k in ("sent_s", "received_s")):
        raise ValueError("Nonfinite packet timestamps")
    if event["received_s"] < event["sent_s"] - .05:
        raise ValueError("Clocks skewed; negative transport latency")
    return event


def delivery_metrics(events, expected_period_s=.05):
    if not numeric(expected_period_s) or not 0 < expected_period_s <= 5:
        raise ValueError("Invalid sample period")
    groups = {}
    for event in events:
        item = valid_packet(event)
        groups.setdefault((item["channel"], item["session"]), []).append(item)
    answer = []
    for (channel, session), packets in sorted(groups.items()):
        seen = set()
        unique = []
        repeats, reordered = 0, 0
        high = -1
        for pkt in packets:
            seq = pkt["seq"]
            if seq in seen:
                repeats += 1
            else:
                seen.add(seq)
                unique.append(pkt)
                if seq < high:
                    reordered += 1
                high = max(high, seq)
        span = max(seen) - min(seen) + 1
        missing = span - len(seen)
        latencies = [1000 * max(0., p["received_s"] - p["sent_s"])
                     for p in unique]
        intervals = [b["received_s"]-a["received_s"]
                     for a, b in zip(unique, unique[1:])]
        jitter = [(v-expected_period_s)*1000 for v in intervals]
        valid_intervals = [v for v in intervals if v >= 0]
        answer.append({
            "channel": channel, "session": session,
            "observed": len(packets), "unique": len(seen),
            "seq_range": [min(seen), max(seen)],
            "missing_in_observed_range": missing,
            "loss_fraction_observed_range": round(missing/span, 6),
            "duplicates": repeats, "reordered": reordered,
            "nonmonotonic_receipt_intervals": len(intervals)-len(valid_intervals),
            "latency_ms": {
                "mean": round(mean(latencies), 3),
                "p95": round(percentile(latencies, .95), 3),
                "max": round(max(latencies), 3),
            },
            "receive_jitter_rms_ms": round(
                math.sqrt(mean([x*x for x in jitter])), 3) if jitter else None,
            "warning": ("Only sequence gaps BETWEEN first and last observed seq; "
                        "cannot detect missing tail, disconnected publisher, "
                        "network vs executor delay or MCU timing."),
            "source_type": "ROS2_SOFTWARE_DIAGNOSTIC_ONLY",
        })
    return answer


def valid_step(row):
    if not isinstance(row, dict) or set(row) != {
            "kind", "t_s", "desired_mps", "response_mps", "source_type"}:
        raise ValueError("Invalid step sample fields")
    if row["kind"] != "step" or row["source_type"] not in (
            "MOCK_KINEMATIC_ONLY", "REAL_ODOM_RECORDED_OFFLINE"):
        raise ValueError("Unknown step response origin")
    if not all(numeric(row[k]) for k in ("t_s", "desired_mps", "response_mps")):
        raise ValueError("Nonfinite step response sample")
    if abs(row["desired_mps"]) > 1 or abs(row["response_mps"]) > 5:
        raise ValueError("Invalid speed sample range")
    return row


def step_metrics(rows, tolerance_fraction=.05):
    samples = [valid_step(x) for x in rows]
    if len(samples) < 4:
        raise ValueError("At least four samples required")
    if len({x["source_type"] for x in samples}) != 1:
        raise ValueError("Cannot mix simulated and recorded REAL sources")
    times = [x["t_s"] for x in samples]
    if any(b <= a for a, b in zip(times, times[1:])):
        raise ValueError("Step sample time must increase")
    base = samples[0]["desired_mps"]
    change = next((i for i, v in enumerate(samples)
                   if abs(v["desired_mps"]-base) > .02), None)
    if change is None:
        raise ValueError("No target speed step")
    target = samples[change]["desired_mps"]
    amplitude = target-base
    sign = 1 if amplitude > 0 else -1
    # Limit analysis to the initial constant-command segment, stop/reverse
    # is a separate event and must not distort steady-state speed.
    stop = next((i for i in range(change+1, len(samples))
                 if abs(samples[i]["desired_mps"]-target) > .001),
                len(samples))
    segment = samples[change:stop]
    if len(segment) < 3:
        raise ValueError("Too few post-step samples")
    t0 = samples[change]["t_s"]
    bresponse = samples[change-1]["response_mps"]
    def crossing(level):
        return next((s["t_s"]-t0 for s in segment
                     if sign*(s["response_mps"]-bresponse) >=
                     abs(amplitude)*level), None)
    t10, t90 = crossing(.1), crossing(.9)
    steady = mean([s["response_mps"] for s in segment[-min(5, len(segment)):]])
    overshoot = max(0., max(sign*(s["response_mps"]-target) for s in segment))
    threshold = tolerance_fraction*abs(amplitude)
    settle = next((s["t_s"]-t0 for i, s in enumerate(segment)
                   if all(abs(x["response_mps"]-target) <= threshold
                          for x in segment[i:])), None)
    return {
        "schema": "car_keyboard_step_response_v1",
        "source_type": samples[0]["source_type"],
        "hardware_authorized": False,
        "step_t_s": round(t0, 4),
        "initial_command_mps": base,
        "target_command_mps": target,
        "rise_time_10_90_s": round(t90-t10, 4) if t10 is not None and t90 is not None else None,
        "settling_time_5pct_s": round(settle, 4) if settle is not None else None,
        "steady_response_mps": round(steady, 5),
        "steady_error_mps": round(target-steady, 5),
        "overshoot_mps": round(overshoot, 5),
        "samples": len(segment),
        "notice": "These metrics depend on supplied source; MOCK is not real STM32 PID response.",
    }


def mecanum_wheels(vx, vy, wz, radius=.05, half_length=.145, half_width=.13):
    """Conventional X-roller mecanum geometry; wheel/sign wiring unverified."""
    if not all(numeric(v) for v in (vx, vy, wz, radius, half_length, half_width)):
        raise ValueError("Finite mecanum geometry required")
    if radius <= 0 or half_length <= 0 or half_width <= 0:
        raise ValueError("Positive geometry required")
    arm = half_length+half_width
    return tuple((v/radius) for v in (
        vx-vy-arm*wz, vx+vy+arm*wz, vx+vy-arm*wz, vx-vy+arm*wz))


def mecanum_body(wheels, radius=.05, half_length=.145, half_width=.13):
    if len(wheels) != 4 or not all(numeric(v) for v in wheels):
        raise ValueError("Four finite wheel speeds required")
    if not all(numeric(v) and v > 0 for v in (radius, half_length, half_width)):
        raise ValueError("Invalid geometry")
    a, b, c, d = wheels
    arm = half_length+half_width
    return (radius*(a+b+c+d)/4,
            radius*(-a+b+c-d)/4,
            radius*(-a+b-c+d)/(4*arm))


def reference_trace(command_rows, initial=(0., 0., 0.)):
    """Integrate body-frame velocity using previous-sample Euler, NO sensors."""
    if len(initial) != 3 or not all(numeric(v) for v in initial):
        raise ValueError("Invalid integration origin")
    x, y, yaw = initial
    trace = [[round(x, 5), round(y, 5), round(yaw, 5)]]
    last = None
    for row in command_rows:
        t, vx, vy, wz = (row.get(k) for k in ("t_s", "vx", "vy", "wz"))
        if not all(numeric(v) for v in (t, vx, vy, wz)):
            raise ValueError("Nonfinite command trace")
        if last is not None:
            dt = t-last[0]
            if not 0 < dt <= .5:
                raise ValueError("Invalid command sample gap")
            c, s = math.cos(yaw), math.sin(yaw)
            x += (c*last[1]-s*last[2])*dt
            y += (s*last[1]+c*last[2])*dt
            yaw = math.atan2(math.sin(yaw+last[3]*dt),
                             math.cos(yaw+last[3]*dt))
            trace.append([round(x, 5), round(y, 5), round(yaw, 5)])
        last = (t, vx, vy, wz)
    return trace


def read_jsonl(path):
    lines = Path(path).read_text(encoding="utf-8").splitlines()
    if len(lines) > MAX_RECORDS:
        raise ValueError("Too many offline replay events")
    events = []
    for i, raw in enumerate(lines, 1):
        if len(raw) > 1024:
            raise ValueError("Oversized replay event line")
        try:
            event = json.loads(raw,
                               parse_constant=lambda _: (_ for _ in ()).throw(
                                   ValueError("Nonfinite value")))
        except json.JSONDecodeError as exc:
            raise ValueError(f"Invalid JSONL line {i}") from exc
        if not isinstance(event, dict):
            raise ValueError("Replay records must be objects")
        if event.get("kind") == "packet":
            events.append(valid_packet(event))
        elif event.get("kind") == "step":
            events.append(valid_step(event))
        else:
            raise ValueError("Unknown replay event kind")
    return events


def replay(events):
    packets = [x for x in events if x["kind"] == "packet"]
    steps = [x for x in events if x["kind"] == "step"]
    return {
        "schema": SCHEMA, "source_type": "OFFLINE_RECORDED_OR_SYNTHETIC",
        "hardware_authorized": False, "stm32_tx_enabled": False,
        "packets": delivery_metrics(packets),
        "step_response": step_metrics(steps) if steps else None,
        "records": len(events),
        "limitations": ("Offline transport stats are not physical serial throughput; "
                        "step metrics only reflect their explicit source_type."),
    }


def generate_fixture():
    packets = []
    for seq in range(80):
        if seq in (17, 18, 41):
            continue
        sent = 1000 + seq*.05
        packets.append({"kind": "packet", "channel": "pc_local_ros2",
                        "session": "offline-1", "seq": seq,
                        "sent_s": round(sent, 4),
                        "received_s": round(sent + .012 + .002*(seq%3), 4)})
    packets.append(dict(packets[12]))
    packets.insert(30, packets.pop(18))
    steps = []
    for n in range(130):
        t = n*.05
        desired = .16 if .5 <= t < 4.5 else 0.
        response = (0 if t < .5 else
                    .16*(1-math.exp(-(t-.5)/.33)))
        if t >= 4.5:
            response = .16*math.exp(-(t-4.5)/.25)
        steps.append({"kind": "step", "t_s": round(t, 4),
                      "desired_mps": desired, "response_mps": round(response, 6),
                      "source_type": "MOCK_KINEMATIC_ONLY"})
    return packets+steps


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", help="Replay local JSONL, never connects ROS2/UART")
    parser.add_argument("--out-dir", default="/tmp/car_keyboard_offline_analysis")
    args = parser.parse_args(argv)
    events = read_jsonl(args.input) if args.input else generate_fixture()
    report = replay(events)
    out = Path(args.out_dir)
    out.mkdir(parents=True, exist_ok=True)
    if not args.input:
        (out / "synthetic_events.jsonl").write_text(
            "".join(json.dumps(x, separators=(",", ":"))+"\n"
                    for x in events), encoding="utf-8")
    (out / "report.json").write_text(
        json.dumps(report, ensure_ascii=False, indent=2)+"\n", encoding="utf-8")
    lines = [
        "# 键盘 ROS 2 离线诊断报告", "",
        "数据来源：" + report["source_type"],
        "实体电机授权：false；不会发送 UART /cmd_vel", "",
    ]
    for p in report["packets"]:
        lines += [
            f"## {p['channel']} / {p['session']}",
            f"接收 {p['observed']}，唯一包 {p['unique']}，中间缺序 {p['missing_in_observed_range']}，"
            f"重复 {p['duplicates']}，乱序 {p['reordered']}",
            f"单机估算时延平均 {p['latency_ms']['mean']} ms，P95 {p['latency_ms']['p95']} ms",
            "",
        ]
    if report["step_response"]:
        s = report["step_response"]
        lines += [
            "## 软件速度阶跃响应",
            f"10–90% 上升时间: {s['rise_time_10_90_s']} s",
            f"5% 稳定时间: {s['settling_time_5pct_s']} s",
            f"稳定误差: {s['steady_error_mps']} m/s",
            "", s["notice"], "",
        ]
    (out / "report.md").write_text("\n".join(lines), encoding="utf-8")
    print(json.dumps({"out_dir": str(out), "packet_groups": len(report["packets"]),
                      "step_source": report["step_response"]["source_type"]
                      if report["step_response"] else None,
                      "stm32_tx_enabled": False}, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
