#!/usr/bin/env python3
"""A PC-only end-to-end HTTP -> real ROS2 pub/sub -> MOCK motion smoke test.

Run when website and local ROS2 launcher are already running. This script
NEVER opens UART or publishes /cmd_vel; it refuses Pi/STM32 route. It sends
only temporary local keyboard intents and always requests STOP in finally.
"""
from __future__ import annotations

import argparse
import json
import sys
import time
from urllib.parse import urlsplit
from urllib.request import Request, urlopen


def json_request(url, payload=None):
    raw = (json.dumps(payload).encode("utf-8") if payload is not None else None)
    request = Request(
        url, data=raw,
        headers={"Content-Type": "application/json"} if raw is not None else {},
        method="POST" if raw is not None else "GET")
    with urlopen(request, timeout=1.0) as result:
        return json.loads(result.read(16000))


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--server", default="http://127.0.0.1:8090")
    parser.add_argument("--seconds", type=float, default=3.0)
    args = parser.parse_args(argv)
    p = urlsplit(args.server)
    if p.scheme != "http" or p.hostname not in ("localhost", "127.0.0.1", "::1"):
        parser.error("Smoke test can run only against the same PC localhost")
    if not 1.0 <= args.seconds <= 8.0:
        parser.error("Test duration must be between 1 and 8 seconds")
    endpoint = args.server.rstrip("/") + "/api/keyboard"
    info = json_request(endpoint+"/health")
    if (info.get("route_mode") != "local_ros2" or
        info.get("hardware_authorized") is not False or
        info.get("stm32_tx_enabled") is not False):
        parser.error("Only unprivileged LOCAL ROS2 mode is allowed")
    previous = int(info["desired"]["last_client_seq"])
    counter = max(int(time.time()*1e6), previous+10)
    x0 = info["mock"]["x_m"]
    rows = []
    print("[SMOKE] PC-only ROS2 loopback, no motors; sending 0.12 m/s for",
          args.seconds, "s", flush=True)
    try:
        until = time.monotonic()+args.seconds
        while time.monotonic() < until:
            counter += 1
            body = {"action": "keys", "client_seq": counter,
                    "keys": ["w"], "speed_mps": .12,
                    "turn_rps": .30, "deadman": True}
            json_request(endpoint+"/intent", body)
            # GET dryrun performs one bounded model step.
            mock = json_request(endpoint+"/dryrun")
            health = json_request(endpoint+"/health")
            rows.append({"reason": health["reason"],
                         "ros2_online": health["ros2_feedback"]["online"],
                         "vx": health["desired"]["vx"],
                         "model_x_m": mock["x_m"],
                         "mock_state": mock["state"],
                         "collision": mock["collision_reason"]})
            time.sleep(.1)
    finally:
        counter += 1
        try:
            json_request(endpoint+"/intent",
                         {"action": "stop", "client_seq": counter})
        except Exception as exc:
            print("[SMOKE][WARN] STOP request failed; 300ms TTL still applies:",
                  str(exc), file=sys.stderr)
    after = json_request(endpoint+"/dryrun")
    health = json_request(endpoint+"/health")
    nonzero_echo = any(r["ros2_online"] for r in rows)
    moved = x0-after["x_m"] > .004
    result = {
        "source_type": "PC_ROS2_PLUS_MOCK_ONLY",
        "hardware_authorized": False, "stm32_tx_enabled": False,
        "keyboard_intents_sent": len(rows),
        "received_any_ros2_echo": nonzero_echo,
        "mock_moved_west_over_4mm": moved,
        "initial_x_m": x0, "final_x_m": after["x_m"],
        "last_collision": after["collision_reason"],
        "latest_reason": health["reason"],
        "observations": rows[-6:],
    }
    print(json.dumps(result, ensure_ascii=False, indent=2))
    if not nonzero_echo:
        print("[DIAGNOSE] No ROS2 subscription echo: inspect ROS_DOMAIN_ID,"
              " subscribers, and POST /api/keyboard/ros2_feedback", file=sys.stderr)
        return 3
    if not moved:
        print("[DIAGNOSE] ROS2 delivered but MOCK did not move. Inspect"
              " collision_reason, reset map, and web redraw.", file=sys.stderr)
        return 4
    print("[PASS] Real PC ROS2 pub/sub reached the software vehicle model."
          " No real odom, Pi or STM32 involved.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
