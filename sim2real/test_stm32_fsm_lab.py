"""Isolated MOCK STM32 CAR-LINK/PTy mission FSM lab for the /real view.

Preserves the existing PTY MockPeer + SimExecutor. Commands submitted here are
strictly TEST-only control of *anonymous* Unix PTYs; never opens a real serial
device, publishes ROS commands, or writes /api/real telemetry.
"""
from __future__ import annotations

import ipaddress
import json
import math
import sys
import threading
import time
from pathlib import Path
from urllib.parse import urlsplit

LAB_PATH = "/api/test/stm32_fsm_lab"
ROOT = Path(__file__).resolve().parents[1]
SIM_CODE = ROOT / "algorithm" / "car_ws" / "src" / "car_rule_mission"
if str(SIM_CODE) not in sys.path:
    sys.path.insert(0, str(SIM_CODE))
from mission_execution import MissionError, parse_task_code
from mission_test_session import PtyMissionSession
from rule_mission_core import PlanError, build_mission

LOCK = threading.RLock()
SESSION = None
CASE = None
GENERATION = 0
UPDATED_AT = 0.0
FAULTS = {"none": (None, None),
          "failed_at_3": (3, None),
          "missing_ack_at_2": (None, 2)}
ZONES = {"upper_right", "lower_right"}
MAX_RUN_STEPS = 80


def _safe_snapshot():
    if SESSION is None:
        return None
    # Snapshot retains the original untouched Test-session shape. This
    # endpoint does not promote a discrete road node to actual odometry.
    return SESSION.snapshot()


def status():
    with LOCK:
        return {
            "schema": "car_stm32_pty_fsm_lab_v1",
            "source_type": "MOCK",
            "mode": "SIM_ONLY",
            "hardware_authorized": False,
            "transport": "ANONYMOUS_PTY_CARLINK_TEST_ONLY",
            "model_kind": "PYTHON_SIMEXECUTOR_NOT_COMPILED_STM32",
            "endpoint_scope": "TEST_ONLY_DISPLAYED_IN_REAL_PAGE",
            "generation": GENERATION,
            "case": dict(CASE) if CASE is not None else None,
            "snapshot": _safe_snapshot(),
            "age_s": round(time.monotonic() - UPDATED_AT, 2) if UPDATED_AT else None,
        }


def _exact_keys(payload, keys):
    if not isinstance(payload, dict) or set(payload) != keys:
        raise ValueError("Unexpected fields in mock action")


def _start(payload):
    global SESSION, CASE, GENERATION, UPDATED_AT
    _exact_keys(payload, {"action", "start_zone", "task_code",
                          "blocked_roads", "fault"})
    zone, task, roads, fault = (payload["start_zone"], payload["task_code"],
                              payload["blocked_roads"], payload["fault"])
    if zone not in ZONES or fault not in FAULTS:
        raise ValueError("Invalid zone or fault")
    if not isinstance(task, str) or len(task) != 15:
        raise ValueError("Invalid task code")
    parse_task_code(task)
    if not isinstance(roads, list) or len(roads) > 8:
        raise ValueError("Blocked roads must be an array of at most 8")
    if not all(isinstance(x, str) and len(x) <= 8 for x in roads):
        raise ValueError("Invalid blocked road")
    plan = build_mission(zone, roads, provenance="DEMO_ONLY")
    if plan["status"] != "DEMO_ROUTE_ONLY":
        raise ValueError("NO_ROUTE; simulated STM32 will not depart")
    fail_at, drop_ack_at = FAULTS[fault]
    # Construct new PTY before closing the old session. An invalid new
    # request cannot destroy the previously running test.
    next_session = PtyMissionSession(plan, task, fail_at=fail_at,
                                     drop_ack_at=drop_ack_at)
    old = SESSION
    SESSION = next_session
    CASE = {"start_zone": zone, "task_code": task,
            "blocked_roads": sorted(roads), "fault": fault}
    GENERATION += 1
    UPDATED_AT = time.monotonic()
    if old is not None:
        old.close()


def _step(limit):
    global UPDATED_AT
    if SESSION is None:
        raise ValueError("Start a mock session first")
    events = []
    for _ in range(limit):
        if SESSION.state in ("DONE", "FAULT"):
            break
        if SESSION.state != "READY":
            raise ValueError("Unexpected state in mock PTY session")
        events.extend(SESSION.advance())
    UPDATED_AT = time.monotonic()
    return events


def action(payload):
    global SESSION, CASE, GENERATION, UPDATED_AT
    if not isinstance(payload, dict):
        raise ValueError("JSON object expected")
    command = payload.get("action")
    with LOCK:
        if command == "start":
            _start(payload)
        elif command in ("step", "run", "reset"):
            _exact_keys(payload, {"action"})
            if command == "reset":
                if SESSION is not None:
                    SESSION.close()
                    SESSION = None
                    CASE = None
                    GENERATION += 1
                UPDATED_AT = time.monotonic()
            else:
                _step(1 if command == "step" else MAX_RUN_STEPS)
        else:
            raise ValueError("Unknown TEST-only action")
        return status()


def _local(address):
    try:
        return ipaddress.ip_address(address).is_loopback
    except ValueError:
        return False


def handle_get(handler):
    if urlsplit(handler.path).path != LAB_PATH:
        return False
    handler.respond_json(200, status())
    return True


def handle_post(handler):
    if urlsplit(handler.path).path != LAB_PATH:
        return False
    if not _local(handler.client_address[0]):
        handler.respond_json(403, {"error": "Only localhost may control the mock PTY"})
        return True
    origin = handler.headers.get("Origin")
    if origin and urlsplit(origin).netloc.lower() != handler.headers.get("Host", "").lower():
        handler.respond_json(403, {"error": "Cross-origin mock action blocked"})
        return True
    try:
        if handler.headers.get("Content-Type", "").split(";", 1)[0].strip().lower() != "application/json":
            raise ValueError("JSON required")
        size = int(handler.headers.get("Content-Length", "0"))
        if not 0 < size <= 1024:
            raise ValueError("Invalid mock request size")
        payload = json.loads(handler.rfile.read(size),
                             parse_constant=lambda _: (_ for _ in ()).throw(
                                 ValueError("Nonfinite JSON not permitted")))
        result = action(payload)
    except (ValueError, TypeError, KeyError, UnicodeError, PlanError,
            MissionError, OSError) as exc:
        handler.respond_json(400, {"error": str(exc)[:160],
                                   "source_type": "MOCK"})
        return True
    handler.respond_json(200, result)
    return True


def reset_for_tests():
    """Private test teardown: closes only this module's anonymous PTY."""
    global SESSION, CASE, GENERATION, UPDATED_AT
    with LOCK:
        if SESSION is not None:
            SESSION.close()
        SESSION = None
        CASE = None
        GENERATION = 0
        UPDATED_AT = 0.0
