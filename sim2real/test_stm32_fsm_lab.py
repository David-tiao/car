"""TEST-only STM32 FSM workbench: preserved CAR-LINK PTY or real C code host gate.

The native mode builds *actual* STM32 RaceTaskPlan/RaceTaskGate sources in a
disposable shared library. Neither mode uses a real UART, ROS, /cmd_vel or motors.
The web page plays topology-node animation, not physical odometry.
"""
from __future__ import annotations

import ctypes
import ipaddress
import json
import shutil
import subprocess
import sys
import tempfile
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
from road_topology import NODES, shortest

LOCK = threading.RLock()
SESSION = None
CASE = None
GENERATION = 0
UPDATED_AT = 0.0
LAST_TRANSITION = None
FAULTS = {"none": (None, None),
          "failed_at_3": (3, None),
          "missing_ack_at_2": (None, 2)}
ZONES = {"upper_right", "lower_right"}
ENGINES = {"pty", "code_fsm"}
MAX_RUN_STEPS = 80
TASK_KINDS = ("PICK", "GO_ROUGH", "PLACE_ROUGH", "RETRIEVE_ROUGH",
              "GO_STORAGE", "PLACE_STORAGE", "GO_MATERIAL",
              "STACK_STORAGE", "RETURN_START")
TASK_GATE_STATES = ("READY", "WAIT_ACK", "WAIT_RESULT", "DONE", "FAULT")
TASK_GATE_ERRORS = ("NONE", "REJECTED", "FAILED", "ACK_TIMEOUT",
                    "RESULT_TIMEOUT", "CANCELLED", "PLAN")


def node_position(name):
    if name not in NODES:
        raise ValueError("Unknown node in simulated topology")
    return [round(float(v), 4) for v in NODES[name]]


class HostTaskEvent(ctypes.Structure):
    _fields_ = [
        ("cmd_id", ctypes.c_uint32),
        ("kind", ctypes.c_uint8), ("batch", ctypes.c_uint8),
        ("item", ctypes.c_uint8), ("color", ctypes.c_uint8),
        ("position", ctypes.c_uint8), ("state", ctypes.c_uint8),
        ("error", ctypes.c_uint8), ("completed", ctypes.c_uint8),
        ("hardware_authorized", ctypes.c_uint8),
    ]


class CTaskSession:
    """Isolated, GCC-compiled STM32 C task gate; no physical control path."""

    def __init__(self, plan, task, fault):
        if shutil.which("gcc") is None:
            raise ValueError("C 状态机模式需要主机安装 gcc")
        self.tmp = tempfile.TemporaryDirectory(prefix="car_stm32_cfsm_")
        try:
            so = Path(self.tmp.name) / "libstm32_task_gate_host.so"
            base = ROOT / "STM32F4"
            cmd = ["gcc", "-shared", "-fPIC", "-O2", "-std=c99",
                   "-Wall", "-Wextra", "-Werror", "-I" + str(base / "App"),
                   str(base / "App/race_task_plan.c"),
                   str(base / "App/race_task_gate.c"),
                   str(base / "tests/race/task_gate_host_adapter.c"),
                   "-o", str(so)]
            subprocess.run(cmd, check=True, capture_output=True, text=True,
                           timeout=25)
            self.lib = ctypes.CDLL(str(so))
            self.lib.HostTask_Init.argtypes = [
                ctypes.POINTER(ctypes.c_uint8), ctypes.POINTER(ctypes.c_uint8)]
            self.lib.HostTask_Init.restype = ctypes.c_int
            self.lib.HostTask_Step.argtypes = [
                ctypes.c_uint32, ctypes.c_int, ctypes.POINTER(HostTaskEvent)]
            self.lib.HostTask_Step.restype = ctypes.c_int
            for method in ("HostTask_State", "HostTask_Completed",
                           "HostTask_PlanIndex", "HostTask_PlanCount"):
                getattr(self.lib, method).restype = ctypes.c_int
            task_parts = parse_task_code(task)
            colors = (ctypes.c_uint8 * 6)(
                *(task_parts.batch1_colors + task_parts.batch2_colors))
            rings = (ctypes.c_uint8 * 6)(
                *(task_parts.batch1_rough_rings + task_parts.batch2_rough_rings))
            self.total = self.lib.HostTask_Init(colors, rings)
            if self.total != 34:
                raise ValueError("C 任务表生成失败，预期 34 个动作")
        except Exception:
            self.tmp.cleanup()
            raise

        self.fault = fault
        self.plan = plan
        self.start_node = "B3"  # RaceTaskPlan begins after QR at material area
        self.node = self.start_node
        self.position = tuple(NODES[self.node])
        self.events = []
        self.last = {"cmd_id": 0, "kind": "READY", "event": "READY"}
        self.pick_count = self.place_count = self.stack_count = 0
        self.home_confirmed = False
        self.now_ms = 0
        self.error = ""

    @property
    def state(self):
        return TASK_GATE_STATES[self.lib.HostTask_State()]

    def advance(self):
        if self.state != "READY":
            return []
        before = self.node
        event = HostTaskEvent()
        command_number = self.lib.HostTask_Completed() + 1
        fault_code = (
            2 if self.fault == "failed_at_3" and command_number == 3 else
            1 if self.fault == "missing_ack_at_2" and command_number == 2 else 0
        )
        if not self.lib.HostTask_Step(self.now_ms, fault_code, ctypes.byref(event)):
            raise ValueError("C FSM refused task transition")
        if event.hardware_authorized:
            raise ValueError("C test gate illegally authorized hardware")
        kind = TASK_KINDS[event.kind]
        success = self.state != "FAULT"
        trail = [before]
        if success:
            goal = {
                "GO_ROUGH": "B1", "GO_STORAGE": "A2",
                "GO_MATERIAL": "B3",
                "RETURN_START": self.plan["waypoints"][0],
            }.get(kind)
            if goal is not None:
                trail = shortest(before, goal, set(self.plan["blocked_roads"]))
                if not trail:
                    raise ValueError("C task requires a blocked route")
                self.node = goal
                self.position = tuple(NODES[goal])
            if kind in ("PICK", "RETRIEVE_ROUGH"):
                self.pick_count += 1
            if kind in ("PLACE_ROUGH", "PLACE_STORAGE", "STACK_STORAGE"):
                self.place_count += 1
            if kind == "STACK_STORAGE":
                self.stack_count += 1
            if kind == "RETURN_START":
                self.home_confirmed = True
        else:
            self.error = TASK_GATE_ERRORS[event.error]
        result = {"cmd_id": int(event.cmd_id),
                  "event": "COMPLETED" if success else "FAULT",
                  "reason": self.error}
        self.events.append(result)
        self.last = {"cmd_id": int(event.cmd_id), "kind": kind,
                     "event": result["event"], "batch": int(event.batch) + 1,
                     "color_id": int(event.color), "ring_id": int(event.position)}
        self.now_ms += 31000
        return trail

    def snapshot(self):
        return {
            "schema": "car_c_race_task_gate_v1",
            "source_type": "MOCK", "mode": "SIM_ONLY",
            "hardware_authorized": False,
            "state": self.state, "error": self.error,
            "completed_steps": self.lib.HostTask_Completed(),
            "total_steps": self.total,
            "pick_count": self.pick_count,
            "place_count": self.place_count,
            "stack_count": self.stack_count,
            "home_confirmed": self.home_confirmed,
            "cmd_id": self.last["cmd_id"], "kind": self.last["kind"],
            "event": self.last["event"], "stage_index": 0,
            "node": self.node, "x_m": round(self.position[0], 4),
            "y_m": round(self.position[1], 4),
            "pose_source": "DISCRETE_ROAD_NODE",
            "mock_executions": self.lib.HostTask_Completed(),
            "events": self.events[-16:],
        }

    def close(self):
        self.tmp.cleanup()


def _safe_snapshot():
    return None if SESSION is None else SESSION.snapshot()


def status():
    with LOCK:
        engine = CASE["engine"] if CASE else None
        return {
            "schema": "car_stm32_pty_fsm_lab_v2",
            "source_type": "MOCK", "mode": "SIM_ONLY",
            "hardware_authorized": False,
            "engine": engine,
            "transport": ("ANONYMOUS_PTY_CARLINK_TEST_ONLY" if engine != "code_fsm"
                          else "NATIVE_GCC_C_FSM_NO_UART"),
            "model_kind": ("PYTHON_SIMEXECUTOR_NOT_COMPILED_STM32"
                           if engine != "code_fsm" else
                           "COMPILED_STM32_RACETASKPLAN_AND_GATE_HOST_ONLY"),
            "endpoint_scope": "TEST_ONLY",
            "generation": GENERATION,
            "case": dict(CASE) if CASE is not None else None,
            "snapshot": _safe_snapshot(),
            "transition": LAST_TRANSITION,
            "age_s": round(time.monotonic() - UPDATED_AT, 2) if UPDATED_AT else None,
        }


def _exact_keys(payload, keys):
    if not isinstance(payload, dict) or set(payload) != keys:
        raise ValueError("Unexpected fields in mock action")


def _start(payload):
    global SESSION, CASE, GENERATION, UPDATED_AT, LAST_TRANSITION
    allowed = {"action", "start_zone", "task_code", "blocked_roads", "fault"}
    keys = set(payload) if isinstance(payload, dict) else set()
    if keys not in (allowed, allowed | {"engine"}):
        raise ValueError("Unexpected fields in mock start")
    engine = payload.get("engine", "pty")
    zone, task, roads, fault = (payload["start_zone"], payload["task_code"],
                              payload["blocked_roads"], payload["fault"])
    if zone not in ZONES or fault not in FAULTS or engine not in ENGINES:
        raise ValueError("Invalid zone, engine or fault")
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
    if engine == "pty":
        fail_at, drop_ack_at = FAULTS[fault]
        new_session = PtyMissionSession(plan, task, fail_at=fail_at,
                                         drop_ack_at=drop_ack_at)
    else:
        new_session = CTaskSession(plan, task, fault)
    old = SESSION
    SESSION = new_session
    CASE = {"start_zone": zone, "task_code": task,
            "blocked_roads": sorted(roads), "fault": fault, "engine": engine}
    GENERATION += 1
    UPDATED_AT = time.monotonic()
    LAST_TRANSITION = {
        "cmd_id": 0, "kind": "START",
        "from_node": new_session.node, "to_node": new_session.node,
        "node_path": [new_session.node],
        "points": [node_position(new_session.node)],
        "executed": False,
    }
    if old is not None:
        old.close()


def _step(limit):
    global UPDATED_AT, LAST_TRANSITION
    if SESSION is None:
        raise ValueError("Start a mock session first")
    for _ in range(limit):
        if SESSION.state in ("DONE", "FAULT"):
            break
        if SESSION.state != "READY":
            raise ValueError("Unexpected state in mock STM32 session")
        before = SESSION.node
        if CASE["engine"] == "pty":
            SESSION.advance()
            trail = [before, SESSION.node] if SESSION.node != before else [before]
        else:
            trail = SESSION.advance()
        state = SESSION.snapshot()
        # A FAILED movement must never animate to a new node.
        if state["state"] == "FAULT":
            trail = [before]
        LAST_TRANSITION = {
            "cmd_id": state["cmd_id"], "kind": state["kind"],
            "from_node": before, "to_node": SESSION.node,
            "node_path": trail, "points": [node_position(n) for n in trail],
            "executed": (state["state"] != "FAULT"),
        }
    UPDATED_AT = time.monotonic()


def action(payload):
    global SESSION, CASE, GENERATION, UPDATED_AT, LAST_TRANSITION
    if not isinstance(payload, dict):
        raise ValueError("JSON object expected")
    with LOCK:
        command = payload.get("action")
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
                LAST_TRANSITION = None
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
        handler.respond_json(403, {"error": "Only localhost may control the mock STM32"})
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
            MissionError, OSError, subprocess.CalledProcessError,
            subprocess.TimeoutExpired) as exc:
        handler.respond_json(400, {"error": str(exc)[:160],
                                   "source_type": "MOCK"})
        return True
    handler.respond_json(200, result)
    return True


def reset_for_tests():
    global SESSION, CASE, GENERATION, UPDATED_AT, LAST_TRANSITION
    with LOCK:
        if SESSION is not None:
            SESSION.close()
        SESSION = None
        CASE = None
        GENERATION = 0
        UPDATED_AT = 0.0
        LAST_TRANSITION = None
