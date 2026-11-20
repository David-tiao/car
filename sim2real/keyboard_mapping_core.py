"""Stage-5 keyboard mapping set: local intent, REAL odom comparison, no hardware TX.

A browser may *request* a velocity. This never opens a UART or authorizes
motion. The PC ROS2 publisher and Pi ROS2 guarded receiver remain separate.
The only actual data source is the pre-existing REAL /odom HTTP telemetry.
"""
from __future__ import annotations

from collections import deque
import ipaddress
import json
import math
import threading
import time
from urllib.parse import urlsplit
from keyboard_dryrun_chassis import KeyboardChassisTwin, valid_blocked_cells
from keyboard_offline_analysis import valid_packet, delivery_metrics

PATH = "/api/keyboard"
KEYS = frozenset("wasdqe")
MAX_SPEED = 0.25  # m/s, design limit for OFFLINE command candidates
MAX_TURN = 0.5    # rad/s
TTL_S = 0.30
ODOM_MAX_AGE_S = 0.75
ROUTE_MODES = frozenset(("local_ros2", "pi_stm32"))
ROS_FEEDBACK_TIMEOUT_S = 0.75
ROS_TOPICS = {"local_ros2": "/car/teleop/local_test_twist",
              "pi_stm32": "/car/teleop/stm32_candidate"}


def _finite(value, limit):
    return type(value) in (int, float) and math.isfinite(value) and 0 <= value <= limit


def _angle(value):
    return math.atan2(math.sin(value), math.cos(value))


def mapped_twist(keys, speed, turn):
    """Body-frame command (+x forward, +y left, +z CCW). No motor API."""
    x = int("w" in keys) - int("s" in keys)
    y = int("a" in keys) - int("d" in keys)
    w = int("q" in keys) - int("e" in keys)
    norm = math.hypot(x, y)
    return {"vx": round(speed * x / norm, 5) if norm else 0.0,
            "vy": round(speed * y / norm, 5) if norm else 0.0,
            "wz": round(turn * w, 5)}


class KeyboardMapping:
    def __init__(self):
        self.lock = threading.RLock()
        self.generation = 0
        self.seq = 0
        self.last_client_seq = -1
        self.route_mode = "local_ros2"  # Works without Raspberry Pi
        self.ros_receipts = {}
        self.ros_history = deque(maxlen=600)
        self.diag_packets = deque(maxlen=700)
        self.digital_twin = KeyboardChassisTwin()
        self.last_mock_reset_seq = -1
        self.command = {"vx": 0.0, "vy": 0.0, "wz": 0.0}
        self.keys = []
        self.deadman = False
        self.last_command_mono = 0.0
        self.last_command_wall = 0.0
        self.history = deque(maxlen=600)
        self.last_odom_seq = 0
        self.reference_pose = None
        self.last_ref_time = None
        self.last_ref_twist = None
        self.latest_error = None
        self.stop_settle_until = 0.0

    def _live(self, now):
        return self.deadman and self.last_command_mono > 0 and (
            now - self.last_command_mono <= TTL_S)

    def effective(self, now=None):
        now = time.monotonic() if now is None else now
        with self.lock:
            active = self._live(now)
            return {"seq": self.seq, "generation": self.generation,
                    "last_client_seq": self.last_client_seq,
                    "active": active, "deadman": self.deadman if active else False,
                    "keys": self.keys if active else [],
                    "vx": self.command["vx"] if active else 0.0,
                    "vy": self.command["vy"] if active else 0.0,
                    "wz": self.command["wz"] if active else 0.0,
                    "age_ms": int(1000 * (now-self.last_command_mono))
                    if self.last_command_mono else None,
                    "timeout_ms": int(TTL_S * 1000),
                    "frame_id": "base_link",
                    "route_mode": self.route_mode,
                    "hardware_authorized": False,
                    "transport": "PC_ROS2_CANDIDATE_ONLY"}

    def accept(self, payload):
        if not isinstance(payload, dict):
            raise ValueError("Keyboard intent must be an object")
        action = payload.get("action")
        client_seq = payload.get("client_seq")
        if (type(client_seq) is not int or not 0 <= client_seq < 9007199254740991):
            raise ValueError("Monotonic client_seq required")
        if action == "stop":
            if set(payload) != {"action", "client_seq"}:
                raise ValueError("Unexpected stop fields")
            keys, speed, turn, deadman = [], 0.0, 0.0, False
        elif action == "route":
            if set(payload) != {"action", "client_seq", "route_mode"} or (
                    payload["route_mode"] not in ROUTE_MODES):
                raise ValueError("Unknown or malformed keyboard route")
            keys, speed, turn, deadman = [], 0.0, 0.0, False
        elif action == "keys":
            if set(payload) != {"action", "client_seq", "keys", "speed_mps", "turn_rps", "deadman"}:
                raise ValueError("Unexpected keyboard intent fields")
            keys = payload["keys"]
            speed, turn, deadman = (payload["speed_mps"], payload["turn_rps"],
                                     payload["deadman"])
            if not isinstance(keys, list) or len(keys) > 6 or (
                    len(set(map(str, keys))) != len(keys)) or any(
                    type(k) is not str or k not in KEYS for k in keys):
                raise ValueError("Only unique WASDQE keys allowed")
            if not _finite(speed, MAX_SPEED) or not _finite(turn, MAX_TURN):
                raise ValueError("Speed limits exceeded")
            if type(deadman) is not bool:
                raise ValueError("Deadman must be boolean")
            if not deadman:
                keys = []
        else:
            raise ValueError("Only keys/stop are supported")
        with self.lock:
            if client_seq <= self.last_client_seq:
                raise ValueError("STALE_KEYBOARD_INTENT: older than stop/reset")
            self.last_client_seq = client_seq
            if action == "route":
                self.route_mode = payload["route_mode"]
                # A new route is a new experiment. Previous loopback receipts
                # and odom error samples must not appear as fresh route data.
                self.ros_receipts.clear()
                self.ros_history.clear()
                self.diag_packets.clear()
                self.history.clear()
                self.latest_error = None
                self.last_odom_seq = 0
                self.digital_twin.reset(
                    self.digital_twin.zone, self.digital_twin.blocked_cells)
            new_keys = sorted(keys)
            new_twist = mapped_twist(new_keys, speed, turn)
            changed = (new_keys != self.keys or deadman != self.deadman or
                       new_twist != self.command)
            self.keys = new_keys
            self.deadman = deadman
            self.command = new_twist
            self.last_command_mono = time.monotonic()
            # The 10Hz browser heartbeat renews the watchdog, but must NOT
            # invalidate the most recent REAL odom sample on every refresh.
            if changed or not self.last_command_wall or action in ("stop", "route"):
                self.last_command_wall = time.time()
            self.seq += 1
            if action == "stop":
                # Measure residual REAL odom velocity for up to 1.2 s after
                # STOP. This is only diagnostics; no motor command is sent.
                self.stop_settle_until = time.monotonic() + 1.2
                self.generation += 1
            elif action == "route":
                self.stop_settle_until = 0.0
                self.generation += 1
            elif self.deadman:
                self.stop_settle_until = 0.0
            if action in ("stop", "route"):
                self.last_ref_time = None
                self.last_ref_twist = None
                self.reference_pose = None
            return self.effective()

    def observe_ros2(self, payload):
        """Log an observed ROS2 topic; never promote ROS echo to real odometry."""
        if not isinstance(payload, dict) or set(payload) != {
                "source", "topic", "stamp_s", "frame_id", "vx", "vy", "wz"}:
            raise ValueError("Invalid ROS2 observed-message schema")
        source = payload["source"]
        if type(source) is not str or source not in ROS_TOPICS or (
                payload["topic"] != ROS_TOPICS[source] or
                payload["frame_id"] != "base_link"):
            raise ValueError("Unknown ROS2 feedback topic/frame")
        if not all(type(payload[k]) in (int, float) and math.isfinite(payload[k])
                   for k in ("vx", "vy", "wz", "stamp_s")):
            raise ValueError("Invalid ROS2 feedback values")
        if (math.hypot(payload["vx"], payload["vy"]) > MAX_SPEED + 1e-5 or
                abs(payload["wz"]) > MAX_TURN + 1e-5):
            raise ValueError("ROS2 feedback exceeds candidate limits")
        wall = time.time()
        # The transport duration is approximate wall-time latency, not MCU ACK.
        lag = wall - payload["stamp_s"]
        if not -0.15 <= lag <= 2.0:
            raise ValueError("ROS2 feedback stamp stale or clock mismatch")
        record = {"source": source, "topic": payload["topic"],
                  "frame_id": "base_link",
                  "received_wall_s": wall,
                  "stamp_s": float(payload["stamp_s"]),
                  "received_mono": time.monotonic(),
                  "estimated_ros2_latency_ms": round(max(0.0, lag) * 1000, 2),
                  "twist": {k: round(float(payload[k]), 5)
                            for k in ("vx", "vy", "wz")}}
        with self.lock:
            command = self.effective()
            record["desired_at_receive"] = {k: command[k] for k in ("vx", "vy", "wz")}
            record["delta"] = {k: round(command[k] - record["twist"][k], 5)
                               for k in ("vx", "vy", "wz")}
            record["delta"]["linear_norm_mps"] = round(math.hypot(
                record["delta"]["vx"], record["delta"]["vy"]), 5)
            self.ros_receipts[source] = record
            self.ros_history.append(record)
        return {"accepted": True, "source": source,
                "hardware_authorized": False, "stm32_tx_enabled": False}

    def observe_diagnostic_packet(self, payload):
        """Record a publisher-numbered ROS2 heartbeat, not a motor receipt."""
        packet = dict(valid_packet(payload))
        # Currently only PC-origin seq diagnostics are instrumented; Pi
        # candidate speed echoes are checked separately and not counted.
        if packet["channel"] != "pc_local_ros2":
            raise ValueError("Pi packet sequence diagnostics not connected")
        with self.lock:
            if self.route_mode != "local_ros2":
                raise ValueError("Local diagnostics only in PC route")
            self.diag_packets.append(packet)
        return {"accepted": True, "source_type": "PC_ROS2_DIAG_HEARTBEAT_ONLY",
                "hardware_authorized": False}

    def network_diagnostics(self, include_history=False):
        with self.lock:
            selected = self.route_mode
            packets = list(self.diag_packets) if selected == "local_ros2" else []
        series = delivery_metrics(packets) if packets else []
        return {
            "schema": "car_keyboard_ros2_packet_diagnostic_v1",
            "source_type": "PC_ROS2_DIAG_HEARTBEAT_ONLY",
            "hardware_authorized": False, "stm32_tx_enabled": False,
            "mode": selected, "connected": bool(series),
            "groups": series,
            "packet_history": packets if include_history else None,
            "limitations": ("Diagnostics are a separate numbered ROS2 String "
                            "heartbeat, not TwistStamped/motor/MCU ACK. "
                            "Sequence gaps count only the observed span."),
        }

    def ros2_status(self, now=None):
        now = time.monotonic() if now is None else now
        with self.lock:
            selected = self.route_mode
            rec = self.ros_receipts.get(selected)
            if not rec:
                return {"status": ("WAIT_LOCAL_ROS2" if selected == "local_ros2"
                                   else "WAIT_PI_ROS2"), "online": False,
                        "source": selected, "observed": None, "delta": None}
            age_s = now - rec["received_mono"]
            if age_s > ROS_FEEDBACK_TIMEOUT_S:
                return {"status": "STALE_ROS2_TOPIC", "online": False,
                        "source": selected, "age_ms": round(age_s * 1000),
                        "observed": None, "delta": None}
            observed = rec["twist"]
            command = self.effective(now)
            # A mirrored ROS2 TwistStamped is only proof of topic delivery.
            # Do NOT label it as a motor measurement or speed tracking error.
            diff = {k: round(command[k] - observed[k], 5)
                    for k in ("vx", "vy", "wz")}
            return {"status": ("LOCAL_ROS2_ECHO" if selected == "local_ros2"
                               else "PI_ROS2_CANDIDATE"),
                    "online": True, "source": selected,
                    "topic": rec["topic"],
                    "age_ms": round(age_s * 1000),
                    "approx_ros2_latency_ms": rec["estimated_ros2_latency_ms"],
                    "observed": dict(observed), "delta": diff,
                    "is_actual_vehicle_velocity": False,
                    "physical_feedback_available": False}

    def dryrun_snapshot(self):
        """Advance ONLY the mock chassis with fresh RECEIVED local ROS2 feedback."""
        with self.lock:
            now = time.monotonic()
            source = self.ros2_status(now)
            # Explicitly require a fresh local ROS2 subscriber receipt;
            # keyboard HTTP acceptance alone never drives this mock.
            command = self.effective(now)
            can_simulate = (self.route_mode == "local_ros2" and
                            source.get("online") is True and
                            source.get("source") == "local_ros2" and
                            command["active"])
            twist = source["observed"] if can_simulate else None
            result = self.digital_twin.advance(
                now, twist, accepted_ros2=can_simulate)
            result["input_provenance"] = (
                "OBSERVED_PC_ROS2_LOCAL_TEST_TWIST" if can_simulate else
                "ZERO_NO_FRESH_LOCAL_ROS2_OR_DEADMAN")
            result["real_feedback_is_separate"] = True
            return result

    def dryrun_action(self, payload):
        if not isinstance(payload, dict):
            raise ValueError("Invalid mock simulation action")
        with self.lock:
            action = payload.get("action")
            if action == "reset":
                keys = set(payload)
                if keys not in ({"action", "start_zone", "blocked_cells"},
                                {"action", "start_zone", "blocked_cells",
                                 "reset_seq"}):
                    raise ValueError("Invalid mock reset payload")
                # The UI includes a monotonic reset token. If two scene
                # refreshes race over HTTP, the old one must not put the
                # simulated car back on an obsolete map.
                seq = payload.get("reset_seq")
                if seq is not None and (
                        type(seq) is not int or seq <= self.last_mock_reset_seq
                        or seq > 9007199254740991):
                    raise ValueError("STALE_MOCK_RESET")
                cells = valid_blocked_cells(payload["blocked_cells"])
                zone = payload["start_zone"]
                if zone not in ("upper_right", "lower_right"):
                    raise ValueError("Unknown mock start zone")
                if seq is not None:
                    self.last_mock_reset_seq = seq
                return self.digital_twin.reset(zone, cells)
            if action == "fault":
                if set(payload) != {"action", "fault_mode"}:
                    raise ValueError("Invalid mock fault payload")
                return self.digital_twin.set_fault(payload["fault_mode"])
            raise ValueError("Only mock reset/fault actions are available")

    def link_health(self):
        """Explain where a local keyboard->ROS2->MOCK run is stopped.

        Source-typed inspection only. Reading this diagnostic never advances
        the mock, alters keyboard intent, or sends any motor output.
        """
        with self.lock:
            command = self.effective()
            feedback = self.ros2_status()
            model = self.digital_twin.snapshot()
            if self.route_mode != "local_ros2":
                reason, hint = ("PI_ROUTE_SELECTED",
                                "请选择本机 ROS2 回环；当前没有树莓派")
            elif not command["active"]:
                reason, hint = ("DEADMAN_NOT_ACTIVE",
                                "勾选键盘使能并持续按住 Shift+W；也可按住本机测试按钮")
            elif not any(abs(command[k]) > 0.00001 for k in ("vx", "vy", "wz")):
                reason, hint = ("NO_MOVEMENT_KEYS",
                                "Shift 已使能，但没有按住 W/A/S/D/Q/E")
            elif feedback.get("online") is not True:
                reason, hint = ("ROS2_ECHO_OFFLINE",
                                "期望速度已产生；检查电脑 ROS2 双节点和 /car/teleop/local_test_twist 回传")
            elif not any(abs(feedback["observed"][k]) > 0.00001
                         for k in ("vx", "vy", "wz")):
                reason, hint = ("ROS2_ECHO_ZERO",
                                "ROS2 话题已接收，但速度仍为零；检查按键发送、时间戳与刷新")
            elif model["state"] == "OBSTACLE_STOP":
                reason, hint = ("MOCK_OBSTACLE_STOP",
                                "模拟底盘撞到禁行格/固定区；请复位模拟底盘并检查红色障碍")
            elif model["fault_mode"] != "none":
                reason, hint = ("MOCK_FAULT_SELECTED",
                                "当前启用了模拟故障：" + model["fault_mode"])
            elif model["state"] == "RUNNING":
                reason, hint = ("MOCK_MOVING",
                                "ROS2 已接收，模拟车已经计算位移；若图上没动请检查紫色 MOCK 图层")
            else:
                reason, hint = ("WAIT_MOCK_POLL",
                                "ROS2 已收到非零速度；检查 /api/keyboard/dryrun 定时刷新")
            return {
                "schema": "car_keyboard_link_health_v1",
                "source_type": "LOCAL_ROS2_AND_MOCK_DIAGNOSTIC_ONLY",
                "hardware_authorized": False,
                "stm32_tx_enabled": False,
                "reason": reason, "hint": hint,
                "route_mode": self.route_mode,
                "desired": command,
                "ros2_feedback": feedback,
                "mock": {k: model[k] for k in
                         ("state", "fault_mode", "collision_reason", "x_m",
                          "y_m", "velocity", "input_twist", "samples",
                          "generation")},
            }

    def compare(self, row, now=None):
        """Compare only when REAL odom is fresh and received AFTER intent.

        Relative integrated reference is *not* truth/map localization.
        """
        now = time.monotonic() if now is None else now
        command = self.effective(now)
        with self.lock:
            if not row or not row.get("online") or not row.get("sample"):
                return {"status": "NO_REAL_ODOM", "error": None, "measured": None}
            sample = row["sample"]
            age_s = row.get("age_s")
            if age_s is None or not math.isfinite(age_s) or age_s > ODOM_MAX_AGE_S:
                return {"status": "STALE_ODOM", "error": None, "measured": None}
            if not str(sample.get("source", "")).startswith("REAL:") or (
                    sample.get("source_type") != "REAL"):
                return {"status": "NOT_REAL_CHANNEL", "error": None, "measured": None}
            if str(sample.get("frame")) != "odom":
                return {"status": "WRONG_FRAME", "error": None, "measured": None}
            if str(sample.get("twist_frame")) != "base_link":
                return {"status": "WRONG_TWIST_FRAME", "error": None,
                        "measured": None}
            vals = [sample.get(k) for k in ("x", "y", "yaw", "vx", "vy", "wz")]
            if not all(type(v) in (float, int) and math.isfinite(v) for v in vals):
                return {"status": "INVALID_ODOM", "error": None, "measured": None}
            measured = {k: round(float(sample[k]), 5) for k in ("vx", "vy", "wz")}
            pose = {k: round(float(sample[k]), 5) for k in ("x", "y", "yaw")}
            if sample.get("received_at", 0) < self.last_command_wall:
                return {"status": "WAITING_FOR_NEW_ODOM", "error": None,
                        "measured": measured, "pose": pose}
            settling = (not command["active"] and
                        self.stop_settle_until > now)
            if not command["active"] and not settling:
                return {"status": "DEADMAN_NOT_HELD", "error": None,
                        "measured": measured, "pose": pose}
            seq = sample.get("seq")
            if type(seq) is not int:
                return {"status": "INVALID_ODOM", "error": None, "measured": None}
            # Only append once per actual odom observation; browser polling
            # must not inflate the RMS or make duplicates look like new data.
            if seq != self.last_odom_seq:
                self.last_odom_seq = seq
                err = {k: round(command[k]-measured[k], 5)
                       for k in ("vx", "vy", "wz")}
                err["linear_norm_mps"] = round(math.hypot(err["vx"], err["vy"]), 5)
                reference_error = None
                stamp = float(sample.get("received_at"))
                if self.reference_pose is None:
                    self.reference_pose = [float(sample["x"]), float(sample["y"]),
                                           float(sample["yaw"])]
                elif (self.last_ref_time is not None and
                      0 <= stamp-self.last_ref_time <= 0.8 and
                      self.last_ref_twist is not None):
                    dt = stamp-self.last_ref_time
                    x, y, yaw = self.reference_pose
                    previous = self.last_ref_twist
                    x += (math.cos(yaw)*previous["vx"] -
                          math.sin(yaw)*previous["vy"])*dt
                    y += (math.sin(yaw)*previous["vx"] +
                          math.cos(yaw)*previous["vy"])*dt
                    yaw = _angle(yaw+previous["wz"]*dt)
                    self.reference_pose = [x, y, yaw]
                if self.reference_pose:
                    reference_error = {
                        "relative_position_error_m": round(math.hypot(
                            self.reference_pose[0]-sample["x"],
                            self.reference_pose[1]-sample["y"]), 5),
                        "relative_yaw_error_rad": round(_angle(
                            self.reference_pose[2]-sample["yaw"]), 5)}
                self.last_ref_time = stamp
                self.last_ref_twist = dict(command)
                self.latest_error = {**err, **(reference_error or {})}
                self.history.append({
                    "time_s": round(stamp, 3), "odom_seq": seq,
                    "command": {k: command[k] for k in ("vx", "vy", "wz")},
                    "actual": measured, "actual_pose": pose,
                    "integrated_target_pose": {
                        "x": round(self.reference_pose[0], 5),
                        "y": round(self.reference_pose[1], 5),
                        "yaw": round(self.reference_pose[2], 5)
                    } if self.reference_pose is not None else None,
                    "error": dict(self.latest_error),
                    "sample_source": sample["source"], "frame": sample["frame"]})
            errors = [e["error"]["linear_norm_mps"] for e in self.history]
            rms = math.sqrt(sum(v*v for v in errors)/len(errors)) if errors else None
            return {"status": ("STOP_SETTLING" if settling else
                                "COMPARABLE_ODOM_ONLY"),
                    "measured": measured, "pose": pose,
                    "error": self.latest_error,
                    "rms_linear_mps": round(rms, 5) if rms is not None else None,
                    "samples": len(errors), "odom_age_s": round(age_s, 3),
                    "warning": "Relative commanded integration is not absolute localization; motor TX remains disabled."}

    def snapshot(self, real_row):
        command = self.effective()
        return {"schema": "car_keyboard_mapping_v1",
                "source_type": "KEYBOARD_DESIRED_PLUS_REAL_ODOM",
                "hardware_authorized": False,
                "stm32_tx_enabled": False,
                "pc_ros_topic": "/car/teleop/desired_twist",
                "pi_candidate_topic": "/car/teleop/stm32_candidate",
                "route_mode": self.route_mode,
                "desired": command,
                "ros2_feedback": self.ros2_status(),
                "comparison": self.compare(real_row)}

    def reset(self):
        self.__init__()


STATE = KeyboardMapping()


def _local(ip):
    try:
        return ipaddress.ip_address(ip).is_loopback
    except ValueError:
        return False


def handle_get(handler, real_reader):
    path = urlsplit(handler.path).path
    if path not in (PATH + "/status", PATH + "/intent", PATH + "/history",
                    PATH + "/dryrun", PATH + "/mock_acceptance",
                    PATH + "/diagnostics", PATH + "/diag_history",
                    PATH + "/health"):
        return False
    if not _local(handler.client_address[0]):
        handler.respond_json(403, {"error": "Keyboard diagnostics restricted to localhost"})
        return True
    if path.endswith("/intent"):
        handler.respond_json(200, STATE.effective())
    elif path.endswith("/dryrun"):
        handler.respond_json(200, STATE.dryrun_snapshot())
    elif path.endswith("/mock_acceptance"):
        from keyboard_fault_lab import acceptance
        handler.respond_json(200, acceptance())
    elif path.endswith("/health"):
        handler.respond_json(200, STATE.link_health())
    elif path.endswith("/diagnostics"):
        handler.respond_json(200, STATE.network_diagnostics())
    elif path.endswith("/diag_history"):
        handler.respond_json(200, STATE.network_diagnostics(include_history=True))
    elif path.endswith("/history"):
        with STATE.lock:
            handler.respond_json(200, {"schema": "car_keyboard_error_history_v1",
                                       "hardware_authorized": False,
                                       "samples": list(STATE.history),
                                       "ros2_samples": [
                                           {k: v for k, v in record.items()
                                            if k != "received_mono"}
                                           for record in STATE.ros_history]})
    else:
        handler.respond_json(200, STATE.snapshot(real_reader()))
    return True


def handle_post(handler):
    path = urlsplit(handler.path).path
    if path not in (PATH + "/intent", PATH + "/ros2_feedback",
                    PATH + "/dryrun", PATH + "/diagnostic_packet"):
        return False
    if not _local(handler.client_address[0]):
        handler.respond_json(403, {"error": "Keyboard requires localhost"})
        return True
    origin = handler.headers.get("Origin")
    if origin and urlsplit(origin).netloc.lower() != handler.headers.get("Host", "").lower():
        handler.respond_json(403, {"error": "Cross-origin command rejected"})
        return True
    try:
        if handler.headers.get("Content-Type", "").split(";", 1)[0].strip().lower() != "application/json":
            raise ValueError("JSON required")
        size = int(handler.headers.get("Content-Length", "0"))
        if not 0 < size <= 512:
            raise ValueError("Invalid keyboard request size")
        payload = json.loads(handler.rfile.read(size),
                             parse_constant=lambda _: (_ for _ in ()).throw(ValueError("Nonfinite")))
        if path.endswith("/ros2_feedback"):
            result = STATE.observe_ros2(payload)
        elif path.endswith("/diagnostic_packet"):
            result = STATE.observe_diagnostic_packet(payload)
        elif path.endswith("/dryrun"):
            result = STATE.dryrun_action(payload)
        else:
            result = STATE.accept(payload)
    except (TypeError, ValueError, UnicodeError) as exc:
        handler.respond_json(400, {"error": str(exc)[:160]})
        return True
    handler.respond_json(200, result)
    return True
