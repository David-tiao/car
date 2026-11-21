"""Offline mapping acceptance: keyboard intents, ROS2 handoff, REAL odom errors.

Requires only Python stdlib. Never sends anything to physical serial/motors.
"""
import io
import json
import math
from pathlib import Path
import sys
import time
import unittest

HERE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HERE))
# Grouped modules remain importable under direct unittest discovery.
import pathlib as _sim2real_pathlib
for _module_group in ("keyboard", "bridges", "labs"):
    sys.path.insert(0, str(_sim2real_pathlib.Path(__file__).resolve().parents[1] / _module_group))
import keyboard_mapping_core as mapping
import keyboard_ros2_pc as pc
import keyboard_ros2_pi as pi


class FakeHandler:
    def __init__(self, route, payload=None, address="127.0.0.1", origin=None):
        self.path = route
        self.client_address = (address, 12345)
        body = json.dumps(payload).encode("utf-8") if payload is not None else b""
        self.rfile = io.BytesIO(body)
        self.headers = {"Content-Type": "application/json", "Content-Length": str(len(body)),
                        "Host": "127.0.0.1:8090"}
        if origin is not None:
            self.headers["Origin"] = origin
        self.status = self.result = None

    def respond_json(self, status, data):
        self.status, self.result = status, data


def keys(n, held=True, pressed=None, speed=0.12, turn=0.3):
    return {"action": "keys", "client_seq": n, "keys": pressed or [],
            "speed_mps": speed, "turn_rps": turn, "deadman": held}


def real_sample(seq, vx=0.09, vy=0.0, wz=0.0, origin="REAL:ros2_odom"):
    return {"online": True, "age_s": 0.05, "sample": {
        "source_type": "REAL", "source": origin, "seq": seq,
        "received_at": time.time()+0.1, "frame": "odom",
        "twist_frame": "base_link",
        "x": 1.0+seq*0.01, "y": 1.0, "yaw": 0.0,
        "vx": vx, "vy": vy, "wz": wz}}


class MappingTests(unittest.TestCase):
    def setUp(self):
        self.state = mapping.KeyboardMapping()

    def test_command_signs_diagonal_normalization_and_deadman(self):
        x = self.state.accept(keys(10, pressed=["w", "a", "q"]))
        self.assertTrue(x["active"])
        self.assertAlmostEqual(math.hypot(x["vx"], x["vy"]), 0.12, places=4)
        self.assertGreater(x["vx"], 0)
        self.assertGreater(x["vy"], 0)
        self.assertEqual(x["wz"], 0.3)
        x = self.state.accept(keys(11, pressed=["w", "s", "q", "e"]))
        self.assertEqual((x["vx"], x["vy"], x["wz"]), (0.0, 0.0, 0.0))
        x = self.state.accept(keys(12, held=False, pressed=["w"]))
        self.assertFalse(x["active"])
        self.assertEqual(x["vx"], 0)

    def test_expiry_zero_and_stop_rejects_reordered_keypress(self):
        x = self.state.accept(keys(25, pressed=["w"]))
        later = self.state.effective(self.state.last_command_mono+mapping.TTL_S+0.001)
        self.assertFalse(later["active"])
        self.assertEqual(later["vx"], 0.0)
        self.state.accept({"action": "stop", "client_seq": 27})
        with self.assertRaisesRegex(ValueError, "STALE_KEYBOARD_INTENT"):
            self.state.accept(keys(26, pressed=["w"]))
        self.assertEqual(self.state.effective()["vx"], 0.0)

    def test_bounds_and_unexpected_actuators_rejected(self):
        rejected = [
            keys(1, pressed=["g"]), keys(1, pressed=["W"]),
            keys(1, pressed=["w", "w"]),
            keys(1, speed=2.5, pressed=["w"]),
            keys(1, turn=float("inf"), pressed=["e"]),
            {**keys(1, pressed=["w"]), "actuator": "grab"},
            {"action": "stop", "client_seq": -1},
            {"action": "move_motor", "client_seq": 1},
        ]
        for payload in rejected:
            with self.subTest(payload=payload), self.assertRaises(ValueError):
                self.state.accept(payload)

    def test_real_odom_comparison_no_mock_or_stale(self):
        self.state.accept(keys(100, pressed=["w"]))
        empty = self.state.compare(None)
        self.assertEqual(empty["status"], "NO_REAL_ODOM")
        synthetic = self.state.compare(real_sample(1, origin="TEST:mock"))
        self.assertEqual(synthetic["status"], "NOT_REAL_CHANNEL")
        stale = real_sample(1)
        stale["age_s"] = 5
        self.assertEqual(self.state.compare(stale)["status"], "STALE_ODOM")
        wrong = real_sample(1)
        wrong["sample"]["frame"] = "map"
        self.assertEqual(self.state.compare(wrong)["status"], "WRONG_FRAME")
        bad_twist = real_sample(1)
        bad_twist["sample"]["twist_frame"] = "odom"
        self.assertEqual(self.state.compare(bad_twist)["status"], "WRONG_TWIST_FRAME")
        good = self.state.compare(real_sample(1))
        self.assertEqual(good["status"], "COMPARABLE_ODOM_ONLY")
        self.assertAlmostEqual(good["error"]["vx"], 0.03, places=5)
        self.assertEqual(good["samples"], 1)
        self.assertIn("pose", good)
        self.assertIn("actual_pose", self.state.history[-1])
        self.assertIn("integrated_target_pose", self.state.history[-1])
        dup = self.state.compare(real_sample(1))
        self.assertEqual(dup["samples"], 1)
        second = self.state.compare(real_sample(2, vx=0.06))
        self.assertEqual(second["samples"], 2)
        self.assertGreater(second["rms_linear_mps"], 0)
        self.assertIn("relative_position_error_m", second["error"])

    def test_stop_reports_measured_braking_residual_only_for_short_window(self):
        self.state.accept(keys(1, pressed=["w"]))
        self.state.accept({"action": "stop", "client_seq": 2})
        report = self.state.compare(real_sample(10, vx=0.04))
        self.assertEqual(report["status"], "STOP_SETTLING")
        self.assertAlmostEqual(report["error"]["vx"], -0.04)
        self.assertEqual(report["error"]["linear_norm_mps"], 0.04)
        self.state.stop_settle_until = time.monotonic() - 1
        report = self.state.compare(real_sample(11, vx=0.02))
        self.assertEqual(report["status"], "DEADMAN_NOT_HELD")
        self.assertIsNone(report["error"])

    def test_no_sensor_no_fabricated_measurement_or_error(self):
        self.state.accept(keys(1, pressed=["w"]))
        x = self.state.snapshot({"online": False, "sample": None, "age_s": None})
        self.assertIsNone(x["comparison"]["error"])
        self.assertIsNone(x["comparison"]["measured"])
        self.assertFalse(x["stm32_tx_enabled"])

    def test_http_local_only_and_disallowed_origins(self):
        original = mapping.STATE
        mapping.STATE = mapping.KeyboardMapping()
        try:
            for addr, origin in (("192.168.1.3", None),
                                 ("127.0.0.1", "https://evil.invalid")):
                req = FakeHandler(mapping.PATH+"/intent", keys(5, pressed=["w"]),
                                  addr, origin)
                self.assertTrue(mapping.handle_post(req))
                self.assertEqual(req.status, 403)
            req = FakeHandler(mapping.PATH+"/intent", keys(10, pressed=["w"]))
            self.assertTrue(mapping.handle_post(req))
            self.assertEqual(req.status, 200)
            self.assertTrue(req.result["active"])
            status = FakeHandler(mapping.PATH+"/status")
            self.assertTrue(mapping.handle_get(
                status, lambda: {"online": False, "sample": None, "age_s": None}))
            self.assertEqual(status.status, 200)
            self.assertFalse(status.result["hardware_authorized"])
            old = FakeHandler(mapping.PATH+"/intent", keys(9, pressed=["e"]))
            mapping.handle_post(old)
            self.assertEqual(old.status, 400)
            denied = FakeHandler(mapping.PATH+"/history", address="10.0.0.8")
            mapping.handle_get(denied, lambda: None)
            self.assertEqual(denied.status, 403)
            self.assertFalse(mapping.handle_post(
                FakeHandler("/api/keyboard/command", {"vx": 1})))
        finally:
            mapping.STATE = original

    def test_two_routes_default_to_pc_local_without_pi(self):
        self.assertEqual(self.state.effective()["route_mode"], "local_ros2")
        self.assertEqual(self.state.snapshot(None)["ros2_feedback"]["status"],
                         "WAIT_LOCAL_ROS2")
        self.assertEqual(pc.select_topic(self.state.effective()),
                         "/car/teleop/local_test_twist")
        self.state.accept(keys(10, pressed=["w"]))
        route = self.state.accept({
            "action": "route", "client_seq": 11, "route_mode": "pi_stm32"})
        self.assertEqual(route["route_mode"], "pi_stm32")
        self.assertFalse(route["active"])
        self.assertEqual(route["vx"], 0.0)
        self.assertEqual(pc.select_topic(route), "/car/teleop/desired_twist")
        with self.assertRaisesRegex(ValueError, "STALE_KEYBOARD_INTENT"):
            self.state.accept(keys(10, pressed=["w"]))
        self.assertEqual(self.state.ros2_status()["status"], "WAIT_PI_ROS2")

    def test_ros2_local_echo_is_not_measured_real_vehicle_velocity(self):
        self.state.accept(keys(1, pressed=["w"], speed=.12))
        packet = {"source": "local_ros2",
                  "topic": "/car/teleop/local_test_twist",
                  "stamp_s": time.time(), "frame_id": "base_link",
                  "vx": .12, "vy": 0.0, "wz": 0.0}
        response = self.state.observe_ros2(packet)
        self.assertTrue(response["accepted"])
        status = self.state.snapshot(
            {"online": False, "age_s": None, "sample": None})
        self.assertEqual(status["ros2_feedback"]["status"], "LOCAL_ROS2_ECHO")
        self.assertTrue(status["ros2_feedback"]["online"])
        self.assertEqual(status["ros2_feedback"]["delta"]["vx"], 0.0)
        self.assertIs(status["ros2_feedback"]["is_actual_vehicle_velocity"], False)
        self.assertIsNone(status["comparison"]["measured"])
        self.assertEqual(status["comparison"]["status"], "NO_REAL_ODOM")
        self.assertEqual(len(self.state.ros_history), 1)
        self.assertEqual(self.state.ros_history[0]["delta"]["vx"], 0.0)
        self.assertEqual(self.state.ros2_status(
            time.monotonic() + mapping.ROS_FEEDBACK_TIMEOUT_S + .1
        )["status"], "STALE_ROS2_TOPIC")

    def test_pi_mode_only_sees_pi_candidate_not_local_receipt(self):
        self.state.accept({"action": "route", "client_seq": 1,
                           "route_mode": "pi_stm32"})
        base = {"stamp_s": time.time(), "frame_id": "base_link",
                "vx": 0.05, "vy": 0., "wz": 0.}
        self.state.observe_ros2(
            {**base, "source": "local_ros2",
             "topic": "/car/teleop/local_test_twist"})
        self.assertEqual(self.state.ros2_status()["status"], "WAIT_PI_ROS2")
        self.state.observe_ros2(
            {**base, "source": "pi_stm32",
             "topic": "/car/teleop/stm32_candidate"})
        self.assertEqual(self.state.ros2_status()["status"], "PI_ROS2_CANDIDATE")
        self.assertFalse(self.state.ros2_status()["physical_feedback_available"])
        self.assertFalse(self.state.snapshot(None)["stm32_tx_enabled"])

    def test_ros2_feedback_rejects_stale_wrong_topic_fake_data(self):
        good = {"source": "local_ros2",
                "topic": "/car/teleop/local_test_twist",
                "stamp_s": time.time(), "frame_id": "base_link",
                "vx": 0., "vy": 0., "wz": 0.}
        invalid = [
            {**good, "source": "UNKNOWN"},
            {**good, "topic": "/cmd_vel"},
            {**good, "stamp_s": time.time()-8},
            {**good, "frame_id": "odom"},
            {**good, "vx": 2.0},
            {**good, "wz": float("nan")},
            {**good, "uart_tx": True}
        ]
        for data in invalid:
            with self.subTest(data=data), self.assertRaises(ValueError):
                self.state.observe_ros2(data)

    def test_ros2_feedback_http_is_loopback_only(self):
        original = mapping.STATE
        mapping.STATE = mapping.KeyboardMapping()
        try:
            packet = {"source": "local_ros2",
                      "topic": "/car/teleop/local_test_twist",
                      "stamp_s": time.time(), "frame_id": "base_link",
                      "vx": 0., "vy": 0., "wz": 0.}
            success = FakeHandler(mapping.PATH+"/ros2_feedback", packet)
            self.assertTrue(mapping.handle_post(success))
            self.assertEqual(success.status, 200)
            denied = FakeHandler(mapping.PATH+"/ros2_feedback", packet,
                                 "192.168.10.5")
            mapping.handle_post(denied)
            self.assertEqual(denied.status, 403)
            bad_origin = FakeHandler(mapping.PATH+"/ros2_feedback", packet,
                                     origin="https://untrusted.example")
            mapping.handle_post(bad_origin)
            self.assertEqual(bad_origin.status, 403)
        finally:
            mapping.STATE = original

    def test_ros2_bridging_is_topic_only_without_hardware_sender(self):
        import keyboard_ros2_web_feedback as bridge
        self.assertEqual(bridge.TOPICS["local_ros2"],
                         "/car/teleop/local_test_twist")
        self.assertEqual(bridge.TOPICS["pi_stm32"],
                         "/car/teleop/stm32_candidate")
        validated = bridge.validate_message(
            "local_ros2", "base_link", time.time(), .1, .0, .1)
        self.assertEqual(validated["vx"], .1)
        with self.assertRaises(ValueError):
            bridge.validate_message(
                "local_ros2", "odom", time.time(), .1, .0, .1)

    def test_pc_ros2_publisher_contract_and_pi_guard_limits(self):
        self.assertTrue(pc.is_local_url("http://127.0.0.1:8090"))
        self.assertFalse(pc.is_local_url("http://192.168.0.1:8090"))
        self.assertEqual(pc.parse_snapshot(self.state.effective()),
                         (0.0, 0.0, 0.0))
        cmd = self.state.accept(keys(2, pressed=["w", "q"]))
        self.assertEqual(pc.parse_snapshot(cmd), (0.12, 0, 0.3))
        self.assertEqual(pi.validate_candidate(0.1, 0.0, 0.3, "base_link"),
                         (0.1, 0.0, 0.3))
        for args in ((1.0, 0, 0, "base_link"),
                     (0.1, 0.1, 0, "odom"),
                     (0, 0, float("nan"), "base_link")):
            self.assertIsNone(pi.validate_candidate(*args))


if __name__ == "__main__":
    unittest.main(verbosity=2)
