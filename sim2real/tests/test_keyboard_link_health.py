"""Reason-coded health for browser -> localhost -> ROS2 -> MOCK link.

Tests no hardware, no rclpy and no browser. The model may only move when
the local ROS2 observed-message entry exists and deadman remains held.
"""
import io
import json
import pathlib
import sys
import time
import unittest

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
import keyboard_mapping_core as mapping


def intent(seq, keys=("w",), enabled=True):
    return {"action": "keys", "client_seq": seq,
            "keys": list(keys), "speed_mps": .12,
            "turn_rps": .3, "deadman": enabled}


def observed(vx=.12):
    return {
        "source": "local_ros2",
        "topic": "/car/teleop/local_test_twist",
        "stamp_s": time.time(),
        "frame_id": "base_link",
        "vx": vx, "vy": 0.0, "wz": 0.0,
    }


class Handler:
    path = "/api/keyboard/health"
    client_address = ("127.0.0.1", 4421)
    def respond_json(self, code, payload):
        self.code = code
        self.payload = payload


class HealthContractTests(unittest.TestCase):
    def setUp(self):
        self.state = mapping.KeyboardMapping()

    def test_no_keys_indicates_deadman_not_active(self):
        state = self.state.link_health()
        self.assertEqual(state["reason"], "NO_KEYS_OR_TIMEOUT")
        self.assertFalse(state["hardware_authorized"])
        self.assertFalse(state["stm32_tx_enabled"])

    def test_key_without_hold_is_not_movement(self):
        self.state.accept(intent(1, enabled=False))
        self.assertEqual(self.state.link_health()["reason"], "NO_KEYS_OR_TIMEOUT")
        self.state.accept(intent(2, keys=(), enabled=True))
        self.assertEqual(self.state.link_health()["reason"], "NO_KEYS_OR_TIMEOUT")

    def test_keyboard_is_active_but_ros2_topic_offline(self):
        self.state.accept(intent(1))
        health = self.state.link_health()
        self.assertEqual(health["reason"], "ROS2_ECHO_OFFLINE")
        self.assertGreater(health["desired"]["vx"], 0)
        self.assertEqual(health["mock"]["velocity"]["vx"], 0)

    def test_ros2_topic_online_but_received_zero(self):
        self.state.accept(intent(1))
        self.state.observe_ros2(observed(0.0))
        self.assertEqual(self.state.link_health()["reason"], "ROS2_ECHO_ZERO")
        self.assertEqual(self.state.dryrun_snapshot()["input_twist"]["vx"], 0)

    def test_ros2_received_nonzero_then_mock_moves(self):
        self.state.accept(intent(1))
        self.state.observe_ros2(observed(.12))
        first = self.state.dryrun_snapshot()
        self.assertEqual(first["input_provenance"],
                         "OBSERVED_PC_ROS2_LOCAL_TEST_TWIST")
        self.state.digital_twin.last_time = time.monotonic() - .12
        after = self.state.dryrun_snapshot()
        self.assertGreater(after["velocity"]["vx"], 0)
        self.assertLess(after["x_m"], first["x_m"])
        self.assertEqual(self.state.link_health()["reason"], "MOCK_MOVING")

    def test_collision_and_mock_fault_explain_stopped_car(self):
        self.state.accept(intent(1))
        self.state.observe_ros2(observed(.12))
        twin = self.state.digital_twin
        twin.reset("upper_right", [23])
        twin.advance(0., observed_vel(), True)
        for step in range(1, 30):
            result = twin.advance(step*.1, observed_vel(), True)
            if result["state"] == "OBSTACLE_STOP":
                break
        self.assertEqual(result["state"], "OBSTACLE_STOP")
        self.assertEqual(self.state.link_health()["reason"], "MOCK_OBSTACLE_STOP")
        twin.reset("upper_right", [])
        twin.set_fault("motor_stall")
        self.assertEqual(self.state.link_health()["reason"], "MOCK_FAULT_SELECTED")

    def test_local_health_endpoint_fails_closed_and_pi_mode_status(self):
        original = mapping.STATE
        mapping.STATE = self.state
        try:
            h = Handler()
            self.assertTrue(mapping.handle_get(h, lambda: None))
            self.assertEqual(h.code, 200)
            self.assertEqual(h.payload["reason"], "NO_KEYS_OR_TIMEOUT")
            h.client_address = ("192.168.1.14", 3312)
            mapping.handle_get(h, lambda: None)
            self.assertEqual(h.code, 403)
            self.state.accept({"action": "route", "client_seq": 2,
                               "route_mode": "pi_stm32"})
            self.assertEqual(self.state.link_health()["reason"], "PI_ROUTE_SELECTED")
        finally:
            mapping.STATE = original

    def test_browser_panel_exposes_hold_to_drive_and_diagnostics(self):
        web = (ROOT / "web/keyboard_mapping_panel.js").read_text(encoding="utf-8")
        self.assertIn("按住 W 前进（鼠标/触屏）", web)
        self.assertIn("deadmanHeld", web)
        self.assertIn("/health", web)
        self.assertIn("pointercapture", web.lower())
        self.assertIn("API_ERROR", web)


def observed_vel():
    return {"vx": .12, "vy": 0., "wz": 0.}


if __name__ == "__main__":
    unittest.main(verbosity=2)
