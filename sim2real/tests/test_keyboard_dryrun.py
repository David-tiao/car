"""Pure software mecanum mock / mapping API fault matrix.

No ROS2 runtime, serial port, Raspberry Pi or motor can be accessed here.
"""
import io
import json
import math
from pathlib import Path
import sys
import time
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
# Grouped modules remain importable under direct unittest discovery.
import pathlib as _sim2real_pathlib
for _module_group in ("keyboard", "bridges", "labs"):
    sys.path.insert(0, str(_sim2real_pathlib.Path(__file__).resolve().parents[1] / _module_group))
from keyboard_dryrun_chassis import (
    KeyboardChassisTwin, cell_box, valid_blocked_cells, FOOTPRINT_RADIUS_M)
import keyboard_mapping_core as mapping


class StubRequest:
    def __init__(self, path, payload=None, client="127.0.0.1"):
        self.path = path
        self.client_address = (client, 12345)
        self.headers = {
            "Content-Type": "application/json",
            "Host": "127.0.0.1:8090",
        }
        body = json.dumps(payload).encode("utf-8") if payload else b""
        self.headers["Content-Length"] = str(len(body))
        self.rfile = io.BytesIO(body)
        self.code = None
        self.data = None

    def respond_json(self, code, data):
        self.code, self.data = code, data


DRIVE = {"vx": .2, "vy": 0., "wz": 0.}
STOP = {"vx": 0., "vy": 0., "wz": 0.}


class DigitalTwinTests(unittest.TestCase):
    def test_snake_cells_and_validation(self):
        self.assertEqual(cell_box(17), (.96, 1.44, 1.44, 1.92))
        self.assertEqual(cell_box(23), (1.44, 1.92, 1.92, 2.4))
        self.assertEqual(valid_blocked_cells([17, 3]), [3, 17])
        for bad in (([3, 3]), ([-1]), ([25]), ([True]),
                    ([1, 2, 3]), ["17"], None):
            with self.subTest(bad=bad), self.assertRaises(ValueError):
                valid_blocked_cells(bad)

    def test_initial_pose_and_provenance_never_real_odom(self):
        twin = KeyboardChassisTwin()
        row = twin.snapshot()
        self.assertEqual((row["x_m"], row["y_m"]), (2.16, 2.16))
        self.assertEqual(row["source_type"], "MOCK_KINEMATIC_ONLY")
        self.assertFalse(row["real_odom"])
        self.assertFalse(row["hardware_authorized"])
        self.assertTrue(row["ros2_transport_required"])
        twin.reset("lower_right", [])
        self.assertEqual(twin.snapshot()["y_m"], .24)

    def test_w_accelerates_and_local_deadman_timeout_coasts_to_stop(self):
        twin = KeyboardChassisTwin()
        twin.advance(0.0, DRIVE, True)
        slow = twin.advance(.1, DRIVE, True)
        self.assertGreater(slow["velocity"]["vx"], 0.0)
        self.assertLess(slow["velocity"]["vx"], .2)
        prev_x = slow["x_m"]
        for step in range(2, 8):
            last = twin.advance(step*.1, DRIVE, True)
        self.assertLess(last["x_m"], prev_x)  # initial yaw=pi => W moves west
        for step in range(8, 25):
            last = twin.advance(step*.1, DRIVE, False)
        self.assertEqual(last["velocity"]["vx"], 0.0)
        self.assertEqual(last["input_twist"]["vx"], 0.0)
        self.assertNotEqual(last["source_type"], "REAL")

    def test_red_cell_causes_latched_collision_and_no_teleport(self):
        twin = KeyboardChassisTwin()
        twin.reset("upper_right", [23])
        twin.advance(0.0, DRIVE, True)
        last = None
        for i in range(1, 20):
            last = twin.advance(i*.1, DRIVE, True)
            if last["state"] == "OBSTACLE_STOP":
                break
        self.assertEqual(last["state"], "OBSTACLE_STOP")
        self.assertEqual(last["collision_reason"], "BLOCKED_CELL")
        self.assertGreaterEqual(last["x_m"], 1.92+FOOTPRINT_RADIUS_M-.002)
        stopped_at = (last["x_m"], last["y_m"])
        after = twin.advance(4, DRIVE, True)
        self.assertEqual((after["x_m"], after["y_m"]), stopped_at)
        self.assertEqual(after["velocity"], STOP)
        self.assertEqual(twin.reset("upper_right", [])["state"], "READY")

    def test_motor_stall_and_link_loss_show_nonzero_model_error(self):
        for mode in ("motor_stall", "link_loss"):
            with self.subTest(mode=mode):
                twin = KeyboardChassisTwin()
                twin.set_fault(mode)
                twin.advance(0., DRIVE, True)
                row = twin.advance(.2, DRIVE, True)
                self.assertEqual(row["velocity"]["vx"], 0.)
                self.assertEqual(row["input_twist"]["vx"], .2)
                self.assertAlmostEqual(row["model_error"]["linear_norm_mps"], .2)
                self.assertEqual(row["error_kind"], "MOCK_COMMANDED_VS_MOCK_KINEMATIC")

    def test_response_delay_is_visible_without_faking_odom(self):
        normal = KeyboardChassisTwin()
        slow = KeyboardChassisTwin()
        slow.set_fault("slow_response")
        for obj in (normal, slow):
            obj.advance(0, DRIVE, True)
        n = normal.advance(.15, DRIVE, True)
        s = slow.advance(.15, DRIVE, True)
        self.assertGreater(n["velocity"]["vx"], s["velocity"]["vx"])
        self.assertGreater(s["model_error"]["vx"], n["model_error"]["vx"])

    def test_deterministic_same_inputs_and_bounded_dt(self):
        first, second = KeyboardChassisTwin(), KeyboardChassisTwin()
        for i in range(14):
            f = first.advance(i*.08, DRIVE, True)
            s = second.advance(i*.08, DRIVE, True)
        self.assertEqual(f["trace"], s["trace"])
        self.assertEqual(f["velocity"], s["velocity"])
        self.assertGreater(f["sim_time_s"], 0)
        before = first.run_seconds
        first.advance(1000, STOP, False)
        self.assertLessEqual(first.run_seconds-before, .301)

    def test_wrong_simulator_actions_rejected(self):
        core = mapping.KeyboardMapping()
        for req in (
            {"action": "reset", "start_zone": "upper_right",
             "blocked_cells": [0, 1, 2]},
            {"action": "fault", "fault_mode": "physical_uart"},
            {"action": "reset", "start_zone": "unknown",
             "blocked_cells": []},
            {"action": "cmd_vel", "vx": .4},
        ):
            with self.subTest(req=req), self.assertRaises(ValueError):
                core.dryrun_action(req)

    def test_core_needs_subscribed_ros2_feedback_not_just_keypress(self):
        core = mapping.KeyboardMapping()
        core.accept({"action": "keys", "client_seq": 10, "deadman": True,
                     "keys": ["w"], "speed_mps": .12, "turn_rps": .2})
        a = core.dryrun_snapshot()
        self.assertEqual(a["input_provenance"], "ZERO_NO_FRESH_LOCAL_ROS2_OR_DEADMAN")
        self.assertEqual(a["input_twist"]["vx"], 0.0)
        core.observe_ros2({
            "source": "local_ros2", "topic": "/car/teleop/local_test_twist",
            "stamp_s": time.time(), "frame_id": "base_link",
            "vx": .12, "vy": 0., "wz": 0.})
        after = core.dryrun_snapshot()
        self.assertEqual(after["input_provenance"],
                         "OBSERVED_PC_ROS2_LOCAL_TEST_TWIST")
        self.assertFalse(after["hardware_authorized"])
        core.accept({"action": "route", "client_seq": 11,
                     "route_mode": "pi_stm32"})
        disabled = core.dryrun_snapshot()
        self.assertEqual(disabled["input_provenance"],
                         "ZERO_NO_FRESH_LOCAL_ROS2_OR_DEADMAN")

    def test_mock_http_localhost_and_real_channel_isolation(self):
        original = mapping.STATE
        mapping.STATE = mapping.KeyboardMapping()
        try:
            reset = {"action": "reset", "start_zone": "lower_right",
                     "blocked_cells": [17]}
            h = StubRequest("/api/keyboard/dryrun", reset)
            self.assertTrue(mapping.handle_post(h))
            self.assertEqual(h.code, 200)
            self.assertEqual(h.data["blocked_cells"], [])
            h = StubRequest("/api/keyboard/dryrun")
            self.assertTrue(mapping.handle_get(h, lambda: None))
            self.assertEqual(h.data["source_type"], "MOCK_KINEMATIC_ONLY")
            self.assertFalse(h.data["real_odom"])
            unauthorized = StubRequest("/api/keyboard/dryrun", reset, "192.168.1.10")
            mapping.handle_post(unauthorized)
            self.assertEqual(unauthorized.code, 403)
            self.assertFalse(mapping.handle_post(
                StubRequest("/api/real/velocity", {"vx": .1})))
        finally:
            mapping.STATE = original

    def test_ui_file_displays_both_sim_and_real_provenance(self):
        ui = (ROOT / "web/keyboard_mapping_panel.js").read_text(encoding="utf-8")
        self.assertIn("MOCK_KINEMATIC_ONLY", ui)
        self.assertIn("field_preview_m_not_odom", ui)
        self.assertIn("preview_mode!=='direct'", ui)
        self.assertNotIn("sim2real:field-changed", ui)
        self.assertNotIn("motor_stall", ui)
        self.assertNotIn("link_loss", ui)
        self.assertIn("keyboardMockChassisLayer", ui)


if __name__ == "__main__":
    unittest.main(verbosity=2)
