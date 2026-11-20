"""Recovery regressions for live Shift/pointer deadman and scene-reset races.

Offline only, verifies strict server TTL, stale mock reset rejection, MOCK
source separation, and existing /keyboard UI contracts. No ROS2 or motors.
"""
from pathlib import Path
import sys
import time
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
import keyboard_mapping_core as mapping


def keyboard(seq, vx=.12):
    return {"action": "keys", "client_seq": seq, "keys": ["w"],
            "speed_mps": vx, "turn_rps": .3, "deadman": True}


class KeyboardRecoveryTests(unittest.TestCase):
    def setUp(self):
        self.state = mapping.KeyboardMapping()

    def test_mouse_hold_repeated_heartbeat_renews_deadman_ttl(self):
        first = self.state.accept(keyboard(100))
        self.assertTrue(first["active"])
        # Simulated elapsed time beyond 300ms means stopped, even if UI's
        # key/button looks held. This is the essential fail-safe condition.
        self.state.last_command_mono -= 0.5
        self.assertFalse(self.state.effective()["active"])
        self.assertEqual(self.state.effective()["vx"], 0.)
        # The browser MUST renew keys while held; once a new heartbeat
        # arrives the desired velocity can resume without restarting ROS2.
        second = self.state.accept(keyboard(101))
        self.assertTrue(second["active"])
        self.assertEqual(second["vx"], .12)
        self.assertGreater(second["seq"], first["seq"])

    def test_out_of_order_mock_map_reset_is_rejected(self):
        a = self.state.dryrun_action({
            "action": "reset", "start_zone": "upper_right",
            "blocked_cells": [], "reset_seq": 1001})
        self.assertEqual(a["zone"], "upper_right")
        b = self.state.dryrun_action({
            "action": "reset", "start_zone": "lower_right",
            "blocked_cells": [17], "reset_seq": 1003})
        self.assertEqual(b["zone"], "lower_right")
        self.assertEqual(b["blocked_cells"], [17])
        before = self.state.digital_twin.snapshot()
        with self.assertRaisesRegex(ValueError, "STALE_MOCK_RESET"):
            self.state.dryrun_action({
                "action": "reset", "start_zone": "upper_right",
                "blocked_cells": [], "reset_seq": 1002})
        self.assertEqual(self.state.digital_twin.snapshot(), before)

    def test_invalid_reset_does_not_consume_sequence(self):
        with self.assertRaises(ValueError):
            self.state.dryrun_action({
                "action": "reset", "start_zone": "UNKNOWN",
                "blocked_cells": [], "reset_seq": 800})
        ok = self.state.dryrun_action({
            "action": "reset", "start_zone": "upper_right",
            "blocked_cells": [], "reset_seq": 800})
        self.assertEqual(ok["zone"], "upper_right")
        with self.assertRaisesRegex(ValueError, "STALE_MOCK_RESET"):
            self.state.dryrun_action({
                "action": "reset", "start_zone": "upper_right",
                "blocked_cells": [], "reset_seq": 800})

    def test_mock_map_reset_cannot_be_mistaken_for_stm32_motion(self):
        self.state.accept(keyboard(1))
        saved = self.state.digital_twin.snapshot()
        assert saved["source_type"] == "MOCK_KINEMATIC_ONLY"
        self.state.dryrun_action({
            "action": "reset", "start_zone": "upper_right",
            "blocked_cells": [], "reset_seq": 1})
        self.assertTrue(self.state.effective()["active"])
        self.assertEqual(self.state.snapshot(
            {"online": False, "sample": None, "age_s": None})["comparison"]["status"],
            "NO_REAL_ODOM")
        self.assertFalse(self.state.digital_twin.snapshot()["hardware_authorized"])

    def test_ui_includes_pointer_deadman_in_periodic_heartbeat(self):
        ui = (ROOT / "web/keyboard_mapping_panel.js").read_text("utf-8")
        self.assertIn("if(deadmanHeld()&&active.size>0&&!document.hidden)sendNow()", ui)
        self.assertIn("const deadmanHeld=()=>enabled.checked&&(shift||holdTest)", ui)
        self.assertIn("payload.reset_seq=nextSeq()", ui)
        self.assertIn("signature===lastMapSignature", ui)
        self.assertIn("requestSeq<=refreshAppliedSeq", ui)
        self.assertIn("refreshAppliedSeq=++refreshRequestSeq", ui)
        self.assertIn("请松开再重新按住 Shift+W", ui)

    def test_competition_robot_is_hidden_in_keyboard_mode(self):
        source = (ROOT / "web/app.js").read_text("utf-8")
        self.assertIn("if(window.CarWebMode?.mode==='keyboard')return;", source)
        ui = (ROOT / "web/keyboard_mapping_panel.js").read_text("utf-8")
        self.assertIn("keyboardMockChassisLayer", ui)


if __name__ == "__main__":
    unittest.main(verbosity=2)
