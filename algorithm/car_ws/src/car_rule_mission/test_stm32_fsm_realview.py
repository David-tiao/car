#!/usr/bin/env python3
"""End-to-end offline TEST lab tests: real anonymous PTY + MOCK STM32 feedback."""
import io
import json
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(ROOT / "sim2real"))
import test_stm32_fsm_lab as lab
import web_mode_gateway


class HTTP:
    def __init__(self, path, payload=None, client="127.0.0.1", origin=None):
        self.path = path
        self.client_address = (client, 53100)
        body = json.dumps(payload).encode("utf-8") if payload is not None else b""
        self.rfile = io.BytesIO(body)
        self.headers = {"Content-Type": "application/json",
                        "Content-Length": str(len(body)),
                        "Host": "127.0.0.1:8090"}
        if origin:
            self.headers["Origin"] = origin
        self.response = None

    def respond_json(self, status, payload):
        self.response = (status, payload)


def start(**changes):
    data = {"action": "start", "start_zone": "upper_right",
            "task_code": "156+123+516+231", "blocked_roads": [],
            "fault": "none"}
    data.update(changes)
    return data


class MockSTM32RealViewTests(unittest.TestCase):
    def setUp(self):
        lab.reset_for_tests()

    def tearDown(self):
        lab.reset_for_tests()

    def test_reused_original_pty_simulator_executes_all_steps(self):
        response = lab.action(start())
        self.assertEqual(response["transport"], "ANONYMOUS_PTY_CARLINK_TEST_ONLY")
        self.assertEqual(response["model_kind"], "PYTHON_SIMEXECUTOR_NOT_COMPILED_STM32")
        self.assertFalse(response["hardware_authorized"])
        self.assertIsNotNone(lab.SESSION)
        self.assertGreaterEqual(lab.SESSION.master, 0)
        self.assertGreaterEqual(lab.SESSION.slave, 0)
        result = lab.action({"action": "run"})
        s = result["snapshot"]
        self.assertEqual(s["state"], "DONE", s["error"])
        self.assertEqual((s["pick_count"], s["place_count"], s["stack_count"]),
                         (12, 12, 3))
        self.assertTrue(s["home_confirmed"])
        self.assertEqual(s["completed_steps"], s["total_steps"])
        self.assertEqual(s["mock_executions"], s["total_steps"])
        self.assertEqual(s["pose_source"], "DISCRETE_ROAD_NODE")

    def test_one_step_and_subsequent_run(self):
        first = lab.action(start(start_zone="lower_right",
                                 blocked_roads=["B3-C3"]))
        self.assertEqual(first["snapshot"]["state"], "READY")
        second = lab.action({"action": "step"})["snapshot"]
        self.assertEqual(second["completed_steps"], 1)
        self.assertEqual(second["state"], "READY")
        third = lab.action({"action": "run"})["snapshot"]
        self.assertEqual(third["state"], "DONE")

    def test_actuator_failure_latches_and_blocks_future_step(self):
        lab.action(start(fault="failed_at_3"))
        error = lab.action({"action": "run"})["snapshot"]
        self.assertEqual(error["state"], "FAULT")
        self.assertEqual(error["mock_executions"], 3)
        self.assertEqual(error["completed_steps"], 2)
        again = lab.action({"action": "step"})["snapshot"]
        self.assertEqual(again["mock_executions"], 3)

    def test_ack_dropout_latches_and_does_not_repeat_command(self):
        lab.action(start(fault="missing_ack_at_2"))
        failed = lab.action({"action": "run"})["snapshot"]
        self.assertEqual(failed["state"], "FAULT")
        self.assertIn("TIMEOUT", failed["error"])
        self.assertEqual(failed["mock_executions"], 1)

    def test_bad_start_does_not_reset_existing_pty(self):
        lab.action(start())
        original = lab.SESSION
        for payload in (start(task_code="156+112+516+231"),
                        start(start_zone="invalid"),
                        start(blocked_roads=["A1-A2", "A1-B1", "A2-B2", "A2-A3",
                                             "B1-B2", "B2-B3", "C1-C2", "C2-C3", "X9-Z9"]),
                        start(fault="real_uart")):
            with self.subTest(payload=payload), self.assertRaises(ValueError):
                lab.action(payload)
            self.assertIs(lab.SESSION, original)

    def test_http_test_only_not_real_and_localhost_post(self):
        p = lab.LAB_PATH
        self.assertFalse(lab.handle_get(HTTP("/api/real/stm32_fsm_lab")))
        self.assertFalse(lab.handle_post(HTTP("/api/real/stm32_fsm_lab", start())))
        for client, origin in (("10.8.0.3", None),
                               ("127.0.0.1", "http://evil.invalid")):
            with self.subTest(client=client,origin=origin):
                req = HTTP(p, start(), client=client, origin=origin)
                self.assertTrue(lab.handle_post(req))
                self.assertEqual(req.response[0], 403)
        req = HTTP(p, start())
        self.assertTrue(lab.handle_post(req))
        self.assertEqual(req.response[0], 200)
        self.assertFalse(req.response[1]["hardware_authorized"])
        get = HTTP(p)
        self.assertTrue(lab.handle_get(get))
        self.assertEqual(get.response[0], 200)

    def test_reject_fake_real_and_command_injection(self):
        lab.action(start())
        for payload in ({"action": "move"}, {"action": "step", "cmd_vel": 1},
                        {"action": "start", "source_type": "REAL"},
                        {"action": "reset", "hardware_authorized": True}):
            req = HTTP(lab.LAB_PATH, payload)
            self.assertTrue(lab.handle_post(req))
            self.assertEqual(req.response[0], 400)
        self.assertEqual(lab.status()["snapshot"]["state"], "READY")

    def test_separate_real_channel_is_not_mutated(self):
        before = web_mode_gateway.CHANNELS["real"]["ros"]["seq"]
        lab.action(start())
        lab.action({"action": "run"})
        after = web_mode_gateway.CHANNELS["real"]["ros"]["seq"]
        self.assertEqual(before, after)
        self.assertIsNone(web_mode_gateway.CHANNELS["real"]["ros"]["snapshot"])
        self.assertFalse(lab.status()["hardware_authorized"])

    def test_reset_closes_pty_descriptors(self):
        lab.action(start())
        old = lab.SESSION
        lab.action({"action": "reset"})
        self.assertEqual((old.master, old.slave), (-1, -1))
        self.assertIsNone(lab.status()["snapshot"])


if __name__ == "__main__":
    unittest.main(verbosity=2)
