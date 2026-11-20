"""Regression tests for TEST-only V4 QR/STM32 contract model and HTTP isolation."""
import io
import json
from pathlib import Path
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "sim2real"))
import test_v4_qr_lab as lab


class StubHTTP:
    def __init__(self, path, payload=None, address="127.0.0.1", origin=None):
        self.path = path
        self.client_address = (address, 45100)
        content = json.dumps(payload).encode("utf-8") if payload is not None else b""
        self.rfile = io.BytesIO(content)
        self.headers = {"Content-Type": "application/json",
                        "Content-Length": str(len(content)), "Host": "127.0.0.1:8080"}
        if origin is not None:
            self.headers["Origin"] = origin
        self.result = None

    def respond_json(self, status, value):
        self.result = (status, value)


class Lab(unittest.TestCase):
    def test_frame_reference_and_corruption(self):
        self.assertEqual(lab.frame(1, "ALIGN,2,7"), "@1,9,ALIGN,2,7*40\n")
        self.assertEqual(lab.decode(lab.frame(23, "JOB,1,23")), (23, "JOB,1,23"))
        with self.assertRaises(ValueError):
            lab.decode(lab.frame(4, "JOB,1,23").replace("JOB", "J0B"))
        for invalid in ("junk", "@1,9,ALIGN,2,7*00\n", "@1,5,JOB,1,23*00\n"):
            with self.subTest(invalid=invalid), self.assertRaises(ValueError):
                lab.decode(invalid)

    def test_scenarios_require_expected_terminal_state(self):
        for case in lab.SCENARIOS:
            with self.subTest(case=case):
                r = lab.run_case(case)
                self.assertTrue(r["passed"], r)
                self.assertFalse(r["hardware_authorized"])
                self.assertEqual(r["model_kind"], "CONTRACT_MODEL_NOT_STM32_FIRMWARE")
                if case in ("success", "duplicate_seq"):
                    self.assertEqual((r["state"], r["accepted_qr"]), ("DONE", 1))
                elif case == "cancel":
                    self.assertEqual(r["state"], "CANCELLED")
                else:
                    self.assertEqual((r["state"], r["accepted_qr"]), ("FAULT", 0))

    def test_qr_needs_job_ack_and_matched_session(self):
        experiment = lab.QrExperiment()
        experiment.begin()
        self.assertFalse(experiment.receive("QR,156+123+516+231,23"))
        self.assertEqual(experiment.accepted_qr, 0)
        self.assertFalse(experiment.ack_qr())
        self.assertTrue(experiment.receive("ACK,JOB,1"))
        self.assertFalse(experiment.receive("QR,156+123+516+231,22"))
        self.assertTrue(experiment.receive("QR,156+123+516+231,23"))
        self.assertEqual(experiment.accepted_qr, 1)
        self.assertTrue(experiment.ack_qr())
        self.assertEqual(experiment.state, "WAIT_DONE")
        self.assertTrue(experiment.receive("DONE,QR"))
        self.assertTrue(experiment.receive("IDLE"))
        self.assertEqual(experiment.state, "DONE")

    def test_invalid_tasks_and_duplicate_frame(self):
        for code in ("156+112+516+231", "156+123+516", "156+123+624+231",
                     "156+123+516+234", ""):
            self.assertFalse(lab.valid_task(code))
        x = lab.QrExperiment()
        x.begin()
        self.assertTrue(x.receive("ACK,JOB,1"))
        seq = x.rx_seq
        self.assertTrue(x.receive("QR,156+123+516+231,23"))
        self.assertFalse(x.receive("QR,156+123+516+231,23", seq=seq))
        self.assertEqual(x.accepted_qr, 1)

    def test_http_rejects_real_or_cross_origin(self):
        path = "/api/test/v4_qr_lab"
        for address, origin in (("192.168.1.3", None),
                                ("127.0.0.1", "https://evil.invalid")):
            h = StubHTTP(path, {"scenario": "success"}, address, origin)
            self.assertTrue(lab.handle_post(h))
            self.assertEqual(h.result[0], 403)

    def test_http_accepts_only_predefined_scenarios(self):
        path = "/api/test/v4_qr_lab"
        for payload in ({"action": "send_motor"}, {"scenario": "MOVE"},
                        {"scenario": "success", "hardware_authorized": True}):
            h = StubHTTP(path, payload)
            self.assertTrue(lab.handle_post(h))
            self.assertEqual(h.result[0], 400)
        good = StubHTTP(path, {"scenario": "success"})
        self.assertTrue(lab.handle_post(good))
        self.assertEqual(good.result[0], 200)
        self.assertEqual(good.result[1]["state"], "DONE")
        get = StubHTTP(path)
        self.assertTrue(lab.handle_get(get))
        self.assertEqual(get.result[0], 200)
        self.assertFalse(get.result[1]["hardware_authorized"])

    def test_web_lab_not_available_on_real(self):
        self.assertFalse(lab.handle_get(StubHTTP("/api/real/v4_qr_lab")))
        self.assertFalse(lab.handle_post(StubHTTP("/api/real/v4_qr_lab", {"scenario":"success"})))


if __name__ == "__main__":
    unittest.main(verbosity=2)
