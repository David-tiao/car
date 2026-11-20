"""Off-hardware test for CLI end-to-end diagnostic's STOP and refusal guards.

The live smoke script is executed by the user on real PC ROS2. Here a fake
HTTP server is injected to verify it cannot skip the STOP safety command.
"""
import contextlib
import io
from pathlib import Path
import sys
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import keyboard_local_smoke as smoke


class LocalSmokeTests(unittest.TestCase):
    def test_mock_transport_completes_and_always_stops(self):
        requests = []
        clock = [0.0]
        def fake_now():
            clock[0] += .14
            return clock[0]
        def request(url, payload=None):
            if url.endswith("/health"):
                return {
                    "route_mode": "local_ros2",
                    "hardware_authorized": False, "stm32_tx_enabled": False,
                    "desired": {"last_client_seq": -1},
                    "mock": {"x_m": 2.16},
                    "reason": "MOCK_MOVING",
                    "ros2_feedback": {"online": True},
                }
            if url.endswith("/intent"):
                requests.append(payload)
                return {"ok": True}
            if url.endswith("/dryrun"):
                return {"x_m": 2.10, "state": "RUNNING",
                        "collision_reason": ""}
            raise AssertionError(url)
        with patch.object(smoke, "json_request", side_effect=request), \
             patch.object(smoke.time, "monotonic", side_effect=fake_now), \
             patch.object(smoke.time, "sleep", return_value=None), \
             contextlib.redirect_stdout(io.StringIO()), \
             contextlib.redirect_stderr(io.StringIO()):
            code = smoke.main(["--seconds", "1"])
        self.assertEqual(code, 0)
        self.assertGreaterEqual(len(requests), 2)
        self.assertTrue(all(x["action"] == "keys" for x in requests[:-1]))
        self.assertEqual(requests[-1]["action"], "stop")

    def test_web_not_running_fails_before_sending_motion(self):
        with patch.object(smoke, "json_request",
                          side_effect=ConnectionRefusedError("offline")):
            with self.assertRaises(ConnectionRefusedError):
                smoke.main(["--seconds", "1"])

    def test_future_pi_route_is_refused_and_never_sends(self):
        seen = []
        def request(url, payload=None):
            seen.append((url, payload))
            return {"route_mode": "pi_stm32",
                    "hardware_authorized": False,
                    "stm32_tx_enabled": False}
        with patch.object(smoke, "json_request", side_effect=request):
            with self.assertRaises(SystemExit):
                smoke.main(["--seconds", "1"])
        self.assertEqual(len(seen), 1)
        self.assertIsNone(seen[0][1])

    def test_nonlocal_urls_rejected(self):
        with self.assertRaises(SystemExit):
            smoke.main(["--server", "http://192.168.1.25:8090"])


if __name__ == "__main__":
    unittest.main(verbosity=2)
