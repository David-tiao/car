"""Offline tests: browser-triggered ROS2 keyboard loopback stays local and single-owned."""
from __future__ import annotations

import sys
from pathlib import Path
from types import SimpleNamespace
import tempfile
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
# Grouped modules remain importable under direct unittest discovery.
import pathlib as _sim2real_pathlib
for _module_group in ("keyboard", "bridges", "labs"):
    sys.path.insert(0, str(_sim2real_pathlib.Path(__file__).resolve().parents[1] / _module_group))
import keyboard_local_ros2_autostart as auto


class FakeProcess:
    def __init__(self):
        self.exit_code = None
        self.stopped = False
    def poll(self):
        return self.exit_code
    def terminate(self):
        self.stopped = True
        self.exit_code = 0
    def kill(self):
        self.stopped = True
        self.exit_code = -9
    def wait(self, timeout=None):
        return self.exit_code


class FakeHandler:
    def __init__(self, path="/keyboard", client="127.0.0.1", **headers):
        self.path = path
        self.client_address = (client, 43210)
        self.server = SimpleNamespace(server_port=8090, keyboard_autostart_enabled=True)
        self.headers = headers
        self.response = None
    def respond_json(self, code, body):
        self.response = (code, body)


class AutoStartTests(unittest.TestCase):
    def setUp(self):
        temp = tempfile.TemporaryDirectory()
        self.addCleanup(temp.cleanup)
        folder = Path(temp.name)
        setup = folder / "setup.bash"
        script = folder / "start_keyboard_local_ros2.sh"
        setup.write_text("# pretend ROS2\n", encoding="utf-8")
        script.write_text("# pretend launcher\n", encoding="utf-8")
        for name, value in [("SETUP", setup), ("SCRIPT", script), ("LOG", folder / "start.log")]:
            patcher = patch.object(auto, name, value)
            patcher.start()
            self.addCleanup(patcher.stop)
        patcher = patch.dict(auto.os.environ, {"CAR_KEYBOARD_AUTOSTART":"1"})
        patcher.start()
        self.addCleanup(patcher.stop)
        auto._PROCESS = None
        auto._ATTEMPT_AT = 0.0
        auto._ERROR = ""
        self.addCleanup(auto.stop)

    def test_once_for_repeated_local_page_visits(self):
        fake = FakeProcess()
        with patch.object(auto.subprocess, "Popen", return_value=fake) as started:
            self.assertTrue(auto.ensure_started(8090))
            self.assertFalse(auto.ensure_started(8090))
            self.assertEqual(started.call_count, 1)
            argv = started.call_args.args[0]
            self.assertIn("--worker", argv)
            self.assertIn("--parent-pid", argv)
            self.assertIn("8090", argv)
            self.assertTrue(auto.status()["running"])
            self.assertFalse(auto.status()["motor_authorized"])
            auto.stop()
            self.assertTrue(fake.stopped)

    def test_existing_ros2_feedback_does_not_duplicate_publishers(self):
        with patch.object(auto.subprocess, "Popen") as started:
            self.assertFalse(auto.ensure_started(8090, existing_feedback=True))
            started.assert_not_called()

    def test_only_local_browser_visit_triggers_process(self):
        fake = FakeProcess()
        with patch.object(auto.subprocess, "Popen", return_value=fake) as started:
            forbidden = [
                FakeHandler(client="192.168.1.9"),
                FakeHandler(**{"Sec-Fetch-Site":"cross-site"}),
                FakeHandler(**{"Sec-Purpose":"prefetch"}),
                FakeHandler(Origin="https://remote.invalid",Host="127.0.0.1:8090"),
            ]
            for handler in forbidden:
                self.assertFalse(auto.handle_get(handler))
            started.assert_not_called()
            self.assertFalse(auto.handle_get(FakeHandler()))
            self.assertEqual(started.call_count, 1)

    def test_api_status_never_starts_subscriber(self):
        with patch.object(auto.subprocess, "Popen") as started:
            handler = FakeHandler(path="/api/keyboard/autostart")
            self.assertTrue(auto.handle_get(handler))
            self.assertEqual(handler.response[0], 200)
            started.assert_not_called()

    def test_disabled_flag_and_missing_setup_leave_no_process(self):
        with patch.dict(auto.os.environ, {"CAR_KEYBOARD_AUTOSTART":"0"}):
            self.assertFalse(auto.ensure_started(8090))
        with patch.object(auto, "SETUP", Path("/nonexistent/no-ros-setup")):
            self.assertFalse(auto.ensure_started(8090))
            self.assertIn("missing", auto.status()["message"])
            self.assertFalse(auto.status()["running"])


if __name__ == "__main__":
    unittest.main()
