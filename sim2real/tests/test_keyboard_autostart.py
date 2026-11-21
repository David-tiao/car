"""Offline checks: a localhost /keyboard visit owns at most one ROS2 observer.

Tests never start ROS2 or control an actual car.
"""
from __future__ import annotations

from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
# Grouped modules remain importable under direct unittest discovery.
import pathlib as _sim2real_pathlib
for _module_group in ("keyboard", "bridges", "labs"):
    sys.path.insert(0, str(_sim2real_pathlib.Path(__file__).resolve().parents[1] / _module_group))
import keyboard_feedback_autostart as auto


class FakeProcess:
    def __init__(self):
        self.exit_code = None
        self.terminated = False

    def poll(self):
        return self.exit_code

    def terminate(self):
        self.terminated = True
        self.exit_code = 0

    def kill(self):
        self.exit_code = -9

    def wait(self, timeout=None):
        return self.exit_code


class AutoStartTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        folder = Path(self.temp.name)
        self.setup = folder / 'setup.bash'
        self.setup.write_text('# mock ROS setup\n', encoding='utf-8')
        self.bridge = folder / 'observer.py'
        self.bridge.write_text('# mock observer\n', encoding='utf-8')
        self.patches = [
            patch.object(auto, 'ROS_SETUP', self.setup),
            patch.object(auto, 'WS_SETUP', folder / 'workspace.bash'),
            patch.object(auto, 'BRIDGE', self.bridge),
            patch.object(auto, 'LOG_PATH', folder / 'auto.log'),
        ]
        for item in self.patches:
            item.start()
        auto._PROCESS = None
        auto._LAST_ATTEMPT = 0.0
        auto._ERROR = ''

    def tearDown(self):
        auto.stop()
        for item in reversed(self.patches):
            item.stop()
        self.temp.cleanup()

    def test_first_visit_starts_one_subscription_and_refresh_reuses_it(self):
        fake = FakeProcess()
        with patch.object(auto.subprocess, 'Popen', return_value=fake) as popen:
            self.assertTrue(auto.ensure_started(8080))
            self.assertFalse(auto.ensure_started(8080))
            self.assertEqual(popen.call_count, 1)
        argv = popen.call_args.args[0]
        self.assertIn(str(self.bridge), argv)  # Patched path is the injected test observer
        self.assertIn('--parent-pid', argv)
        self.assertIn('http://127.0.0.1:8080', argv)
        self.assertTrue(auto.status()['running'])
        fake.exit_code = 1
        self.assertIn('exited (code 1)', auto.status()['message'])
        fake.exit_code = None
        auto.stop()
        self.assertTrue(fake.terminated)

    def test_existing_bridge_prevents_duplicate_start(self):
        with patch.object(auto.subprocess, 'Popen') as popen:
            self.assertFalse(auto.ensure_started(8080, existing_feedback=True))
            popen.assert_not_called()

    def test_missing_ros2_setup_reports_error_without_spawning(self):
        with patch.object(auto, 'ROS_SETUP', self.setup.parent / 'absent.bash'):
            with patch.object(auto.subprocess, 'Popen') as popen:
                self.assertFalse(auto.ensure_started(8080))
                self.assertIn('ROS2 Humble not found', auto.status()['message'])
                popen.assert_not_called()


if __name__ == '__main__':
    unittest.main()
