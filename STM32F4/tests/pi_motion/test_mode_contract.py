"""Compile-time mode isolation; no real HAL or hardware is involved."""
from pathlib import Path
import subprocess
import unittest

CONFIG = Path(__file__).resolve().parents[2] / 'Config'


class ModeContractTests(unittest.TestCase):
    def check_mode(self, mode, enable, accepted):
        flags = [f'-D{k}={v}' for k, v in (
            ('APP_RUN_MODE', mode), ('PI_MOTION_ENABLE', enable)) if v is not None]
        result = subprocess.run(['cc', '-E', '-x', 'c', '-I' + str(CONFIG), *flags, '-'],
                                input=b'#include "app_mode.h"\n', capture_output=True)
        self.assertEqual(result.returncode == 0, accepted, result.stderr.decode())

    def test_default(self): self.check_mode(None, None, True)
    def test_legacy(self): self.check_mode(0, 0, True)
    def test_servo(self): self.check_mode(1, 0, True)
    def test_stepper(self): self.check_mode(2, 0, True)
    def test_race(self): self.check_mode(3, 0, True)
    def test_pi_explicit(self): self.check_mode(4, 1, True)
    def test_pi_disabled(self): self.check_mode(4, 0, False)
    def test_race_pi_conflict(self): self.check_mode(3, 1, False)
    def test_invalid_enable(self): self.check_mode(4, 2, False)
    def test_invalid_mode(self): self.check_mode(5, 0, False)


if __name__ == '__main__':
    unittest.main()
