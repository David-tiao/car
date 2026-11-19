#!/usr/bin/env python3
"""Pure Python tests: stage-3 incremental PTY replay, no ROS2 or real UART."""
import unittest

from mission_test_session import PtyMissionSession
from rule_mission_core import build_mission

CODE = '156+123+516+231'


class TestSession(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.plan = build_mission('upper_right', ['B3-C3'], provenance='DEMO_ONLY')

    def make(self, **kw):
        return PtyMissionSession(self.plan, CODE, **kw)

    def test_finish_and_track_only_road_nodes(self):
        session = self.make()
        try:
            first = session.snapshot()
            self.assertEqual(first['node'], 'C3')
            self.assertEqual(first['pose_source'], 'DISCRETE_ROAD_NODE')
            self.assertFalse(first['hardware_authorized'])
            while session.state == 'READY':
                events = session.advance()
                self.assertTrue(any(e['event'] == 'ACK' for e in events))
                self.assertTrue(any(e['event'] == 'COMPLETED' for e in events))
            snap = session.snapshot()
            self.assertEqual(snap['state'], 'DONE')
            self.assertEqual((snap['pick_count'], snap['place_count'], snap['stack_count']),
                             (12, 12, 3))
            self.assertEqual(snap['node'], 'C3')
            self.assertTrue(snap['home_confirmed'])
            self.assertEqual(snap['mock_executions'], snap['total_steps'])
            self.assertGreater(len(session.trace), 3)
        finally:
            session.close()

    def test_fail_at_three_never_advances_next_step(self):
        session = self.make(fail_at=3)
        try:
            for _ in range(3):
                session.advance()
            self.assertEqual(session.state, 'FAULT')
            snap = session.snapshot()
            self.assertEqual(snap['mock_executions'], 3)
            self.assertFalse(snap['home_confirmed'])
            self.assertEqual(session.advance(), [])
        finally:
            session.close()

    def test_missing_ack_causes_fault(self):
        session = self.make(drop_ack_at=2)
        try:
            session.advance()
            session.advance()
            self.assertEqual(session.state, 'FAULT')
            self.assertIn('TIMEOUT', session.snapshot()['error'])
            self.assertEqual(session.snapshot()['mock_executions'], 1)
        finally:
            session.close()

    def test_no_real_serial_device_parameters(self):
        session = self.make()
        try:
            self.assertIn(session.master, range(0, 1000000))
            self.assertIn(session.slave, range(0, 1000000))
            self.assertFalse(hasattr(session, 'device_path'))
        finally:
            session.close()


if __name__ == '__main__':
    unittest.main(verbosity=2)
