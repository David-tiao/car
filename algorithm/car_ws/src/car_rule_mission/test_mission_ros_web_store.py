#!/usr/bin/env python3
"""TEST mission HTTP input validation, no HTTP server or ROS required."""
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[4] / 'sim2real'))
import test_mission_store


def example():
    return {'schema': 'car_test_mission_replay_v1', 'source_type': 'MOCK',
            'mode': 'SIM_ONLY', 'hardware_authorized': False,
            'state': 'DONE', 'error': '', 'completed_steps': 36,
            'total_steps': 36, 'pick_count': 12, 'place_count': 12,
            'stack_count': 3, 'home_confirmed': True, 'cmd_id': 36,
            'kind': 'HOME', 'event': 'COMPLETED', 'stage_index': 8,
            'node': 'C3', 'x_m': 2.2, 'y_m': 2.2,
            'pose_source': 'DISCRETE_ROAD_NODE', 'mock_executions': 36,
            'events': [{'cmd_id': 36, 'event': 'COMPLETED'}]}


class StoreValidation(unittest.TestCase):
    def test_valid_snapshot(self):
        x = test_mission_store.validate(example())
        self.assertEqual(x['source_type'], 'MOCK')

    def test_reject_real_and_misleading_odom(self):
        for key, value in (('source_type', 'REAL'),
                           ('hardware_authorized', True),
                           ('pose_source', 'ODOM'),
                           ('mode', 'REAL'),
                           ('x_m', float('nan')),
                           ('completed_steps', 10000)):
            x = example()
            x[key] = value
            with self.subTest(key=key), self.assertRaises(ValueError):
                test_mission_store.validate(x)

    def test_reject_unknown_or_malformed_event(self):
        x = example()
        x['cmd_vel'] = 1
        with self.assertRaises(ValueError):
            test_mission_store.validate(x)
        x = example()
        x['events'] = [{'cmd_id': 12, 'event': 'SPOOFED'}]
        with self.assertRaises(ValueError):
            test_mission_store.validate(x)


if __name__ == '__main__':
    unittest.main(verbosity=2)
