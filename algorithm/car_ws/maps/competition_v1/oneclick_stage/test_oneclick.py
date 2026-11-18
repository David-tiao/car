#!/usr/bin/env python3
"""Offline tests; do not source ROS, access the serial port or run motors."""
import json
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

import main
from mission_export import ZONES, STAGES, validate_route_message, flatten_legs


class OneclickTests(unittest.TestCase):
    def test_mission_order(self):
        for key, seq in ZONES.items():
            self.assertEqual(len(seq), 9)
            self.assertEqual(seq[0], seq[-1])
            self.assertEqual(seq[1:-1], ('C2', 'B3', 'B1', 'A2', 'B3', 'B1', 'A2'))
            self.assertEqual(len(STAGES), len(seq))

    def test_valid_route_only_after_lock(self):
        seq = ZONES['upper_right']
        status = {'status': 'LOCKED_READY', 'frozen': True, 'source': 'LIDAR_CONFIRMED',
                  'conflict': False, 'waypoints': list(seq),
                  'blocked': [], 'mission_routes': [[a, b] for a,b in zip(seq,seq[1:])]}
        self.assertEqual(len(validate_route_message(status,seq)), 8)
        status['status'] = 'WAIT_FOR_LOCK'
        self.assertIsNone(validate_route_message(status,seq))

    def test_no_route_fails_closed(self):
        seq = ZONES['lower_right']
        status = {'status': 'LOCKED_READY', 'frozen': True, 'source': 'LIDAR_CONFIRMED',
                  'conflict': False, 'waypoints': list(seq), 'blocked': [],
                  'mission_routes': [[a,b] for a,b in zip(seq,seq[1:])]}
        status['mission_routes'][0] = ['A2','C3']
        self.assertIsNone(validate_route_message(status,seq))

    def test_manual_demo_route_is_rejected(self):
        seq = ZONES['upper_right']
        status = {'status': 'LOCKED_READY', 'frozen': True, 'source': 'MANUAL_TEST',
                  'conflict': False, 'waypoints': list(seq), 'blocked': [],
                  'mission_routes': [[a,b] for a,b in zip(seq,seq[1:])]}
        self.assertIsNone(validate_route_message(status,seq))

    def test_flatten(self):
        self.assertEqual(flatten_legs([['A','B'],['B','C','D']]), ['A','B','C','D'])

    def test_bad_config_prevents_tf_guessing(self):
        with tempfile.TemporaryDirectory() as d:
            p = Path(d)/'config.json'
            cfg = {'expected_obstacles':2,'start_zone':'upper_right','serial_port':None,
                   'show_rviz':True, 'stationary_tf':{'enabled':True,
                                                    'map_to_base':None,'base_to_laser':None}}
            p.write_text(json.dumps(cfg))
            with self.assertRaisesRegex(ValueError, '已测量'):
                main.config(p)
            cfg['stationary_tf']['enabled']=False
            p.write_text(json.dumps(cfg))
            self.assertEqual(main.config(p)['start_zone'], 'upper_right')

    def test_port_changes_only(self):
        source = 'port: /dev/ttyUSB0\nframe_id: laser_frame\n'
        new = main.x3_param_text(source, '/dev/serial/by-id/test')
        self.assertIn('port: /dev/serial/by-id/test', new)
        self.assertIn('frame_id: laser_frame', new)

    def test_dependency_preflight_reports_missing(self):
        with patch.object(main, 'REQUIRED', (Path('/nonexistent/source1'),)):
            with self.assertRaisesRegex(RuntimeError, '缺少原有工程文件'):
                main.check_inputs()

    def test_map_generation_into_logs_only(self):
        import zipfile
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)/'project'
            run = Path(folder)/'logs'
            root.mkdir()
            run.mkdir()
            # Reuse the actual installed map generator, never replace its geometry.
            generator = main.ROOT / 'generate_static_map.py'
            if not generator.is_file():
                self.skipTest('原工程地图生成器缺失，当前仅测试新增脚本')
            import shutil
            shutil.copy2(generator, root/'generate_static_map.py')
            with patch.object(main,'ROOT',root):
                a,b = main.map_pair(run)
                self.assertTrue(a.is_file())
                self.assertTrue(b.is_file())
                self.assertFalse((root/'competition_navigation_map.yaml').exists())


if __name__ == '__main__':
    unittest.main(verbosity=2)
