#!/usr/bin/env python3
"""无需 ROS / 雷达 / STM32 的纯 Python 测试；直接导入现有拓扑代码。"""
import math
import unittest
from copy import deepcopy

from rule_mission_core import (NODES, EDGES, ZONES, STAGES, PlanError,
                               build_mission, verify_real_lock)


def station_snapshot():
    # 完全是测试数据，绝不是现场测量结果。
    return {'frame_id': 'map', 'source': 'LIDAR_STATIC', 'frozen': True,
            'stations': {
                'qr_board': {'valid': True, 'x_m': 2.35, 'y_m': 1.20,
                             'yaw_rad': 0.0, 'confidence': 0.91},
                'material': {'valid': True, 'x_m': 1.20, 'y_m': 2.35,
                             'confidence': 0.93},
            }}


def valid_status():
    return {'status': 'LOCKED_READY', 'ready': True, 'frozen': True,
            'conflict': False, 'source': 'LIDAR_CONFIRMED',
            'blocked': ['B3-C3'], 'mission_routes': [['C3', 'C2', 'B2', 'B3']]}


class RuleMissionTests(unittest.TestCase):
    def test_actual_topology_is_reused(self):
        self.assertEqual(len(NODES), 9)
        self.assertEqual(len(EDGES), 12)
        self.assertEqual(len(STAGES), 9)

    def test_all_mission_stages_and_start_zone(self):
        for zone in ZONES:
            plan = build_mission(zone, [], provenance='DEMO_ONLY')
            self.assertEqual(plan['status'], 'DEMO_ROUTE_ONLY')
            self.assertEqual(len(plan['legs']), 8)
            self.assertEqual(plan['waypoints'][0], plan['waypoints'][-1])
            self.assertEqual(plan['path_nodes'][0], plan['waypoints'][0])
            self.assertEqual(plan['path_nodes'][-1], plan['waypoints'][0])
            self.assertEqual(plan['legs'][0]['action'], 'SCAN_QR')
            self.assertEqual(plan['legs'][-1]['action'], 'RETURN_START')
            self.assertFalse(plan['commands_enabled'])
            self.assertTrue(plan['NOT_FOR_NAV2_FOLLOWPATH'])

    def test_dijkstra_avoids_locked_road(self):
        plan = build_mission('upper_right', ['B3-C3'])
        self.assertTrue(plan['path_nodes'])
        self.assertTrue(all('B3-C3' not in leg['road_ids'] for leg in plan['legs']))
        self.assertEqual(plan['blocked_roads'], ['B3-C3'])

    def test_unreachable_returns_no_route_and_no_nodes(self):
        plan = build_mission('upper_right', ['B3-C3', 'C2-C3'])
        self.assertEqual(plan['status'], 'NO_ROUTE')
        self.assertEqual(plan['legs'], [])
        self.assertEqual(plan['path_nodes'], [])

    def test_reversed_edge_normalizes(self):
        plan = build_mission('lower_right', ['C3-B3'])
        self.assertEqual(plan['blocked_roads'], ['B3-C3'])

    def test_rejects_unknown_or_duplicate_edges(self):
        for roads in (['C3-A3'], ['B3-C3', 'C3-B3'], ['A1-A1'], [4]):
            with self.subTest(roads=roads), self.assertRaises(PlanError):
                build_mission('upper_right', roads)

    def test_lock_proof_checks_required_flags(self):
        s = valid_status()
        self.assertEqual(verify_real_lock(s), {'B3-C3'})
        for key, value in [('status', 'WAIT_FOR_LOCK'), ('ready', False),
                           ('frozen', False), ('conflict', True),
                           ('source', 'MANUAL_TEST'), ('blocked', ['A3-C1'])]:
            bad = deepcopy(s)
            bad[key] = value
            with self.subTest(key=key), self.assertRaises(PlanError):
                verify_real_lock(bad)

    def test_station_observation_does_not_authorize_motion(self):
        plan = build_mission('upper_right', ['B3-C3'], station_snapshot(),
                             provenance='LIDAR_CONFIRMED')
        self.assertEqual(plan['status'], 'PREVIEW_WITH_STATIONS')
        self.assertEqual(plan['legs'][0]['target_station'], 'qr_board')
        self.assertEqual(plan['legs'][1]['target_station'], 'material')
        self.assertEqual(plan['legs'][4]['target_station'], 'material')
        self.assertEqual(plan['station_positions']['material']['approach_anchor'], 'B3')
        self.assertFalse(plan['station_positions']['material']['stop_pose_calibrated'])
        self.assertFalse(plan['commands_enabled'])

    def test_low_confidence_does_not_become_station_ready(self):
        data = station_snapshot()
        data['stations']['qr_board']['confidence'] = 0.1
        plan = build_mission('upper_right', [], data, 'LIDAR_CONFIRMED')
        self.assertEqual(plan['status'], 'PREVIEW_WAIT_STATIONS')

    def test_missing_station_is_explicit(self):
        data = station_snapshot()
        data['stations'].pop('material')
        plan = build_mission('upper_right', [], data, 'LIDAR_CONFIRMED')
        self.assertEqual(plan['stations_state'], 'WAIT_STATIONS')
        self.assertNotIn('material', plan['station_positions'])

    def test_rejects_bad_frame_source_coordinates_and_yaw(self):
        for change in (
            lambda x: x.update(frame_id='base_link'),
            lambda x: x.update(source='MANUAL_TEST'),
            lambda x: x['stations']['material'].update(x_m=float('nan')),
            lambda x: x['stations']['material'].update(x_m=0.10),
            lambda x: x['stations']['qr_board'].update(yaw_rad=10.0),
        ):
            data = station_snapshot()
            change(data)
            with self.assertRaises(PlanError):
                build_mission('upper_right', [], data, 'LIDAR_CONFIRMED')

    def test_only_legal_graph_edges_in_plan(self):
        plan = build_mission('upper_right', ['A2-B2', 'B1-B2'])
        for leg in plan['legs']:
            for a, b in zip(leg['node_path'], leg['node_path'][1:]):
                self.assertIn({a, b}, [{u, v} for u, v in EDGES.values()])
            self.assertTrue(math.isfinite(leg['distance_m']))


if __name__ == '__main__':
    unittest.main(verbosity=2)
