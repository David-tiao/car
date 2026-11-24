#!/usr/bin/env python3
"""Offline tests; requires only Python 3, NOT ROS, YDLIDAR or Nav2."""
import math
import sys
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent / 'topology_stage6'))

from lock_core import ObstacleLock, laser_clusters, roads_at
from lock_obstacles import static_map_free
from road_topology import EDGES, canonical, shortest, plan_mission, validate_lock


class FixedRouteTests(unittest.TestCase):
    def test_graph_and_dijkstra_excludes_whole_road(self):
        self.assertEqual(len(EDGES), 12)
        self.assertEqual(canonical('B2-A2'), 'A2-B2')
        p = shortest('A2', 'B2', {'A2-B2'})
        self.assertTrue(p)
        self.assertNotEqual(p, ['A2', 'B2'])
        self.assertFalse(any({a, b} == {'A2', 'B2'} for a, b in zip(p, p[1:])))

    def test_full_route_unavailable(self):
        self.assertIsNone(plan_mission(['A1', 'C3'], {'A1-A2', 'A1-B1'}))

    def test_junction_closes_all_touching_roads(self):
        self.assertEqual(set(roads_at(1.20, 1.20)),
                         {'A2-B2', 'B2-C2', 'B1-B2', 'B2-B3'})

    def test_double_obstacle_same_road(self):
        detector = ObstacleLock(expected=2, required_hits=2, settle_frames=2)
        for _ in range(4):
            detector.observe([(.70, 1.20), (.90, 1.20)])
        s = detector.status()
        self.assertTrue(s['locked'])
        self.assertEqual(len(s['obstacles']), 2)
        self.assertEqual(s['blocked_roads'], ['A2-B2'])

    def test_confirm_and_never_unlock_after_clear(self):
        detector = ObstacleLock(expected=2, required_hits=2, settle_frames=2)
        for _ in range(4):
            detector.observe([(.70, 1.20), (2.05, .73)])
        first = detector.status()
        self.assertTrue(first['locked'])
        self.assertEqual(len(first['blocked_roads']), 2)
        for _ in range(50):
            detector.observe([])
        self.assertEqual(detector.status()['blocked_roads'], first['blocked_roads'])
        self.assertEqual(detector.status()['obstacles'], first['obstacles'])

    def test_third_target_conflict_does_not_modify_lock(self):
        detector = ObstacleLock(expected=2, required_hits=2, settle_frames=2)
        for _ in range(4):
            detector.observe([(.70, 1.20), (2.05, .73)])
        first = detector.status()
        for _ in range(3):
            detector.observe([(.70, 1.20), (2.05, .73), (1.20, 1.55)])
        self.assertEqual(detector.state, 'CONFLICT')
        self.assertEqual(detector.status()['blocked_roads'], first['blocked_roads'])

    def test_lock_schema_rejects_partial(self):
        self.assertIsNone(validate_lock({'state': 'SCANNING', 'locked': False}, 2))
        valid = {'state': 'LOCKED', 'locked': True,
                 'map_received': True, 'expected_obstacles': 2,
                 'corridor_radius': 0.23,
                 'obstacles': [{'x': .7, 'y': 1.2, 'roads': ['A2-B2']},
                               {'x': 2.05, 'y': .73, 'roads': ['C1-C2']}],
                 'blocked_roads': ['A2-B2', 'C1-C2']}
        self.assertEqual(validate_lock(valid, 2), frozenset({'A2-B2', 'C1-C2'}))
        valid['blocked_roads'] = ['A2-B2']
        self.assertIsNone(validate_lock(valid, 2))

    def test_lock_missing_map_or_wrong_expected_is_rejected(self):
        detector = ObstacleLock(expected=2, required_hits=2, settle_frames=2)
        for _ in range(4):
            detector.observe([(.70, 1.20), (2.05, .73)])
        s = detector.status()
        self.assertIsNone(validate_lock(s, 2))
        s['map_received'] = True
        self.assertIsNotNone(validate_lock(s, 2))
        self.assertIsNone(validate_lock(s, 3))

    def test_rejects_misdeclared_road_at_valid_coordinate(self):
        detector = ObstacleLock(expected=2, required_hits=2, settle_frames=2)
        for _ in range(4):
            detector.observe([(.70, 1.20), (2.05, .73)])
        s = detector.status()
        s['map_received'] = True
        s['obstacles'][0]['roads'] = ['B1-C1']
        s['blocked_roads'] = ['B1-C1', 'C1-C2']
        self.assertIsNone(validate_lock(s, 2))

    def test_invalid_detection_params_and_non_finite_targets(self):
        with self.assertRaises(ValueError):
            ObstacleLock(corridor_radius=float('nan'))
        detector = ObstacleLock(expected=1, required_hits=2, settle_frames=2)
        for _ in range(4):
            detector.observe([(float('nan'), 1.2), (.70, 1.20)])
        self.assertTrue(detector.status()['locked'])

    def test_static_map_only_accepts_known_free_cells(self):
        from types import SimpleNamespace as NS
        info = NS(origin=NS(position=NS(x=0., y=0.),
                            orientation=NS(x=0., y=0., z=0., w=1.)),
                  resolution=1., width=2, height=2)
        grid = NS(header=NS(frame_id='map'), info=info, data=[0, 100, -1, 0])
        self.assertTrue(static_map_free(grid, .5, .5))
        self.assertFalse(static_map_free(grid, 1.5, .5))
        self.assertFalse(static_map_free(grid, .5, 1.5))
        grid.header.frame_id = 'odom'
        self.assertFalse(static_map_free(grid, .5, .5))

    def test_clustering_rejects_invalid_metadata(self):
        self.assertEqual(laser_clusters([1.], 0, 0), [])
        self.assertEqual(laser_clusters([1.], 0, .02, min_range=2, max_range=1), [])

    def test_gazebo_two_obstacles_crossing_scan_seam(self):
        # Analytic ray/circle intersections matching the C3 SIM fixture.
        # A target directly behind the robot straddles -pi/+pi.
        for count, step in ((720, math.tau / 719), (720, math.tau / 720)):
            for sign in (1, -1):
                first = -sign * math.pi
                targets = [(-.55, 0.), (-.85, -.55)]
                readings = []
                for i in range(count):
                    a = first + i * sign * step
                    dx, dy = math.cos(a), math.sin(a)
                    hits = []
                    for x, y in targets:
                        projection = x * dx + y * dy
                        disc = .025**2 - (x*dy-y*dx)**2
                        if projection > 0 and disc >= 0:
                            hits.append(projection - math.sqrt(disc))
                    readings.append(min(hits, default=math.inf))
                groups = laser_clusters(readings, first, sign * step)
                self.assertEqual(len(groups), 2, groups)
                engine = ObstacleLock()
                for _ in range(8):
                    engine.observe([(2.05+x, 2.05+y) for x,y,_ in groups])
                self.assertEqual(engine.state, 'LOCKED')
                self.assertEqual(engine.status()['blocked_roads'], ['B2-B3', 'B3-C3'])
                # A genuine third compact object remains a conflict.
                for _ in range(5):
                    engine.observe([(2.05+x, 2.05+y) for x,y,_ in groups] + [(2.05, 1.55)])
                self.assertEqual(engine.state, 'CONFLICT')

    def test_partial_fov_and_invalid_endpoint_not_joined(self):
        # Short partial scan: endpoints close in space but unrelated runs.
        readings = [1., 1., math.inf, math.inf, 1., 1.]
        self.assertEqual(len(laser_clusters(readings, 0., .01)), 2)
        step = math.tau / 719
        readings = [math.inf] * 720
        readings[1:4] = [.5] * 3
        readings[-3:] = [.5] * 3
        self.assertEqual(len(laser_clusters(readings, -math.pi, step)), 2)

    def test_seam_join_before_span_and_count_filters(self):
        step = math.tau / 719
        readings = [math.inf] * 720
        readings[0] = .55
        readings[-1] = .55
        self.assertEqual(laser_clusters(readings, -math.pi, step), [])
        self.assertEqual(len(laser_clusters(readings, -math.pi, math.tau / 720)), 1)
        readings[:20] = [.55] * 20
        readings[-20:] = [.55] * 20
        # Each half is compact; their full circular span exceeds .14m.
        self.assertEqual(laser_clusters(readings, -math.pi, step), [])

    def test_clustering(self):
        # A tiny circular target over 3 adjacent angles around 1.0 m.
        readings = [float('inf')] * 50
        readings[20:23] = [1.0, .99, 1.0]
        groups = laser_clusters(readings, -.25, .01)
        self.assertEqual(len(groups), 1)
        self.assertEqual(groups[0][2], 3)


if __name__ == '__main__':
    unittest.main(verbosity=2)
