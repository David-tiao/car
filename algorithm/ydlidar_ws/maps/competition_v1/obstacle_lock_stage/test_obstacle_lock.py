#!/usr/bin/env python3
import unittest
from obstacle_lock_core import ObstacleLocker, classify_road_cell, shortest

class LockTests(unittest.TestCase):
    def test_two_objects_lock_and_route(self):
        lk = ObstacleLocker()
        for i in range(21):
            lk.observe([(1.62+0.003*(i%3), 2.05), (0.35, 0.79+0.002*(i%2))])
        self.assertEqual(lk.state, 'LOCKED')
        self.assertEqual(len(lk.locked_objects), 2)
        self.assertEqual(len(lk.blocked_cells), 2)
        path = lk.route('C3','A3')
        self.assertTrue(path)
        self.assertFalse(any(f'{a}-{b}' in lk.blocked_cells or f'{b}-{a}' in lk.blocked_cells
                             for a,b in zip(path,path[1:])))
        for i in range(5):
            lk.observe([])
        self.assertEqual(lk.state,'LOCKED')
        self.assertEqual(len(lk.blocked_cells),2)

    def test_not_two_not_ready(self):
        lk=ObstacleLocker()
        for _ in range(25): lk.observe([(1.62,2.05)])
        self.assertEqual(lk.state,'WAIT_TWO_OBSTACLES')
        self.assertEqual(lk.route('C3','A3'),[])

    def test_obstacles_found_at_different_times(self):
        lk=ObstacleLocker()
        for _ in range(12): lk.observe([(1.62,2.05)])
        for _ in range(13): lk.observe([(0.35,0.79)])
        self.assertEqual(lk.state,'LOCKED')
        self.assertEqual(len(lk.blocked_cells),2)

    def test_three_objects_conflict(self):
        lk=ObstacleLocker()
        for _ in range(25): lk.observe([(1.62,2.05), (0.35,0.79), (2.05,0.79)])
        self.assertEqual(lk.state,'CONFLICT')
        self.assertFalse(lk.route('C3','A3'))

    def test_third_after_locked_conflict(self):
        lk=ObstacleLocker()
        for _ in range(25): lk.observe([(1.62,2.05), (0.35,0.79)])
        self.assertEqual(lk.state,'LOCKED')
        for _ in range(5): lk.observe([(2.05,0.79)])
        self.assertEqual(lk.state,'CONFLICT')
        self.assertEqual(len(lk.locked_objects),2)

    def test_same_road_two_objects(self):
        lk=ObstacleLocker()
        for _ in range(25): lk.observe([(1.49,2.05), (1.87,2.05)])
        self.assertEqual(lk.state,'LOCKED')
        self.assertEqual(len(lk.locked_objects),2)
        self.assertEqual(len(lk.blocked_cells),1)

    def test_near_intersection_no_guess(self):
        cell, why = classify_road_cell(1.20,1.20)
        self.assertIsNone(cell)
        self.assertIn(why,('AMBIGUOUS_CELL_BORDER','NEAR_INTERSECTION'))

if __name__ == '__main__': unittest.main()
