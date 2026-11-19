#!/usr/bin/env python3
"""Offline mission simulation tests; no ROS, serial, lidar, or motor hardware."""
import unittest
from mission_execution import (MissionError, SimExecutor, compile_workflow,
                               parse_task_code)

ACTIONS = ('SCAN_QR', 'PICK_BATCH_1', 'ROUGH_PROCESS_BATCH_1',
           'STORE_BATCH_1', 'PICK_BATCH_2', 'ROUGH_PROCESS_BATCH_2',
           'STACK_BATCH_2', 'RETURN_START')
NODES = ('C3', 'C2', 'B3', 'B1', 'A2', 'B3', 'B1', 'A2', 'C3')
CODE = '156+123+516+231'


def fake_plan():
    legs = []
    for i, (a, b) in enumerate(zip(NODES, NODES[1:]), 1):
        legs.append(dict(stage_index=i, action=ACTIONS[i-1], start_node=a, goal_node=b,
                         node_path=[a, b], road_ids=[a+'-'+b]))
    return {'status': 'DEMO_ROUTE_ONLY', 'commands_enabled': False,
            'NOT_FOR_NAV2_FOLLOWPATH': True, 'legs': legs, 'blocked_roads': []}


class WorkflowTests(unittest.TestCase):
    def test_qr_validation(self):
        self.assertEqual(parse_task_code(CODE).batch2_rough_rings, (2, 3, 1))
        for bad in ('123+123+124+231', '111+123+111+123',
                    '156+112+516+231', '156+123+516+122',
                    '156+123+516+234', '156+123+516+231 ', '', None):
            with self.subTest(bad=bad), self.assertRaises(MissionError):
                parse_task_code(bad)

    def test_24_manipulations_and_stacking_by_first_batch_color(self):
        steps = compile_workflow(fake_plan(), CODE)
        self.assertEqual(len([s for s in steps if s.kind == 'MOVE_NODE']), 8)
        self.assertEqual(len([s for s in steps if s.kind.startswith('PICK_')]), 12)
        self.assertEqual(len([s for s in steps if s.kind.startswith('PLACE_')]), 9)
        self.assertEqual(len([s for s in steps if s.kind == 'STACK_STORE']), 3)
        self.assertEqual([(s.color_id, s.ring_id) for s in steps if s.kind == 'STACK_STORE'],
                         [(5, 2), (1, 1), (6, 3)])
        self.assertEqual([(s.color_id, s.ring_id) for s in steps
                          if s.kind == 'PLACE_ROUGH' and s.batch == 2],
                         [(5, 2), (1, 3), (6, 1)])
        self.assertEqual(steps[-1].kind, 'HOME')
        self.assertTrue(all(s.intent()['hardware_authorized'] is False for s in steps))

    def test_bad_plan_cannot_be_promoted_to_commands(self):
        for mutate in (
            lambda p: p.update(commands_enabled=True),
            lambda p: p.update(status='NO_ROUTE'),
            lambda p: p['legs'].pop(),
            lambda p: p['legs'][1].update(start_node='BOGUS'),
            lambda p: p['legs'][2].update(road_ids=[]),
            lambda p: p.update(blocked_roads=['C3-C2']),
        ):
            p = fake_plan()
            mutate(p)
            with self.assertRaises(MissionError):
                compile_workflow(p, CODE)

    def test_happy_path_requires_correlated_ack_and_completion(self):
        e = SimExecutor(compile_workflow(fake_plan(), CODE), CODE)
        now = 0
        while e.state != 'DONE':
            intent = e.issue(now)
            self.assertIsNotNone(intent)
            self.assertIsNone(e.issue(now))
            self.assertFalse(e.feedback(intent['cmd_id'] + 999, 'COMPLETED', now))
            self.assertFalse(e.feedback(intent['cmd_id'], 'COMPLETED', now))
            self.assertTrue(e.feedback(intent['cmd_id'], 'ACK', now + 1))
            self.assertFalse(e.feedback(intent['cmd_id'], 'ACK', now + 2))
            self.assertTrue(e.feedback(intent['cmd_id'], 'COMPLETED', now + 3,
                                       qr_text=CODE if intent['kind'] == 'SCAN_QR' else None))
            self.assertFalse(e.feedback(intent['cmd_id'], 'COMPLETED', now + 4))
            now += 10
        summary = e.summary()
        self.assertEqual((summary['pick_count'], summary['place_count'], summary['stack_count']),
                         (12, 12, 3))
        self.assertTrue(summary['home_confirmed'])

    def test_rejection_failure_and_timeouts_latch_fault(self):
        steps = compile_workflow(fake_plan(), CODE)
        e = SimExecutor(steps, CODE, ack_timeout_ms=10)
        e.issue(0)
        self.assertEqual(e.tick(11), 'FAULT')
        self.assertIsNone(e.issue(12))
        e = SimExecutor(steps, CODE, result_timeout_ms=10)
        cmd = e.issue(0)
        e.feedback(cmd['cmd_id'], 'ACK', 1)
        self.assertEqual(e.tick(12), 'FAULT')
        for fail in ('REJECTED', 'FAILED'):
            e = SimExecutor(steps, CODE)
            cmd = e.issue(0)
            self.assertTrue(e.feedback(cmd['cmd_id'], fail, 1))
            self.assertEqual(e.state, 'FAULT')

    def test_scan_must_match_and_be_acknowledged(self):
        steps = compile_workflow(fake_plan(), CODE)
        e = SimExecutor(steps, CODE)
        move = e.issue(0)
        e.feedback(move['cmd_id'], 'ACK', 1)
        e.feedback(move['cmd_id'], 'COMPLETED', 2)
        scan = e.issue(3)
        self.assertEqual(scan['kind'], 'SCAN_QR')
        e.feedback(scan['cmd_id'], 'ACK', 4)
        e.feedback(scan['cmd_id'], 'COMPLETED', 5, qr_text='123+123+123+123')
        self.assertEqual(e.state, 'FAULT')
        self.assertEqual(e.summary()['pick_count'], 0)

    def test_monotonic_time_guard(self):
        e = SimExecutor(compile_workflow(fake_plan(), CODE), CODE)
        e.issue(100)
        with self.assertRaises(MissionError):
            e.tick(99)


if __name__ == '__main__':
    unittest.main(verbosity=2)
