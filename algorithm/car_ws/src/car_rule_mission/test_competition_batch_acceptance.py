#!/usr/bin/env python3
"""Offline regression for phase-4 QA determinism, report and fail-closed matrix."""
import tempfile
import unittest
from pathlib import Path

from competition_batch_acceptance import (audit_plan, format_markdown, make_task_code,
                                          run_in_memory, run_suite)
from mission_execution import parse_task_code
from rule_mission_core import build_mission
import random


class Phase4Tests(unittest.TestCase):
    def test_generated_tasks_are_valid_and_reproducible(self):
        a = random.Random(20271120)
        b = random.Random(20271120)
        sequences = [make_task_code(a) for _ in range(25)]
        self.assertEqual(sequences, [make_task_code(b) for _ in range(25)])
        for code in sequences:
            parsed = parse_task_code(code)
            self.assertEqual(set(parsed.batch1_colors), set(parsed.batch2_colors))

    def test_audit_and_memory_execution(self):
        code = '156+123+516+231'
        plan = build_mission('upper_right', ['B3-C3'], provenance='DEMO_ONLY')
        steps = audit_plan(plan, {'B3-C3'}, code)
        summary = run_in_memory(steps, code)
        self.assertEqual(summary['state'], 'DONE')
        self.assertTrue(summary['home_confirmed'])

    def test_batch_covers_reachable_and_no_route(self):
        report = run_suite(total=6, seed=123, wire_samples=1)
        self.assertTrue(report['passed'], report.get('errors'))
        self.assertEqual(report['total_cases'], 6)
        self.assertGreater(report['summary'].get('PASS_SIM', 0), 0)
        self.assertGreater(report['summary'].get('PASS_NO_ROUTE', 0), 0)
        self.assertEqual(report['wire_samples_completed'], 1)
        self.assertTrue(all(event['ok'] for event in report['fault_matrix']))
        self.assertFalse(report['hardware_authorized'])

    def test_report_format_distinguishes_simulation(self):
        report = run_suite(total=3, seed=3, wire_samples=0)
        md = format_markdown(report)
        self.assertIn('SIM_ONLY', md)
        self.assertIn('未授权真实运动', md)
        self.assertIn('故障注入矩阵', md)
        self.assertIn('虚拟串口', md)
        self.assertEqual(len(report['cases']), 3)


if __name__ == '__main__':
    unittest.main(verbosity=2)
