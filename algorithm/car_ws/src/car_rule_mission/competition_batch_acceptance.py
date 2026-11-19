#!/usr/bin/env python3
"""Phase 4: offline batch QA for competition route + task + CAR-LINK MOCK protocol.

Never opens physical UART, runs ROS, or publishes motor commands. All motion
and manipulation outcomes are simulated. Reports are evidence of software
consistency only, not physical navigation or grasp accuracy.
"""
import argparse
import itertools
import json
import os
import random
import tempfile
from collections import Counter
from datetime import datetime, timezone
from pathlib import Path

from mission_execution import MissionError, SimExecutor, compile_workflow, parse_task_code
from mission_wire_sim import run_loopback
from rule_mission_core import EDGES, NODES, PlanError, ZONES, build_mission


def make_task_code(rng):
    # Match the parser's explicit V1 domain: three distinct IDs from 1..6,
    # second batch uses the same colors in a different possible order.
    colors = rng.sample(list(range(1, 7)), 3)
    second = rng.sample(colors, 3)
    rings_a = rng.sample([1, 2, 3], 3)
    rings_b = rng.sample([1, 2, 3], 3)
    return '+'.join(''.join(map(str, group)) for group in
                    (colors, rings_a, second, rings_b))


def _assert(cond, detail):
    if not cond:
        raise AssertionError(detail)


def audit_plan(plan, blocked, code):
    """Check actual original road topology without trusting status labels alone."""
    _assert(plan.get('status') == 'DEMO_ROUTE_ONLY', 'unexpected route status')
    _assert(plan.get('commands_enabled') is False, 'unsafe motion authorization')
    _assert(plan.get('NOT_FOR_NAV2_FOLLOWPATH') is True, 'path guard absent')
    legs = plan['legs']
    _assert(len(legs) == 8, 'expected eight stages')
    _assert(plan['waypoints'][0] == plan['waypoints'][-1], 'route does not return')
    _assert(sum(len(leg['node_path']) - 1 for leg in legs) ==
            len(plan['path_nodes']) - 1, 'flattened route mismatch')
    for i, leg in enumerate(legs):
        _assert(leg['stage_index'] == i + 1, 'stage misordered')
        _assert(leg['node_path'][0] == leg['start_node'], 'leg origin mismatch')
        _assert(leg['node_path'][-1] == leg['goal_node'], 'leg goal mismatch')
        for a, b, road in zip(leg['node_path'], leg['node_path'][1:], leg['road_ids']):
            _assert(road in EDGES and road not in blocked, 'illegal or blocked road')
            _assert(set(EDGES[road]) == {a, b}, 'route edge not adjacent')
            _assert(a in NODES and b in NODES, 'unknown node')
    steps = compile_workflow(plan, code)
    _assert(steps[-1].kind == 'HOME', 'last step must confirm HOME')
    _assert(all(s.cmd_id == i for i, s in enumerate(steps, 1)), 'unstable cmd_id')
    _assert(sum(s.kind == 'PICK_RAW' for s in steps) == 6, 'raw pick count')
    _assert(sum(s.kind == 'PICK_ROUGH' for s in steps) == 6, 'rough pick count')
    _assert(sum(s.kind == 'PLACE_ROUGH' for s in steps) == 6, 'rough placement count')
    _assert(sum(s.kind == 'PLACE_STORE' for s in steps) == 3, 'store count')
    _assert(sum(s.kind == 'STACK_STORE' for s in steps) == 3, 'stack count')
    task = parse_task_code(code)
    base = dict(zip(task.batch1_colors, task.batch1_rough_rings))
    _assert([(s.color_id, s.ring_id) for s in steps if s.kind == 'STACK_STORE']
            == [(c, base[c]) for c in task.batch2_colors], 'color-to-base mapping')
    return steps


def run_in_memory(steps, task_code):
    """A deterministic, protocol-independent safety state-machine check."""
    engine = SimExecutor(steps, task_code)
    clock = 0
    while engine.state == 'READY':
        intent = engine.issue(clock)
        _assert(intent['hardware_authorized'] is False, 'hardware command seen')
        _assert(engine.feedback(intent['cmd_id'], 'COMPLETED', clock + 1) is False,
                'completion incorrectly accepted before ACK')
        _assert(engine.feedback(intent['cmd_id'], 'ACK', clock + 2), 'ACK rejected')
        _assert(engine.feedback(intent['cmd_id'], 'COMPLETED', clock + 3,
                                qr_text=task_code if intent['kind'] == 'SCAN_QR' else None),
                'completion rejected')
        clock += 10
    summary = engine.summary()
    _assert(summary['state'] == 'DONE', 'in-memory mission did not finish')
    _assert((summary['pick_count'], summary['place_count'], summary['stack_count'])
            == (12, 12, 3), 'incorrect manipulation counts')
    _assert(summary['home_confirmed'], 'home not confirmed')
    return summary


def fault_matrix(steps, task_code):
    checks = (
        ('FAILED_at_3', {'fail_at': 3}, 'FAULT'),
        ('NO_ACK_at_2', {'drop_ack_at': 2}, 'FAULT'),
        ('BAD_CRC_at_2', {'corrupt_crc_at': 2}, 'FAULT'),
        ('QR_MISMATCH', {'corrupt_qr': True}, 'FAULT'),
        ('DUPLICATE_CMD_3', {'duplicate_at': 3}, 'DONE'),
    )
    results = []
    for label, options, expected in checks:
        result = run_loopback(steps, task_code, **options)
        actual = result['state']
        ok = actual == expected and result['hardware_authorized'] is False
        if label == 'QR_MISMATCH':
            ok = ok and result['pick_count'] == 0
        if label == 'DUPLICATE_CMD_3':
            ok = ok and result['wire']['mock_executions'] == len(steps)
        if label in ('FAILED_at_3', 'NO_ACK_at_2', 'BAD_CRC_at_2'):
            ok = ok and result['completed_steps'] < len(steps)
        results.append({'case': label, 'expected': expected, 'state': actual,
                        'ok': bool(ok), 'error': result.get('error', ''),
                        'executions': result['wire']['mock_executions']})
    return results


def _case_set(total, seed):
    rng = random.Random(seed)
    all_edges = sorted(EDGES)
    # Deterministic regression: no obstacles, isolated endpoints (NO_ROUTE),
    # plus random configurations of 0..3 locked roads.
    fixed = [tuple(), tuple(sorted(['C1-C2', 'B1-C1'])),
             tuple(sorted(['C2-C3', 'B3-C3']))]
    for i in range(total):
        blocked = fixed[i] if i < len(fixed) else tuple(sorted(
            rng.sample(all_edges, rng.randint(0, min(3, len(all_edges))))))
        yield i + 1, ('upper_right' if i % 2 == 0 else 'lower_right'), blocked, make_task_code(rng)


def run_suite(total=100, seed=20271120, wire_samples=5):
    if not 3 <= total <= 2000:
        raise ValueError('cases must be within 3..2000')
    if not 0 <= wire_samples <= 20:
        raise ValueError('wire-samples must be within 0..20')
    records = []
    counts = Counter()
    wire_done = 0
    error_messages = []
    for ident, zone, blocked, task_code in _case_set(total, seed):
        row = {'id': ident, 'start_zone': zone, 'blocked_roads': list(blocked),
               'task_code': task_code, 'status': 'NOT_RUN', 'detail': ''}
        try:
            plan = build_mission(zone, blocked, provenance='DEMO_ONLY')
            if plan['status'] == 'NO_ROUTE':
                _assert(plan['legs'] == [] and plan['path_nodes'] == [],
                        'NO_ROUTE contains phantom path')
                row['status'] = 'PASS_NO_ROUTE'
            else:
                steps = audit_plan(plan, set(blocked), task_code)
                summary = run_in_memory(steps, task_code)
                row['status'] = 'PASS_SIM'
                row['steps'] = len(steps)
                row['road_distance_m'] = plan['total_road_distance_m']
                row['summary'] = summary
                if wire_done < wire_samples:
                    result = run_loopback(steps, task_code)
                    _assert(result['state'] == 'DONE', 'wire replay did not finish')
                    _assert(result['wire']['mock_executions'] == len(steps),
                            'mock action count mismatch')
                    row['wire'] = 'PASS'
                    wire_done += 1
        except (AssertionError, ValueError, TypeError, KeyError, OSError) as exc:
            row['status'] = 'FAIL'
            row['detail'] = '{}: {}'.format(type(exc).__name__, exc)
            error_messages.append('case {}: {}'.format(ident, row['detail']))
        counts[row['status']] += 1
        records.append(row)

    # Complete fault matrix on the known traversable baseline, not on arbitrary
    # randomly generated maps that may be inaccessible.
    baseline_code = '156+123+516+231'
    baseline_plan = build_mission('upper_right', [], provenance='DEMO_ONLY')
    baseline_steps = audit_plan(baseline_plan, set(), baseline_code)
    failures = fault_matrix(baseline_steps, baseline_code)
    success = counts['FAIL'] == 0 and all(f['ok'] for f in failures)
    return {'schema': 'car_competition_offline_acceptance_v1',
            'mode': 'SIM_ONLY', 'hardware_authorized': False,
            'generated_at_utc': datetime.now(timezone.utc).isoformat(timespec='seconds'),
            'seed': seed, 'total_cases': total, 'wire_samples_requested': wire_samples,
            'wire_samples_completed': wire_done, 'summary': dict(counts),
            'fault_matrix': failures, 'passed': bool(success),
            'warnings': [
                'TEST-only synthetic ACK/COMPLETED are not actual STM32 firmware feedback',
                'Road graph endpoints are not actual calibrated stop poses',
                'No lidar localization, collision dynamics, robot manipulation or real UART test'],
            'errors': error_messages, 'cases': records}


def format_markdown(report):
    counts = report['summary']
    lines = [
        '# 工创小车第四阶段：离线比赛全流程验收报告',
        '',
        '- 生成时间 (UTC)：' + report['generated_at_utc'],
        '- 运行模式：SIM_ONLY；未授权真实运动',
        '- 随机种子：' + str(report['seed']),
        '- 场景总数：' + str(report['total_cases']),
        '- 可达场景通过：' + str(counts.get('PASS_SIM', 0)),
        '- 不可达场景正确拒绝：' + str(counts.get('PASS_NO_ROUTE', 0)),
        '- 场景失败：' + str(counts.get('FAIL', 0)),
        '- 通过虚拟串口执行的完整场景：' + str(report['wire_samples_completed']),
        '- 最终结论：' + ('PASS（仅软件模拟）' if report['passed'] else 'FAIL'),
        '', '## 故障注入矩阵', '',
        '| 场景 | 预期 | 实际 | 结果 |',
        '| --- | --- | --- | --- |',
    ]
    for row in report['fault_matrix']:
        lines.append('| {} | {} | {} | {} |'.format(
            row['case'], row['expected'], row['state'],
            'PASS' if row['ok'] else 'FAIL'))
    lines.extend(['', '## 安全限制', ''])
    for warning in report['warnings']:
        lines.append('- ' + warning)
    if report['errors']:
        lines.extend(['', '## 错误详情', ''])
        for err in report['errors'][:50]:
            lines.append('- ' + err.replace('|', '/'))
    return '\n'.join(lines) + '\n'


def _atomic_write(path, content):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, name = tempfile.mkstemp(prefix='.' + path.name + '.', dir=str(path.parent))
    try:
        with os.fdopen(fd, 'w', encoding='utf-8') as out:
            out.write(content)
        os.replace(name, path)
    finally:
        if os.path.exists(name):
            os.unlink(name)


def main():
    p = argparse.ArgumentParser(description='2027 工创智能搬运 TEST-only batch QA')
    p.add_argument('--cases', type=int, default=100)
    p.add_argument('--seed', type=int, default=20271120)
    p.add_argument('--wire-samples', type=int, default=5)
    p.add_argument('--output-dir', default='/tmp/car_phase4_acceptance')
    args = p.parse_args()
    if not (3 <= args.cases <= 2000 and 0 <= args.wire_samples <= 20):
        p.error('cases must be 3..2000, wire-samples must be 0..20')
    try:
        report = run_suite(args.cases, args.seed, args.wire_samples)
        folder = Path(args.output_dir).expanduser().resolve()
        _atomic_write(folder / 'acceptance.json', json.dumps(report, ensure_ascii=False,
                                                             indent=2, allow_nan=False) + '\n')
        _atomic_write(folder / 'acceptance.md', format_markdown(report))
        print(json.dumps({'passed': report['passed'], 'summary': report['summary'],
                          'fault_matrix': report['fault_matrix'],
                          'json': str(folder / 'acceptance.json'),
                          'markdown': str(folder / 'acceptance.md')},
                         ensure_ascii=False, indent=2))
        return 0 if report['passed'] else 1
    except (PlanError, MissionError, AssertionError, OSError, ValueError) as exc:
        p.exit(2, 'ERROR: {}\n'.format(exc))


if __name__ == '__main__':
    raise SystemExit(main())
