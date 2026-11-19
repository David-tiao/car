#!/usr/bin/env python3
"""Offline demo: existing Dijkstra road plan + pure simulated workflow.

Only SIM_ONLY intents and synthetic acknowledgements. No ROS2, UART or motors.
"""
import argparse
import json
from mission_execution import MissionError, SimExecutor, compile_workflow
from rule_mission_core import PlanError, build_mission


def main():
    parser = argparse.ArgumentParser(description='2027 比赛任务模拟闭环；不会控制真实硬件')
    parser.add_argument('--start-zone', choices=('upper_right', 'lower_right'), default='upper_right')
    parser.add_argument('--task-code', default='156+123+516+231')
    parser.add_argument('--block', action='append', default=[])
    parser.add_argument('--fail-at', type=int, help='可选：指定 cmd_id 模拟执行失败')
    args = parser.parse_args()
    try:
        plan = build_mission(args.start_zone, args.block, provenance='DEMO_ONLY')
        if plan['status'] == 'NO_ROUTE':
            print(json.dumps({'state': 'NO_ROUTE', 'hardware_authorized': False}, ensure_ascii=False))
            return 3
        steps = compile_workflow(plan, args.task_code)
        engine = SimExecutor(steps, args.task_code)
        simulated_ms = 0
        events = []
        while engine.state not in ('DONE', 'FAULT'):
            cmd = engine.issue(simulated_ms)
            if cmd is None:
                raise MissionError('模拟任务状态机停止推进')
            engine.feedback(cmd['cmd_id'], 'ACK', simulated_ms + 1)
            outcome = 'FAILED' if cmd['cmd_id'] == args.fail_at else 'COMPLETED'
            engine.feedback(cmd['cmd_id'], outcome, simulated_ms + 2,
                            qr_text=args.task_code if cmd['kind'] == 'SCAN_QR' else None)
            events.append({'id': cmd['cmd_id'], 'kind': cmd['kind'], 'result': outcome})
            simulated_ms += 10
        summary = engine.summary()
        print(json.dumps({'preview_status': plan['status'], 'start_zone': args.start_zone,
                          'blocked_roads': plan['blocked_roads'], 'road_distance_m': plan['total_road_distance_m'],
                          'workflow': summary, 'events': events}, ensure_ascii=False, indent=2))
        return 0 if engine.state == 'DONE' and summary['pick_count'] == 12 and summary['place_count'] == 12 else 4
    except (PlanError, MissionError, ValueError) as exc:
        parser.exit(2, 'ERROR: {}\n'.format(exc))


if __name__ == '__main__':
    raise SystemExit(main())
