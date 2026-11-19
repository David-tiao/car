#!/usr/bin/env python3
"""Pure offline competition workflow; produces SIM_ONLY intents, NEVER UART/motor commands.

The real road planner remains rule_mission_core.build_mission. This module only
compiles its eight route legs plus a validated 2027 QR task into an auditable
sequence and advances on correlated ACK / COMPLETED events.
"""
from dataclasses import asdict, dataclass
import re


class MissionError(ValueError):
    """Invalid mission input or unsafe attempted state transition."""


@dataclass(frozen=True)
class TaskCode:
    batch1_colors: tuple
    batch1_rough_rings: tuple
    batch2_colors: tuple
    batch2_rough_rings: tuple


def parse_task_code(text):
    """Example: 156+123+516+231; no spaces, duplicate digits or unknown colors."""
    if not isinstance(text, str) or re.fullmatch(r'[1-6]{3}\+[1-3]{3}\+[1-6]{3}\+[1-3]{3}', text) is None:
        raise MissionError('任务码必须是四组三位数字，例如 156+123+516+231')
    a, b, c, d = (tuple(map(int, part)) for part in text.split('+'))
    if len(set(a)) != 3 or len(set(c)) != 3 or set(a) != set(c):
        raise MissionError('每批颜色必须各不相同，且两批颜色集合相同')
    if set(b) != {1, 2, 3} or set(d) != {1, 2, 3}:
        raise MissionError('粗加工环号必须是 1、2、3 的排列')
    return TaskCode(a, b, c, d)


@dataclass(frozen=True)
class SimStep:
    cmd_id: int
    kind: str
    stage_index: int
    node: str = ''
    road_id: str = ''
    batch: int = 0
    color_id: int = 0
    ring_id: int = 0

    def intent(self):
        return dict(asdict(self), interface='SIM_ONLY', hardware_authorized=False)


STAGE_ACTIONS = ('SCAN_QR', 'PICK_BATCH_1', 'ROUGH_PROCESS_BATCH_1',
                 'STORE_BATCH_1', 'PICK_BATCH_2', 'ROUGH_PROCESS_BATCH_2',
                 'STACK_BATCH_2', 'RETURN_START')


def compile_workflow(plan, task_text):
    """Make route-node intents and 24 manipulation intents, not motor setpoints.

    A road node is only a topology anchor, not an actual camera/arm stop pose.
    This function intentionally does not import serial, ROS2 or hardware APIs.
    """
    code = parse_task_code(task_text)
    if not isinstance(plan, dict) or plan.get('status') not in (
            'DEMO_ROUTE_ONLY', 'PREVIEW_WAIT_STATIONS', 'PREVIEW_WITH_STATIONS'):
        raise MissionError('仅接受已有 rule_mission_core 的正常道路预览')
    if plan.get('commands_enabled') is not False or plan.get('NOT_FOR_NAV2_FOLLOWPATH') is not True:
        raise MissionError('路线不能声明真实运动权限')
    legs = plan.get('legs')
    if not isinstance(legs, list) or len(legs) != 8:
        raise MissionError('比赛路线必须包含 8 段')
    blocked = set(plan.get('blocked_roads', []))
    base_ring_by_color = dict(zip(code.batch1_colors, code.batch1_rough_rings))
    steps = []

    def add(kind, stage, **fields):
        steps.append(SimStep(len(steps) + 1, kind, stage, **fields))

    previous_node = None
    for stage, leg in enumerate(legs, 1):
        if not isinstance(leg, dict) or leg.get('stage_index') != stage or leg.get('action') != STAGE_ACTIONS[stage - 1]:
            raise MissionError('路线阶段序号或任务语义不一致')
        nodes, roads = leg.get('node_path'), leg.get('road_ids')
        if not isinstance(nodes, list) or not nodes or not all(isinstance(n, str) and n for n in nodes):
            raise MissionError('第 {} 段缺少合法道路节点'.format(stage))
        if not isinstance(roads, list) or len(roads) != len(nodes) - 1:
            raise MissionError('第 {} 段道路数量不匹配'.format(stage))
        if nodes[0] != leg.get('start_node') or nodes[-1] != leg.get('goal_node'):
            raise MissionError('第 {} 段起终点不一致'.format(stage))
        if previous_node is not None and nodes[0] != previous_node:
            raise MissionError('相邻路段首尾不连续')
        if any(not isinstance(road, str) or not road or road in blocked for road in roads):
            raise MissionError('路线经过封锁道路或包含非法道路')
        previous_node = nodes[-1]
        for node, road in zip(nodes[1:], roads):
            add('MOVE_NODE', stage, node=node, road_id=road)
        if stage == 1:
            add('SCAN_QR', stage)
        elif stage in (2, 5):
            batch = 1 if stage == 2 else 2
            colors = code.batch1_colors if batch == 1 else code.batch2_colors
            for color in colors:
                add('PICK_RAW', stage, batch=batch, color_id=color)
        elif stage in (3, 6):
            batch = 1 if stage == 3 else 2
            colors = code.batch1_colors if batch == 1 else code.batch2_colors
            rings = code.batch1_rough_rings if batch == 1 else code.batch2_rough_rings
            for color, ring in zip(colors, rings):
                add('PLACE_ROUGH', stage, batch=batch, color_id=color, ring_id=ring)
            for color, ring in zip(colors, rings):
                add('PICK_ROUGH', stage, batch=batch, color_id=color, ring_id=ring)
        elif stage == 4:
            for color in code.batch1_colors:
                add('PLACE_STORE', stage, batch=1, color_id=color,
                    ring_id=base_ring_by_color[color])
        elif stage == 7:
            for color in code.batch2_colors:
                add('STACK_STORE', stage, batch=2, color_id=color,
                    ring_id=base_ring_by_color[color])
        else:
            add('HOME', stage)
    if len([s for s in steps if s.kind.startswith('PICK_')]) != 12:
        raise MissionError('抓取总数必须为 12')
    if len([s for s in steps if s.kind.startswith('PLACE_')]) != 9:
        raise MissionError('非码垛放置总数必须为 9')
    if len([s for s in steps if s.kind == 'STACK_STORE']) != 3:
        raise MissionError('码垛总数必须为 3')
    return tuple(steps)


class SimExecutor:
    """Single in-flight simulated command with ACK, terminal result and timeouts.

    Designed to catch protocol/state-machine mistakes *before* a real transport
    is written. It never authorizes real motion. Monotonic ms supplied by caller.
    """
    def __init__(self, steps, expected_qr, ack_timeout_ms=1000, result_timeout_ms=30000):
        parse_task_code(expected_qr)
        if not steps or any(s.cmd_id != i for i, s in enumerate(steps, 1)):
            raise MissionError('步骤不能为空，cmd_id 必须从 1 连续递增')
        if ack_timeout_ms <= 0 or result_timeout_ms <= 0:
            raise MissionError('超时阈值必须为正整数')
        self.steps = tuple(steps)
        self.expected_qr = expected_qr
        self.ack_timeout_ms = ack_timeout_ms
        self.result_timeout_ms = result_timeout_ms
        self.index = 0
        self.state = 'READY'
        self.deadline_ms = None
        self.last_time_ms = -1
        self.error = ''
        self.completed = []

    def _clock(self, now_ms):
        if type(now_ms) is not int or now_ms < self.last_time_ms or now_ms < 0:
            raise MissionError('时间必须是非递减的整数毫秒')
        self.last_time_ms = now_ms

    def _fault(self, reason):
        self.state = 'FAULT'
        self.error = reason
        self.deadline_ms = None

    def issue(self, now_ms):
        self._clock(now_ms)
        if self.state != 'READY':
            return None
        self.state = 'WAIT_ACK'
        self.deadline_ms = now_ms + self.ack_timeout_ms
        return self.steps[self.index].intent()

    def feedback(self, cmd_id, event, now_ms, qr_text=None):
        self._clock(now_ms)
        if self.state not in ('WAIT_ACK', 'WAIT_RESULT'):
            return False
        current = self.steps[self.index]
        if type(cmd_id) is not int or cmd_id != current.cmd_id:
            return False  # stale/duplicate/foreign feedback; never advances
        if self.deadline_ms is not None and now_ms > self.deadline_ms:
            self._fault('TIMEOUT cmd_id={}'.format(cmd_id))
            return False
        if event in ('REJECTED', 'FAILED'):
            self._fault('{} cmd_id={}'.format(event, cmd_id))
            return True
        if event == 'ACK' and self.state == 'WAIT_ACK':
            self.state = 'WAIT_RESULT'
            self.deadline_ms = now_ms + self.result_timeout_ms
            return True
        if event == 'COMPLETED' and self.state == 'WAIT_RESULT':
            if current.kind == 'SCAN_QR' and qr_text != self.expected_qr:
                self._fault('QR_MISMATCH cmd_id={}'.format(cmd_id))
                return True
            self.completed.append(current)
            self.index += 1
            self.deadline_ms = None
            self.state = 'DONE' if self.index == len(self.steps) else 'READY'
            return True
        return False  # COMPLETED before ACK, duplicate ACK, unknown event

    def tick(self, now_ms):
        self._clock(now_ms)
        if self.deadline_ms is not None and now_ms > self.deadline_ms:
            self._fault('TIMEOUT cmd_id={}'.format(self.steps[self.index].cmd_id))
        return self.state

    def summary(self):
        return {'mode': 'SIM_ONLY', 'hardware_authorized': False,
                'state': self.state, 'error': self.error,
                'completed_steps': len(self.completed), 'total_steps': len(self.steps),
                'pick_count': sum(s.kind.startswith('PICK_') for s in self.completed),
                'place_count': sum(s.kind.startswith('PLACE_') or s.kind == 'STACK_STORE'
                                   for s in self.completed),
                'stack_count': sum(s.kind == 'STACK_STORE' for s in self.completed),
                'home_confirmed': bool(self.completed and self.completed[-1].kind == 'HOME')}
