#!/usr/bin/env python3
"""ROS 2 -> /api/test/inspection, consuming only CAR-LINK MOCK topic feedback.

No command publisher, no hardware access, and no synthetic browser data.
Missing/unvalidated physical quantities remain None rather than fabricated 0.
"""
from __future__ import annotations

import argparse
from collections import deque
import json
import math
import sys
import time
from urllib.error import URLError, HTTPError
from urllib.request import Request, urlopen

TOPICS = {
    'chassis': '/car/chassis/telemetry',
    'wheels': '/car/chassis/wheels',
    'link': '/car/stm32/link_status',
    'mission': '/car/mission/state',
    'vision': '/car/vision/event',
    'feedback': '/car/stm32/command_feedback',
    'mechanism': '/car/mechanism/state',
    'power': '/car/power',
}
FEEDBACK = {0x90: 'ACK（仅收到，不代表完成）', 0x91: 'REJECTED',
            0x92: 'STARTED', 0x93: 'PROGRESS', 0x94: 'COMPLETED',
            0x95: 'FAILED', 0x96: 'CONTROL_STATUS'}


def finite(value):
    """Return None for invalid quantities; NEVER convert missing values to zero."""
    if isinstance(value, bool):
        return value
    if isinstance(value, int):
        return value
    if isinstance(value, float):
        return round(value, 5) if math.isfinite(value) else None
    return None


def selected(msg, fields):
    return {name: finite(getattr(msg, name)) if isinstance(getattr(msg, name), (float, int))
            and not isinstance(getattr(msg, name), bool) else getattr(msg, name)
            for name in fields}


def wheel_snapshot(m):
    result = {}
    for field in ('target_pulse_per_s', 'actual_pulse_per_s', 'target_mps', 'actual_mps'):
        values = list(getattr(m, field))
        if len(values) != 4:
            raise ValueError('Expected four wheel channels FL/FR/RL/RR')
        result[field] = [finite(value) for value in values]
    result['max_abs_pid_output'] = finite(m.max_abs_pid_output)
    result['per_wheel_pid_output_valid'] = bool(m.per_wheel_pid_output_valid)
    return result


def decode(name, msg):
    if name == 'chassis':
        return selected(msg, ('x_m', 'y_m', 'yaw_rad', 'vx_mps', 'vy_mps', 'wz_radps',
                              'chassis_mode', 'target_reached', 'imu_valid', 'target_x_m', 'target_y_m'))
    if name == 'wheels':
        return wheel_snapshot(msg)
    if name == 'link':
        return selected(msg, ('connected', 'heartbeat_fresh', 'received_frames', 'received_heartbeats',
                              'crc_errors', 'length_errors', 'version_errors', 'payload_errors',
                              'discarded_bytes', 'peer_session_id'))
    if name == 'mission':
        return selected(msg, ('phase_code', 'status_code', 'action_index', 'error_code',
                              'plan_index', 'plan_done', 'race_state_semantics_known'))
    if name == 'vision':
        result = selected(msg, ('event_type', 'raw_line', 'task_code', 'step', 'color_id', 'ring_id',
                                'pixel_u', 'pixel_v', 'pixel_position_valid', 'metric_position_valid',
                                'station', 'operation'))
        result['raw_line'] = result['raw_line'][:140]
        return result
    if name == 'feedback':
        return selected(msg, ('event_type', 'cmd_id', 'original_command_type', 'status_code',
                              'progress_percent', 'phase_code', 'error_code', 'control_owner'))
    if name == 'mechanism':
        return selected(msg, ('gripper_valid', 'gripper_angle_deg', 'turntable_valid',
                              'turntable_angle_deg', 'lift_valid', 'lift_height_mm',
                              'extension_valid', 'extension_mm', 'operation_active', 'fault_code'))
    if name == 'power':
        return selected(msg, ('driver_voltage_valid', 'driver_voltage_v', 'board_voltage_valid', 'board_voltage_v'))
    raise KeyError(name)


class MonitorState:
    def __init__(self):
        self.messages = {}
        self.received = {name: deque(maxlen=48) for name in TOPICS}
        self.events = deque(maxlen=24)
        self.event_serial = 0
        self.last_event_key = {}
        self.totals = {name: 0 for name in TOPICS}

    def event(self, kind, text, fingerprint=None):
        if fingerprint is not None and self.last_event_key.get(kind) == fingerprint:
            return
        if fingerprint is not None:
            self.last_event_key[kind] = fingerprint
        self.event_serial += 1
        self.events.append({'id': self.event_serial, 'kind': kind, 'text': text})

    def consume(self, name, msg, now=None):
        now = time.monotonic() if now is None else now
        sample = decode(name, msg)
        self.messages[name] = sample
        self.received[name].append(now)
        self.totals[name] += 1
        if name == 'vision':
            kind = sample.get('event_type') or 'VISION'
            self.event('VISION', f'{kind} | step={sample.get("step")} | color={sample.get("color_id")} | pixel=({sample.get("pixel_u")}, {sample.get("pixel_v")})',
                       (kind, sample.get('step'), sample.get('raw_line'), sample.get('pixel_u'), sample.get('pixel_v')))
        elif name == 'feedback':
            v = sample.get('event_type')
            self.event('COMMAND', f'合成反馈 {FEEDBACK.get(v, str(v))} | cmd={sample.get("cmd_id")} | {sample.get("progress_percent")}%')
        elif name == 'mission':
            self.event('MISSION', f'模拟任务 phase={sample.get("phase_code")} / status={sample.get("status_code")} / action={sample.get("action_index")}',
                       (sample.get('phase_code'), sample.get('status_code'), sample.get('action_index')))
        elif name == 'link':
            self.event('LINK', '串口心跳正常' if sample.get('connected') and sample.get('heartbeat_fresh') else '串口无有效心跳',
                       (sample.get('connected'), sample.get('heartbeat_fresh')))

    def snapshot(self, now=None):
        now = time.monotonic() if now is None else now
        status = {}
        for name, times in self.received.items():
            age = max(0, now-times[-1]) if times else None
            hz = (len(times)-1)/(times[-1]-times[0]) if len(times)>1 and times[-1]>times[0] else 0.0
            status[name] = {'count': self.totals[name],
                            'age_s': round(age, 3) if age is not None else None,
                            'hz': round(hz, 1), 'fresh': bool(age is not None and age <= 1.5)}
        return {'schema': 'car_test_inspection_v1', 'source_type': 'MOCK',
                'topics': status, 'samples': self.messages,
                'events': list(self.events), 'control_enabled': False}


def main():
    parser = argparse.ArgumentParser(description='CAR-LINK PTY TEST telemetry and event monitor (read-only)')
    parser.add_argument('--server', default='http://127.0.0.1:8080')
    parser.add_argument('--hz', type=float, default=4)
    args = parser.parse_args()
    if not args.server.startswith(('http://127.0.0.1:', 'http://localhost:', 'http://[::1]:')):
        parser.error('TEST HTTP publisher must target localhost')
    if not 1 <= args.hz <= 10:
        parser.error('hz must be between 1 and 10')
    try:
        import rclpy
        from rclpy.node import Node
        from car_interfaces.msg import (ChassisTelemetry, WheelTelemetry, LinkStatus, MissionState,
                                        VisionEvent, CommandFeedback, MechanismState, PowerTelemetry)
    except ImportError as exc:
        parser.error('Missing ROS 2 Humble or car_interfaces (source workspace and rebuild): ' + str(exc))
    types = dict(zip(TOPICS, (ChassisTelemetry, WheelTelemetry, LinkStatus, MissionState,
                               VisionEvent, CommandFeedback, MechanismState, PowerTelemetry)))
    rclpy.init()

    class TestMonitor(Node):
        def __init__(self):
            super().__init__('car_web_test_monitor')
            self.state = MonitorState()
            self.last_warn = 0.
            self.subs = [self.create_subscription(types[name], topic,
                         lambda msg, key=name: self.receive(key, msg), 10)
                         for name, topic in TOPICS.items()]
            self.timer = self.create_timer(1/args.hz, self.send)
            self.get_logger().info('TEST ONLY: subscribing to CAR-LINK mock ROS2 topics; no publishers/commands')

        def receive(self, name, msg):
            try:
                self.state.consume(name, msg)
            except (ValueError, KeyError, TypeError, AttributeError) as exc:
                self.get_logger().warning(f'Discarded invalid {name} message: {exc}')

        def send(self):
            data = json.dumps(self.state.snapshot(), ensure_ascii=False, allow_nan=False,
                              separators=(',', ':')).encode('utf-8')
            request = Request(args.server.rstrip('/') + '/api/test/inspection', data=data,
                              method='POST', headers={'Content-Type': 'application/json'})
            try:
                with urlopen(request, timeout=.55) as res:
                    if res.status != 200:
                        raise OSError(f'HTTP {res.status}')
            except (OSError, HTTPError, URLError) as exc:
                now = time.monotonic()
                if now-self.last_warn > 5:
                    self.last_warn = now
                    self.get_logger().warning(f'TEST inspection HTTP unavailable: {exc}')

    node = TestMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
