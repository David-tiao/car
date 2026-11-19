"""TEST-only read-only diagnostic snapshot store for CAR-LINK simulator.

This module never publishes commands. It is deliberately separate from
/api/real/* so synthetic MCU events cannot populate the real dashboard.
"""
from __future__ import annotations

import ipaddress
import json
import math
import threading
import time
from urllib.parse import urlsplit

LOCK = threading.RLock()
TOPICS = ('chassis', 'wheels', 'link', 'mission', 'vision', 'feedback', 'mechanism', 'power')
ALLOWED_SAMPLE_KEYS = {
    'chassis': {'x_m', 'y_m', 'yaw_rad', 'vx_mps', 'vy_mps', 'wz_radps', 'chassis_mode', 'target_reached', 'imu_valid', 'target_x_m', 'target_y_m'},
    'wheels': {'target_pulse_per_s', 'actual_pulse_per_s', 'target_mps', 'actual_mps', 'max_abs_pid_output', 'per_wheel_pid_output_valid'},
    'link': {'connected', 'heartbeat_fresh', 'received_frames', 'received_heartbeats', 'crc_errors', 'length_errors', 'version_errors', 'payload_errors', 'discarded_bytes', 'peer_session_id'},
    'mission': {'phase_code', 'status_code', 'action_index', 'error_code', 'plan_index', 'plan_done', 'race_state_semantics_known'},
    'vision': {'event_type', 'raw_line', 'task_code', 'step', 'color_id', 'ring_id', 'pixel_u', 'pixel_v', 'pixel_position_valid', 'metric_position_valid', 'station', 'operation'},
    'feedback': {'event_type', 'cmd_id', 'original_command_type', 'status_code', 'progress_percent', 'phase_code', 'error_code', 'control_owner'},
    'mechanism': {'gripper_valid', 'gripper_angle_deg', 'turntable_valid', 'turntable_angle_deg', 'lift_valid', 'lift_height_mm', 'extension_valid', 'extension_mm', 'operation_active', 'fault_code'},
    'power': {'driver_voltage_valid', 'driver_voltage_v', 'board_voltage_valid', 'board_voltage_v'},
}
STORE = {'seq': 0, 'last_mono': 0.0, 'snapshot': None}


def _local(address):
    try:
        return ipaddress.ip_address(address).is_loopback
    except ValueError:
        return False


def _atom(value, depth=0):
    if depth > 2:
        raise ValueError('Nested sample too deep')
    if value is None or isinstance(value, bool):
        return value
    if isinstance(value, (int, float)):
        if not math.isfinite(value) or abs(value) > 1e12:
            raise ValueError('Invalid numeric sample')
        return value
    if isinstance(value, str):
        return value[:160]
    if isinstance(value, list):
        if len(value) > 8:
            raise ValueError('Unexpected array length')
        return [_atom(x, depth + 1) for x in value]
    raise ValueError('Unexpected sample type')


def validate(payload):
    if not isinstance(payload, dict) or payload.get('schema') != 'car_test_inspection_v1':
        raise ValueError('Unsupported TEST schema')
    if payload.get('source_type') != 'MOCK':
        raise ValueError('Only MOCK inspection accepted by TEST endpoint')
    topics = payload.get('topics')
    samples = payload.get('samples')
    events = payload.get('events', [])
    if not isinstance(topics, dict) or not isinstance(samples, dict) or not isinstance(events, list) or len(events) > 30:
        raise ValueError('Malformed inspection data')
    if set(topics) - set(TOPICS) or set(samples) - set(TOPICS):
        raise ValueError('Unknown ROS topic')
    clean_topics = {}
    clean_samples = {}
    for name, value in topics.items():
        if not isinstance(value, dict):
            raise ValueError('Malformed topic status')
        count = value.get('count', 0)
        age = value.get('age_s')
        hz = value.get('hz', 0)
        if type(count) is not int or not 0 <= count <= 2**53:
            raise ValueError('Bad topic count')
        for n in (age, hz):
            if n is not None and (not isinstance(n, (int, float)) or isinstance(n, bool) or not math.isfinite(n) or n < 0 or n > 100000):
                raise ValueError('Bad topic frequency or age')
        clean_topics[name] = {'count': count, 'age_s': age, 'hz': hz,
                              'fresh': bool(age is not None and age <= 1.5)}
    for name, value in samples.items():
        if not isinstance(value, dict) or set(value) - ALLOWED_SAMPLE_KEYS[name]:
            raise ValueError('Unexpected sample fields')
        clean_samples[name] = {key: _atom(val) for key, val in value.items()}
    clean_events = []
    for item in events:
        if not isinstance(item, dict):
            raise ValueError('Bad event')
        event_id = item.get('id')
        if type(event_id) is not int or not 0 <= event_id <= 2**53:
            raise ValueError('Bad event ID')
        kind = item.get('kind')
        if kind not in ('VISION', 'MISSION', 'COMMAND', 'LINK'):
            raise ValueError('Bad event type')
        clean_events.append({'id': event_id, 'kind': kind, 'text': str(item.get('text', ''))[:160]})
    return {'schema': 'car_test_inspection_v1', 'source_type': 'MOCK',
            'topics': clean_topics, 'samples': clean_samples, 'events': clean_events,
            'control_enabled': False}


def read():
    with LOCK:
        age = time.monotonic() - STORE['last_mono'] if STORE['seq'] else None
        return {'source_type': 'MOCK', 'seq': STORE['seq'],
                'online': age is not None and age < 2,
                'age_s': round(age, 3) if age is not None else None,
                'snapshot': STORE['snapshot']}


def handle_get(handler):
    if urlsplit(handler.path).path != '/api/test/inspection':
        return False
    handler.respond_json(200, read())
    return True


def handle_post(handler):
    if urlsplit(handler.path).path != '/api/test/inspection':
        return False
    if not _local(handler.client_address[0]):
        handler.respond_json(403, {'error': 'TEST publisher must use loopback'})
        return True
    origin = handler.headers.get('Origin')
    if origin and urlsplit(origin).netloc.lower() != handler.headers.get('Host', '').lower():
        handler.respond_json(403, {'error': 'Cross-origin publish forbidden'})
        return True
    try:
        if handler.headers.get('Content-Type', '').split(';', 1)[0].strip().lower() != 'application/json':
            raise ValueError('JSON required')
        size = int(handler.headers.get('Content-Length', '0'))
        if not 0 < size <= 24*1024:
            raise ValueError('Body must be <= 24 KiB')
        def reject_constant(x):
            raise ValueError('Nonfinite value: ' + x)
        payload = json.loads(handler.rfile.read(size), parse_constant=reject_constant)
        cleaned = validate(payload)
    except (ValueError, TypeError, UnicodeError, json.JSONDecodeError) as exc:
        handler.respond_json(400, {'error': str(exc)[:160]})
        return True
    with LOCK:
        STORE['seq'] += 1
        STORE['last_mono'] = time.monotonic()
        STORE['snapshot'] = cleaned
        seq = STORE['seq']
    handler.respond_json(200, {'ok': True, 'seq': seq, 'source_type': 'MOCK'})
    return True
