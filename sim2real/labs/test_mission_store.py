"""Independent TEST-only mission replay store. Never accepts REAL or commands."""
import ipaddress
import json
import math
import threading
import time
from urllib.parse import urlsplit

LOCK = threading.RLock()
STATE = {'seq': 0, 'received_at': 0.0, 'snapshot': None}
ALLOWED = {'schema', 'source_type', 'mode', 'hardware_authorized', 'state', 'error',
           'completed_steps', 'total_steps', 'pick_count', 'place_count',
           'stack_count', 'home_confirmed', 'cmd_id', 'kind', 'event', 'stage_index',
           'node', 'x_m', 'y_m', 'pose_source', 'mock_executions', 'events'}
STATES = {'READY', 'WAIT_ACK', 'WAIT_RESULT', 'DONE', 'FAULT'}
EVENTS = {'READY', 'WAIT_ACK', 'COMPLETED', 'FAULT', 'WAIT_RESULT'}
NODES = {'A1', 'A2', 'A3', 'B1', 'B2', 'B3', 'C1', 'C2', 'C3'}


def validate(payload):
    if not isinstance(payload, dict) or set(payload) != ALLOWED:
        raise ValueError('Invalid TEST mission fields')
    if (payload['schema'] != 'car_test_mission_replay_v1'
            or payload['source_type'] != 'MOCK' or payload['mode'] != 'SIM_ONLY'
            or payload['hardware_authorized'] is not False
            or payload['pose_source'] != 'DISCRETE_ROAD_NODE'):
        raise ValueError('Not a mock mission replay')
    if payload['state'] not in STATES or payload['event'] not in EVENTS:
        raise ValueError('Invalid mock state')
    if payload['node'] not in NODES:
        raise ValueError('Invalid road node')
    if not isinstance(payload['kind'], str) or len(payload['kind']) > 32:
        raise ValueError('Invalid step kind')
    if not isinstance(payload['error'], str) or len(payload['error']) > 160:
        raise ValueError('Invalid error text')
    for key in ('x_m', 'y_m'):
        val = payload[key]
        if type(val) not in (int, float) or not math.isfinite(val) or not 0 <= val <= 2.4:
            raise ValueError('Invalid mock road position')
    for key in ('completed_steps', 'total_steps', 'pick_count', 'place_count',
                'stack_count', 'cmd_id', 'stage_index', 'mock_executions'):
        val = payload[key]
        if type(val) is not int or not 0 <= val <= 10000:
            raise ValueError('Invalid mock counter')
    if payload['completed_steps'] > payload['total_steps'] or payload['total_steps'] < 1:
        raise ValueError('Impossible mock progress')
    if type(payload['home_confirmed']) is not bool:
        raise ValueError('Invalid home flag')
    events = payload['events']
    if not isinstance(events, list) or len(events) > 16:
        raise ValueError('Invalid event list')
    cleaned = []
    for event in events:
        if not isinstance(event, dict) or not set(event) <= {'cmd_id', 'event', 'event_type', 'reason'}:
            raise ValueError('Invalid event object')
        if type(event.get('cmd_id')) is not int or event['cmd_id'] < 1 or event['cmd_id'] > 10000:
            raise ValueError('Invalid event command ID')
        if event.get('event') not in ('ACK', 'STARTED', 'PROGRESS', 'COMPLETED',
                                       'REJECTED', 'FAILED', 'FAULT'):
            raise ValueError('Invalid event type')
        cleaned.append({'cmd_id': event['cmd_id'], 'event': event['event'],
                        'reason': str(event.get('reason', ''))[:120]})
    safe = dict(payload)
    safe['events'] = cleaned
    return safe


def read():
    with LOCK:
        age = time.monotonic() - STATE['received_at'] if STATE['seq'] else None
        return {'source_type': 'MOCK', 'seq': STATE['seq'], 'online': bool(age is not None and age < 3),
                'age_s': round(age, 2) if age is not None else None,
                'snapshot': STATE['snapshot']}


def handle_get(handler):
    if urlsplit(handler.path).path != '/api/test/mission_replay':
        return False
    handler.respond_json(200, read())
    return True


def handle_post(handler):
    if urlsplit(handler.path).path != '/api/test/mission_replay':
        return False
    try:
        if not ipaddress.ip_address(handler.client_address[0]).is_loopback:
            handler.respond_json(403, {'error': 'TEST localhost only'})
            return True
    except ValueError:
        handler.respond_json(403, {'error': 'Invalid source address'})
        return True
    origin = handler.headers.get('Origin')
    if origin and urlsplit(origin).netloc.lower() != handler.headers.get('Host', '').lower():
        handler.respond_json(403, {'error': 'Cross-origin TEST publish forbidden'})
        return True
    try:
        if handler.headers.get('Content-Type', '').split(';', 1)[0].strip().lower() != 'application/json':
            raise ValueError('JSON Content-Type required')
        size = int(handler.headers.get('Content-Length', '0'))
        if not 0 < size <= 12000:
            raise ValueError('Invalid request size')
        payload = json.loads(handler.rfile.read(size),
                             parse_constant=lambda _: (_ for _ in ()).throw(ValueError('Nonfinite')))
        safe = validate(payload)
    except (ValueError, TypeError, KeyError, UnicodeError) as exc:
        handler.respond_json(400, {'error': str(exc)[:160]})
        return True
    with LOCK:
        STATE['seq'] += 1
        STATE['received_at'] = time.monotonic()
        STATE['snapshot'] = safe
        seq = STATE['seq']
    handler.respond_json(200, {'ok': True, 'seq': seq, 'source_type': 'MOCK'})
    return True
