"""Local ROS2 speed-command feedback API; no browser control and no STM32 output."""
from __future__ import annotations

import ipaddress
import json
import math
import threading
import time
from http.server import SimpleHTTPRequestHandler
from urllib.parse import urlsplit
from keyboard_feedback_autostart import ensure_started, status as bridge_status

_LOCK = threading.Lock()
_RECORD = {'seq': 0, 'received_mono': 0.0, 'sample': None}


def _loopback(addr):
    try:
        return ipaddress.ip_address(addr).is_loopback
    except ValueError:
        return False


def keyboard_get(handler):
    path = urlsplit(handler.path).path
    if path == '/keyboard':
        # Only requests originating on this computer may start a ROS2 subscriber.
        fetch_site = handler.headers.get('Sec-Fetch-Site', '')
        if (getattr(handler.server, 'keyboard_autostart_enabled', False)
                and _loopback(handler.client_address[0])
                and fetch_site in ('', 'none', 'same-origin')):
            with _LOCK:
                current = _RECORD['received_mono']
                has_bridge = bool(current and time.monotonic() - current < 2.0)
            ensure_started(handler.server.server_port, existing_feedback=has_bridge)
        handler.path = '/keyboard.html'
        SimpleHTTPRequestHandler.do_GET(handler)
        return True
    if path == '/api/keyboard/bridge':
        handler.respond_json(200, bridge_status())
        return True
    if path != '/api/keyboard/feedback':
        return False
    with _LOCK:
        age = (time.monotonic() - _RECORD['received_mono']) if _RECORD['seq'] else None
        online = age is not None and age < 2.0
        sample = dict(_RECORD['sample']) if _RECORD['sample'] else None
        handler.respond_json(200, {
            'source_type': 'KEYBOARD',
            'seq': _RECORD['seq'],
            'online': online,
            'age_s': round(age, 3) if age is not None else None,
            'command_recent': bool(online and sample and sample['command_recent']),
            'sample': sample if online else None,
        })
    return True


def keyboard_post(handler):
    if urlsplit(handler.path).path != '/api/keyboard/feedback':
        return False
    if not _loopback(handler.client_address[0]):
        handler.respond_json(403, {'error': 'Only localhost ROS2 bridge may publish feedback'})
        return True
    origin = handler.headers.get('Origin')
    if origin and urlsplit(origin).netloc.lower() != handler.headers.get('Host', '').lower():
        handler.respond_json(403, {'error': 'Cross-origin feedback upload rejected'})
        return True
    try:
        if handler.headers.get('Content-Type', '').split(';', 1)[0].strip().lower() != 'application/json':
            raise ValueError('Expected application/json')
        size = int(handler.headers.get('Content-Length', '0'))
        if not 0 < size <= 2048:
            raise ValueError('Invalid payload size')
        raw = json.loads(handler.rfile.read(size), parse_constant=lambda s: (_ for _ in ()).throw(ValueError('Nonfinite JSON constant')))
        if not isinstance(raw, dict):
            raise ValueError('Expected JSON object')
        sample = {}
        for name in ('x', 'y', 'yaw', 'vx', 'vy', 'wz'):
            value = raw.get(name)
            if type(value) not in (int, float) or not math.isfinite(value):
                raise ValueError('Invalid number: ' + name)
            sample[name] = float(value)
        if type(raw.get('command_recent')) is not bool:
            raise ValueError('command_recent must be boolean')
        sample['command_recent'] = raw['command_recent']
        topic = raw.get('topic', '/cmd_vel')
        if not isinstance(topic, str) or not topic.startswith('/') or len(topic) > 128:
            raise ValueError('Invalid topic name')
        sample['topic'] = topic
        sample['source'] = 'ros2_cmd_vel_integrated'
        sample['frame'] = 'estimated_local'
    except (ValueError, TypeError, OverflowError, json.JSONDecodeError) as exc:
        handler.respond_json(400, {'error': str(exc)[:160]})
        return True
    with _LOCK:
        _RECORD['seq'] += 1
        sample['seq'] = _RECORD['seq']
        _RECORD['sample'] = sample
        _RECORD['received_mono'] = time.monotonic()
        seq = _RECORD['seq']
    handler.respond_json(200, {'ok': True, 'seq': seq, 'source_type': 'KEYBOARD'})
    return True
