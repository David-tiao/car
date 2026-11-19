"""Mode-specific read-only HTTP inputs for Sim2Real; original APIs remain intact.

Test and real state are physically separate in memory.  This gate is not an
authentication service: only run the REAL ROS bridge against verified hardware.
"""
from __future__ import annotations

import base64
from collections import deque
import ipaddress
import json
import math
import threading
import time
from http.server import SimpleHTTPRequestHandler
from urllib.parse import urlsplit, parse_qs
import test_inspection

MODES = ('test', 'real')
LAYER_NAMES = ('map', 'global_costmap', 'local_costmap')
ALLOWED_ROS_KEYS = ('display_frame', 'scan', 'robot', 'odom', 'paths', 'markers',
                    'cmd_vel', 'tf', 'topics', 'nodes', 'nav', 'diagnostics', 'bridge')
LOCK = threading.RLock()


def _fresh_state():
    return {
        'telemetry': {'seq': 0, 'last_mono': 0.0, 'samples': deque(maxlen=3000)},
        'ros': {'seq': 0, 'last_mono': 0.0, 'snapshot': None},
        'grids': {name: {'seq': 0, 'last_mono': 0.0, 'grid': None} for name in LAYER_NAMES},
    }


CHANNELS = {name: _fresh_state() for name in MODES}
TEST_PATH = {"seq":0,"updated":0.0,"points":[]}


def _is_local(address):
    try:
        return ipaddress.ip_address(address).is_loopback
    except ValueError:
        return False


def _json_body(handler, limit):
    if handler.headers.get('Content-Type', '').split(';', 1)[0].strip().lower() != 'application/json':
        raise ValueError('JSON Content-Type required')
    try:
        length = int(handler.headers.get('Content-Length', '0'))
    except (ValueError, TypeError):
        raise ValueError('Invalid Content-Length') from None
    if not 0 < length <= limit:
        raise ValueError('Invalid body size')
    def reject_constant(value):
        raise ValueError('Nonfinite JSON value: ' + value)
    return json.loads(handler.rfile.read(length), parse_constant=reject_constant)


def _grid(payload):
    if not isinstance(payload, dict) or payload.get('name') not in LAYER_NAMES:
        raise ValueError('Unsupported grid layer')
    width, height = int(payload.get('width', 0)), int(payload.get('height', 0))
    resolution = float(payload.get('resolution', 0))
    ox = float(payload.get('origin_x', 0))
    oy = float(payload.get('origin_y', 0))
    yaw = float(payload.get('origin_yaw', 0))
    if not 1 <= width <= 1200 or not 1 <= height <= 1200 or width * height > 1500000:
        raise ValueError('Invalid grid dimensions')
    if not all(map(math.isfinite, (resolution, ox, oy, yaw))) or not 0.001 <= resolution <= 5:
        raise ValueError('Invalid grid geometry')
    data = payload.get('data')
    if not isinstance(data, str) or len(data) > 2100000 or len(base64.b64decode(data, validate=True)) != width * height:
        raise ValueError('Invalid occupancy data')
    return payload['name'], {'width': width, 'height': height, 'resolution': resolution,
                             'origin_x': ox, 'origin_y': oy, 'origin_yaw': yaw,
                             'frame': str(payload.get('frame', 'map'))[:80], 'data': data}


def _online(then, limit):
    return bool(then and time.monotonic() - then < limit)


def _ros_status(mode):
    with LOCK:
        state = CHANNELS[mode]
        record = state['ros']
        age = time.monotonic() - record['last_mono'] if record['seq'] else None
        result = dict(record['snapshot'] or {})
        result.update({'source_type': mode.upper(),
                       'online': age is not None and age < 3.0,
                       'age_s': round(age, 2) if age is not None else None,
                       'seq': record['seq'],
                       'grids': {k: {'seq': v['seq'], 'frame': (v['grid'] or {}).get('frame', ''),
                                     'width': (v['grid'] or {}).get('width', 0),
                                     'height': (v['grid'] or {}).get('height', 0),
                                     'age_s': round(time.monotonic()-v['last_mono'], 2) if v['seq'] else None}
                                 for k, v in state['grids'].items()}})
        return result


def _telemetry_status(mode, since):
    with LOCK:
        record = CHANNELS[mode]['telemetry']
        age = time.monotonic() - record['last_mono'] if record['seq'] else None
        rows = [r for r in record['samples'] if r['seq'] > since]
        if since == 0:
            rows = rows[-1:]
        return {'source_type': mode.upper(), 'online': age is not None and age < 2.0,
                'age_s': round(age, 3) if age is not None else None,
                'seq': record['seq'], 'samples': rows[-150:]}


def mode_get(handler):
    """Return True when request was handled, else allow the old server to run."""
    if test_inspection.handle_get(handler):
        return True
    path = urlsplit(handler.path).path
    if path in ('/sim', '/test', '/real', '/keyboard'):
        handler.path = '/index.html'
        SimpleHTTPRequestHandler.do_GET(handler)
        return True
    if path == '/api/test/reference_path':
        with LOCK:
            age = time.monotonic()-TEST_PATH['updated'] if TEST_PATH['updated'] else None
            online = age is not None and age < 3.0
            handler.respond_json(200, {
                'source_type':'TEST',
                'seq':TEST_PATH['seq'],
                'online':online,
                'points':TEST_PATH['points'] if online else []
            })
        return True
    if path == '/api/mode/status':
        with LOCK:
            status = {k: {'telemetry_online': _online(v['telemetry']['last_mono'], 2.0),
                          'telemetry_seq': v['telemetry']['seq'],
                          'ros_online': _online(v['ros']['last_mono'], 3.0),
                          'ros_seq': v['ros']['seq']} for k, v in CHANNELS.items()}
        handler.respond_json(200, {'channels': status, 'keyboard_control': False})
        return True
    parts = path.split('/')
    if len(parts) < 4 or parts[1] != 'api' or parts[2] not in MODES:
        return False
    mode, leaf = parts[2], '/'.join(parts[3:])
    if leaf == 'telemetry':
        try:
            since = int(parse_qs(urlsplit(handler.path).query).get('after', ['0'])[0])
            if since < 0: raise ValueError()
        except (ValueError, TypeError):
            handler.respond_json(400, {'error': 'Invalid after sequence'})
            return True
        handler.respond_json(200, _telemetry_status(mode, since))
        return True
    if leaf == 'ros/state':
        handler.respond_json(200, _ros_status(mode))
        return True
    if len(parts) == 6 and parts[3] == 'ros' and parts[4] == 'grid':
        name = parts[5]
        if name not in LAYER_NAMES:
            handler.respond_json(404, {'error': 'Unknown grid layer'})
        else:
            with LOCK:
                record = CHANNELS[mode]['grids'][name]
                handler.respond_json(200, {'seq': record['seq'], 'grid': record['grid']})
        return True
    handler.respond_json(404, {'error': 'Unknown scoped API'})
    return True


def mode_post(handler):
    if test_inspection.handle_post(handler):
        return True
    path = urlsplit(handler.path).path
    if path == '/api/test/reference_path':
        if not _is_local(handler.client_address[0]):
            handler.respond_json(403, {'error':'TEST planner requires localhost'})
            return True
        origin = handler.headers.get('Origin')
        if origin and urlsplit(origin).netloc.lower() != handler.headers.get('Host','').lower():
            handler.respond_json(403, {'error':'Cross-origin planner rejected'})
            return True
        try:
            raw = _json_body(handler,24000)['points']
            if not isinstance(raw,list) or not 2 <= len(raw) <= 250:
                raise ValueError('Expected 2..250 waypoints')
            points = []
            for q in raw:
                if not isinstance(q,list) or len(q)!=2 or any(type(v) not in (int,float) for v in q):
                    raise ValueError('Point must be two numbers')
                x,y = q
                if not (math.isfinite(x) and math.isfinite(y) and 0<=x<=2.4 and 0<=y<=2.4):
                    raise ValueError('Point outside field')
                pt = [round(x,4),round(y,4)]
                if not points or math.dist(pt,points[-1])>.002:
                    points.append(pt)
            if len(points)<2:
                raise ValueError('Need distinct waypoints')
            with LOCK:
                if TEST_PATH['points'] != points:
                    TEST_PATH['seq'] += 1
                    TEST_PATH['points'] = points
                TEST_PATH['updated'] = time.monotonic()
                seq = TEST_PATH['seq']
            handler.respond_json(200, {
                'ok':True,'seq':seq,'count':len(points),'source_type':'TEST'
            })
        except (ValueError,TypeError,KeyError) as e:
            handler.respond_json(400, {'error':str(e)[:160]})
        return True
    if path == '/api/keyboard/command':
        handler.respond_json(403, {'error': 'Real keyboard control is not implemented'})
        return True
    parts = path.split('/')
    if len(parts) != 4 and len(parts) != 5:
        return False
    if parts[1] != 'api' or parts[2] not in MODES:
        return False
    mode = parts[2]
    leaf = '/'.join(parts[3:])
    if leaf not in ('telemetry', 'ros/state', 'ros/grid'):
        handler.respond_json(404, {'error': 'Unknown scoped API'})
        return True
    if not _is_local(handler.client_address[0]):
        handler.respond_json(403, {'error': 'Only loopback publishers are allowed'})
        return True
    origin = handler.headers.get('Origin')
    if origin and urlsplit(origin).netloc.lower() != handler.headers.get('Host', '').lower():
        handler.respond_json(403, {'error': 'Cross-origin publish rejected'})
        return True
    try:
        payload = _json_body(handler, 2*1024*1024 if leaf == 'ros/grid' else 240*1024 if leaf == 'ros/state' else 4096)
        if not isinstance(payload, dict):
            raise ValueError('Expected JSON object')
        if leaf == 'telemetry':
            row = {}
            for key, limit in (('x', 100), ('y', 100), ('yaw', 10000),
                               ('vx', 10), ('vy', 10), ('wz', 100)):
                v = float(payload[key])
                if not math.isfinite(v) or abs(v) > limit:
                    raise ValueError('Invalid '+key)
                row[key] = v
            row['frame'] = str(payload.get('frame', 'odom'))[:64]
            row['source'] = (mode.upper() + ':' + str(payload.get('source', 'ros2_odom')))[:64]
            row['source_type'] = mode.upper()  # NEVER trust a caller-supplied source_type
            with LOCK:
                store = CHANNELS[mode]['telemetry']
                store['seq'] += 1
                row['seq'] = store['seq']
                row['received_at'] = time.time()
                store['last_mono'] = time.monotonic()
                store['samples'].append(row)
                seq = row['seq']
        elif leaf == 'ros/state':
            if not isinstance(payload.get('topics', {}), dict) or not isinstance(payload.get('paths', {}), dict):
                raise ValueError('Invalid ROS topics / paths')
            snapshot = {k: payload[k] for k in ALLOWED_ROS_KEYS if k in payload}
            with LOCK:
                store = CHANNELS[mode]['ros']
                store['seq'] += 1
                store['last_mono'] = time.monotonic()
                store['snapshot'] = snapshot
                seq = store['seq']
        else:
            name, grid = _grid(payload)
            with LOCK:
                store = CHANNELS[mode]['grids'][name]
                store['seq'] += 1
                store['last_mono'] = time.monotonic()
                store['grid'] = grid
                seq = store['seq']
    except (ValueError, TypeError, KeyError, json.JSONDecodeError, base64.binascii.Error) as err:
        handler.respond_json(400, {'error': str(err)[:160]})
        return True
    handler.respond_json(200, {'ok': True, 'seq': seq, 'source_type': mode.upper()})
    return True
