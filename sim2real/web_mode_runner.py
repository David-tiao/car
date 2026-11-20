#!/usr/bin/env python3
"""Own and clean up one four-mode ROS2-to-web session.

The original `car` website command is unchanged.  This supervisor only
launches subprocesses that belong to this session and never kills unrelated
processes or rewrites user shell aliases.
"""
from __future__ import annotations
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
from urllib.request import urlopen

ROOT = Path(__file__).resolve().parent
LOG_DIR = ROOT / 'data' / 'web_modes'
CHILDREN = []
LOG_FILES = []


def stop_children():
    for proc in reversed(CHILDREN):
        if proc.poll() is None:
            try: os.killpg(proc.pid, signal.SIGTERM)
            except ProcessLookupError: pass
    for proc in reversed(CHILDREN):
        try: proc.wait(timeout=4)
        except subprocess.TimeoutExpired:
            if proc.poll() is None:
                try: os.killpg(proc.pid, signal.SIGKILL)
                except ProcessLookupError: pass
                proc.wait(timeout=2)
    for file in LOG_FILES:
        file.close()


def spawn(tag, command):
    LOG_DIR.mkdir(parents=True, exist_ok=True)
    destination = LOG_DIR / (tag + '.log')
    fd = destination.open('a', encoding='utf-8')
    LOG_FILES.append(fd)
    proc = subprocess.Popen(command, stdin=subprocess.DEVNULL, stdout=fd,
                            stderr=subprocess.STDOUT, start_new_session=True)
    CHILDREN.append(proc)
    print(f'[START] {tag}: pid={proc.pid}, log={destination}', flush=True)
    return proc


def probe(url):
    try:
        with urlopen(url, timeout=.8) as res:
            return json.load(res)
    except (OSError, ValueError, json.JSONDecodeError):
        return None


def require_site(port, mode=None):
    url = f'http://127.0.0.1:{port}'
    found = probe(url + '/api/health')
    if found is not None:
        if found.get('service') != 'competition-v13-simulator':
            raise RuntimeError(f'Port {port} is occupied by a non-Sim2Real service')
        if not (probe(url + '/api/mode/status') or {}).get('channels'):
            raise RuntimeError('Existing car web process is an older version; stop car and restart it after installation')
        if mode == 'keyboard' and (probe(url + '/api/keyboard/feedback') or {}).get('source_type') != 'KEYBOARD':
            raise RuntimeError('Running car website is older than keyboard feedback patch; restart car')
        if mode == 'test' and (probe(url + '/api/test/inspection') or {}).get('source_type') != 'MOCK':
            raise RuntimeError('Running car website has old TEST API: stop it and rerun car')
        print('[REUSE] Existing Sim2Real HTTP server (for example, started by car)', flush=True)
        return url
    server = spawn('site', [sys.executable, str(ROOT / 'server.py'), '--host', '127.0.0.1', '--port', str(port)])
    for _ in range(40):
        if server.poll() is not None:
            raise RuntimeError(f'Web server exited with code {server.returncode}; inspect {LOG_DIR / "site.log"}')
        found = probe(url + '/api/health')
        if found and found.get('service') == 'competition-v13-simulator':
            return url
        time.sleep(.15)
    raise RuntimeError(f'Web server failed to become ready: {url}/api/health')


def check_port(path):
    if not path:
        raise RuntimeError('REAL requires CAR_REAL_PORT=/dev/ttyUSB0 or another verified hardware serial device')
    device = Path(path)
    if not device.exists():
        raise RuntimeError(f'Real serial device not found: {path}')
    resolved = str(device.resolve())
    if not resolved.startswith(('/dev/ttyUSB', '/dev/ttyACM', '/dev/ttyAMA', '/dev/ttyS')):
        raise RuntimeError(f'Refusing a mock or unverified serial path: {path} → {resolved}')
    return str(device)


def ros_pkg_exists(package):
    proc = subprocess.run(['ros2', 'pkg', 'prefix', package], capture_output=True,
                          text=True, timeout=8, check=False)
    if proc.returncode:
        raise RuntimeError(f'ROS2 package {package} unavailable; run colcon build and source car_ws/install/setup.bash')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('mode', choices=('test', 'real', 'keyboard'))
    parser.add_argument('--port', type=int, default=int(os.environ.get('CAR_PORT', '8080')))
    parser.add_argument('--real-port', default=os.environ.get('CAR_REAL_PORT', ''))
    parser.add_argument('--topic', default=os.environ.get('CAR_KEYBOARD_TOPIC', '/cmd_vel'))
    parser.add_argument('--stamped', action='store_true')
    parser.add_argument('--hz', type=float, default=20.0)
    parser.add_argument('--timeout', type=float, default=0.5)
    parser.add_argument('--protocol', choices=('carlink', 'ulog43'), default=os.environ.get('CAR_REAL_PROTOCOL', 'carlink'))
    args = parser.parse_args()
    if not 1024 <= args.port <= 65535:
        parser.error('Port must be within 1024..65535')
    if args.mode == 'keyboard':
        url = require_site(args.port, 'keyboard')
        command = [sys.executable, str(ROOT/'ros2_keyboard_feedback_bridge.py'),
                   '--server', url, '--topic', args.topic, '--hz', str(args.hz),
                   '--timeout', str(args.timeout)]
        if args.stamped:
            command.append('--stamped')
        spawn('keyboard_feedback_bridge', command)
        print(f'[WEB] {url}/keyboard', flush=True)
        print('[OBSERVE] Browser follows existing ROS2 velocity topic; NO motor commands issued.', flush=True)
        print('[STOP] Ctrl+C stops only this session and its child processes.', flush=True)
        while True:
            for process in CHILDREN:
                if process.poll() is not None:
                    raise RuntimeError(f'Keyboard child exited (code {process.returncode}); check {LOG_DIR}')
            time.sleep(.4)
    if args.mode == 'test':
        os.environ['ROS_DOMAIN_ID'] = os.environ.get('CAR_TEST_ROS_DOMAIN_ID', '73')
        print('[ISOLATION] TEST domain=' + os.environ['ROS_DOMAIN_ID'], flush=True)
    else:
        check_port(args.real_port)
    package = 'car_serial_link' if args.mode == 'test' or args.protocol == 'carlink' else 'car_stm32_bridge'
    ros_pkg_exists(package)
    port = check_port(args.real_port) if args.mode == 'real' else None
    url = require_site(args.port, args.mode)
    print(f'[WEB] {url}/{args.mode}', flush=True)
    if args.mode == 'test':
        launch = ['ros2', 'launch', 'car_serial_link', 'offline_serial_demo.launch.xml', 'with_debug:=false']
    elif args.protocol == 'carlink':
        launch = ['ros2', 'launch', 'car_serial_link', 'live_readonly.launch.xml', 'port:=' + port]
    else:
        launch = ['ros2', 'launch', 'car_stm32_bridge', 'monitor.launch.xml', 'port:=' + port]
    producer = spawn(args.mode + '_ros', launch)
    time.sleep(1)
    if producer.poll() is not None:
        raise RuntimeError('ROS launch exited early; inspect ' + str(LOG_DIR / (args.mode + '_ros.log')))
    bridge = spawn(args.mode + '_web_bridge', [sys.executable, str(ROOT / 'ros2_visualization_bridge.py'),
        '--server', url, '--web-channel', args.mode,
        '--plan', '/car/navigation/preview_path', '--road-path', '/competition/mission_road_path'])
    if args.mode == 'test':
        inspector = spawn('test_inspection_bridge', [sys.executable, str(ROOT / 'test_telemetry_bridge.py'),
            '--server', url])
        print('[TEST] Collecting wheels, vision, mission, link CRC, synthetic command feedback.', flush=True)
    if args.mode == 'test':
        spawn('test_path_bridge', [sys.executable, str(ROOT/'test_path_bridge.py'), '--server', url])
    print(f'[READY] Open {url}/{args.mode}; actual data may still be OFFLINE until received.', flush=True)
    print('[STOP] Ctrl+C stops only the subprocesses created by this launcher.', flush=True)
    while True:
        for p in CHILDREN:
            if p.poll() is not None:
                raise RuntimeError(f'Child exited with code {p.returncode}: PID {p.pid}; check {LOG_DIR}')
        time.sleep(.4)


def handle_term(signum, frame):
    raise KeyboardInterrupt


if __name__ == '__main__':
    signal.signal(signal.SIGTERM, handle_term)
    try:
        main()
    except KeyboardInterrupt:
        print('\n[STOP] Interrupted by user', flush=True)
    except (RuntimeError, OSError, subprocess.TimeoutExpired) as exc:
        print('[ERROR] ' + str(exc), file=sys.stderr, flush=True)
        sys.exit_code = 2
    finally:
        stop_children()
    if getattr(sys, 'exit_code', 0):
        raise SystemExit(sys.exit_code)
