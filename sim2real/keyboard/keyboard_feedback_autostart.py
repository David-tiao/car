"""Launch one read-only ROS2 command observer when localhost opens /keyboard.

The web server never publishes ROS2 motor commands. Only loopback page visits
may cause a background subscriber to start. Existing command publishers are
owned by the user's ROS2 session, not by this module.
"""
from __future__ import annotations

import os
from pathlib import Path
import subprocess
import threading
import time

ROOT = Path(__file__).resolve().parents[1]
ROS_SETUP = Path('/opt/ros/humble/setup.bash')
WS_SETUP = ROOT.parent / 'algorithm/car_ws/install/setup.bash'
BRIDGE = ROOT / 'bridges/ros2_keyboard_feedback_bridge.py'
LOG_PATH = ROOT / 'data/web_modes/keyboard_autostart.log'
_LOCK = threading.RLock()
_PROCESS = None
_LAST_ATTEMPT = 0.0
_ERROR = ''
_COOLDOWN = 5.0


def status():
    """Small status for the browser; never starts a process."""
    with _LOCK:
        exit_code = _PROCESS.poll() if _PROCESS is not None else None
        running = _PROCESS is not None and exit_code is None
        message = '' if running else _ERROR
        if not running and not message and _PROCESS is not None:
            message = f'ROS2 observer exited (code {exit_code}); see local log'
        return {
            'running': running,
            'enabled': ROS_SETUP.is_file(),
            'message': message,
            'topic': os.environ.get('CAR_KEYBOARD_TOPIC', '/cmd_vel'),
            'log': 'sim2real/data/web_modes/keyboard_autostart.log',
        }


def ensure_started(port, *, existing_feedback=False):
    """Idempotently start a subscriber for the server's own localhost port."""
    global _PROCESS, _LAST_ATTEMPT, _ERROR
    with _LOCK:
        if existing_feedback:
            return False  # A separately started ROS2 bridge already posts data.
        if _PROCESS is not None and _PROCESS.poll() is None:
            return False
        now = time.monotonic()
        if now - _LAST_ATTEMPT < _COOLDOWN:
            return False
        _LAST_ATTEMPT = now
        if not ROS_SETUP.is_file():
            _ERROR = 'ROS2 Humble not found at /opt/ros/humble/setup.bash'
            return False
        if not BRIDGE.is_file():
            _ERROR = 'Missing ROS2 keyboard feedback bridge'
            return False
        if not (1 <= int(port) <= 65535):
            _ERROR = 'Invalid local web server port'
            return False
        topic = os.environ.get('CAR_KEYBOARD_TOPIC', '/cmd_vel')
        stamped = os.environ.get('CAR_KEYBOARD_STAMPED', '').lower() in ('1', 'true', 'yes')
        hz = os.environ.get('CAR_KEYBOARD_HZ', '20')
        timeout = os.environ.get('CAR_KEYBOARD_TIMEOUT', '0.5')
        # All untrusted values are positional arguments, never interpolated into shell.
        source_command = (
            'source "$1" || exit 1; '
            'if [ -f "$2" ]; then source "$2" || exit 1; fi; '
            'shift 2; exec "$@"'
        )
        argv = [
            '/bin/bash', '-c', source_command, 'car-keyboard-ros2',
            str(ROS_SETUP), str(WS_SETUP),
            '/usr/bin/python3', str(BRIDGE),
            '--server', f'http://127.0.0.1:{port}',
            '--topic', topic, '--hz', hz, '--timeout', timeout,
            '--parent-pid', str(os.getpid()),
        ]
        if stamped:
            argv.append('--stamped')
        try:
            LOG_PATH.parent.mkdir(parents=True, exist_ok=True)
            with LOG_PATH.open('ab') as log:
                log.write(b'\n=== Opening /keyboard: start ROS2 read-only observer ===\n')
                log.flush()
                _PROCESS = subprocess.Popen(argv, stdin=subprocess.DEVNULL,
                                            stdout=log, stderr=subprocess.STDOUT,
                                            cwd=ROOT, close_fds=True)
            _ERROR = ''
            return True
        except (OSError, ValueError) as exc:
            _ERROR = 'Unable to start ROS2 observer: ' + str(exc)[:180]
            _PROCESS = None
            return False


def stop():
    """Stop only the ROS2 observer spawned by this particular website."""
    global _PROCESS
    with _LOCK:
        proc = _PROCESS
        _PROCESS = None
    if proc is None or proc.poll() is not None:
        return
    proc.terminate()
    try:
        proc.wait(timeout=2)
    except subprocess.TimeoutExpired:
        proc.kill()
        proc.wait(timeout=2)
