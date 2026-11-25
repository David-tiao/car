"""Start the existing safe PC ROS2 keyboard loopback when localhost opens /keyboard.

The web page keeps its original WASDQE mapping and existing
keyboard_ros2_pc.py / keyboard_ros2_web_feedback.py nodes. This module
does NOT create motor publishers, start a Raspberry Pi guard or talk to UART.
"""
from __future__ import annotations

import argparse
import ipaddress
import os
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time
from urllib.parse import urlsplit

ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT.parent / "algorithm/car_ws/bash/start_keyboard_local_ros2.sh"
SETUP = Path(os.environ.get("CAR_ROS_SETUP", "/opt/ros/humble/setup.bash")).expanduser().resolve()
LOG = ROOT / "data/web_modes/keyboard_local_autostart.log"

_LOCK = threading.RLock()
_PROCESS = None
_ATTEMPT_AT = 0.0
_ERROR = ""
RETRY_SECONDS = 5.0


def _local(address):
    try:
        return ipaddress.ip_address(address).is_loopback
    except ValueError:
        return False


def status():
    with _LOCK:
        result = _PROCESS.poll() if _PROCESS is not None else None
        running = _PROCESS is not None and result is None
        message = "" if running else _ERROR
        if not running and _PROCESS is not None and not message:
            message = f"ROS2 launcher exited ({result}); check log"
        return {
            "enabled": os.environ.get("CAR_KEYBOARD_AUTOSTART", "1") != "0",
            "running": running,
            "message": message,
            "log": "sim2real/data/web_modes/keyboard_local_autostart.log",
            "topic": "/car/teleop/local_test_twist",
            "motor_authorized": False,
        }


def ensure_started(port, *, existing_feedback=False):
    """At most one owner per website. Never auto-start if feedback is already live."""
    global _PROCESS, _ATTEMPT_AT, _ERROR
    with _LOCK:
        if existing_feedback or os.environ.get("CAR_KEYBOARD_AUTOSTART", "1") == "0":
            return False
        if _PROCESS is not None and _PROCESS.poll() is None:
            return False
        now = time.monotonic()
        if now - _ATTEMPT_AT < RETRY_SECONDS:
            return False
        _ATTEMPT_AT = now
        if not SETUP.is_file():
            _ERROR = f"ROS2 Humble missing: {SETUP}"
            return False
        if not SCRIPT.is_file():
            _ERROR = "Existing start_keyboard_local_ros2.sh not found"
            return False
        if not 1 <= int(port) <= 65535:
            _ERROR = "Invalid web port"
            return False
        try:
            LOG.parent.mkdir(parents=True, exist_ok=True)
            with LOG.open("ab") as log:
                log.write(b"\n===== /keyboard local ROS2 auto-start =====\n")
                log.flush()
                _PROCESS = subprocess.Popen(
                    [sys.executable, str(Path(__file__).resolve()),
                     "--worker", "--parent-pid", str(os.getpid()), "--port", str(port)],
                    cwd=ROOT, stdin=subprocess.DEVNULL, stdout=log,
                    stderr=subprocess.STDOUT, close_fds=True, start_new_session=True)
            _ERROR = ""
            return True
        except (OSError, ValueError) as exc:
            _PROCESS = None
            _ERROR = "Cannot launch ROS2 manager: " + str(exc)[:150]
            return False


def handle_get(handler):
    """Return True only when an API response was written; otherwise allow normal routing."""
    path = urlsplit(handler.path).path
    if path == "/api/keyboard/autostart":
        if not _local(handler.client_address[0]):
            handler.respond_json(403, {"error": "localhost only"})
        else:
            handler.respond_json(200, status())
        return True
    if path != "/keyboard":
        return False
    if not (getattr(handler.server, "keyboard_autostart_enabled", False)
            and _local(handler.client_address[0])):
        return False
    if handler.headers.get("Sec-Fetch-Site", "") not in ("", "none", "same-origin"):
        return False
    if handler.headers.get("Sec-Purpose", "") == "prefetch":
        return False
    origin = handler.headers.get("Origin")
    if origin and urlsplit(origin).netloc.lower() != handler.headers.get("Host", "").lower():
        return False
    try:
        # The real mapping module already isolates local loopback and future Pi routes.
        import keyboard_mapping_core
        current = keyboard_mapping_core.STATE.ros2_status()
        ready = current.get("online") is True and current.get("source") == "local_ros2"
    except (ImportError, AttributeError):
        ready = False
    ensure_started(handler.server.server_port, existing_feedback=ready)
    return False


def stop():
    """Terminate only the manager that this website owns."""
    global _PROCESS
    with _LOCK:
        process = _PROCESS
        _PROCESS = None
    if process is None or process.poll() is not None:
        return
    process.terminate()
    try:
        process.wait(timeout=3)
    except subprocess.TimeoutExpired:
        process.kill()
        process.wait(timeout=3)


def _stop_group(process):
    if process is None or process.poll() is not None:
        return
    try:
        os.killpg(process.pid, signal.SIGTERM)
        process.wait(timeout=3)
    except (ProcessLookupError, OSError):
        pass
    except subprocess.TimeoutExpired:
        try:
            os.killpg(process.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        process.wait(timeout=3)


def worker(parent_pid, port):
    """Monitor website lifetime, preventing orphan ROS2 pub/sub processes."""
    def terminate(_signal, _frame):
        raise SystemExit(0)

    signal.signal(signal.SIGTERM, terminate)
    env = os.environ.copy()
    env.setdefault("ROS_DOMAIN_ID", env.get("CAR_KEYBOARD_ROS_DOMAIN_ID", "42"))
    child = None
    try:
        child = subprocess.Popen(
            ["/bin/bash", str(SCRIPT), f"http://127.0.0.1:{port}"],
            cwd=ROOT.parent, env=env, stdin=subprocess.DEVNULL,
            start_new_session=True)
        while os.getppid() == parent_pid:
            rc = child.poll()
            if rc is not None:
                return rc
            time.sleep(0.4)
        print("Website closed: stopping keyboard ROS2 pub/sub", flush=True)
        return 0
    finally:
        _stop_group(child)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--worker", action="store_true")
    parser.add_argument("--parent-pid", type=int, default=0)
    parser.add_argument("--port", type=int, default=8080)
    args = parser.parse_args()
    if not args.worker or args.parent_pid < 1 or not 1 <= args.port <= 65535:
        parser.error("Only --worker --parent-pid N --port N are supported")
    sys.exit(worker(args.parent_pid, args.port))
