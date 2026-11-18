#!/usr/bin/env python3
"""Safe self-hosted server relauncher; replaces an older copy of THIS simulator only."""
from __future__ import annotations
import os
import sys
import subprocess
import signal
import socket
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent
PORT = int(os.environ.get('CAR_PORT', '8080'))


def same_user_simulators():
    found = []
    parent = ROOT.parent.resolve()
    for proc in Path('/proc').iterdir():
        if not proc.name.isdigit() or int(proc.name) == os.getpid():
            continue
        try:
            if proc.stat().st_uid != os.getuid():
                continue
            args = (proc / 'cmdline').read_bytes().split(b'\0')
            cmd = [a.decode('utf-8', 'replace') for a in args if a]
            if not cmd or not any('server.py' in arg for arg in cmd):
                continue
            workdir = (proc / 'cwd').resolve()
            candidates = [(Path(arg) if Path(arg).is_absolute() else workdir / arg).resolve() for arg in cmd[1:] if arg.endswith('/server.py') or arg == 'server.py']
            if not candidates:
                continue
            valid = False
            for path in candidates:
                # Exact current installation, or previous legacy installation under ~/Downloads.
                if path == ROOT / 'server.py':
                    valid = True
                elif path.parent.parent == parent and path.parent.name in ('工创智能搬运_一键开站版',):
                    valid = True
            if valid:
                found.append(int(proc.name))
        except (PermissionError, FileNotFoundError, ProcessLookupError, OSError):
            continue
    return found


def port_available(port):
    with socket.socket() as sock:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            sock.bind(('0.0.0.0', port))
            return True
        except OSError:
            return False


def main():
    if not 1024 <= PORT <= 65535:
        sys.exit('Choose CAR_PORT 1024..65535')
    victims = same_user_simulators()
    if victims:
        print('Restarting previous simulator instance: ' + ', '.join(map(str, victims)), flush=True)
        for pid in victims:
            try: os.kill(pid, signal.SIGTERM)
            except ProcessLookupError: pass
        for _ in range(25):
            if port_available(PORT): break
            time.sleep(.12)
    if not port_available(PORT):
        sys.exit(f'Port {PORT} is busy with a different process. Check: sudo ss -lptn "sport = :{PORT}"')
    url = f'http://127.0.0.1:{PORT}/?build=13.0.0'
    print('Opening ' + url, flush=True)
    if os.environ.get('DISPLAY') or os.environ.get('WAYLAND_DISPLAY'):
        subprocess.Popen(['sh', '-c', f'sleep 1.2; command -v xdg-open >/dev/null && xdg-open "{url}" >/dev/null 2>&1 || true'],
                         stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True)
    os.execvp(sys.executable, [sys.executable, str(ROOT / 'server.py'), '--port', str(PORT)])


if __name__ == '__main__':
    main()
