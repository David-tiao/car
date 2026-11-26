#!/usr/bin/env python3
"""CSI barrel distortion experiment; no installation or actuator access."""
import argparse
import json
import itertools
import os
import re
import select
import shutil
import signal
import subprocess
import sys
import time
from contextlib import contextmanager
from pathlib import Path

try:
    import cv2
    import numpy as np
except ImportError as exc:
    sys.exit('[FAIL] Python OpenCV/NumPy missing: ' + str(exc) + '; no install attempted')


def maps(size, K, D, zoom):
    new = K.copy()
    new[0, 0] *= zoom
    new[1, 1] *= zoom
    return cv2.initUndistortRectifyMap(K, D, None, new, size, cv2.CV_32FC1)


def manual(size, k1, k2):
    w, h = size
    f = float(w)
    K = np.array([[f, 0, (w-1)/2], [0, f, (h-1)/2], [0, 0, 1.]])
    return K, np.array([k1, k2, 0., 0., 0.])


def camera(sensor, device_path):
    hello = shutil.which('rpicam-hello') or shutil.which('libcamera-hello')
    if not hello:
        raise RuntimeError('rpicam-hello/libcamera-hello not found')
    p = subprocess.run([hello, '--list-cameras'], stdout=subprocess.PIPE,
                       stderr=subprocess.STDOUT, text=True, timeout=10, check=True)
    print(p.stdout, flush=True)
    rows = re.findall(r'^\s*(\d+)\s*:\s*(\S+).*?\((/[^)]+)\)\s*$', p.stdout, re.M)
    matches = [r for r in rows if r[1].lower() == sensor.lower() and
               (device_path is None or r[2] == device_path)]
    if len(matches) != 1:
        raise RuntimeError('Camera not uniquely matched; check list and --device-path')
    return matches[0]


@contextmanager
def stream(index, size, fps):
    tool = shutil.which('rpicam-vid') or shutil.which('libcamera-vid')
    if not tool:
        raise RuntimeError('rpicam-vid/libcamera-vid not found')
    w, h = size
    cmd = [tool, '--camera', index, '--timeout', '0', '--nopreview',
           '--codec', 'mjpeg', '--width', str(w), '--height', str(h),
           '--framerate', str(fps), '--output', '-']
    print('[INFO]', ' '.join(cmd), flush=True)
    p = subprocess.Popen(cmd, stdout=subprocess.PIPE, stdin=subprocess.DEVNULL,
                         start_new_session=True, bufsize=0)
    try:
        yield p
    finally:
        for sig, wait in [(signal.SIGTERM, 3), (signal.SIGKILL, 3)]:
            try:
                os.killpg(p.pid, sig)
            except ProcessLookupError:
                break
            try:
                p.wait(timeout=wait)
                break
            except subprocess.TimeoutExpired:
                continue
        p.stdout.close()
        if p.poll() is None:
            raise RuntimeError('camera child not reaped')


def frames(p):
    buf = bytearray()
    deadline = time.monotonic() + 10
    while True:
        if time.monotonic() >= deadline:
            raise RuntimeError('camera frame timeout')
        ready, _, _ = select.select([p.stdout], [], [], .2)
        if not ready:
            continue
        chunk = os.read(p.stdout.fileno(), 65536)
        if not chunk:
            raise RuntimeError('camera stream closed')
        buf.extend(chunk)
        while True:
            start = buf.find(b'\xff\xd8')
            if start < 0:
                buf[:] = buf[-1:]
                break
            if start:
                del buf[:start]
            end = buf.find(b'\xff\xd9', 2)
            if end < 0:
                break
            data = bytes(buf[:end+2])
            del buf[:end+2]
            img = cv2.imdecode(np.frombuffer(data, dtype=np.uint8), cv2.IMREAD_COLOR)
            if img is not None:
                yield img
                deadline = time.monotonic() + 10
        if len(buf) > 16*1024*1024:
            raise RuntimeError('oversized or invalid MJPEG frame')


def load_calibration(path, size, identity):
    obj = json.loads(Path(path).read_text())
    if obj['size'] != list(size) or obj['device_path'] != identity:
        raise ValueError('calibration resolution/device identity mismatch')
    K = np.asarray(obj['K'], dtype=np.float64)
    D = np.asarray(obj['D'], dtype=np.float64)
    if (K.shape != (3, 3) or D.shape != (5,) or not np.isfinite(K).all()
            or not np.isfinite(D).all() or min(K[0, 0], K[1, 1]) <= 0
            or not np.allclose(K[2], [0, 0, 1])):
        raise ValueError('invalid Brown-Conrady calibration')
    return K, D


def show(images, size, calibration):
    name = 'CSI barrel correction - Q/ESC quit'
    cv2.namedWindow(name, cv2.WINDOW_NORMAL)
    if calibration is None:
        cv2.createTrackbar('K1 x100 +60', name, 35, 90, lambda _: None)
        cv2.createTrackbar('K2 x100 +20', name, 20, 70, lambda _: None)
    cv2.createTrackbar('Zoom x100', name, 100, 250, lambda _: None)
    prev = None
    for frame in images:
        if frame.shape[1::-1] != size:
            raise RuntimeError('unexpected decoded frame size')
        zoom = max(100, cv2.getTrackbarPos('Zoom x100', name))/100
        if calibration is None:
            k1 = (cv2.getTrackbarPos('K1 x100 +60', name)-60)/100
            k2 = (cv2.getTrackbarPos('K2 x100 +20', name)-20)/100
            K, D = manual(size, k1, k2)
        else:
            K, D = calibration
        key = (tuple(D), zoom)
        if key != prev:
            mx, my = maps(size, K, D, zoom)
            prev = key
        t = time.monotonic()
        fixed = cv2.remap(frame, mx, my, cv2.INTER_LINEAR)
        ms = (time.monotonic()-t)*1000
        left, right = frame.copy(), fixed.copy()
        cv2.putText(left, 'RAW', (10, 24), 0, .7, (0,255,0), 2)
        cv2.putText(right, 'CALIBRATED' if calibration else 'MANUAL / UNCALIBRATED', (10,24), 0,.5,(0,255,0),1)
        cv2.putText(right, f'k1={D[0]:.2f} k2={D[1]:.2f} zoom={zoom:.2f} remap={ms:.1f}ms', (10,48),0,.45,(0,255,0),1)
        cv2.imshow(name, np.hstack([left,right]))
        keypress = cv2.waitKey(1) & 255
        if keypress in (27, ord('q'), ord('Q')):
            break
        if cv2.getWindowProperty(name, cv2.WND_PROP_VISIBLE) < 1:
            break


def self_test():
    size = (640,480)
    K, D = manual(size, 0, 0)
    mx, my = maps(size, K, D, 1)
    x,y = np.meshgrid(np.arange(size[0]),np.arange(size[1]))
    assert np.max(np.abs(mx-x)) < 1e-3 and np.max(np.abs(my-y)) < 1e-3
    K,D = manual(size,-.25,0)
    mx,my = maps(size,K,D,1)
    # Brown-Conrady inverse sampling: negative k1 maps edge toward principal point.
    assert mx[240, 0] > 0 and mx[240,-1] < 639
    assert abs(mx[240,320]-320) < .01
    img = np.zeros((480,640,3), np.uint8)
    assert cv2.remap(img,mx,my,cv2.INTER_LINEAR).shape == img.shape
    print('[PASS] identity, barrel direction, principal point, remap shape (synthetic only)')


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--sensor', default='ov5647')
    p.add_argument('--device-path')
    p.add_argument('--width', type=int, default=640)
    p.add_argument('--height', type=int, default=480)
    p.add_argument('--fps', type=int, default=30)
    p.add_argument('--calibration', help='JSON: size, device_path, K, D (five coefficients)')
    p.add_argument('--image', help='offline image preview; no camera opened')
    p.add_argument('--self-test', action='store_true')
    a = p.parse_args()
    if a.self_test:
        self_test(); return 0
    if not (160 <= a.width <= 1920 and 120 <= a.height <= 1080 and a.width%2 == a.height%2 == 0 and 1 <= a.fps <= 60):
        p.error('invalid dimensions or fps')
    if not (os.getenv('DISPLAY') or os.getenv('WAYLAND_DISPLAY')):
        raise RuntimeError('run on Pi graphical terminal, not plain SSH')
    if a.image:
        if a.calibration:
            p.error('--image is for manual inspection only')
        image = cv2.imread(a.image)
        if image is None:
            raise RuntimeError('cannot read image')
        show(itertools.repeat(image), image.shape[1::-1], None)
        return 0
    size = (a.width,a.height)
    index, sensor, identity = camera(a.sensor,a.device_path)
    print('[INFO] selected', index, sensor, identity, flush=True)
    calibration = load_calibration(a.calibration,size,identity) if a.calibration else None
    with stream(index,size,a.fps) as proc:
        show(frames(proc),size,calibration)
    return 0


if __name__ == '__main__':
    signal.signal(signal.SIGTERM, lambda *_: sys.exit(143))
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        sys.exit(130)
    except (RuntimeError, OSError, ValueError, KeyError, subprocess.SubprocessError, cv2.error) as exc:
        print('[FAIL]', exc, file=sys.stderr)
        sys.exit(1)
    finally:
        try: cv2.destroyAllWindows()
        except cv2.error: pass
