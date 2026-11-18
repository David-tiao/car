#!/usr/bin/env python3
# /// script
# requires-python = ">=3.12"
# dependencies = ["openmv", "opencv-python", "numpy"]
# ///
"""OpenMV H7 Plus chessboard auto-capture. Runs on the COMPUTER, not the camera.

Reads the OpenMV USB IDE frame stream through the official `openmv` host package;
detects a 9x6 inner-corner chessboard using OpenCV; saves clean source images
and separately saves annotated previews. Does NOT write the camera's main.py.

Requires Python 3.12+; pip install openmv opencv-python numpy
Usage: python 01_auto_capture_pc.py --port /dev/ttyACM0 --count 30
"""
from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path
from typing import Iterator

try:
    import cv2
    import numpy as np
except ImportError:
    sys.exit('Please install: python -m pip install opencv-python numpy')


CAMERA_SCRIPT = '''import csi
cam = csi.CSI()
cam.reset()
cam.pixformat(csi.RGB565)
cam.framesize(csi.QVGA)
cam.snapshot(time=2000)
print("CAMERA_READY_QVGA")
while True:
    cam.snapshot()
'''

BOARD = (9, 6)  # Inner corners (10 x 7 printed square grid)
BASE = Path(__file__).resolve().parent


def get_port(requested: str | None) -> str:
    if requested:
        return requested
    try:
        from serial.tools import list_ports
    except ImportError:
        sys.exit('Please install: python -m pip install openmv')
    choices = []
    for dev in list_ports.comports():
        label = ' '.join([str(dev.device), str(dev.description), str(dev.manufacturer or '')]).lower()
        if 'openmv' in label or 'ttyacm' in label:
            choices.append(dev.device)
    if len(choices) == 1:
        return choices[0]
    if not choices:
        sys.exit('No USB OpenMV found. Try --port /dev/ttyACM0 (Linux) or --port COM3 (Windows).')
    sys.exit('Multiple candidate ports: %s. Select one with --port.' % ', '.join(choices))


def find_board(image_bgr: np.ndarray):
    """Return corners as Nx1x2 float32 or None; locate on computer, not OpenMV."""
    gray = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2GRAY)
    # Guard against the almost completely black image seen in the IDE screenshot.
    if gray.mean() < 28 or gray.std() < 18:
        return None, 'TOO DARK / LOW CONTRAST'
    if gray.mean() > 242:
        return None, 'OVEREXPOSED'
    corners = None
    found = False
    if hasattr(cv2, 'findChessboardCornersSB'):
        try:
            found, corners = cv2.findChessboardCornersSB(gray, BOARD)
        except cv2.error:
            pass
    if not found:
        flags = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE
        found, corners = cv2.findChessboardCorners(gray, BOARD, flags)
        if found:
            criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_MAX_ITER, 40, 0.001)
            corners = cv2.cornerSubPix(gray, corners, (5, 5), (-1, -1), criteria)
    if not found or corners is None:
        return None, 'NO COMPLETE 9x6 BOARD'
    corners = np.asarray(corners, dtype=np.float32).reshape(-1, 1, 2)
    if len(corners) != BOARD[0] * BOARD[1]:
        return None, 'CORNER COUNT ERROR'
    xy = corners.reshape(-1, 2)
    h, w = gray.shape
    # Avoid truncated boards and boards too small for trustworthy calibration.
    if (xy[:, 0].min() < 8 or xy[:, 0].max() > w - 9 or
            xy[:, 1].min() < 8 or xy[:, 1].max() > h - 9):
        return corners, 'BOARD TOO CLOSE TO EDGE'
    minx, miny = np.min(xy, axis=0)
    maxx, maxy = np.max(xy, axis=0)
    if (maxx - minx) * (maxy - miny) < 0.02 * w * h:
        return corners, 'MOVE BOARD CLOSER'
    focus = float(cv2.Laplacian(gray, cv2.CV_64F).var())
    if focus < 18:
        return corners, 'BLURRY / REFOCUS'
    return corners, 'OK'


def board_pose(corners: np.ndarray) -> tuple[float, float, float, float, float, float]:
    xy = corners.reshape(BOARD[1], BOARD[0], 2)
    flat = xy.reshape(-1, 2)
    c = flat.mean(axis=0)
    w = np.linalg.norm(xy[0, -1] - xy[0, 0])
    h = np.linalg.norm(xy[-1, 0] - xy[0, 0])
    direction = xy[0, -1] - xy[0, 0]
    angle = math.degrees(math.atan2(float(direction[1]), float(direction[0])))
    # Tilt/skew signature from local grid spacing variation; helps avoid duplicates.
    left = float(np.linalg.norm(xy[-1, 0] - xy[0, 0]))
    right = float(np.linalg.norm(xy[-1, -1] - xy[0, -1]))
    perspective = right / max(1.0, left)
    return float(c[0]), float(c[1]), float(w), float(h), float(angle), perspective


def is_new_pose(current, previous) -> bool:
    if previous is None:
        return True
    cx, cy, w, h, angle, pers = current
    px, py, pw, ph, pa, pp = previous
    center_shift = math.hypot(cx - px, cy - py)
    scale_change = abs(math.log(max(0.1, (w * h) / max(0.1, pw * ph))))
    rotation_change = abs((angle - pa + 180) % 360 - 180)
    perspective_change = abs(pers - pp)
    return (center_shift > 13 or scale_change > 0.11 or rotation_change > 8 or
            perspective_change > 0.14)


def to_bgr(frame: dict) -> np.ndarray:
    w = int(frame['width'])
    h = int(frame['height'])
    if w != 320 or h != 240:
        raise ValueError('Expected QVGA 320x240, got %dx%d. Check camera mode.' % (w, h))
    data = frame['data']  # Official host API returns RGB888, not native RGB565.
    if len(data) != w * h * 3:
        raise ValueError('Frame RGB length mismatch: got %d, expected %d' % (len(data), w*h*3))
    rgb = np.frombuffer(data, dtype=np.uint8).reshape(h, w, 3)
    return cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)


def camera_frames(port: str) -> Iterator[dict]:
    """Execute temporary RAM-only capture code; stream over the IDE USB protocol."""
    try:
        from openmv import Camera
    except ImportError:
        sys.exit('Missing OpenMV host package. Use Python 3.12+ and: pip install openmv')
    print('Opening OpenMV on', port)
    print('IMPORTANT: disconnect or close OpenMV IDE first; only one app owns the USB port.')
    with Camera(port) as cam:
        cam.stop()
        cam.exec(CAMERA_SCRIPT)
        cam.streaming(True)
        last_frame_time = time.monotonic()
        try:
            while True:
                frame = cam.read_frame()
                if frame is not None:
                    last_frame_time = time.monotonic()
                    yield frame
                else:
                    time.sleep(0.02)
                stdout = cam.read_stdout()
                if stdout and ('Traceback' in stdout or 'Error' in stdout):
                    raise RuntimeError('OpenMV script error: ' + stdout.strip())
                if time.monotonic() - last_frame_time > 20.0:
                    raise RuntimeError('No camera frames for 20 seconds; check sensor/USB and IDE connection.')
        finally:
            try:
                cam.streaming(False)
            finally:
                cam.stop()


def local_test_frames(folder: Path) -> Iterator[dict]:
    """For offline checks only; hardware usage never needs this switch."""
    files = sorted(p for p in folder.iterdir() if p.suffix.lower() in ('.jpg', '.jpeg', '.png'))
    if not files:
        raise ValueError('No sample images in %s' % folder)
    for path in files:
        img = cv2.imread(str(path), cv2.IMREAD_COLOR)
        if img is None:
            continue
        h, w = img.shape[:2]
        rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        yield {'width': w, 'height': h, 'data': rgb.tobytes()}


def record_images(frames, image_dir: Path, marked_dir: Path, count: int,
                  min_interval: float, preview: bool, simulate: bool = False) -> int:
    image_dir.mkdir(parents=True, exist_ok=True)
    marked_dir.mkdir(parents=True, exist_ok=True)
    # Continue numbering when an earlier session left images behind.
    numbers = []
    for path in image_dir.glob('cal_*.png'):
        if path.stem[4:].isdigit():
            numbers.append(int(path.stem[4:]))
    next_number = max(numbers, default=0) + 1
    saved_count = 0
    last_pose = None
    last_save_time = -1e9
    frames_seen = 0
    first_warned = False
    last_status_print = -1e9
    if preview:
        cv2.namedWindow('OpenMV calibration - press Q to stop', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('OpenMV calibration - press Q to stop', 960, 720)
    for frame in frames:
        frames_seen += 1
        now = time.monotonic()
        bgr = to_bgr(frame)
        corners, status = find_board(bgr)
        view = bgr.copy()
        if corners is not None:
            cv2.drawChessboardCorners(view, BOARD, corners, True)
        ready = status == 'OK'
        if ready:
            pose = board_pose(corners)
            if now - last_save_time < min_interval:
                status = 'WAIT INTERVAL'
            elif not is_new_pose(pose, last_pose):
                status = 'MOVE / TILT BOARD'
            else:
                filename = f'cal_{next_number:03d}.png'
                raw_path = image_dir / filename
                marked_path = marked_dir / filename
                if not cv2.imwrite(str(raw_path), bgr):
                    raise OSError('Failed to save ' + str(raw_path))
                # Marked preview is ONLY for human checking; never used for calibration!
                view_for_save = view.copy()
                cv2.putText(view_for_save, 'DETECTED 9x6', (8, 22), cv2.FONT_HERSHEY_SIMPLEX,
                            0.52, (0, 255, 0), 1, cv2.LINE_AA)
                if not cv2.imwrite(str(marked_path), view_for_save):
                    raise OSError('Failed to save ' + str(marked_path))
                saved_count += 1
                next_number += 1
                last_pose = pose
                last_save_time = now
                status = 'SAVED ' + filename
                print(f'[{saved_count}/{count}] {status}', flush=True)
        if status.startswith('TOO DARK') and not first_warned:
            print('WARNING: Image is dark. Remove lens cover, add light, point at a white sheet.')
            first_warned = True
        if not preview and now - last_status_print > 4.0:
            print(f'Frames: {frames_seen} | saved: {saved_count}/{count} | {status}', flush=True)
            last_status_print = now
        if preview:
            color = (0, 210, 0) if status.startswith('SAVED') else (0, 220, 240)
            cv2.rectangle(view, (0, 0), (319, 51), (0, 0, 0), -1)
            cv2.putText(view, f'Saved {saved_count}/{count}', (9, 19),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 1, cv2.LINE_AA)
            cv2.putText(view, status[:40], (9, 42),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.43, color, 1, cv2.LINE_AA)
            cv2.imshow('OpenMV calibration - press Q to stop', view)
            key = cv2.waitKey(1) & 0xFF
            if key in (ord('q'), ord('Q'), 27):
                break
        if saved_count >= count:
            break
        if simulate:
            # Each file stands for a different capture moment in offline testing.
            last_save_time = -1e9
    if preview:
        cv2.destroyAllWindows()
    print('Completed: %d saved, %d frames processed.' % (saved_count, frames_seen))
    print('RAW images:   ', image_dir)
    print('MARKED images:', marked_dir)
    return saved_count


def main():
    parser = argparse.ArgumentParser(description='Automatically save OpenMV chessboard images to PC')
    parser.add_argument('--port', default=None, help='USB serial port, e.g. /dev/ttyACM0 or COM3')
    parser.add_argument('--count', type=int, default=30, help='Number of accepted photos (default 30)')
    parser.add_argument('--interval', type=float, default=1.0, help='Minimum seconds between saves')
    parser.add_argument('--images', default=str(BASE / 'images'), help='Raw original output folder')
    parser.add_argument('--marked', default=str(BASE / 'marked'), help='Annotated output folder')
    parser.add_argument('--no-preview', action='store_true', help='No cv2.imshow GUI')
    parser.add_argument('--test-images', type=Path, help='Offline source photos for testing only')
    args = parser.parse_args()
    if args.count < 1 or args.interval < 0:
        parser.error('--count must be >=1 and --interval must be >=0')
    print('Auto detect: 9 x 6 inner corners, QVGA 320 x 240; no SD card needed.')
    if args.test_images:
        frames = local_test_frames(args.test_images)
    else:
        frames = camera_frames(get_port(args.port))
    try:
        result = record_images(frames, Path(args.images).expanduser().resolve(),
                               Path(args.marked).expanduser().resolve(), args.count,
                               args.interval, not args.no_preview,
                               simulate=args.test_images is not None)
        if result < args.count:
            print('Stopped early; you can run the same command again to collect more angles.')
    except KeyboardInterrupt:
        print('\nStopped by user; any saved images remain on disk.')
    except Exception as e:
        print('ERROR:', e, file=sys.stderr)
        print('Check: OpenMV IDE disconnected, OpenMV powered, USB port permissions, board visible.', file=sys.stderr)
        sys.exit(1)
    finally:
        close_fn = getattr(frames, 'close', None)
        if close_fn:
            close_fn()


if __name__ == '__main__':
    main()
