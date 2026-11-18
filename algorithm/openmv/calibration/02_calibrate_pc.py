"""Calibrate OpenMV QVGA camera intrinsics using chessboard JPEGs.

Install: pip install numpy opencv-python
Run: python 02_calibrate_pc.py --images images --square-mm 20
Board: 9x6 *inner corners*, 10x7 physical squares.
"""
import argparse
import glob
import json
from pathlib import Path
import sys

try:
    import cv2
    import numpy as np
except ImportError:
    sys.exit('Missing OpenCV/NumPy: python -m pip install numpy opencv-python')


def main():
    p = argparse.ArgumentParser(description='Calibrate 320x240 OpenMV intrinsics')
    p.add_argument('--images', default='images', help='Folder containing OpenMV JPG pictures')
    p.add_argument('--square-mm', type=float, default=20.0, help='Measured physical chessboard square width in mm')
    p.add_argument('--cols', type=int, default=9, help='Inner checkerboard corners across')
    p.add_argument('--rows', type=int, default=6, help='Inner checkerboard corners down')
    p.add_argument('--output', default='camera_intrinsics.json')
    args = p.parse_args()
    if args.square_mm <= 0 or args.cols < 3 or args.rows < 3:
        p.error('Invalid dimensions')

    paths = []
    for pattern in ('*.jpg', '*.jpeg', '*.png', '*.bmp'):
        paths += glob.glob(str(Path(args.images) / pattern))
    paths = sorted(set(paths))
    if not paths:
        sys.exit('No images found in ' + args.images)

    shape = (args.cols, args.rows)
    grid = np.zeros((args.cols * args.rows, 3), np.float32)
    grid[:, :2] = np.mgrid[0:args.cols, 0:args.rows].T.reshape(-1, 2)
    grid *= args.square_mm

    object_points, image_points = [], []
    accepted = []
    expected_wh = None
    for path in paths:
        img = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
        if img is None:
            print('SKIP unreadable', path)
            continue
        height, width = img.shape
        if expected_wh is None:
            expected_wh = (width, height)
        if (width, height) != expected_wh:
            sys.exit('Inconsistent image resolution; do not mix camera modes or image sizes')
        corners = None
        found = False
        if hasattr(cv2, 'findChessboardCornersSB'):
            found, corners = cv2.findChessboardCornersSB(img, shape)
        if not found:
            found, corners = cv2.findChessboardCorners(
                img, shape, cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE)
            if found:
                criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 40, 0.001)
                corners = cv2.cornerSubPix(img, corners, (5, 5), (-1, -1), criteria)
        if found:
            object_points.append(grid.copy())
            image_points.append(np.asarray(corners, np.float32).reshape(-1, 1, 2))
            accepted.append(path)
        else:
            print('SKIP no complete chessboard', Path(path).name)

    print('Accepted:', len(accepted), '/', len(paths))
    if len(accepted) < 15:
        sys.exit('Not enough complete/diverse chessboard views (need >=15; recommend 20-35)')
    if expected_wh != (320, 240):
        sys.exit('Your project uses QVGA 320x240; captured images are %r. Re-capture in QVGA.' % (expected_wh,))

    rms, K, dist, rvecs, tvecs = cv2.calibrateCamera(
        object_points, image_points, expected_wh, None, None)
    sq_error, count = 0.0, 0
    for obj, corners, rv, tv in zip(object_points, image_points, rvecs, tvecs):
        predicted, _ = cv2.projectPoints(obj, rv, tv, K, dist)
        delta = (corners - predicted).reshape(-1, 2)
        sq_error += float(np.sum(delta ** 2))
        count += delta.shape[0]
    point_rms = (sq_error / count) ** 0.5
    output = {
        'resolution': list(expected_wh),
        'board_inner_corners': list(shape),
        'square_size_mm': args.square_mm,
        'accepted_images': len(accepted),
        'fx_px': float(K[0, 0]),
        'fy_px': float(K[1, 1]),
        'cx_px': float(K[0, 2]),
        'cy_px': float(K[1, 2]),
        'distortion_k1_k2_p1_p2_k3': [float(x) for x in dist.ravel()[:5]],
        'opencv_rms_px': float(rms),
        'corner_reprojection_rms_px': float(point_rms),
        'accepted_files': [Path(x).name for x in accepted],
    }
    with open(args.output, 'w', encoding='utf-8') as f:
        json.dump(output, f, indent=2, ensure_ascii=False)
    print('\n========== Paste into demo/src_openmv/config.py =========')
    print('FX_PX = %.6f' % output['fx_px'])
    print('FY_PX = %.6f' % output['fy_px'])
    print('CX_PX = %.6f' % output['cx_px'])
    print('CY_PX = %.6f' % output['cy_px'])
    print('===========================================================')
    print('OpenCV RMS: %.3f px; per-corner RMS: %.3f px' % (rms, point_rms))
    print('Distortion coefficients:', output['distortion_k1_k2_p1_p2_k3'])
    print('Wrote:', args.output)
    if point_rms > 1.0:
        print('WARNING: reprojection RMS >1px; inspect board quality and retake more diverse images')
    if (K[0, 0] < 100 or K[0, 0] > 1500 or K[1, 1] < 100 or K[1, 1] > 1500
            or not (0 < K[0, 2] < expected_wh[0]) or not (0 < K[1, 2] < expected_wh[1])
            or abs(float(K[0, 0] / K[1, 1]) - 1.0) > 0.25):
        print('WARNING: intrinsics look unusual for a standard QVGA lens; results may be unstable.')
        print('        Re-capture diverse board angles, check print scale, and verify physical distances.')
    print('NOTE: existing OpenMV metric_from_circle() does NOT yet compensate lens distortion.')
    print('Use the center of frame for early measurements, then add correction/validation.')


if __name__ == '__main__':
    main()
