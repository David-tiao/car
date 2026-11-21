#!/usr/bin/env python3
"""TEST 路径跟踪验收：只读取数据，不发送控制指令。"""

import argparse
import json
import math
import time
from datetime import datetime
from pathlib import Path
from urllib.request import urlopen


def get(path):
    with urlopen('http://127.0.0.1:8080' + path, timeout=2) as r:
        return json.load(r)


def project(x, y, points):
    """计算车体中心到规划路径的最短距离和投影进度。"""
    best = float('inf')
    at = 0.0
    total = 0.0

    for a, b in zip(points, points[1:]):
        dx = b[0] - a[0]
        dy = b[1] - a[1]
        d2 = dx * dx + dy * dy

        if d2:
            t = ((x-a[0])*dx + (y-a[1])*dy) / d2
            t = max(0.0, min(1.0, t))
        else:
            t = 0.0

        error = math.hypot(
            x - a[0] - t*dx,
            y - a[1] - t*dy
        )

        length = math.sqrt(d2)

        if error < best:
            best = error
            at = total + t*length

        total += length

    return best, at/total if total else 0.0


def run(args):
    route = get('/api/test/reference_path')
    points = route.get('points', [])

    if (
        route.get('source_type') != 'TEST'
        or not route.get('online')
        or len(points) < 2
    ):
        raise RuntimeError(
            'TEST 路径离线，请打开 /test 并启动测试集'
        )

    route_seq = route['seq']

    errors = []
    odom_seq = 0
    prev = None

    max_progress = 0.0
    travelled = 0.0
    outside = 0
    jumps = 0
    failure = ''

    start = time.monotonic()

    print(
        f'[START] 路线版本={route_seq}, '
        f'航点数={len(points)}',
        flush=True
    )

    while time.monotonic() - start < args.seconds:

        ref = get('/api/test/reference_path')

        if (
            not ref.get('online')
            or ref.get('seq') != route_seq
        ):
            failure = '验收期间规划路线发生变化或离线'
            break

        packet = get(
            f'/api/test/telemetry?after={odom_seq}'
        )

        if (
            packet.get('source_type') != 'TEST'
            or not packet.get('online')
        ):
            failure = 'TEST 里程计离线'
            break

        for s in packet.get('samples', []):

            seq = s.get('seq')
            x = s.get('x')
            y = s.get('y')

            if (
                type(seq) is not int
                or seq <= odom_seq
                or not all(
                    type(v) in (int, float)
                    and math.isfinite(v)
                    for v in (x, y)
                )
            ):
                continue

            odom_seq = seq

            error, progress = project(x, y, points)

            errors.append(error)

            max_progress = max(
                max_progress,
                progress
            )

            if not (
                0 <= x <= 2.4
                and 0 <= y <= 2.4
            ):
                outside += 1

            if prev:
                d = math.hypot(
                    x-prev['x'],
                    y-prev['y']
                )
                travelled += d

                t1 = s.get('received_at')
                t0 = prev.get('received_at')

                if (
                    isinstance(t1, (int,float))
                    and isinstance(t0, (int,float))
                ):
                    dt = t1-t0

                    if (
                        0 < dt < 2
                        and d > max(0.12, 1.2*dt+0.04)
                    ):
                        jumps += 1

            prev = s

        time.sleep(0.2)

    ordered = sorted(errors)

    if errors:
        rms = math.sqrt(
            sum(e*e for e in errors) / len(errors)
        )
        p95 = ordered[
            math.ceil(len(ordered)*0.95)-1
        ]
        max_error = max(errors)
    else:
        rms = None
        p95 = None
        max_error = None

    reasons = [failure] if failure else []

    if len(errors) < args.min_samples:
        reasons.append('里程计采样数不足')

    if rms is not None and rms > args.rms_max:
        reasons.append(
            f'RMS误差 {rms:.3f}m 超出上限'
        )

    if p95 is not None and p95 > args.p95_max:
        reasons.append(
            f'P95误差 {p95:.3f}m 超出上限'
        )

    if travelled < args.min_travel:
        reasons.append(
            f'行驶距离不足：{travelled:.3f}m'
        )

    if outside or jumps:
        reasons.append(
            f'越界={outside}, 位置跳变={jumps}'
        )

    status = 'FAIL' if reasons else 'PASS'

    report = {
        'schema': 'test_route_qa_v1',
        'source_type': 'TEST',
        'status': status,
        'route_seq': route_seq,
        'points': len(points),
        'samples': len(errors),
        'duration_s': round(
            time.monotonic()-start, 2
        ),
        'rms_error_m': rms,
        'p95_error_m': p95,
        'max_error_m': max_error,
        'progress_estimate_pct': round(
            max_progress*100, 1
        ),
        'travelled_m': round(travelled, 3),
        'outside_samples': outside,
        'odom_jump_events': jumps,
        'failures': reasons
    }

    folder = (
        Path(__file__).resolve().parents[1]
        / 'data/web_modes'
    )

    folder.mkdir(parents=True, exist_ok=True)

    filename = (
        'test_route_qa_'
        + datetime.now().strftime(
            '%Y%m%d_%H%M%S'
        )
        + '.json'
    )

    path = folder / filename

    path.write_text(
        json.dumps(
            report,
            ensure_ascii=False,
            indent=2
        ) + '\n',
        encoding='utf-8'
    )

    print(f'[{status}]')
    print('采样数量:', len(errors))
    print('RMS误差:', rms, 'm')
    print('P95误差:', p95, 'm')
    print('最大误差:', max_error, 'm')
    print('估计路径进度:', round(max_progress*100,1), '%')
    print('实际行驶距离:', round(travelled,3), 'm')
    print('中心越界采样数:', outside)
    print('里程计跳变次数:', jumps)

    for reason in reasons:
        print('[FAIL]', reason)

    print('[REPORT]', path)

    return 2 if reasons else 0


def main():
    parser = argparse.ArgumentParser()

    parser.add_argument(
        '--seconds', type=float, default=60
    )
    parser.add_argument(
        '--rms-max', type=float, default=0.08
    )
    parser.add_argument(
        '--p95-max', type=float, default=0.12
    )
    parser.add_argument(
        '--min-travel', type=float, default=0.15
    )
    parser.add_argument(
        '--min-samples', type=int, default=50
    )

    args = parser.parse_args()

    if not (
        0.5 <= args.seconds <= 3600
        and args.rms_max > 0
        and args.p95_max > 0
        and args.min_travel >= 0
        and args.min_samples >= 1
    ):
        parser.error('验收参数无效')

    try:
        return run(args)
    except (
        OSError,
        ValueError,
        KeyError,
        TypeError,
        RuntimeError
    ) as exc:
        print('[FAIL]', exc)
        return 2


if __name__ == '__main__':
    raise SystemExit(main())
