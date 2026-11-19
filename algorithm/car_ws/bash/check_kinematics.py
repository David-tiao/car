#!/usr/bin/env python3
"""TEST 底盘运动学一致性验收。只读，不发送控制指令。"""

import argparse
import json
import math
import os
import statistics
import sys
import time
from datetime import datetime
from pathlib import Path
from urllib.request import urlopen

# 来自 STM32F4 的参考参数
WHEEL_RADIUS = 0.04
ENCODER_PPR = 440.0
L = 0.093 + 0.110

PULSES_PER_METER = ENCODER_PPR / (
    2.0 * math.pi * WHEEL_RADIUS
)


def finite(x):
    return type(x) in (int, float) and math.isfinite(x)


def fetch():
    url = 'http://127.0.0.1:8080/api/test/inspection'

    with urlopen(url, timeout=2) as res:
        packet = json.load(res)

    if packet.get('source_type') != 'MOCK':
        raise ValueError('数据不是 TEST/MOCK 来源')

    if not packet.get('online'):
        raise ValueError('TEST 诊断接口离线')

    snapshot = packet.get('snapshot')

    if not isinstance(snapshot, dict):
        raise ValueError('诊断数据缺失')

    if snapshot.get('schema') != 'car_test_inspection_v1':
        raise ValueError('诊断协议不匹配')

    if snapshot.get('control_enabled') is not False:
        raise ValueError('TEST 控制隔离状态异常')

    return packet['seq'], snapshot


def calculate(snapshot):
    topics = snapshot.get('topics', {})
    samples = snapshot.get('samples', {})

    for name in ('chassis', 'wheels'):
        if not topics.get(name, {}).get('fresh'):
            raise ValueError(name + ' 数据不新鲜')

    chassis = samples.get('chassis', {})
    wheels = samples.get('wheels', {})

    actual = wheels.get('actual_mps')
    pulses = wheels.get('actual_pulse_per_s')
    target = wheels.get('target_mps')

    if (
        not isinstance(actual, list)
        or len(actual) != 4
        or not all(map(finite, actual))
    ):
        raise ValueError('实际轮速数据无效')

    if (
        not isinstance(pulses, list)
        or len(pulses) != 4
        or not all(map(finite, pulses))
    ):
        raise ValueError('编码器脉冲数据无效')

    fields = ('vx_mps', 'vy_mps', 'wz_radps')

    if not all(finite(chassis.get(k)) for k in fields):
        raise ValueError('底盘速度数据无效')

    # STM32F4 麦克纳姆轮正运动学
    # 固定轮序：FL FR RL RR
    fl, fr, rl, rr = actual

    vx = (fl + fr + rl + rr) / 4.0
    vy = (-fl + fr + rl - rr) / 4.0
    wz = (-fl + fr - rl + rr) / (4.0 * L)

    # 与 CAR-LINK 上报的底盘实际速度比较
    speed_error = math.hypot(
        vx - chassis['vx_mps'],
        vy - chassis['vy_mps']
    )

    yaw_error = abs(wz - chassis['wz_radps'])

    # 检查脉冲速度与轮面线速度换算
    pulse_error = max(
        abs(p / PULSES_PER_METER - v)
        for p, v in zip(pulses, actual)
    )

    # 电机目标/实际误差：仅观测，不作为运动学失败条件
    wheel_tracking = None

    if (
        isinstance(target, list)
        and len(target) == 4
        and all(map(finite, target))
    ):
        wheel_tracking = math.sqrt(
            sum(
                (a - t) ** 2
                for a, t in zip(actual, target)
            ) / 4
        )

    return {
        'speed_error_mps': speed_error,
        'yaw_error_radps': yaw_error,
        'pulse_error_mps': pulse_error,
        'wheel_tracking_rmse_mps': wheel_tracking
    }


def p95(values):
    values = sorted(values)
    index = math.ceil(len(values) * .95) - 1
    return values[index]


def process_counts():
    """只统计当前项目、ROS_DOMAIN_ID=73 的测试节点。"""
    root = str(Path(__file__).resolve().parents[3])

    names = {
        'car_serial_mock_mcu': [],
        'car_serial_bridge': []
    }

    for proc in Path('/proc').iterdir():
        if not proc.name.isdigit():
            continue

        try:
            raw = (proc / 'cmdline').read_bytes()
            args = raw.split(b'\0')

            if not args or not args[0]:
                continue

            binary = args[0].decode(errors='replace')
            name = os.path.basename(binary)

            if name not in names:
                continue

            if not binary.startswith(root + '/'):
                continue

            env = (proc / 'environ').read_bytes().split(b'\0')

            if b'ROS_DOMAIN_ID=73' in env:
                names[name].append(int(proc.name))

        except (OSError, PermissionError):
            continue

    return names


def main():
    parser = argparse.ArgumentParser()

    parser.add_argument(
        '--seconds', type=float, default=15
    )

    parser.add_argument(
        '--min-samples', type=int, default=12
    )

    parser.add_argument(
        '--max-speed-error', type=float, default=.08
    )

    parser.add_argument(
        '--max-yaw-error', type=float, default=.15
    )

    args = parser.parse_args()

    if not (
        2 <= args.seconds <= 600
        and args.min_samples >= 1
        and args.max_speed_error > 0
        and args.max_yaw_error > 0
    ):
        parser.error('参数不合法')

    print('[START] STM32F4 运动学一致性检测')
    print('[PARAM] 轮半径:', WHEEL_RADIUS, 'm')
    print('[PARAM] 转动系数 L:', L, 'm')
    print('[PARAM] pulses/m:', round(PULSES_PER_METER, 4))

    start = time.monotonic()
    seq = -1
    rows = []
    errors = []

    while time.monotonic() - start < args.seconds:
        try:
            current_seq, snapshot = fetch()

            if current_seq != seq:
                seq = current_seq
                rows.append(calculate(snapshot))

        except (OSError, ValueError, KeyError, TypeError) as exc:
            errors.append(str(exc))

        time.sleep(.25)

    processes = process_counts()
    failures = []

    if len(rows) < args.min_samples:
        failures.append(
            f'有效采样不足：{len(rows)}/{args.min_samples}'
        )

    checks = (
        ('speed_error_mps', args.max_speed_error),
        ('yaw_error_radps', args.max_yaw_error),
        ('pulse_error_mps', .02)
    )

    for name, limit in checks:
        if rows and p95([r[name] for r in rows]) > limit:
            failures.append(
                f'{name} 的 P95 超过阈值 {limit}'
            )

    if any(len(v) > 1 for v in processes.values()):
        failures.append('存在重复的 TEST 模拟 MCU 或串口桥')

    metrics = {}

    for name in (
        'speed_error_mps',
        'yaw_error_radps',
        'pulse_error_mps',
        'wheel_tracking_rmse_mps'
    ):
        values = [
            row[name] for row in rows
            if row[name] is not None
        ]

        if values:
            metrics[name] = {
                'mean': statistics.fmean(values),
                'p95': p95(values),
                'max': max(values)
            }

    status = 'FAIL' if failures else 'PASS'

    report = {
        'schema': 'car_test_kinematics_v1',
        'status': status,
        'source': 'TEST/MOCK',
        'samples': len(rows),
        'duration_s': args.seconds,
        'parameters': {
            'wheel_radius_m': WHEEL_RADIUS,
            'L_m': L,
            'encoder_ppr': ENCODER_PPR,
            'pulses_per_meter': PULSES_PER_METER
        },
        'metrics': metrics,
        'test_processes': processes,
        'failures': failures,
        'fetch_errors': errors[:4]
    }

    folder = (
        Path(__file__).resolve().parents[3]
        / 'sim2real/data/web_modes'
    )

    folder.mkdir(parents=True, exist_ok=True)

    filename = (
        'test_kinematics_'
        + datetime.now().strftime('%Y%m%d_%H%M%S_%f')
        + '.json'
    )

    output = folder / filename

    output.write_text(
        json.dumps(report, ensure_ascii=False, indent=2) + '\n',
        encoding='utf-8'
    )

    print(f'\n[{status}] 有效采样：{len(rows)}')

    for name, result in metrics.items():
        print(
            f'{name}: '
            f'P95={result["p95"]:.5f}, '
            f'MAX={result["max"]:.5f}'
        )

    for reason in failures:
        print('[FAIL]', reason)

    if errors:
        print('[INFO] 采集异常示例:', errors[0])

    print('[REPORT]', output)

    return 2 if failures else 0


if __name__ == '__main__':
    sys.exit(main())
