#!/usr/bin/env python3
"""One-click STATIC scan/lock/topology preview on Ubuntu ROS2 Humble.

No cmd_vel, Nav2 FollowPath, odometry spoofing, serial motor commands or STM32 control.
Only processes launched by this supervisor are stopped by this supervisor.
"""
import argparse
import fcntl
import json
import math
import logging
from logging.handlers import RotatingFileHandler
import os
from pathlib import Path
import re
import shutil
import signal
import subprocess
import sys
import threading
import time

from mission_export import ZONES

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
WS = ROOT.parent.parent
RUN_HOME = Path(os.environ.get('XDG_STATE_HOME', str(Path.home() / '.local/state'))) / 'competition_oneclick'
REQUIRED = (
    ROOT / 'generate_static_map.py',
    ROOT / 'simple_lock_stage' / 'lock_obstacles.py',
    ROOT / 'simple_lock_stage' / 'preflight_fixed_route.py',
    ROOT / 'simple_lock_stage' / 'lock_core.py',
    ROOT / 'topology_stage6' / 'road_topology.py',
    WS / 'config' / 'x3.yaml',
)


def say(level, message):
    print(f'[{time.strftime("%Y-%m-%dT%H:%M:%S%z")}] [{level}] {message}', flush=True)


def config(path=HERE / 'oneclick_config.json'):
    d = json.loads(path.read_text(encoding='utf-8'))
    if not isinstance(d, dict):
        raise ValueError('oneclick_config.json 必须为 JSON 对象')
    if d.get('start_zone') not in ZONES:
        raise ValueError('start_zone 仅支持 upper_right / lower_right')
    if type(d.get('expected_obstacles')) is not int or d['expected_obstacles'] < 1:
        raise ValueError('expected_obstacles 必须为正整数')
    if not isinstance(d.get('show_rviz'), bool):
        raise ValueError('show_rviz 必须为 true/false')
    port = d.get('serial_port')
    if port is not None and (not isinstance(port, str) or not port.startswith('/dev/')):
        raise ValueError('serial_port 必须为 null 或 /dev/ 开头的设备路径')
    tf = d.get('stationary_tf', {})
    if not isinstance(tf, dict) or not isinstance(tf.get('enabled'), bool):
        raise ValueError('stationary_tf.enabled 必须为布尔值')
    if tf['enabled']:
        for field, n in [('map_to_base', 3), ('base_to_laser', 4)]:
            values = tf.get(field)
            if (not isinstance(values, list) or len(values) != n or
                    any(not isinstance(v, (int, float)) or isinstance(v, bool) or
                        not math.isfinite(v) for v in values)):
                raise ValueError(f'{field} 必须提供 {n} 个已测量的有限实数')
    return d


def check_inputs():
    absent = [str(p) for p in REQUIRED if not p.is_file()]
    if absent:
        raise RuntimeError('缺少原有工程文件，未尝试重建/覆盖：\n  ' + '\n  '.join(absent))
    topology = (ROOT / 'topology_stage6' / 'road_topology.py').read_text(encoding='utf-8')
    if '--mission' not in topology or '--expected-obstacles' not in topology:
        raise RuntimeError('现有 road_topology.py 不是支持固定多段任务的 V2.1 版本')
    d = config()
    text = (WS / 'config' / 'x3.yaml').read_text(encoding='utf-8')
    port_match = re.search(r'^\s*port:\s*([^#\s]+)', text, flags=re.M)
    frame_match = re.search(r'^\s*frame_id:\s*([^#\s]+)', text, flags=re.M)
    if not port_match or not frame_match:
        raise RuntimeError('x3.yaml 缺少 port 或 frame_id')
    frame = frame_match.group(1).strip('"\'')
    if not re.fullmatch(r'[A-Za-z][A-Za-z0-9_]*', frame):
        raise RuntimeError('非法雷达 frame_id')
    port = d.get('serial_port') or port_match.group(1).strip('"\'')
    if not port.startswith('/dev/') or '\n' in port:
        raise RuntimeError('非法雷达串口路径')
    return d, port, frame, text


def run_command(args, timeout=10, allowed=(0,)):
    result = subprocess.run(args, text=True, stdout=subprocess.PIPE,
                            stderr=subprocess.STDOUT, timeout=timeout, check=False)
    if result.returncode not in allowed:
        raise RuntimeError(f'命令失败 ({result.returncode}): {" ".join(map(str,args))}\n{result.stdout[-1300:]}')
    return result.stdout.strip()


def verify_ros():
    if not shutil.which('ros2'):
        raise RuntimeError('未找到 ros2 命令，确认 /opt/ros/humble 和驱动工作空间已安装')
    for pkg in ('nav2_map_server', 'ydlidar_ros2_driver', 'tf2_ros'):
        run_command(['ros2', 'pkg', 'prefix', pkg], timeout=8)


def pub_count(topic):
    if topic not in run_command(['ros2', 'topic', 'list'], timeout=9).splitlines():
        return 0
    result = run_command(['ros2', 'topic', 'info', topic], timeout=9)
    m = re.search(r'Publisher count:\s*(\d+)', result)
    if not m:
        raise RuntimeError('无法判断话题发布者数量：' + topic + '\n' + result)
    return int(m.group(1))


def check_conflict():
    for topic in ('/scan', '/map', '/keepout_filter_mask', '/costmap_filter_info',
                  '/competition/obstacle_lock_status', '/competition/road_status'):
        count = pub_count(topic)
        if count:
            raise RuntimeError(f'已有 {count} 个节点正在发布 {topic}；请手动关闭旧实例，本脚本不会结束其他进程')


def map_pair(run_dir):
    files = (ROOT / 'competition_navigation_map.yaml', ROOT / 'competition_navigation_map.pgm',
             ROOT / 'competition_keepout_mask.yaml', ROOT / 'competition_keepout_mask.pgm')
    if all(p.is_file() for p in files):
        say('MAP', '复用现有已知比赛静态地图与 Keepout 掩膜，不覆盖文件')
        return files[0], files[2]
    say('MAP', '静态地图不完整，调用原有 V5 生成器，仅在本次日志目录生成全新副本')
    out = run_dir / 'generated_map'
    out.mkdir()
    shutil.copy2(ROOT / 'generate_static_map.py', out / 'generate_static_map.py')
    run_command([sys.executable, str(out / 'generate_static_map.py')], timeout=25)
    for name in ('competition_navigation_map.yaml', 'competition_navigation_map.pgm',
                 'competition_keepout_mask.yaml', 'competition_keepout_mask.pgm'):
        if not (out / name).is_file():
            raise RuntimeError('地图生成文件缺失：' + name)
    return out / 'competition_navigation_map.yaml', out / 'competition_keepout_mask.yaml'


def x3_param_text(original, port):
    # Keep vendor's other lidar parameters intact; override only the configured port.
    result, n = re.subn(r'(?m)^(\s*port:\s*)[^#\n]+',
                        lambda m: m.group(1) + port, original, count=1)
    if n != 1:
        raise RuntimeError('无法安全更新临时雷达串口配置')
    return result


class Supervisor:
    def __init__(self, log_dir):
        self.log_dir = log_dir
        self.procs = []
        self.handles = []
        self.stopping = False

    def _record_children(self):
        # The external manager can safely recover only verifiably owned descendants.
        token = os.environ.get('ONECLICK_RUN_ID')
        if not token:
            return
        records = []
        for name, proc in self.procs:
            try:
                ticks = int(Path(f'/proc/{proc.pid}/stat').read_text().rsplit(')', 1)[1].split()[19])
            except (OSError, ValueError, IndexError):
                continue
            records.append({'name': name, 'pid': proc.pid, 'ticks': ticks,
                            'required': name != 'rviz'})
        output = self.log_dir / 'children.json'
        tmp = output.with_name('.children_%d.tmp' % os.getpid())
        tmp.write_text(json.dumps(records, ensure_ascii=False, indent=2), encoding='utf-8')
        os.replace(tmp, output)

    def launch(self, name, argv):
        # Bound each module log to 5 MiB x 3 files, preserving two rotated copies.
        handler = RotatingFileHandler(str(self.log_dir / (name + '.log')),
                                      maxBytes=5 * 1024 * 1024, backupCount=2,
                                      encoding='utf-8')
        handler.setFormatter(logging.Formatter('%(asctime)s [%(name)s] %(message)s'))
        try:
            p = subprocess.Popen([str(v) for v in argv], stdout=subprocess.PIPE,
                                 stderr=subprocess.STDOUT, start_new_session=True)
        except Exception:
            handler.close()
            raise
        self.procs.append((name, p))
        self._record_children()

        def pump():
            try:
                with p.stdout:
                    for raw in iter(p.stdout.readline, b''):
                        record = logging.LogRecord(name, logging.INFO, __file__, 0,
                                                   raw.decode('utf-8', errors='replace').rstrip('\n'),
                                                   (), None)
                        handler.emit(record)
            finally:
                handler.close()

        thread = threading.Thread(target=pump, name='oneclick-log-' + name, daemon=True)
        self.handles.append(thread)
        thread.start()
        say('START', f'{name} pid={p.pid} log={self.log_dir}/{name}.log')
        time.sleep(.2)
        self.check_children()
        return p

    def check_children(self):
        for name, proc in self.procs:
            rc = proc.poll()
            if rc is not None:
                file = self.log_dir / (name + '.log')
                tail = file.read_text(encoding='utf-8', errors='replace')[-900:] if file.exists() else ''
                raise RuntimeError(f'{name} 已提前退出({rc})，日志：{file}\n{tail}')

    def stop(self):
        if self.stopping:
            return
        self.stopping = True
        for name, proc in reversed(self.procs):
            if proc.poll() is None:
                try:
                    os.killpg(proc.pid, signal.SIGTERM)
                except ProcessLookupError:
                    pass
        for name, proc in reversed(self.procs):
            try:
                proc.wait(timeout=4)
            except subprocess.TimeoutExpired:
                # No kill -9; retain evidence and report if graceful stop fails.
                say('WARN', f'{name} 未在 4 秒内退出；请人工检查 PID={proc.pid}')
        for thread in self.handles:
            thread.join(timeout=1.0)
        say('STOP', '仅向本次脚本创建的进程组发送 SIGTERM')


def activate(supervisor, name):
    node = '/competition_oneclick_' + name
    expiry = time.monotonic() + 16
    while time.monotonic() < expiry:
        supervisor.check_children()
        try:
            s = run_command(['ros2', 'lifecycle', 'get', node], timeout=4)
        except RuntimeError:
            time.sleep(.3)
            continue
        if 'unconfigured' in s:
            run_command(['ros2', 'lifecycle', 'set', node, 'configure'], timeout=16)
        s = run_command(['ros2', 'lifecycle', 'get', node], timeout=5)
        if 'inactive' in s:
            run_command(['ros2', 'lifecycle', 'set', node, 'activate'], timeout=16)
        state = run_command(['ros2', 'lifecycle', 'get', node], timeout=5)
        if 'active [3]' in state:
            say('READY', f'{node} {state}')
            return
        time.sleep(.4)
    raise RuntimeError('生命周期未进入 active：' + node)


def start_tf_if_configured(sv, settings, frame):
    tf = settings['stationary_tf']
    if not tf['enabled']:
        say('TF', '未启用静止标定 TF；要求外部提供真实有效的 map->' + frame + ' 变换')
        return
    mx, my, yaw = tf['map_to_base']
    lx, ly, lz, lyaw = tf['base_to_laser']
    say('WARN', '启用人工实测静态 TF：仅允许小车保持静止！移动时坐标将失效')
    common = ['ros2', 'run', 'tf2_ros', 'static_transform_publisher']
    sv.launch('map_base_tf', common + ['--x', str(mx), '--y', str(my), '--z', '0',
             '--roll', '0', '--pitch', '0', '--yaw', str(yaw),
             '--frame-id', 'map', '--child-frame-id', 'base_link'])
    sv.launch('base_laser_tf', common + ['--x', str(lx), '--y', str(ly), '--z', str(lz),
             '--roll', '0', '--pitch', '0', '--yaw', str(lyaw),
             '--frame-id', 'base_link', '--child-frame-id', frame])


def run(args):
    settings, port, frame, original = check_inputs()
    say('CHECK', f'工程={ROOT}  启停区={settings["start_zone"]}  雷达帧={frame}')
    say('ROUTE', ' → '.join(ZONES[settings['start_zone']]))
    say('WARN', '当前仅预览：二维码实际位置、抓取停车点尚未标定；不会驱动车辆')
    if args.check:
        say('CHECK_PASS', '关键工程文件、配置结构已检查；未检查 ROS/串口硬件')
        return 0
    verify_ros()
    if not Path(port).exists():
        raise RuntimeError('雷达未连接或设备路径不存在：' + port + '（可在 oneclick_config.json 配置 serial_port）')
    if not os.access(port, os.R_OK | os.W_OK):
        raise RuntimeError('没有雷达串口读写权限：' + port + '，请检查设备权限')
    if shutil.which('fuser'):
        used = subprocess.run(['fuser', port], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        if used.returncode == 0:
            raise RuntimeError('雷达串口已经被其他程序占用：' + port + '；不会自动关闭其他程序')
    RUN_HOME.mkdir(parents=True, exist_ok=True)
    lock_file = open(RUN_HOME / 'instance.lock', 'w')
    try:
        fcntl.flock(lock_file.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError as exc:
        raise RuntimeError('本项目一键启动实例已经运行，请关闭原窗口再启动') from exc
    check_conflict()
    managed = os.environ.get('ONECLICK_LOG_DIR')
    log_dir = Path(managed) if managed else (RUN_HOME / 'logs' / (time.strftime('%Y%m%d_%H%M%S') + '_' + str(os.getpid())))
    # The manager pre-creates its own run directory; no other locations are accepted.
    if managed and (log_dir.resolve().parent != (RUN_HOME / 'logs').resolve() or not log_dir.is_dir()):
        raise RuntimeError('管理器提供的日志目录非法：' + str(log_dir))
    log_dir.mkdir(parents=True, exist_ok=True)
    say('LOG', str(log_dir))
    sv = Supervisor(log_dir)
    stopped = False
    conflict_reported = False

    def handle(sig, _):
        raise KeyboardInterrupt

    old_int = signal.signal(signal.SIGINT, handle)
    old_term = signal.signal(signal.SIGTERM, handle)
    old_hup = signal.signal(signal.SIGHUP, handle)
    try:
        physical_map, keepout = map_pair(log_dir)
        prefix = run_command(['ros2', 'pkg', 'prefix', 'nav2_map_server'], timeout=8)
        exe_dir = Path(prefix) / 'lib/nav2_map_server'
        for exe in ('map_server', 'costmap_filter_info_server'):
            if not (exe_dir / exe).is_file():
                raise RuntimeError('缺少 Nav2 可执行程序：' + str(exe_dir / exe))
        map_exe = str(exe_dir / 'map_server')
        sv.launch('map', [map_exe, '--ros-args', '-r', '__node:=competition_oneclick_map',
                          '-p', f'yaml_filename:={physical_map}', '-p', 'topic_name:=/map',
                          '-p', 'frame_id:=map'])
        sv.launch('mask', [map_exe, '--ros-args', '-r', '__node:=competition_oneclick_mask',
                           '-p', f'yaml_filename:={keepout}', '-p', 'topic_name:=/keepout_filter_mask',
                           '-p', 'frame_id:=map'])
        sv.launch('info', [str(exe_dir / 'costmap_filter_info_server'), '--ros-args',
                           '-r', '__node:=competition_oneclick_info', '-p', 'type:=0',
                           '-p', 'filter_info_topic:=/costmap_filter_info',
                           '-p', 'mask_topic:=/keepout_filter_mask', '-p', 'base:=0.0',
                           '-p', 'multiplier:=1.0'])
        for name in ('map', 'mask', 'info'):
            activate(sv, name)
        settings_file = log_dir / 'x3_runtime.yaml'
        settings_file.write_text(x3_param_text(original, port), encoding='utf-8')
        start_tf_if_configured(sv, settings, frame)
        sv.launch('lidar', ['ros2', 'run', 'ydlidar_ros2_driver', 'ydlidar_ros2_driver_node',
                            '--ros-args', '--params-file', str(settings_file)])
        say('WAIT', '等待 /scan、/map 与地图到雷达的 TF；雷达未接入或位姿未标定时会安全停止')
        try:
            run_command([sys.executable, str(ROOT / 'simple_lock_stage' / 'preflight_fixed_route.py'),
                         '--mode', 'inputs', '--timeout', '25'], timeout=34)
        except Exception as exc:
            raise RuntimeError('输入话题/TF 校验未通过，未启动障碍物锁定：' + str(exc)) from exc
        say('READY', '雷达扫描、静态地图、TF 已收到；物理坐标精度仍须实测验证')
        expected = str(settings['expected_obstacles'])
        sv.launch('lock', [sys.executable, str(ROOT / 'simple_lock_stage' / 'lock_obstacles.py'),
                            '--expected', expected, '--snapshot', str(log_dir / 'obstacles_locked.json')])
        sv.launch('topology', [sys.executable, str(ROOT / 'topology_stage6' / 'road_topology.py'),
                                '--expected-obstacles', expected, '--mission',
                                *ZONES[settings['start_zone']]])
        sv.launch('mission_export', [sys.executable, str(HERE / 'mission_export.py'),
                                     '--zone', settings['start_zone'],
                                     '--output', str(log_dir / 'mission_locked.json')])
        run_command([sys.executable, str(ROOT / 'simple_lock_stage' / 'preflight_fixed_route.py'),
                     '--mode', 'modules', '--timeout', '15'], timeout=22)
        if settings['show_rviz']:
            if not shutil.which('rviz2'):
                raise RuntimeError('缺少 rviz2，无法打开规划预览')
            sv.launch('rviz', ['rviz2', '-d', str(HERE / 'oneclick_preview.rviz')])
        say('RUNNING', '正在识别障碍物：确认后一次封路并输出整场任务道路路线预览')
        say('INFO', '完整路线保存到：' + str(log_dir / 'mission_locked.json'))
        say('INFO', '没有 Nav2 FollowPath / STM32 输出，按 Ctrl+C 或关闭本终端退出')
        while True:
            time.sleep(.7)
            if settings['show_rviz'] and any(n == 'rviz' and p.poll() is not None for n, p in sv.procs):
                say('INFO', 'RViz 窗口已关闭；结束本次一键启动流程')
                return 0
            sv.check_children()
            report = log_dir / 'mission_locked.json'
            if report.is_file() and not stopped:
                say('LOCKED', '道路封闭及任务路线已冻结；查看 RViz 和 mission_locked.json')
                stopped = True
            if stopped and not conflict_reported:
                latest = json.loads(report.read_text(encoding='utf-8'))
                if latest.get('safety_state') == 'CONFLICT_STOP':
                    say('CONFLICT', '出现额外目标冲突，已清空任务路线预览；仅保留原有封路锁定记录')
                    conflict_reported = True
    except KeyboardInterrupt:
        say('INFO', '收到退出请求')
        return 0
    finally:
        sv.stop()
        signal.signal(signal.SIGINT, old_int)
        signal.signal(signal.SIGTERM, old_term)
        signal.signal(signal.SIGHUP, old_hup)
        lock_file.close()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--check', action='store_true', help='仅检查文件和配置，绝不启动硬件')
    args = parser.parse_args()
    result = 2
    try:
        result = run(args)
    except (RuntimeError, ValueError, OSError, subprocess.TimeoutExpired, json.JSONDecodeError) as exc:
        say('ERROR', str(exc))
        result = 2
    finally:
        # A graceful GUI close/stop is distinguishable from an unexpected crash.
        if os.environ.get('ONECLICK_RUN_ID') and os.environ.get('ONECLICK_LOG_DIR'):
            base = Path(os.environ['ONECLICK_LOG_DIR'])
            if base.is_dir() and base.parent.resolve() == (RUN_HOME / 'logs').resolve():
                completed = {'result': result, 'time': time.time(),
                             'run_id': os.environ['ONECLICK_RUN_ID']}
                part = base / '.run_result.tmp'
                part.write_text(json.dumps(completed), encoding='utf-8')
                os.replace(part, base / 'run_result.json')
    return result


if __name__ == '__main__':
    sys.exit(main())
