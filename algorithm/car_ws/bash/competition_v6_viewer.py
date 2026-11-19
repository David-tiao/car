#!/usr/bin/env python3
"""V6 competition map + lidar + honest road-planning preview, ROS 2 Humble.

No chassis/Nav2 commands. The real road-lock stack is enabled ONLY when the
user's existing oneclick_config.json contains measured, enabled stationary TF.
An off-field inset shows raw /scan WITHOUT pretending that it is map-registered.
"""
import argparse
import fcntl
import json
import math
import os
from pathlib import Path
import re
import shutil
import signal
import subprocess
import sys
import tempfile
import time
from datetime import datetime

HOME = Path.home()
HERE = Path(__file__).resolve().parent
WS = HERE.parent
ROOT = WS / 'maps/competition_v1'
MAP = ROOT / 'v6_static_preview'
STATE = HOME / '.local/state/competition_v6_viewer'
LOGS = STATE / 'logs'
RUN_FILE = STATE / 'runtime.json'
ROS = Path('/opt/ros/humble/setup.bash')
OVERLAY = WS / 'install/setup.bash' if (WS / 'install/setup.bash').is_file() else WS / 'install_oneclick_lidar/setup.bash'
EXTRA_OVERLAY = WS / 'install_oneclick_lidar/setup.bash'
CONFIG = WS / 'config/x3.yaml'
MISSION_CONFIG = ROOT / 'oneclick_stage/oneclick_config.json'
RVIZ_TEMPLATE = HERE / 'preview.rviz'
VISUAL = HERE / 'visual_overlay.py'
LOCK = ROOT / 'simple_lock_stage/lock_obstacles.py'
TOPO = ROOT / 'topology_stage6/road_topology.py'
START_TIMEOUT = 28
STOP_TIMEOUT = 9


class RequestedStop(Exception):
    pass


def on_stop(sig, frame):
    raise RequestedStop('收到关闭信号 {}'.format(sig))


def ensure_dirs():
    for d in (STATE, LOGS):
        d.mkdir(parents=True, exist_ok=True, mode=0o700)
        d.chmod(0o700)


def say(level, msg):
    line = '[{}] [{}] {}'.format(datetime.now().astimezone().isoformat(timespec='seconds'), level, msg)
    print(line, flush=True)
    try:
        ensure_dirs()
        with (LOGS / 'launcher.log').open('a', encoding='utf-8') as f:
            f.write(line + '\n')
    except OSError as exc:
        print('[WARN] 日志写入失败: {}'.format(exc), file=sys.stderr)


def atomic_write(path, data, mode=0o600):
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, tmp = tempfile.mkstemp(prefix='.' + path.name + '-', dir=str(path.parent))
    try:
        with os.fdopen(fd, 'wb') as f:
            f.write(data)
            f.flush()
            os.fsync(f.fileno())
        os.chmod(tmp, mode)
        os.replace(tmp, path)
    finally:
        if os.path.exists(tmp):
            os.unlink(tmp)


def save_state(state):
    atomic_write(RUN_FILE, (json.dumps(state, ensure_ascii=False, indent=2) + '\n').encode('utf-8'))


def load_state():
    if not RUN_FILE.is_file():
        return None
    try:
        st = json.loads(RUN_FILE.read_text(encoding='utf-8'))
        if st.get('project') != 'competition_v6_viewer' or not isinstance(st.get('children'), dict):
            raise ValueError('project/children 不符合预期')
        return st
    except (OSError, ValueError, TypeError) as exc:
        raise RuntimeError('运行记录损坏；为防误杀拒绝清理: {}'.format(exc)) from exc


def proc_info(pid):
    try:
        text = Path('/proc/{}/stat'.format(int(pid))).read_text()
        fields = text[text.rfind(')') + 2:].split()
        return dict(state=fields[0], pgid=int(fields[2]), sid=int(fields[3]), start=int(fields[19]))
    except (OSError, ValueError, TypeError, IndexError):
        return None


def same_process(record, leader=False):
    try:
        pid = int(record['pid'])
        info = proc_info(pid)
        return bool(info and info['state'] != 'Z' and info['start'] == int(record['start'])
                    and (not leader or (info['pgid'] == pid and info['sid'] == pid)))
    except (KeyError, TypeError, ValueError):
        return False


def group_members(pgid):
    try:
        pgid = int(pgid)
    except (TypeError, ValueError):
        return []
    found = []
    for d in Path('/proc').iterdir():
        if d.name.isdigit():
            info = proc_info(d.name)
            if info and info['state'] != 'Z' and info['pgid'] == pgid:
                found.append(int(d.name))
    return found


def owned_stop(st, processes=None):
    """Never kill a group whose recorded leader identity cannot be verified."""
    errors = []
    children = st.get('children', {})
    for name, record in reversed(list(children.items())):
        pid = record.get('pid')
        if same_process(record, leader=True):
            try:
                os.killpg(pid, signal.SIGTERM)
                say('STOP', '{} SIGTERM pgid={}'.format(name, pid))
            except ProcessLookupError:
                pass
            except OSError as exc:
                errors.append('{}: {}'.format(name, exc))
        elif group_members(pid):
            errors.append('{}: 组长身份失效但仍有组成员；拒绝误杀'.format(name))
    deadline = time.monotonic() + STOP_TIMEOUT
    while time.monotonic() < deadline:
        if not any(group_members(r.get('pid')) for r in children.values()):
            break
        time.sleep(.15)
    for name, record in reversed(list(children.items())):
        if group_members(record.get('pid')) and same_process(record, leader=True):
            try:
                os.killpg(int(record['pid']), signal.SIGKILL)
                say('WARN', '{} 超时，已确认组长身份后 SIGKILL'.format(name))
            except OSError as exc:
                errors.append('{}: SIGKILL: {}'.format(name, exc))
    if processes:
        for p in processes.values():
            try:
                p.wait(timeout=1)
            except subprocess.TimeoutExpired:
                pass
    deadline = time.monotonic() + 2
    while time.monotonic() < deadline and any(group_members(r.get('pid')) for r in children.values()):
        time.sleep(.15)
    for name, record in children.items():
        if group_members(record.get('pid')):
            errors.append('{}: 进程组 {} 未完全退出'.format(name, record['pid']))
    return errors


def env_command(*cmd):
    # 先加载 Humble，再加载当前 car_ws；单独雷达 overlay 可选。
    # 只组装命令，原有模块进程身份校验与安全停止逻辑不变。
    return ['bash', '-c',
            'set -e; set +u; source "$1"; '
            'if [ -f "$2" ]; then source "$2"; fi; '
            'if [ -f "$3" ] && [ "$3" != "$2" ]; then source "$3"; fi; '
            'shift 3; exec "$@"',
            'competition_v6_viewer', str(ROS), str(OVERLAY),
            str(EXTRA_OVERLAY), *map(str, cmd)]


def call(args, timeout=9):
    return subprocess.run(env_command(*args), text=True, stdout=subprocess.PIPE,
                          stderr=subprocess.PIPE, timeout=timeout, check=False)


def required_call(args, timeout=9):
    res = call(args, timeout)
    if res.returncode:
        raise RuntimeError('命令 {} 失败 ({}): {}'.format(' '.join(args), res.returncode,
                          (res.stderr or res.stdout).strip()[-700:]))
    return res.stdout


def read_yaml_device():
    s = CONFIG.read_text(encoding='utf-8')
    p = re.search(r'^\s*port:\s*[\'"]?([^\s\'"#]+)', s, re.M)
    f = re.search(r'^\s*frame_id:\s*[\'"]?([^\s\'"#]+)', s, re.M)
    if not p or not f or not p.group(1).startswith('/dev/'):
        raise RuntimeError('x3.yaml 的 port / frame_id 不合法')
    return p.group(1), f.group(1)


def read_stationary_calibration():
    """An explicit user-measured TF, NOT a default inferred from the rule drawing."""
    if not MISSION_CONFIG.is_file():
        raise RuntimeError('缺少已有配置 {}'.format(MISSION_CONFIG))
    obj = json.loads(MISSION_CONFIG.read_text(encoding='utf-8'))
    if obj.get('start_zone') != 'upper_right':
        raise RuntimeError('当前工程起点不是已确认的 upper_right')
    expected = obj.get('expected_obstacles')
    if type(expected) is not int or expected < 1:
        raise RuntimeError('expected_obstacles 必须为正整数')
    tf = obj.get('stationary_tf', {})
    if not isinstance(tf, dict) or type(tf.get('enabled')) is not bool:
        raise RuntimeError('stationary_tf.enabled 配置缺失')
    if not tf['enabled']:
        return expected, None
    values = []
    for name, n in (('map_to_base', 3), ('base_to_laser', 4)):
        v = tf.get(name)
        if not isinstance(v, list) or len(v) != n or any(type(x) not in (float, int) or
                                                          not math.isfinite(x) for x in v):
            raise RuntimeError('stationary_tf.{} 缺少实测坐标；严禁补假数'.format(name))
        values.append(v)
    (x, y, yaw), (lx, ly, lz, lyaw) = values
    if not (0 <= x <= 2.4 and 0 <= y <= 2.4 and 0 <= lz <= .6 and
            abs(lx) <= .6 and abs(ly) <= .6 and abs(yaw) <= math.pi and abs(lyaw) <= math.pi):
        raise RuntimeError('静态 TF 数值超出校验范围（位置米、角度弧度）；拒绝启用')
    return expected, values


def preflight(offline=False):
    files = [ROS, OVERLAY, CONFIG, MISSION_CONFIG, RVIZ_TEMPLATE, VISUAL,
             MAP / 'competition_navigation_map.yaml', MAP / 'competition_navigation_map.pgm',
             MAP / 'competition_keepout_mask.yaml', MAP / 'competition_keepout_mask.pgm',
             MAP / 'verify_static_map.py', MAP / 'zones.json', LOCK,
             ROOT / 'simple_lock_stage/lock_core.py', TOPO]
    missing = [str(p) for p in files if not p.is_file()]
    if missing:
        raise RuntimeError('必要文件缺失: {}'.format(' | '.join(missing)))
    for tool in ('bash', 'python3', 'fuser'):
        if not shutil.which(tool):
            raise RuntimeError('必需工具缺失：{}'.format(tool))
    checked = subprocess.run(['/usr/bin/python3', str(MAP / 'verify_static_map.py')],
                             cwd=str(MAP), capture_output=True, text=True, timeout=20)
    if checked.returncode:
        raise RuntimeError('地图校验失败: {}'.format((checked.stderr or checked.stdout)[-800:]))
    port, frame = read_yaml_device()
    expected, measured = read_stationary_calibration()
    if not offline:
        if not os.environ.get('DISPLAY') and not os.environ.get('WAYLAND_DISPLAY'):
            raise RuntimeError('没有图形桌面环境')
        if not os.access(port, os.R_OK | os.W_OK):
            raise RuntimeError('雷达串口不可访问: {}'.format(port))
        occupancy = subprocess.run(['fuser', port], stdout=subprocess.PIPE,
                                   stderr=subprocess.PIPE, timeout=5)
        if occupancy.returncode == 0:
            raise RuntimeError('串口 {} 被占用（{}）；拒绝结束未知进程'.format(port,
                              occupancy.stdout.decode(errors='replace').strip()))
        if occupancy.returncode not in (0, 1):
            raise RuntimeError('无法确认串口是否被占用: fuser code={}'.format(occupancy.returncode))
        for pkg in ('nav2_map_server', 'ydlidar_ros2_driver', 'tf2_ros'):
            required_call(['ros2', 'pkg', 'prefix', pkg])
        required_call(['bash', '-c', 'command -v rviz2'])
        nodes = required_call(['ros2', 'node', 'list']).splitlines()
        for name in ('/competition_v6_map_server', '/competition_v6_mask_server',
                     '/ydlidar_ros2_driver_node', '/competition_v6_static_visual',
                     '/competition_obstacle_lock', '/competition_stage6_road_topology'):
            if name in nodes:
                raise RuntimeError('节点 {} 已存在，可能属于其他实例；拒绝冲突启动'.format(name))
        known_topics = set(required_call(['ros2', 'topic', 'list']).splitlines())
        for topic in ('/map', '/keepout_filter_mask', '/scan', '/competition/v6/scene'):
            if topic not in known_topics:
                continue
            result = required_call(['ros2', 'topic', 'info', topic])
            m = re.search(r'Publisher count:\s*(\d+)', result)
            if not m:
                raise RuntimeError('无法查明话题 {} 的发布者'.format(topic))
            if int(m.group(1)):
                raise RuntimeError('话题 {} 已有发布者；拒绝混用外部数据'.format(topic))
    say('PASS', 'V6 地图文件/依赖/串口检查通过；雷达帧={}；障碍目标={}'.format(frame, expected))
    say('INFO', ('实测静止 TF 已启用：本次会尝试锁路（小车必须静止）' if measured else
                 '未提供实测 TF：只显示原始雷达实时预览、地图和静态 Dijkstra 基线；不执行真实封路'))
    return port, frame, expected, measured


# Direct rclpy subscriptions avoid repeated short-lived ros2 topic echo processes.
PROBE_SCRIPT = r'''
import math, sys, time
import rclpy
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, qos_profile_sensor_data
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
mode, topic, frame = sys.argv[1:4]
rclpy.init()
n = rclpy.create_node('competition_v6_probe_' + str(__import__('os').getpid()), enable_rosout=False)
messages = []
if mode in ('map','status'):
    qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL, reliability=QoSReliabilityPolicy.RELIABLE)
    sub = n.create_subscription(OccupancyGrid if mode == 'map' else String, topic, messages.append, qos)
else:
    sub = n.create_subscription(LaserScan, topic, messages.append, qos_profile_sensor_data)
try:
    end = time.monotonic() + 10
    while not messages and time.monotonic() < end:
        rclpy.spin_once(n, timeout_sec=.2)
    if not messages:
        raise RuntimeError('NO_MESSAGE topic={} publisher_count={}'.format(topic, n.count_publishers(topic)))
    m = messages[0]
    if mode != 'status' and m.header.frame_id != frame:
        raise RuntimeError('frame_id={} expected={}'.format(m.header.frame_id, frame))
    if mode == 'status':
        import json
        data = json.loads(m.data)
        if not isinstance(data, dict):
            raise RuntimeError('status is not a JSON object')
        print('STATUS_VALID topic={} keys={}'.format(topic, sorted(data)))
    elif mode == 'map':
        if (m.info.width != 280 or m.info.height != 280 or abs(m.info.resolution-.01) > 1e-5
            or len(m.data) != 78400):
            raise RuntimeError('wrong map shape/resolution')
        print('MAP_VALID topic={} size=280x280 resolution=0.01'.format(topic))
    else:
        good = sum(1 for v in m.ranges if math.isfinite(v) and max(.10, m.range_min) <= v <= min(8., m.range_max))
        if good < 10:
            raise RuntimeError('scan valid points={} (<10)'.format(good))
        print('SCAN_VALID topic={} valid_points={} frame={}'.format(topic, good, frame))
finally:
    n.destroy_node()
    rclpy.shutdown()
'''

TF_PROBE = r'''
import sys,time,rclpy
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener
rclpy.init()
n=rclpy.create_node('competition_v6_tf_probe',enable_rosout=False)
b=Buffer(); listener=TransformListener(b,n)
try:
    end=time.monotonic()+10
    while time.monotonic()<end:
        rclpy.spin_once(n,timeout_sec=.2)
        if b.can_transform('map',sys.argv[1],Time()):
            print('TF_VALID map -> '+sys.argv[1]);break
    else:raise RuntimeError('map -> '+sys.argv[1]+' not available')
finally:
    n.destroy_node();rclpy.shutdown()
'''


def probe(mode, topic, frame, parent):
    if parent.poll() is not None:
        raise RuntimeError('进程 {} 已退出 code={}'.format(topic, parent.returncode))
    try:
        output = required_call(['/usr/bin/python3', '-c', PROBE_SCRIPT, mode, topic, frame], 15)
    except subprocess.TimeoutExpired as exc:
        raise RuntimeError('{} 订阅超时：{}'.format(topic, exc)) from exc
    say('PASS', output.strip()[-500:])


def lifecycle(name, parent):
    deadline = time.monotonic() + START_TIMEOUT
    last = ''
    while time.monotonic() < deadline:
        if parent.poll() is not None:
            raise RuntimeError('{} 进程已退出 code={}'.format(name, parent.returncode))
        try:
            res = call(['ros2', 'lifecycle', 'get', '/' + name], 4)
            last = (res.stdout + res.stderr).strip()[-300:]
            if res.returncode == 0:
                for transition in ('configure', 'activate'):
                    required_call(['ros2', 'lifecycle', 'set', '/' + name, transition], 9)
                value = required_call(['ros2', 'lifecycle', 'get', '/' + name], 6)
                if not value.strip().startswith('active'):
                    raise RuntimeError('{} 状态={}，没有激活'.format(name, value.strip()))
                say('PASS', '{} lifecycle active'.format(name))
                return
        except subprocess.TimeoutExpired as exc:
            last = str(exc)
        time.sleep(.35)
    raise RuntimeError('{} lifecycle 超时：{}'.format(name, last))


def create_rviz(calibrated):
    data = RVIZ_TEMPLATE.read_text(encoding='utf-8')
    if calibrated:
        # Exact labeled block replacements; refuse silently mismatched config.
        for name in ('REAL LOCKED ROAD PATH (calibrated only)',
                     'REAL LOCKED ROAD GRAPH (calibrated only)',
                     'REAL LOCKED OBSTACLES (calibrated only)',
                     'REGISTERED LaserScan (calibrated only)'):
            anchor = '      Enabled: false\n      Name: ' + name + '\n'
            if data.count(anchor) != 1:
                raise RuntimeError('RViz 配置未找到待启用显示：' + name)
            data = data.replace(anchor, '      Enabled: true\n      Name: ' + name + '\n', 1)
    path = STATE / 'runtime_preview.rviz'
    atomic_write(path, data.encode('utf-8'), 0o600)
    return path


def recent_lines(path, count=24):
    try:
        return '\n'.join(path.read_text(errors='replace').splitlines()[-count:])
    except OSError:
        return '(日志不可读)'


def wait_visual(proc, file, log):
    deadline = time.monotonic() + 12
    while time.monotonic() < deadline:
        if proc.poll() is not None:
            raise RuntimeError('场地可视化提前退出(code={}):\n{}'.format(proc.returncode, recent_lines(log)))
        if file.is_file():
            msg = json.loads(file.read_text(encoding='utf-8'))
            if msg.get('state') != 'PUBLISHED' or msg.get('pid') != proc.pid:
                raise RuntimeError('场地显示就绪文件与进程不一致')
            say('PASS', 'V6 彩色场地、9节点12路段、实时雷达预览和 Dijkstra 基线节点已发布')
            return
        time.sleep(.15)
    raise RuntimeError('场地可视化超时: {}'.format(recent_lines(log)))


def run_start(desktop=False):
    ensure_dirs()
    with (STATE / 'run.lock').open('a+') as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            raise RuntimeError('另一启动实例仍在执行；请关闭上一个窗口')
        legacy = STATE / 'pids'
        if legacy.is_file() and legacy.stat().st_size:
            raise RuntimeError('发现旧版未核实的 pids 记录；拒绝覆盖：{}'.format(legacy))
        st = load_state()
        if st:
            if same_process(st.get('supervisor', {})):
                raise RuntimeError('旧启动器尚在运行；无法取得独占控制权')
            left = owned_stop(st)
            if left:
                raise RuntimeError('旧实例仍有未确认进程: {}'.format('; '.join(left)))
        port, frame, expected, calibration = preflight()
        own = proc_info(os.getpid())
        stamp = datetime.now().strftime('%Y%m%d_%H%M%S_%f')
        st = {'project':'competition_v6_viewer','version':3,
              'supervisor':{'pid':os.getpid(),'start':own['start']},
              'started':datetime.now().astimezone().isoformat(),'stamp':stamp,
              'mode': 'STATIONARY_LOCK' if calibration else 'UNREGISTERED_PREVIEW',
              'status':'STARTING','children':{}}
        save_state(st)
        signal_list = (signal.SIGTERM, signal.SIGHUP, signal.SIGINT)
        old_handlers = {sig:signal.getsignal(sig) for sig in signal_list}
        for sig in signal_list:
            signal.signal(sig, on_stop)
        children = {}

        def spawn(name, args):
            logfile = LOGS / '{}_{}.log'.format(stamp, name)
            with logfile.open('a', encoding='utf-8') as out:
                p = subprocess.Popen(env_command(*args), start_new_session=True,
                                     stdin=subprocess.DEVNULL, stdout=out,
                                     stderr=subprocess.STDOUT, close_fds=True)
            children[name] = p
            info = proc_info(p.pid)
            if info is None:
                raise RuntimeError('{} 创建失败：{}'.format(name, recent_lines(logfile)))
            st['children'][name] = {'pid':p.pid,'start':info['start'],'log':str(logfile)}
            save_state(st)
            say('INFO', '{} PID={} 日志={}'.format(name,p.pid,logfile))
            return p, logfile

        error = None
        leftovers = []
        try:
            map_p, _ = spawn('map', ['ros2','run','nav2_map_server','map_server','--ros-args',
                                    '-r','__node:=competition_v6_map_server',
                                    '-p','yaml_filename:={}'.format(MAP/'competition_navigation_map.yaml'),
                                    '-p','topic_name:=map','-p','frame_id:=map'])
            lifecycle('competition_v6_map_server', map_p)
            probe('map', '/map', 'map', map_p)

            mask, _ = spawn('mask', ['ros2','run','nav2_map_server','map_server','--ros-args',
                                     '-r','__node:=competition_v6_mask_server',
                                     '-p','yaml_filename:={}'.format(MAP/'competition_keepout_mask.yaml'),
                                     '-p','topic_name:=keepout_filter_mask','-p','frame_id:=map'])
            lifecycle('competition_v6_mask_server', mask)
            probe('map', '/keepout_filter_mask', 'map', mask)

            if calibration:
                (mx,my,yaw),(lx,ly,lz,lyaw) = calibration
                cmd = ['ros2','run','tf2_ros','static_transform_publisher']
                spawn('tf_map_base', cmd + ['--x',str(mx),'--y',str(my),'--z','0',
                      '--roll','0','--pitch','0','--yaw',str(yaw),
                      '--frame-id','map','--child-frame-id','base_link'])
                spawn('tf_base_laser', cmd + ['--x',str(lx),'--y',str(ly),'--z',str(lz),
                      '--roll','0','--pitch','0','--yaw',str(lyaw),
                      '--frame-id','base_link','--child-frame-id',frame])
                required_call(['/usr/bin/python3','-c',TF_PROBE,frame], 15)
                say('PASS','已校验 map -> {} TF；仅适用于静止测试'.format(frame))

            lidar, _ = spawn('lidar', ['ros2','run','ydlidar_ros2_driver','ydlidar_ros2_driver_node',
                                       '--ros-args','--params-file',CONFIG])
            probe('scan','/scan',frame,lidar)

            ready = LOGS / (stamp + '_scene_ready.json')
            visual, vislog = spawn('visual', ['/usr/bin/python3',VISUAL,'--zones',MAP/'zones.json',
                    '--topology',TOPO,'--ready-file',ready])
            wait_visual(visual, ready, vislog)

            if calibration:
                lock, _ = spawn('lock', ['/usr/bin/python3',LOCK,'--expected',str(expected)])
                topology, _ = spawn('topology',['/usr/bin/python3',TOPO,'--start','C3',
                                         '--goal','A3','--expected-obstacles',str(expected)])
                probe('status','/competition/obstacle_lock_status','',lock)
                probe('status','/competition/road_status','',topology)
                say('WARN','真实障碍物锁路已启动，但 C3→A3 仍是测试目标，非扫码决定的正式任务顺序')

            rviz_file = create_rviz(bool(calibration))
            rviz, _ = spawn('rviz', ['rviz2','-d',rviz_file])
            time.sleep(1.2)
            if rviz.poll() is not None:
                raise RuntimeError('RViz 打开失败，code={}'.format(rviz.returncode))
            st['status']='RUNNING'
            save_state(st)
            say('RUNNING','地图、彩色场地、实时原始扫描、道路拓扑、Dijkstra 基线和 RViz 已启动')
            if not calibration:
                say('WARN','原始扫描为场地外独立预览，非地图配准；真实障碍物锁路需已测量静态 TF')
            say('INFO','关闭 RViz 或日志窗口将自动关闭本次地图、雷达和可视化进程')
            while True:
                time.sleep(.35)
                if rviz.poll() is not None:
                    say('INFO','RViz 已关闭，开始清理')
                    break
                for name, proc in children.items():
                    if name != 'rviz' and proc.poll() is not None:
                        raise RuntimeError('运行中模块 {} 退出 code={}：日志={}'.format(
                            name,proc.returncode, st['children'][name]['log']))
        except (RequestedStop, KeyboardInterrupt) as exc:
            say('INFO','请求停止：{}'.format(exc))
        except BaseException as exc:
            error = str(exc)
            say('ERROR','启动/运行失败：{}'.format(error))
        finally:
            for sig in signal_list:
                signal.signal(sig, signal.SIG_IGN)
            st['status']='STOPPING'
            save_state(st)
            leftovers = owned_stop(st, children)
            st['status'] = 'UNKNOWN' if leftovers else ('FAILED' if error else 'STOPPED')
            if error:
                st['failure'] = error
            if leftovers:
                st['cleanup_error'] = '; '.join(leftovers)
                say('ERROR','清理未完全成功：{}'.format(st['cleanup_error']))
            save_state(st)
            say('INFO','退出状态={}；原始日志保留'.format(st['status']))
            for sig, handler in old_handlers.items():
                signal.signal(sig,handler)
        if error:
            raise RuntimeError(error + ('；清理：' + '; '.join(leftovers) if leftovers else ''))
        if leftovers:
            raise RuntimeError('清理不完整：' + '; '.join(leftovers))


def managed_stop():
    ensure_dirs()
    with (STATE / 'manage.lock').open('a+') as f:
        fcntl.flock(f, fcntl.LOCK_EX)
        st=load_state()
        if not st:
            say('INFO','STOPPED: 无运行记录')
            return
        sup=st.get('supervisor',{})
        if same_process(sup):
            cmdline=Path('/proc/{}/cmdline'.format(sup['pid'])).read_bytes().replace(b'\0',b' ')
            if b'competition_v6_viewer.py' not in cmdline:
                raise RuntimeError('管理器进程身份与记录不匹配，拒绝结束')
            os.kill(int(sup['pid']),signal.SIGTERM)
            deadline=time.monotonic()+STOP_TIMEOUT+12
            while time.monotonic()<deadline and same_process(sup):
                time.sleep(.2)
            if same_process(sup):
                raise RuntimeError('旧管理器没有按时退出，拒绝强制停止未知进程')
        else:
            leftovers=owned_stop(st)
            if leftovers:
                raise RuntimeError('旧实例无法确认全部关闭：'+'; '.join(leftovers))
            st['status']='STOPPED'
            save_state(st)
        st=load_state()
        if st and st.get('status') not in ('STOPPED','FAILED'):
            raise RuntimeError('旧管理器退出状态不明确：{}'.format(st.get('status')))
        if st and any(group_members(r.get('pid')) for r in st.get('children',{}).values()):
            raise RuntimeError('旧实例仍有组成员，未确认退出')
        say('INFO','旧 V6 实例已安全退出，仅处理本项目的进程')


def status():
    st=load_state()
    if st is None:
        say('INFO','STOPPED: 无运行记录')
        return 0
    sup=same_process(st.get('supervisor',{}))
    alive={key:bool(group_members(r.get('pid'))) for key,r in st['children'].items()}
    if sup and st.get('status')=='RUNNING' and all(alive.values()):
        label='RUNNING'
    elif not sup and not any(alive.values()):
        label='STOPPED' if st.get('status')=='STOPPED' else 'FAILED'
    else:
        label='UNKNOWN'
    say('INFO','{} 模式={} 进程={}'.format(label,st.get('mode'),alive))
    return 0 if label in ('RUNNING','STOPPED') else 1


def logs():
    arr=sorted(LOGS.glob('*.log'),key=lambda f:f.stat().st_mtime,reverse=True)
    for f in arr[:6]:
        print('\n---- {} ----'.format(f))
        print(recent_lines(f,25))


def main():
    parser=argparse.ArgumentParser(description='V6 一键地图+原始雷达+真实锁路（需要测量 TF）；不控制电机')
    parser.add_argument('action',nargs='?',default='start',choices=['start','stop','restart','status','doctor','logs'])
    parser.add_argument('--offline',action='store_true')
    parser.add_argument('--desktop',action='store_true')
    args=parser.parse_args()
    ensure_dirs()
    code=0
    try:
        if args.action in ('start','restart'):
            managed_stop()
            run_start(desktop=args.desktop)
        elif args.action=='stop':
            managed_stop()
        elif args.action=='status':
            code=status()
        elif args.action=='doctor':
            preflight(offline=args.offline)
        elif args.action=='logs':
            logs()
    except (RequestedStop,KeyboardInterrupt) as exc:
        say('INFO','用户请求结束：{}'.format(exc))
    except (RuntimeError,ValueError,OSError,subprocess.TimeoutExpired) as exc:
        say('ERROR','{} 失败：{}'.format(args.action,exc))
        code=1
    if code and args.desktop and sys.stdin.isatty():
        try:
            input('启动失败，查看上方具体错误；按回车关闭窗口……')
        except (EOFError,KeyboardInterrupt):
            pass
    return code


if __name__=='__main__':
    raise SystemExit(main())
