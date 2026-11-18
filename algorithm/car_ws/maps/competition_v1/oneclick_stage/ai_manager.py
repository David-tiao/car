#!/usr/bin/env python3
"""Lifecycle management for the existing stationary radar/locked-road preview.

No motor output. Only processes carrying this launch's identity are signalled.
Works on Linux / Ubuntu 22.04, Python 3.10+.
"""
from __future__ import annotations

import argparse
from datetime import datetime
import fcntl
import json
import os
from pathlib import Path
import platform
import re
import secrets
import shutil
import signal
import subprocess
import sys
import time
import xml.etree.ElementTree as ET

import main as pipeline

ROOT = Path(__file__).resolve().parent.parent
HERE = Path(__file__).resolve().parent
WS = ROOT.parent.parent
LIDAR_PACKAGE = 'ydlidar_ros2_driver'
LIDAR_SOURCE = WS / 'src' / LIDAR_PACKAGE
LIDAR_BUILD = WS / 'build_oneclick_lidar'
LIDAR_INSTALL = WS / 'install_oneclick_lidar'
LIDAR_LOG = WS / 'log_oneclick_lidar'
STATE_HOME = Path(os.environ.get('XDG_STATE_HOME', str(Path.home() / '.local/state'))) / 'competition_oneclick'
STATE_FILE = STATE_HOME / 'state.json'
MANAGE_LOCK = STATE_HOME / 'lifecycle.lock'
TIMEOUT_START = 110
TIMEOUT_STOP = 14


def stamp():
    return datetime.now().astimezone().isoformat(timespec='seconds')


def report(level, message, log_dir=None):
    line = f'[{stamp()}] [{level}] [launcher] {message}'
    print(line, flush=True)
    if log_dir is not None:
        with (log_dir / 'manager.log').open('a', encoding='utf-8') as fp:
            fp.write(line + '\n')


def secure_dir(path):
    path.mkdir(parents=True, exist_ok=True, mode=0o700)
    if not path.is_dir() or path.is_symlink():
        raise RuntimeError('运行目录不是安全的普通目录：' + str(path))
    os.chmod(path, 0o700)


def atomic_json(path, value):
    secure_dir(path.parent)
    tmp = path.with_name(f'.{path.name}.{os.getpid()}.tmp')
    try:
        with tmp.open('x', encoding='utf-8') as fp:
            os.chmod(tmp, 0o600)
            json.dump(value, fp, ensure_ascii=False, indent=2)
            fp.flush()
            os.fsync(fp.fileno())
        os.replace(tmp, path)
    finally:
        if tmp.exists():
            tmp.unlink()


def load_state():
    if not STATE_FILE.exists():
        return None
    try:
        obj = json.loads(STATE_FILE.read_text(encoding='utf-8'))
    except (ValueError, OSError) as exc:
        raise RuntimeError('运行状态文件损坏，请勿自动清理进程：' + str(exc)) from exc
    if not isinstance(obj, dict) or obj.get('project') != str(ROOT) or not isinstance(obj.get('run_id'), str):
        raise RuntimeError('运行状态文件项目归属异常，拒绝操作；请人工核查：' + str(STATE_FILE))
    return obj


def save_state(obj):
    obj = dict(obj)
    obj['updated_at'] = stamp()
    atomic_json(STATE_FILE, obj)


def proc_start(pid):
    """Linux procfs start time (kernel clock ticks since boot), not wall time."""
    try:
        data = Path(f'/proc/{int(pid)}/stat').read_text()
        return int(data.rsplit(')', 1)[1].split()[19])
    except (FileNotFoundError, ProcessLookupError):
        return None


def proc_env(pid):
    try:
        return set(Path(f'/proc/{int(pid)}/environ').read_bytes().split(b'\0'))
    except (FileNotFoundError, ProcessLookupError):
        return None


def identity(pid, start, run_id):
    """alive, dead, or unknown (PID reused / permission denied / identity conflict)."""
    actual = proc_start(pid)
    if actual is None:
        return 'dead'
    if actual != start:
        return 'unknown'
    try:
        if Path(f'/proc/{pid}').stat().st_uid != os.getuid():
            return 'unknown'
        env = proc_env(pid)
        if env is None:
            return 'dead'
        if (f'ONECLICK_RUN_ID={run_id}'.encode() not in env or
                f'ONECLICK_ROOT={ROOT}'.encode() not in env):
            return 'unknown'
    except (OSError, PermissionError):
        return 'unknown'
    return 'alive'


def owned_processes(run_id):
    """Find descendants even if supervisor died. Never match by process name alone."""
    found = []
    for proc in Path('/proc').iterdir():
        if not proc.name.isdigit():
            continue
        pid = int(proc.name)
        try:
            if proc.stat().st_uid != os.getuid():
                continue
            env = proc_env(pid)
            if env is None:
                continue
            if (f'ONECLICK_RUN_ID={run_id}'.encode() in env and
                    f'ONECLICK_ROOT={ROOT}'.encode() in env):
                start = proc_start(pid)
                if start is not None:
                    found.append((pid, start))
        except (OSError, PermissionError):
            continue
    return found


def is_zombie(pid):
    try:
        return Path(f'/proc/{pid}/stat').read_text().rsplit(')',1)[1].strip().startswith('Z ')
    except OSError:
        return False


def signal_owned(pid, ticks, run_id):
    if identity(pid, ticks, run_id) != 'alive':
        return False
    try:
        os.kill(pid, signal.SIGTERM)
    except ProcessLookupError:
        return False
    return True


def current_status(obj):
    if obj is None:
        return 'STOPPED', '尚未找到管理记录'
    p = obj.get('supervisor_pid')
    tick = obj.get('supervisor_start_ticks')
    if not isinstance(p, int) or not isinstance(tick, int):
        return obj.get('state', 'UNKNOWN'), '无受管进程记录'
    found = identity(p, tick, obj['run_id'])
    if found == 'unknown':
        return 'UNKNOWN', 'PID 身份不匹配或无法验证；不会向该 PID 发信号'
    if found == 'dead' or is_zombie(p):
        children = [(pid, ticks) for pid, ticks in owned_processes(obj['run_id']) if not is_zombie(pid)]
        if children:
            return 'FAILED', f'管理进程已退出，仍有 {len(children)} 个本项目标记进程存活'
        if obj.get('state') in ('STOPPED', 'FAILED'):
            return obj['state'], '进程已退出'
        result_path = Path(obj['log_dir']) / 'run_result.json'
        if result_path.is_file():
            try:
                result = json.loads(result_path.read_text(encoding='utf-8'))
                if result.get('run_id') == obj['run_id'] and result.get('result') == 0:
                    return 'STOPPED', '管理进程正常退出（例如关闭 RViz）'
            except (OSError, ValueError):
                pass
        return 'FAILED', '管理进程意外退出（没有完整的停止记录）'
    if obj.get('state') == 'STOPPING':
        return 'STOPPING', '正在停止'
    if obj.get('state') == 'STARTING':
        return 'STARTING', '各模块正在等待就绪'
    if obj.get('state') in ('RUNNING', 'DEGRADED'):
        mission = Path(obj['log_dir']) / 'mission_locked.json'
        if mission.is_file():
            try:
                if json.loads(mission.read_text(encoding='utf-8')).get('safety_state') == 'CONFLICT_STOP':
                    return 'FAILED', '新增障碍物冲突，路径预览已失效'
            except (OSError, ValueError):
                return 'UNKNOWN', '锁定路线文件无法解析'
        registry = Path(obj['log_dir']) / 'children.json'
        if registry.exists():
            try:
                children = json.loads(registry.read_text(encoding='utf-8'))
                for child in children:
                    if child.get('required', True) and identity(child['pid'], child['ticks'], obj['run_id']) != 'alive':
                        return 'FAILED', f"必需模块进程异常退出：{child.get('name', 'unknown')}"
            except (KeyError, TypeError, ValueError, OSError):
                return 'UNKNOWN', '子进程身份记录损坏'
        else:
            return 'UNKNOWN', '未找到模块进程登记文件'
        return 'RUNNING', '已检查管理进程与已登记的必需模块进程；ROS 实时数据请另行实测'
    return obj.get('state', 'UNKNOWN'), '状态记录未完成'


def manager_lock():
    secure_dir(STATE_HOME)
    fp = MANAGE_LOCK.open('a+')
    os.chmod(MANAGE_LOCK, 0o600)
    try:
        fcntl.flock(fp.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        fp.close()
        raise RuntimeError('另一个 start/stop/restart 正在操作，请勿并发启动')
    return fp


def tail(path, size=25):
    if not path.is_file():
        return ''
    return '\n'.join(path.read_text(encoding='utf-8', errors='replace').splitlines()[-size:])


def _legacy_lidar_overlay_valid():
    """Use the old install_x3 overlay only when its driver executable really resolves."""
    node = WS / 'install_x3' / LIDAR_PACKAGE / 'lib' / LIDAR_PACKAGE / (LIDAR_PACKAGE + '_node')
    try:
        resolved = node.resolve(strict=True)
        return resolved.is_file() and os.access(resolved, os.X_OK)
    except OSError:
        return False


def ros_environment_prefix():
    """Build a safe Bash prefix; ROS setup files are sourced with nounset disabled."""
    ros_setup = Path('/opt/ros/humble/setup.bash')
    steps = ['set -e', 'set +u', f'source {shlex_quote(str(ros_setup))}']
    base_overlay = WS / 'install' / 'setup.bash'
    if base_overlay.is_file():
        steps.append(f'source {shlex_quote(str(base_overlay))}')
    lidar_overlay = LIDAR_INSTALL / 'setup.bash'
    if lidar_overlay.is_file():
        steps.append(f'source {shlex_quote(str(lidar_overlay))}')
    elif _legacy_lidar_overlay_valid():
        legacy = WS / 'install_x3' / 'setup.bash'
        if legacy.is_file():
            steps.append(f'source {shlex_quote(str(legacy))}')
    steps.append('set -u')
    return '; '.join(steps)


def ros_shell(command, timeout=12):
    script = ros_environment_prefix() + '; ' + command
    return subprocess.run(['bash', '-c', script], text=True,
                          stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                          timeout=timeout, check=False)


def ros_package_prefix(package):
    result = ros_shell('ros2 pkg prefix ' + shlex_quote(package), timeout=12)
    if result.returncode != 0:
        return None, result.stdout.strip()
    prefix = result.stdout.strip().splitlines()[-1] if result.stdout.strip() else ''
    return (prefix or None), result.stdout.strip()


def lidar_executable(prefix):
    if not prefix:
        return None
    node = Path(prefix) / 'lib' / LIDAR_PACKAGE / (LIDAR_PACKAGE + '_node')
    try:
        resolved = node.resolve(strict=True)
    except OSError:
        return None
    return resolved if resolved.is_file() and os.access(resolved, os.X_OK) else None


def find_ydlidar_sdk_config():
    candidates = []
    env_dir = os.environ.get('ydlidar_sdk_DIR')
    if env_dir:
        candidates.append(Path(env_dir).expanduser())
    candidates.extend([
        Path('/usr/local/lib/cmake/ydlidar_sdk'),
        Path('/usr/lib/x86_64-linux-gnu/cmake/ydlidar_sdk'),
        Path('/usr/lib/cmake/ydlidar_sdk'),
    ])
    for directory in candidates:
        config_file = directory / 'ydlidar_sdkConfig.cmake'
        if config_file.is_file():
            return directory.resolve()
    return None


def lidar_build_prerequisites():
    """Return (ready, details); never installs system packages or edits global state."""
    package_xml = LIDAR_SOURCE / 'package.xml'
    if not package_xml.is_file():
        return False, '缺少驱动源码：' + str(package_xml)
    try:
        package_name = ET.parse(package_xml).getroot().findtext('name')
    except (ET.ParseError, OSError) as exc:
        return False, '驱动 package.xml 无法解析：' + str(exc)
    if package_name != LIDAR_PACKAGE:
        return False, f'驱动源码包名不匹配：期望 {LIDAR_PACKAGE}，实际 {package_name!r}'
    sdk_dir = find_ydlidar_sdk_config()
    if sdk_dir is None:
        return False, '缺少 YDLIDAR SDK CMake 配置：未找到 /usr/local/lib/cmake/ydlidar_sdk/ydlidar_sdkConfig.cmake；不会自动安装系统级 SDK'
    try:
        result = ros_shell('command -v colcon', timeout=10)
    except subprocess.TimeoutExpired:
        return False, '检查 colcon 超时'
    if result.returncode != 0 or not result.stdout.strip():
        return False, 'ROS 环境加载后找不到 colcon；请先安装/配置 ROS 2 colcon 构建工具'
    colcon_path = result.stdout.strip().splitlines()[-1]
    return True, f'源码={LIDAR_SOURCE}；SDK={sdk_dir}；colcon={colcon_path}'


def _check_build_directory():
    source = LIDAR_SOURCE.resolve()
    cache = LIDAR_BUILD / LIDAR_PACKAGE / 'CMakeCache.txt'
    if LIDAR_BUILD.exists():
        if not LIDAR_BUILD.is_dir() or LIDAR_BUILD.is_symlink():
            raise RuntimeError('YDLIDAR 构建目录不是普通目录，拒绝覆盖：' + str(LIDAR_BUILD))
        if cache.is_file():
            home = None
            for line in cache.read_text(encoding='utf-8', errors='replace').splitlines():
                if line.startswith('CMAKE_HOME_DIRECTORY:INTERNAL='):
                    home = line.split('=', 1)[1]
                    break
            if home and Path(home).resolve() != source:
                raise RuntimeError('检测到旧路径生成的 YDLIDAR CMake 缓存，拒绝自动清理：' + str(cache) + '；请人工确认后处理')
        elif any(LIDAR_BUILD.iterdir()):
            raise RuntimeError('YDLIDAR 构建目录已有未知内容且没有可验证的 CMakeCache，拒绝覆盖：' + str(LIDAR_BUILD))
    if LIDAR_INSTALL.exists():
        if not LIDAR_INSTALL.is_dir() or LIDAR_INSTALL.is_symlink():
            raise RuntimeError('YDLIDAR 安装目录不是普通目录，拒绝覆盖：' + str(LIDAR_INSTALL))
        if any(LIDAR_INSTALL.iterdir()) and not (LIDAR_INSTALL / 'setup.bash').is_file():
            raise RuntimeError('YDLIDAR 安装目录已有未知内容，拒绝覆盖：' + str(LIDAR_INSTALL))


def ensure_lidar_driver():
    """Build only the missing ROS driver into a dedicated workspace overlay."""
    if not Path('/opt/ros/humble/setup.bash').is_file():
        raise RuntimeError('未找到 ROS 2 Humble 环境：/opt/ros/humble/setup.bash')
    prefix, detail = ros_package_prefix(LIDAR_PACKAGE)
    if prefix:
        executable = lidar_executable(prefix)
        if executable is None:
            raise RuntimeError(f'ROS 已注册 {LIDAR_PACKAGE}，但驱动可执行文件不存在或不可执行：{Path(prefix) / "lib" / LIDAR_PACKAGE / (LIDAR_PACKAGE + "_node")}')
        report('PASS', f'YDLIDAR 驱动已安装：prefix={prefix} executable={executable}')
        return prefix

    ready, reason = lidar_build_prerequisites()
    if not ready:
        raise RuntimeError('YDLIDAR 驱动未安装，且无法自动构建：' + reason)
    report('WARN', 'ROS 环境未找到 ydlidar_ros2_driver；检测到源码和 SDK，开始项目内首次构建')
    report('BUILD', reason)
    _check_build_directory()
    STATE_HOME.mkdir(parents=True, exist_ok=True, mode=0o700)
    build_log_dir = STATE_HOME / 'logs'
    secure_dir(build_log_dir)
    build_log = build_log_dir / ('driver_build_' + datetime.now().strftime('%Y%m%d_%H%M%S_%f') + '.log')
    report('BUILD_LOG', '构建完整输出将保存到：' + str(build_log))
    colcon_result = ros_shell('command -v colcon', timeout=10)
    if colcon_result.returncode != 0 or not colcon_result.stdout.strip():
        raise RuntimeError('YDLIDAR 构建前复检失败：ROS 环境中 colcon 不可用')
    colcon_path = colcon_result.stdout.strip().splitlines()[-1]
    sdk_dir = find_ydlidar_sdk_config()
    if sdk_dir is None:
        raise RuntimeError('YDLIDAR SDK CMake 配置在构建前消失；停止构建')
    build_command = (
        'cd ' + shlex_quote(str(WS)) + ' && ' + shlex_quote(colcon_path) + ' --log-base ' + shlex_quote(str(LIDAR_LOG)) + ' build' +
        ' --base-paths ' + shlex_quote(str(LIDAR_SOURCE)) +
        ' --build-base ' + shlex_quote(str(LIDAR_BUILD)) +
        ' --install-base ' + shlex_quote(str(LIDAR_INSTALL)) +
        ' --packages-select ' + LIDAR_PACKAGE +
        ' --symlink-install --cmake-args -Dydlidar_sdk_DIR=' + shlex_quote(str(sdk_dir))
    )
    build_script = ros_environment_prefix() + '; ' + build_command
    timed_out = False
    with build_log.open('w', encoding='utf-8') as log_fp:
        os.chmod(build_log, 0o600)
        proc = subprocess.Popen(['bash', '-c', build_script], cwd=str(WS),
                                stdout=log_fp, stderr=subprocess.STDOUT,
                                start_new_session=True)
        try:
            return_code = proc.wait(timeout=900)
        except subprocess.TimeoutExpired:
            timed_out = True
            try:
                os.killpg(proc.pid, signal.SIGTERM)
            except ProcessLookupError:
                pass
            try:
                proc.wait(timeout=8)
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(proc.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
                proc.wait(timeout=3)
            return_code = proc.returncode
    build_output = build_log.read_text(encoding='utf-8', errors='replace')
    if timed_out:
        raise RuntimeError(f'YDLIDAR 驱动构建超时（900秒），本次构建进程组已终止。日志={build_log}\n末尾输出：\n' + '\n'.join(build_output.splitlines()[-35:]))
    if return_code != 0:
        tail_text = '\n'.join(build_output.splitlines()[-35:])
        raise RuntimeError(f'YDLIDAR 驱动构建失败，退出码={return_code}\n构建日志={build_log}\n末尾输出：\n{tail_text}')
    if not (LIDAR_INSTALL / 'setup.bash').is_file():
        raise RuntimeError('colcon 返回成功，但没有生成安装环境：' + str(LIDAR_INSTALL / 'setup.bash') + '\n日志=' + str(build_log))
    prefix, detail = ros_package_prefix(LIDAR_PACKAGE)
    executable = lidar_executable(prefix)
    if not prefix or executable is None:
        raise RuntimeError('构建命令返回成功，但 ROS 仍无法发现有效驱动可执行文件。检查日志：' + str(build_log) + '\n' + detail)
    report('BUILD_PASS', f'YDLIDAR 驱动构建并验证成功：prefix={prefix} executable={executable} log={build_log}')
    return prefix


def diagnose(live=True, allow_buildable_driver=False):
    failures = []
    report('INFO', f'工程={ROOT} Python={platform.python_version()} 内核={platform.release()} 用户UID={os.getuid()}')
    if not sys.platform.startswith('linux'):
        failures.append('仅支持 Linux/Ubuntu')
    if sys.version_info < (3, 10):
        failures.append('Python 需要 >= 3.10')
    try:
        settings, port, frame, _ = pipeline.check_inputs()
        report('PASS', f'工程文件/配置可读取；起点={settings["start_zone"]} 雷达帧={frame}')
    except (ValueError, OSError, RuntimeError) as exc:
        failures.append('工程文件或配置：' + str(exc))
        port = None
    if not live:
        report('WARN', '离线模式：跳过 ROS、USB 串口及 TF 就绪检查，不代表设备可用')
        for msg in failures:
            report('ERROR', msg)
        return 0 if not failures else 2

    ros_setup = Path('/opt/ros/humble/setup.bash')
    if not ros_setup.is_file():
        failures.append('未找到 ROS 2 Humble：' + str(ros_setup))
    else:
        for package in ('nav2_map_server', 'tf2_ros'):
            try:
                prefix, detail = ros_package_prefix(package)
            except subprocess.TimeoutExpired:
                prefix, detail = None, '检查超时'
            if prefix:
                report('PASS', f'ROS 包 {package}：{prefix}')
            else:
                failures.append(f'ROS 包缺失：{package}；ros2 pkg prefix 输出={detail or "无输出"}')

        try:
            prefix, detail = ros_package_prefix(LIDAR_PACKAGE)
        except subprocess.TimeoutExpired:
            prefix, detail = None, '检查超时'
        if prefix:
            executable = lidar_executable(prefix)
            if executable:
                report('PASS', f'ROS 包 {LIDAR_PACKAGE}：prefix={prefix} executable={executable}')
            else:
                failures.append(f'ROS 已注册 {LIDAR_PACKAGE}，但驱动节点不存在或不可执行：{Path(prefix) / "lib" / LIDAR_PACKAGE / (LIDAR_PACKAGE + "_node")}')
        else:
            ready, reason = lidar_build_prerequisites()
            if ready:
                msg = (f'{LIDAR_PACKAGE} 尚未构建/注册；源码、SDK、colcon 均可用。'
                       f'构建目标：{LIDAR_INSTALL}。执行 start 时会自动构建；也可先执行 ./scripts/ai.sh build-lidar。')
                if allow_buildable_driver:
                    report('WARN', msg)
                else:
                    failures.append(msg)
            else:
                failures.append(f'ROS 包缺失：{LIDAR_PACKAGE}；ros2 输出={detail or "无输出"}；自动构建条件不满足：{reason}')

    if port:
        dev = Path(port)
        if not dev.exists():
            failures.append('雷达设备未连接：' + port)
        elif not os.access(dev, os.R_OK | os.W_OK):
            failures.append('雷达设备没有当前用户读写权限：' + port)
        else:
            tool = shutil.which('fuser') or shutil.which('lsof')
            if tool is None:
                failures.append('缺少 fuser/lsof，无法确认雷达串口是否被占用；未自动关闭任何进程')
            else:
                cmd = ['fuser', port] if Path(tool).name == 'fuser' else ['lsof', '-t', '--', port]
                result = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                                        text=True, timeout=8)
                if result.returncode == 0 and result.stdout.strip():
                    failures.append('雷达串口被占用：' + port + ' 占用PID=' + result.stdout.strip())
                elif result.returncode in (0, 1):
                    report('PASS', '雷达串口可访问、未检测到已知占用：' + port)
                else:
                    failures.append('串口占用检查工具返回异常：' + result.stderr[-300:])
    for msg in failures:
        report('ERROR', msg)
    if failures:
        report('ERROR', 'doctor 未通过：未修改系统、未启动电机；请先处理上述必需条件')
        return 2
    report('PASS', '启动前检查通过；/scan、/map 和 TF 会在服务启动后另行验证')
    return 0

def shlex_quote(s):
    import shlex
    return shlex.quote(s)


def _stop_under_lock(obj, reason='用户请求停止'):
    if obj is None:
        report('INFO', '目前没有受管实例，无须停止')
        return 0
    run_id = obj['run_id']
    pid = obj.get('supervisor_pid')
    ticks = obj.get('supervisor_start_ticks')
    if not isinstance(pid, int) or not isinstance(ticks, int):
        save_state({**obj, 'state': 'STOPPED'})
        return 0
    check = identity(pid, ticks, run_id)
    if check == 'unknown':
        report('ERROR', f'PID={pid} 身份不匹配；安全起见拒绝发送信号')
        save_state({**obj, 'state': 'UNKNOWN'})
        return 3
    save_state({**obj, 'state': 'STOPPING'})
    report('INFO', f'{reason}；只尝试停止可验证归属的项目进程', Path(obj['log_dir']))
    if check == 'alive':
        signal_owned(pid, ticks, run_id)
    deadline = time.monotonic() + TIMEOUT_STOP
    while time.monotonic() < deadline:
        if identity(pid, ticks, run_id) != 'alive' or is_zombie(pid):
            break
        time.sleep(.12)
    # Supervisor may have died abruptly; only process IDs carrying exact run identity may be stopped.
    leftover = [(p,t) for p,t in owned_processes(run_id) if p != pid and not is_zombie(p)]
    for child_pid, child_ticks in leftover:
        signal_owned(child_pid, child_ticks, run_id)
    deadline = time.monotonic() + 4
    while time.monotonic() < deadline:
        remaining = [(p,t) for p,t in owned_processes(run_id) if not is_zombie(p)]
        if not remaining:
            break
        time.sleep(.12)
    else:
        remaining = [(p,t) for p,t in owned_processes(run_id) if not is_zombie(p)]
    if remaining:
        save_state({**obj, 'state': 'FAILED'})
        report('ERROR', '部分本项目进程未正常退出（不会强制杀死）：' + str([p for p,_ in remaining]), Path(obj['log_dir']))
        return 4
    save_state({**obj, 'state': 'STOPPED', 'supervisor_pid': None, 'supervisor_start_ticks': None})
    report('SUCCESS', '本次实例已停止，日志和锁定结果均保留', Path(obj['log_dir']))
    return 0


def _start_under_lock(skip_doctor=False):
    old = load_state()
    status, detail = current_status(old)
    if status == 'RUNNING' or status == 'STARTING':
        report('INFO', f'已有运行实例：{status}；不重复启动、不重新扫描或清空本场封路')
        return 0
    if status == 'UNKNOWN':
        report('ERROR', f'旧实例归属不能验证：{detail}；拒绝覆盖/清理')
        return 3
    if old and old.get('supervisor_pid'):
        if _stop_under_lock(old, '恢复旧的已确认项目实例') != 0:
            return 3
    if not skip_doctor:
        if diagnose(live=True, allow_buildable_driver=True) != 0:
            report('ERROR', '启动前基础环境诊断失败，未创建项目进程')
            return 2
        try:
            ensure_lidar_driver()
        except (RuntimeError, OSError, subprocess.SubprocessError) as exc:
            report('ERROR', 'YDLIDAR 驱动准备失败：' + str(exc))
            return 2
        if diagnose(live=True, allow_buildable_driver=False) != 0:
            report('ERROR', '驱动构建后完整环境复检未通过，未创建项目进程')
            return 2
    now = datetime.now().strftime('%Y%m%d_%H%M%S')
    run_id = now + '_' + secrets.token_hex(6)
    log_dir = STATE_HOME / 'logs' / run_id
    secure_dir(log_dir)
    supervisor_log = log_dir / 'supervisor.log'
    launcher = HERE / 'start_oneclick.sh'
    if not launcher.is_file():
        raise RuntimeError('缺少一键启动包装器：' + str(launcher))
    env = os.environ.copy()
    env.update({'AI_SUPERVISOR_CHILD':'1', 'ONECLICK_RUN_ID':run_id,
                'ONECLICK_ROOT':str(ROOT), 'ONECLICK_LOG_DIR':str(log_dir)})
    report('INFO', '新实例开始启动；日志=' + str(log_dir), log_dir)
    with supervisor_log.open('w', encoding='utf-8') as fp:
        os.chmod(supervisor_log, 0o600)
        proc = subprocess.Popen(['bash', str(launcher)], cwd=str(ROOT), env=env,
                                stdout=fp, stderr=subprocess.STDOUT, start_new_session=True)
    ticks = proc_start(proc.pid)
    if ticks is None:
        report('ERROR', '管理进程启动后立即退出：' + tail(supervisor_log), log_dir)
        return 2
    obj = {'version':2, 'project':str(ROOT), 'run_id':run_id,
           'state':'STARTING', 'supervisor_pid':proc.pid,
           'supervisor_start_ticks':ticks, 'log_dir':str(log_dir),
           'created_at':stamp()}
    save_state(obj)
    deadline = time.monotonic() + TIMEOUT_START
    while time.monotonic() < deadline:
        ident = identity(proc.pid, ticks, run_id)
        if ident == 'unknown':
            report('ERROR', '运行管理进程身份发生变化，拒绝发出信号；请人工检查', log_dir)
            save_state({**obj, 'state':'UNKNOWN'})
            return 3
        if proc.poll() is not None:
            snippet = tail(supervisor_log, 18)
            report('ERROR', '启动阶段=模块初始化；管理进程提前退出，代码=' + str(proc.returncode)
                   + '\n' + snippet, log_dir)
            _stop_under_lock(obj, '启动失败清理')
            save_state({**obj, 'state':'FAILED', 'supervisor_pid':None,
                        'supervisor_start_ticks':None})
            return 2
        # A READY marker is only written after main.py actually checked ROS topic and module readiness.
        if '[RUNNING]' in tail(supervisor_log, 35):
            obj['state'] = 'RUNNING'
            save_state(obj)
            report('SUCCESS', '感知与道路路线预览模块启动就绪（仍不控制电机）', log_dir)
            return 0
        time.sleep(.25)
    report('ERROR', f'启动超时（{TIMEOUT_START}s），最近日志：\n' + tail(supervisor_log, 18), log_dir)
    _stop_under_lock(obj, '启动超时回滚')
    save_state({**obj, 'state':'FAILED', 'supervisor_pid':None,
                'supervisor_start_ticks':None})
    return 2



def cmd_build_lidar():
    with manager_lock():
        try:
            state, detail = current_status(load_state())
            if state in ('RUNNING', 'STARTING', 'STOPPING'):
                raise RuntimeError('项目运行中不允许更新驱动安装覆盖层；请先执行 ./scripts/ai.sh stop')
            ensure_lidar_driver()
            report('SUCCESS', 'YDLIDAR ROS 2 驱动已就绪；可继续执行 doctor 或 start')
            return 0
        except (RuntimeError, OSError, subprocess.SubprocessError) as exc:
            report('ERROR', 'YDLIDAR 驱动构建/检查失败：' + str(exc))
            return 2

def cmd_start():
    with manager_lock():
        return _start_under_lock()


def cmd_stop():
    with manager_lock():
        return _stop_under_lock(load_state())


def cmd_restart():
    with manager_lock():
        if _stop_under_lock(load_state(), '受控重启') != 0:
            return 3
        return _start_under_lock()


def cmd_status():
    obj = load_state()
    state, reason = current_status(obj)
    report('STATUS', state + ' | ' + reason)
    if obj:
        report('INFO', '运行日志：' + str(obj.get('log_dir', 'UNKNOWN')))
        report('INFO', '本次实例：' + obj['run_id'])
    return 0 if state in ('RUNNING', 'STOPPED', 'DEGRADED', 'STARTING') else 2


def cmd_logs(module, count, follow):
    if not re.fullmatch(r'[a-z_]+', module):
        raise RuntimeError('无效模块名称（只能使用英文字母及下划线）')
    if module == 'driver_build':
        candidates = sorted((STATE_HOME / 'logs').glob('driver_build_*.log'))
        if not candidates:
            report('ERROR', '尚无 YDLIDAR 构建日志；先执行 build-lidar 或 start')
            return 2
        path = candidates[-1]
    else:
        obj = load_state()
        if obj is None:
            report('ERROR', '尚无运行日志，先运行 start 或 doctor')
            return 2
        available = {'supervisor','manager','map','mask','info','lidar','map_base_tf',
                     'base_laser_tf','lock','topology','mission_export','rviz'}
        if module not in available:
            raise RuntimeError('未知模块：' + module)
        path = Path(obj['log_dir']) / (module + '.log')
    if not path.exists():
        report('ERROR', '该模块暂未产生日志：' + str(path))
        return 2
    print('[LOG] ' + str(path))
    print(tail(path, count))
    if follow:
        try:
            subprocess.run(['tail', '-n', '0', '-f', str(path)], check=False)
        except KeyboardInterrupt:
            pass
    return 0


def main():
    os.umask(0o077)
    parser = argparse.ArgumentParser(description='工创小车受控一键启动（只做静止雷达与道路规划预览）')
    parser.add_argument('command', choices=['start','stop','restart','status','doctor','logs','build-lidar'])
    parser.add_argument('--offline', action='store_true', help='doctor 离线文件检查，不连接 ROS/雷达')
    parser.add_argument('--module', default='supervisor', help='logs 的模块名')
    parser.add_argument('--lines', type=int, default=35, help='logs 最近行数')
    parser.add_argument('--follow', action='store_true', help='logs 持续查看')
    args = parser.parse_args()
    try:
        if args.command == 'doctor':
            return diagnose(live=not args.offline)
        if args.command == 'start':
            return cmd_start()
        if args.command == 'stop':
            return cmd_stop()
        if args.command == 'restart':
            return cmd_restart()
        if args.command == 'status':
            return cmd_status()
        if args.command == 'logs':
            return cmd_logs(args.module, max(1,min(args.lines,500)), args.follow)
        if args.command == 'build-lidar':
            return cmd_build_lidar()
    except (RuntimeError, OSError, subprocess.SubprocessError, ValueError) as exc:
        report('ERROR', str(exc))
        return 2
    return 2


if __name__ == '__main__':
    sys.exit(main())
