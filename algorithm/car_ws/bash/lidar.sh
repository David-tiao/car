#!/usr/bin/env bash
# 单独调试 YDLIDAR X3：只运行雷达驱动并发布 /scan；不启动小车控制。
# 用法：bash lidar.sh doctor   仅检查环境、驱动、串口，不启动雷达
#       bash lidar.sh start    前台启动雷达，Ctrl+C/关闭终端可结束
# 注意：当 V6 一键程序已运行时，不要同时执行本脚本抢占 /dev/ttyUSB0。
set -Eeuo pipefail
source "$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)/lib/common.sh"
car_check_workspace
WS="$CAR_WS"
CFG="${CAR_LIDAR_CONFIG:-$WS/config/x3.yaml}"
MODE="${1:-start}"
if [[ "$MODE" != start && "$MODE" != doctor ]]; then
  echo "[ERROR] 用法: bash lidar.sh [start|doctor]" >&2; exit 2
fi
if [[ ! -r "$CAR_ROS_SETUP" || ! -r "$CFG" ]]; then
  echo "[ERROR] ROS Humble 或雷达配置不存在：$CFG" >&2; exit 1
fi
# ROS 的 setup.bash 可能访问未定义 AMENT_* 环境变量，加载时临时关闭 -u。
car_source_ros_base
[[ ! -r "$WS/install/setup.bash" ]] || car_source_setup "$WS/install/setup.bash"
[[ ! -r "$WS/install_oneclick_lidar/setup.bash" ]] || car_source_setup "$WS/install_oneclick_lidar/setup.bash"
if ! ros2 pkg prefix ydlidar_ros2_driver >/dev/null 2>&1; then
  echo '[ERROR] 当前 car_ws 没有注册 ydlidar_ros2_driver；先检查/编译驱动，不能把源码存在当成编译成功。' >&2
  exit 1
fi
if ! ros2 pkg executables ydlidar_ros2_driver | grep -Eq '^ydlidar_ros2_driver[[:space:]]+ydlidar_ros2_driver_node$'; then
  echo '[ERROR] 驱动包存在，但缺少可运行的 ydlidar_ros2_driver_node。' >&2; exit 1
fi
PREFIX="$(ros2 pkg prefix ydlidar_ros2_driver)"
[[ -x "$PREFIX/lib/ydlidar_ros2_driver/ydlidar_ros2_driver_node" ]] || {
  echo "[ERROR] 雷达可执行文件缺失或旧符号链接失效：$PREFIX" >&2; exit 1;
}
# 仅从实际 YAML 读取串口路径，不写死 ttyUSB0；拒绝非法路径。
PORT="$(/usr/bin/python3 - "$CFG" <<'PYPORT'
import pathlib,re,sys
s=pathlib.Path(sys.argv[1]).read_text(encoding='utf-8')
m=re.search(r'^\s*port:\s*[\"\']?(/dev/[A-Za-z0-9_./-]+)',s,re.M)
if not m: raise SystemExit('x3.yaml 中的 port 无效')
print(m.group(1))
PYPORT
)"
if [[ ! -r "$PORT" || ! -w "$PORT" ]]; then
  echo "[ERROR] 雷达串口不可读写：$PORT" >&2; exit 1
fi
if command -v fuser >/dev/null 2>&1 && fuser "$PORT" >/dev/null 2>&1; then
  echo "[ERROR] 串口 $PORT 已被占用；不会结束其他进程。" >&2; exit 1
fi
if [[ "$MODE" == doctor ]]; then
  echo "[PASS] 驱动已注册；串口 $PORT 可访问且未检测到占用；未启动雷达。"
  exit 0
fi
echo "[INFO] 启动真实雷达，参数文件=$CFG；结束请按 Ctrl+C。"
# 使用 exec 保持前台进程生命周期；无自动后台进程，也不控制电机。
exec ros2 run ydlidar_ros2_driver ydlidar_ros2_driver_node --ros-args --params-file "$CFG"
