#!/usr/bin/env bash
# 单独调试 YDLIDAR X3：只运行雷达驱动并发布 /scan；不启动小车控制。
# 用法：bash lidar.sh doctor   仅检查环境、驱动、串口，不启动雷达
#       bash lidar.sh start    前台启动雷达，Ctrl+C/关闭终端可结束
#       bash lidar.sh regions       复用现有 /scan + 实测 TF，启动只读区域观察
#       bash lidar.sh start-regions 启动雷达与区域观察（要求已发布实测 TF）
#       bash lidar.sh test-regions  不依赖 ROS/硬件的区域扫描回归测试
#       bash lidar.sh startup            开局复用已有 /scan，静止全场定位快照
#       bash lidar.sh start-startup      单命令雷达驱动+静止全场定位快照
#       bash lidar.sh capture FILE [--frames N]  采集现有 /scan 至本地文件（无 TF）
#       bash lidar.sh calibrate FILE --front-m D --left-m D --front-deg A --left-deg A  离线外参计算
# 注意：当 V6 一键程序已运行时，只使用 regions，禁止重复启动串口。
set -Eeuo pipefail
source "$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)/lib/common.sh"
car_check_workspace
WS="$CAR_WS"
CFG="${CAR_LIDAR_CONFIG:-$WS/config/x3.yaml}"
MODE="${1:-start}"
REGION="$WS/maps/competition_v1/lidar_stage5/scan_regions.py"
case "$MODE" in
  start|doctor|regions|start-regions|test-regions|startup|start-startup|capture|calibrate) ;;
  *) echo "[ERROR] 用法: bash lidar.sh [start|doctor|regions|start-regions|test-regions|capture|calibrate]" >&2; exit 2 ;;
esac
if [[ "$MODE" == calibrate ]]; then
  shift
  exec /usr/bin/python3 "$REGION" calibrate "$@"
fi
if [[ "$MODE" == test-regions ]]; then
  exec /usr/bin/python3 "$REGION" --self-test
fi
if [[ ! -r "$CAR_ROS_SETUP" || ( "$MODE" != regions && "$MODE" != capture && ! -r "$CFG" ) ]]; then
  echo "[ERROR] ROS Humble 或雷达配置不存在：$CFG" >&2; exit 1
fi
# ROS 的 setup.bash 可能访问未定义 AMENT_* 环境变量，加载时临时关闭 -u。
car_source_ros_base
if [[ "$MODE" == capture ]]; then
  shift
  exec /usr/bin/python3 "$REGION" capture "$@"
fi
[[ ! -r "$WS/install/setup.bash" ]] || car_source_setup "$WS/install/setup.bash"
[[ ! -r "$WS/install_oneclick_lidar/setup.bash" ]] || car_source_setup "$WS/install_oneclick_lidar/setup.bash"
if [[ "$MODE" == regions || "$MODE" == start-regions || "$MODE" == startup || "$MODE" == start-startup ]]; then
  [[ -r "$REGION" ]] || { echo "[ERROR] 缺少区域扫描源码：$REGION" >&2; exit 1; }
  command -v flock >/dev/null || { echo '[ERROR] 缺少 flock，无法防止重复启动' >&2; exit 1; }
  STATE="${XDG_STATE_HOME:-$HOME/.local/state}/car-lidar"
  mkdir -p -- "$STATE"
  exec 9>"$STATE/region.lock"
  flock -n 9 || { echo '[ERROR] 已有本用户区域扫描实例；请关闭原终端' >&2; exit 1; }
  # ROS 图检查只读，不能自动结束其他节点。发现窗口有限，不代表全局排他。
  /usr/bin/python3 - "$MODE" <<'PYGRAPH'
import sys, time
import rclpy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener
rclpy.init(args=[])
node = rclpy.create_node('car_region_precheck')
try:
    deadline = time.monotonic()+2.0
    while time.monotonic()<deadline:
        rclpy.spin_once(node, timeout_sec=.1)
    if node.count_publishers('/competition/lidar/regions'):
        raise SystemExit('[ERROR] 区域话题已有发布者；请检查当前 ROS_DOMAIN_ID')
    count = node.count_publishers('/scan')
    expected = 1 if sys.argv[1] in ('regions','startup') else 0
    if count != expected:
        raise SystemExit(f'[ERROR] /scan 发布者数量 {count}，期望 {expected}；不要重复启动雷达')
finally:
    node.destroy_node()
    rclpy.shutdown()
PYGRAPH
  if [[ "$MODE" == regions || "$MODE" == startup ]]; then
    echo '[INFO] 复用已有 /scan，静止且需有真实 map->laser_frame TF。'
    if [[ "$MODE" == startup ]]; then
      exec env CAR_REGION_STARTUP_TIMEOUT=12 /usr/bin/python3 "$REGION" startup
    fi
    exec env CAR_REGION_STARTUP_TIMEOUT=12 /usr/bin/python3 "$REGION"
  fi
fi
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
if [[ "$MODE" == start-regions || "$MODE" == start-startup ]] && ! command -v fuser >/dev/null; then
  echo '[ERROR] 缺少 fuser，无法检查串口冲突' >&2; exit 1
fi
if command -v fuser >/dev/null 2>&1 && fuser "$PORT" >/dev/null 2>&1; then
  echo "[ERROR] 串口 $PORT 已被占用；不会结束其他进程。" >&2; exit 1
fi
if [[ "$MODE" == doctor ]]; then
  echo "[PASS] 驱动已注册；串口 $PORT 可访问且未检测到占用；未启动雷达。"
  exit 0
fi
if [[ "$MODE" == start-regions || "$MODE" == start-startup ]]; then
  LOG="$(mktemp -d "$STATE/region.XXXXXXXX")"
  # 只清理本 shell 的直接子进程，不按名称或串口杀其他程序。
  cleanup_regions() {
    local pid i
    trap - EXIT INT TERM
    for pid in $(jobs -pr); do kill -TERM "$pid" 2>/dev/null || true; done
    for ((i=0;i<30;i++)); do
      [[ -n "$(jobs -pr)" ]] || break
      sleep .1
    done
    for pid in $(jobs -pr); do kill -KILL "$pid" 2>/dev/null || true; done
    wait || true
  }
  trap cleanup_regions EXIT
  trap 'exit 130' INT
  trap 'exit 143' TERM
  echo "[INFO] 联合启动；日志：$LOG；请保持车体静止。"
  "$PREFIX/lib/ydlidar_ros2_driver/ydlidar_ros2_driver_node" --ros-args --params-file "$CFG" >"$LOG/driver.log" 2>&1 &
  driver_pid=$!
  if [[ "$MODE" == start-startup ]]; then
    CAR_REGION_STARTUP_TIMEOUT=12 /usr/bin/python3 "$REGION" startup >"$LOG/regions.log" 2>&1 &
  else
    CAR_REGION_STARTUP_TIMEOUT=12 /usr/bin/python3 "$REGION" >"$LOG/regions.log" 2>&1 &
  fi
  region_pid=$!
  for ((i=0;i<150;i++)); do
    if ! kill -0 "$driver_pid" 2>/dev/null || ! kill -0 "$region_pid" 2>/dev/null; then
      echo "[ERROR] 雷达或区域节点提前退出；检查 $LOG" >&2
      tail -n 20 "$LOG/driver.log" "$LOG/regions.log" >&2 || true
      exit 1
    fi
    if [[ "$MODE" == start-startup ]] && grep -Fq 'STARTUP_SNAPSHOT status=' "$LOG/regions.log"; then
      if grep -Fq 'STARTUP_SNAPSHOT status=BOTH_GEOMETRIES_FOUND' "$LOG/regions.log"; then
        echo '[PASS] 开局全场扫描发现两个几何候选；身份仍需视觉确认，禁止自动运动。'
        break
      fi
      echo '[ERROR] 开局扫描未能确定两个独立几何候选；见 ROS /competition/lidar/startup_targets' >&2
      exit 1
    fi
    if [[ "$MODE" == start-regions ]] && grep -Fq 'READY: valid scan and time-aligned stationary TF' "$LOG/regions.log"; then
      echo '[PASS] 雷达区域观察就绪（未确认目标身份，不授权运动）。'
      break
    fi
    sleep .1
  done
  if [[ "$MODE" == start-startup ]] && grep -Fq 'STARTUP_SNAPSHOT status=BOTH_GEOMETRIES_FOUND' "$LOG/regions.log"; then
    :
  elif [[ "$MODE" == start-regions ]] && grep -Fq 'READY: valid scan and time-aligned stationary TF' "$LOG/regions.log"; then
    :
  else
    echo "[ERROR] 区域扫描未就绪；检查实际 TF、时间戳和 $LOG" >&2
    tail -n 20 "$LOG/driver.log" "$LOG/regions.log" >&2 || true
    exit 1
  fi
  # 任一子进程退出均停止联合实例，不留半运行状态。
  wait -n "$driver_pid" "$region_pid" || true
  echo "[ERROR] 联合实例子进程退出；检查 $LOG" >&2
  exit 1
fi
echo "[INFO] 启动真实雷达，参数文件=$CFG；结束请按 Ctrl+C。"
# 使用 exec 保持前台进程生命周期；无自动后台进程，也不控制电机。
exec ros2 run ydlidar_ros2_driver ydlidar_ros2_driver_node --ros-args --params-file "$CFG"
