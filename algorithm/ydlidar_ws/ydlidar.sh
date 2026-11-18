#!/usr/bin/env bash
# ROS 2 Humble / Algorithm Workspace / YDLIDAR 启动与检查脚本
# 默认工作空间：~/Desktop/algorithm/ydlidar_ws
# 用法：bash ydlidar.sh [all|build|start|check|rviz|help]

set -Eeuo pipefail

WORKSPACE="${ALGORITHM_WS:-$HOME/Desktop/algorithm/ydlidar_ws}"
ROS_SETUP="/opt/ros/humble/setup.bash"
PACKAGE="ydlidar"                    # 注意：包名不是 ydlidar_ros2
LAUNCH_FILE="ydlidar_launch.py"

info() { printf '\n[INFO] %s\n' "$*"; }
warn() { printf '\n[WARN] %s\n' "$*" >&2; }
fail() { printf '\n[ERROR] %s\n' "$*" >&2; exit 1; }

usage() {
  cat <<'HELP'
ROS 2 Humble / YDLIDAR 工具脚本

用法：bash ydlidar.sh [模式]

  all     （默认）在工作空间根目录编译，然后启动雷达驱动
  build   只编译 ydlidar 包，并核对安装路径
  start   只启动已编译的 YDLIDAR 雷达驱动
  check   检查 /scan 话题并尝试接收一帧扫描数据（最长等待 15 秒）
  rviz    打开 RViz2（手动设置 Fixed Frame=laser_frame，添加 /scan）
  help    查看帮助

示例：
  bash ydlidar.sh all
  bash ydlidar.sh check    # 在另一个终端运行
  bash ydlidar.sh rviz     # 再打开一个终端运行

如果工作空间不是 ~/Desktop/algorithm/ydlidar_ws：
  ALGORITHM_WS=/其他路径/ydlidar_ws bash ydlidar.sh all

注意：雷达应按厂家要求正确供电、连接，且参数文件已配置。
本脚本不会自动调整串口权限、设备参数或 TF 发布方式。
HELP
}

source_file() {
  local filename="$1"
  [[ -f "$filename" ]] || fail "缺少环境文件：$filename"
  # ROS 2 的 setup.bash 可能读取未定义的变量；暂时关闭 nounset。
  set +u
  # shellcheck disable=SC1090
  source "$filename"
  set -u
}

source_ros() {
  source_file "$ROS_SETUP"
}

source_workspace() {
  source_file "$WORKSPACE/install/setup.bash"
}

require_workspace() {
  [[ -d "$WORKSPACE/src" ]] || fail "找不到工作空间源码目录：$WORKSPACE/src"
  [[ -f "$WORKSPACE/src/ydlidar_ros2/package.xml" ]] || warn "未在预期目录找到 ydlidar_ros2/package.xml，请确认源码位置和包名。"
}

check_install_prefix() {
  local actual expected
  actual="$(ros2 pkg prefix "$PACKAGE")" || fail "ROS 2 尚未识别包 $PACKAGE；请先运行 build。"
  expected="$WORKSPACE/install/$PACKAGE"
  info "ROS 2 包安装路径：$actual"
  # 如果意外加载了 src 下旧的构建环境，明确报告出来。
  if [[ "$(readlink -f "$actual")" != "$(readlink -f "$expected")" ]]; then
    warn "当前加载的包路径不是期望的工作空间安装目录：$expected"
    warn "请使用新终端，并在工作空间根目录编译与加载 install/setup.bash。"
    return 1
  fi
}

build_package() {
  require_workspace
  source_ros
  command -v colcon >/dev/null 2>&1 || fail "colcon 不可用，请检查 ROS 2 / colcon 安装。"
  cd "$WORKSPACE"
  info "工作空间：$PWD"
  info "开始编译：$PACKAGE（普通构建，不使用 --symlink-install）"
  colcon build --packages-select "$PACKAGE"
  source_workspace
  check_install_prefix
  info "编译及环境检查完成。"
}

start_driver() {
  require_workspace
  source_ros
  source_workspace
  check_install_prefix
  info "启动雷达：ros2 launch $PACKAGE $LAUNCH_FILE"
  info "此命令会持续运行；请另开终端执行 check 或 rviz。"
  exec ros2 launch "$PACKAGE" "$LAUNCH_FILE"
}

check_scan() {
  source_ros
  source_workspace
  info "ROS 2 包：$(ros2 pkg prefix "$PACKAGE" 2>/dev/null || echo '未找到')"
  info "检查当前话题列表："
  ros2 topic list
  if ! ros2 topic list | grep -Fxq '/scan'; then
    fail "未发现 /scan。检查雷达驱动是否启动，以及驱动配置的实际话题名称。"
  fi
  info "/scan 存在，话题信息如下："
  ros2 topic info /scan
  info "尝试接收一帧扫描数据（最长 15 秒；只显示前 26 行）："
  if timeout 15s ros2 topic echo /scan --once | sed -n '1,26p'; then
    info "检测完成。如果有 header.frame_id 和 ranges，说明收到雷达消息。"
  else
    fail "15 秒内未成功接收 /scan 数据。检查雷达连接、驱动日志和 Lifecycle 状态。"
  fi
}

start_rviz() {
  source_ros
  source_workspace
  command -v rviz2 >/dev/null 2>&1 || fail "没有找到 rviz2 命令。"
  info "启动 RViz2。打开后设置 Fixed Frame 为实际 /scan.header.frame_id（常见为 laser_frame）。"
  info "Add → LaserScan → Topic=/scan。"
  exec rviz2
}

MODE="${1:-all}"
case "$MODE" in
  all)   build_package; start_driver ;;
  build) build_package ;;
  start) start_driver ;;
  check) check_scan ;;
  rviz)  start_rviz ;;
  help|-h|--help) usage ;;
  *) usage; fail "未知模式：$MODE" ;;
esac
