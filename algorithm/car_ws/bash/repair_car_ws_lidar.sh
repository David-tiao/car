#!/usr/bin/env bash
# 工创小车：迁移 car_ws 后恢复 YDLIDAR ROS 2 驱动。
# 作用：检查已有 SDK；必要时仅在本工作空间获取/编译 SDK；
#       仅编译 ydlidar_ros2_driver；注册到 V6 启动器已使用的 overlay。
# 不启动雷达，不向 STM32 发送命令；不删除地图/算法/其他 ROS 包。
set -Eeuo pipefail
source "$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)/lib/common.sh"
car_check_workspace
WS="$CAR_WS"
SRC="$WS/src/ydlidar_ros2_driver"
INSTALL="$WS/install_oneclick_lidar"
STAMP="$(date +%Y%m%d_%H%M%S)_$$"
BUILD="$WS/build_lidar_repair_$STAMP"
LOG="$WS/log_lidar_repair_$STAMP"
SDK_SRC="$WS/vendor/YDLidar-SDK"
SDK_PREFIX="$WS/install_oneclick_sdk"
BACKUP=""

err() { echo "[ERROR] $*" >&2; exit 1; }
[[ -f "$SRC/package.xml" ]] || err "缺少驱动源码：$SRC/package.xml"
[[ -f "$WS/config/x3.yaml" ]] || err "缺少雷达配置：$WS/config/x3.yaml"
[[ -r "$CAR_ROS_SETUP" ]] || err "缺少 ROS 2 Humble: $CAR_ROS_SETUP"
for x in colcon cmake git make; do command -v "$x" >/dev/null || err "缺少编译工具：$x"; done
# ROS 环境脚本会读取未定义变量，仅在 source 期间关闭 nounset。
car_source_ros_base

# 优先使用已成功安装的 SDK，避免重新下载与编译。
SDK_DIR=""
SDK_ROOT=""
# Explicit SDK CMake config directory wins; never silently fall back if invalid.
if [[ -n "${CAR_YDLIDAR_SDK_DIR:-}" ]]; then
  [[ -f "$CAR_YDLIDAR_SDK_DIR/ydlidar_sdkConfig.cmake" ]] || err "无效 CAR_YDLIDAR_SDK_DIR: $CAR_YDLIDAR_SDK_DIR"
  SDK_DIR="$(cd -- "$CAR_YDLIDAR_SDK_DIR" && pwd -P)"
else
  prefixes=("$SDK_PREFIX")
  if [[ -n "${CAR_YDLIDAR_SDK_PREFIX:-}" ]]; then
    [[ -d "$CAR_YDLIDAR_SDK_PREFIX" ]] || err "无效 CAR_YDLIDAR_SDK_PREFIX: $CAR_YDLIDAR_SDK_PREFIX"
    prefixes=("$CAR_YDLIDAR_SDK_PREFIX")
  else
    IFS=: read -r -a cmake_prefixes <<< "${CMAKE_PREFIX_PATH:-}"
    prefixes+=("${cmake_prefixes[@]}" /usr/local /usr)
  fi
  for p in "${prefixes[@]}"; do
    [[ -n "$p" && -d "$p" ]] || continue
    for libdir in lib lib64 "lib/$(gcc -print-multiarch 2>/dev/null || true)"; do
      cfg="$p/$libdir/cmake/ydlidar_sdk/ydlidar_sdkConfig.cmake"
      if [[ -f "$cfg" ]]; then
        SDK_ROOT="$(cd -- "$p" && pwd -P)"
        SDK_DIR="$(cd -- "${cfg%/*}" && pwd -P)"
        break 2
      fi
    done
  done
  if [[ -n "${CAR_YDLIDAR_SDK_PREFIX:-}" && -z "$SDK_DIR" ]]; then
    err "指定 SDK prefix 中找不到配置；可显式设置 CAR_YDLIDAR_SDK_DIR"
  fi
fi
if [[ -z "$SDK_DIR" ]]; then
  # 没有 SDK 安装时，优先使用 vendor 源码；没有源码才获取官方 SDK。
  if [[ ! -f "$SDK_SRC/CMakeLists.txt" ]]; then
    [[ ! -e "$SDK_SRC" ]] || err "SDK 源码目录已存在但不完整：$SDK_SRC（未覆盖）"
    mkdir -p "$WS/vendor"
    TEMP="$WS/vendor/.YDLidar-SDK-download-$STAMP"
    echo '[SDK] 本地无可用 SDK，尝试从官方 GitHub 获取源码'
    git clone --depth 1 https://github.com/YDLIDAR/YDLidar-SDK.git "$TEMP" || err 'SDK 下载失败；未修改驱动安装目录'
    [[ -f "$TEMP/CMakeLists.txt" ]] || err '下载的 SDK 缺少 CMakeLists.txt'
    mv "$TEMP" "$SDK_SRC"
  fi
  command -v g++ >/dev/null || err '缺少 g++'
  echo '[SDK] 在 car_ws 内编译 SDK'
  cmake -S "$SDK_SRC" -B "$WS/build_lidar_sdk_repair_$STAMP" \
    -DCMAKE_INSTALL_PREFIX="$SDK_PREFIX" \
    -DBUILD_EXAMPLES=OFF -DBUILD_TEST=OFF -DBUILD_CSHARP=OFF \
    -DBUILD_SHARED_LIBS=OFF
  cmake --build "$WS/build_lidar_sdk_repair_$STAMP" --parallel 2
  cmake --install "$WS/build_lidar_sdk_repair_$STAMP"
  SDK_ROOT="$SDK_PREFIX"
  SDK_DIR="$SDK_PREFIX/lib/cmake/ydlidar_sdk"
  [[ -f "$SDK_DIR/ydlidar_sdkConfig.cmake" ]] || err "SDK 安装失败，找不到 $SDK_DIR"
fi
printf '[PASS] SDK 配置：%s\n' "$SDK_DIR"

# 单独编译雷达驱动时，临时移开 COLCON_IGNORE。
# 编译结束自动恢复，不影响主工作空间原来的包选择策略。
exec 9>"$WS/bash/.lidar_build.lock"
flock -n 9 || err '已有另一个雷达构建任务'

IGNORE="$SRC/COLCON_IGNORE"
SAVED=""

restore_ignore() {
  if [[ -n "$SAVED" && -f "$SAVED" ]]; then
    [[ ! -e "$IGNORE" ]] || {
      echo '[ERROR] 忽略标记恢复冲突' >&2
      return 1
    }
    mv -- "$SAVED" "$IGNORE"
    echo '[RESTORE] 已恢复 COLCON_IGNORE'
  fi
}

trap restore_ignore EXIT

if [[ -f "$IGNORE" && ! -L "$IGNORE" ]]; then
  SAVED="$WS/bash/.COLCON_IGNORE_$STAMP"
  mv -- "$IGNORE" "$SAVED"
  echo '[INFO] 临时移开 COLCON_IGNORE'
elif [[ -e "$IGNORE" || -L "$IGNORE" ]]; then
  err 'COLCON_IGNORE 不是普通文件，拒绝操作'
fi

# 编译前必须确认 colcon 真正发现了驱动包。
PKGS="$(colcon list --paths "$SRC" --names-only)" \
  || err 'colcon list 失败'

echo "[DISCOVER] $PKGS"

grep -Fxq 'ydlidar_ros2_driver' <<< "$PKGS" \
  || err '驱动包尚未被发现，不继续构建'

# 备份旧 overlay，不覆盖移动前生成的安装文件或其他 ROS 功能包。
if [[ -e "$INSTALL" || -L "$INSTALL" ]]; then
  BACKUP="$WS/install_oneclick_lidar.bak_$STAMP"
  mv -- "$INSTALL" "$BACKUP"
  echo "[BACKUP] $BACKUP"
fi

if [[ -n "$SDK_ROOT" ]]; then
  export CMAKE_PREFIX_PATH="$SDK_ROOT${CMAKE_PREFIX_PATH:+:$CMAKE_PREFIX_PATH}"
fi

# colcon 的 --log-base 是全局参数，必须位于 build 之前。
echo '[BUILD] 仅编译 ydlidar_ros2_driver'
if ! (cd "$WS" && colcon --log-base "$LOG" build \
      --paths "$SRC" \
      --build-base "$BUILD" \
      --install-base "$INSTALL" \
      --packages-select ydlidar_ros2_driver \
      --cmake-args -DBUILD_TESTING=OFF "-Dydlidar_sdk_DIR=$SDK_DIR"); then
  echo "[ERROR] 驱动编译失败；构建日志目录：$LOG" >&2
  if [[ -n "$BACKUP" ]]; then
    [[ ! -e "$INSTALL" ]] || mv -- "$INSTALL" "$WS/install_oneclick_lidar.failed_$STAMP"
    mv -- "$BACKUP" "$INSTALL"
    echo '[ROLLBACK] 已恢复原驱动安装目录'
  fi
  exit 1
fi

[[ -r "$INSTALL/setup.bash" ]] || err "构建结束但缺少 $INSTALL/setup.bash"
car_source_setup "$INSTALL/setup.bash"
ros2 pkg prefix ydlidar_ros2_driver || err '包注册验证失败'
ros2 pkg executables ydlidar_ros2_driver | grep -qE '^ydlidar_ros2_driver[[:space:]]+ydlidar_ros2_driver_node$' || err '缺少驱动节点可执行文件'
echo '[PASS] 驱动已在新 car_ws 环境注册，V6 可以找到 ydlidar_ros2_driver'
echo "[LOG] $LOG"
echo '[NEXT] bash bash/lidar.sh doctor'
