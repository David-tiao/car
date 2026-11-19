#!/usr/bin/env bash
# 桌面一键入口：启动 V6 地图 + 原始雷达 + RViz（不控制电机）。
# 重复启动由原 Python 管理器核验并安全停止旧实例；关闭 RViz 会自动清理。
set -Eeuo pipefail
HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
exec /usr/bin/python3 "$HERE/competition_v6_viewer.py" start --desktop "$@"
