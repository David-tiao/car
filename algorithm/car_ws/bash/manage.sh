#!/usr/bin/env bash
# 唯一命令行管理入口。进程身份识别/退出清理由现有 V6 管理器负责。
# 用法：bash manage.sh start|stop|restart|status|doctor|logs [--offline]
#       bash manage.sh lidar [start|doctor]  # 单独测试真实雷达
#       bash manage.sh competition [start|stop|restart|status|doctor|logs]
set -Eeuo pipefail
HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ACTION="${1:-start}"
case "$ACTION" in
  lidar) shift; exec bash "$HERE/lidar.sh" "${@:-}" ;;
  competition) shift; exec bash "$HERE/competition.sh" "$@" ;;
  start|stop|restart|status|doctor|logs)
    exec /usr/bin/python3 "$HERE/competition_v6_viewer.py" "$@" ;;
  *) echo "[ERROR] 不支持的管理动作：$ACTION" >&2; exit 2 ;;
esac
