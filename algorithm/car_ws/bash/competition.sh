#!/usr/bin/env bash
# 原比赛静止锁路流程入口：复用既有 ai.sh，绝不复制或改写 Dijkstra 算法。
# 注意：真实障碍物锁路需要实测 TF；本入口不发送电机运动指令。
# 用法：bash competition.sh doctor | start | stop | restart | status | logs
set -Eeuo pipefail
HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WS="$(cd -- "$HERE/.." && pwd)"
ENTRY="$WS/maps/competition_v1/scripts/ai.sh"
[[ -f "$ENTRY" ]] || { echo "[ERROR] 原比赛入口缺失：$ENTRY" >&2; exit 1; }
exec bash "$ENTRY" "${@:-doctor}"
