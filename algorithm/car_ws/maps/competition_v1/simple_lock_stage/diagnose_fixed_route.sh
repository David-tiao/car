#!/usr/bin/env bash
# Project-local, non-destructive offline quality gate.
set -Eeuo pipefail
STAGE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(dirname "$STAGE")"
STATE="${XDG_STATE_HOME:-$HOME/.local/state}/competition_fixed_route"
mkdir -p "$STATE/logs"
LOG="$STATE/logs/diagnose_$(date +%Y%m%d_%H%M%S)_$$.log"
mode="${1:-check}"
exec > >(tee -a "$LOG") 2>&1
printf '[INFO] log=%s\n' "$LOG"
printf '[INFO] python=%s\n' "$(python3 --version)"
printf '[INFO] ros_setup=%s\n' "$([[ -r /opt/ros/humble/setup.bash ]] && echo PRESENT || echo MISSING)"
case "$mode" in
 env) echo '[INFO] only environment check requested' ;;
 check|build)
    for f in "$STAGE"/*.py "$ROOT/topology_stage6/road_topology.py"; do
        python3 - "$f" <<'PY'
import ast, pathlib, sys
path = pathlib.Path(sys.argv[1]); ast.parse(path.read_text(encoding='utf-8'))
print('[SYNTAX_PASS]', path.name)
PY
    done
    for f in "$STAGE"/*.sh "$ROOT/topology_stage6/start_road_topology.sh"; do
        bash -n "$f"
        printf '[BASH_SYNTAX_PASS] %s\n' "$(basename "$f")"
    done
    if [[ "$mode" == check ]]; then
        (cd "$STAGE" && python3 test_fixed_route.py)
    fi
    echo '[DONE] 本地语法检查与指定测试完成；不代表 ROS/硬件联调通过' ;;
 *) echo "未知诊断模式 $mode (check|env|build)"; exit 2;;
esac
