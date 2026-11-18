#!/usr/bin/env bash
# Only supervises two perception/planning nodes; /scan and /map are external.
set -Eeuo pipefail
STAGE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(dirname "$STAGE")"
RUNTIME="${XDG_STATE_HOME:-$HOME/.local/state}/competition_fixed_route"
STATE="$RUNTIME/instance.state"
LOCK="$RUNTIME/instance.lock"
LOGDIR="$RUNTIME/logs"

msg() { printf '[%s] %s\n' "$1" "$2"; }
init_dirs() { mkdir -p "$RUNTIME" "$LOGDIR"; }
start_ticks() { [[ -r "/proc/$1/stat" ]] && awk '{print $22}' "/proc/$1/stat"; }
is_ours() {
    local pid="$1" started="$2" cmd
    [[ "$pid" =~ ^[0-9]+$ && "$started" =~ ^[0-9]+$ ]] || return 1
    [[ "$(start_ticks "$pid" 2>/dev/null || :)" == "$started" ]] || return 1
    cmd="$(tr '\0' ' ' < "/proc/$pid/cmdline" 2>/dev/null || :)"
    [[ "$cmd" == *"operate_lock.sh"* ]]
}
read_state() {
    [[ -f "$STATE" ]] || return 1
    read -r SUPERVISOR STARTED LOCK_PID TOPO_PID LOGPATH < "$STATE" || return 1
}
status() {
    if ! read_state; then msg STOPPED '没有已记录实例'; return 1; fi
    if ! is_ours "$SUPERVISOR" "$STARTED"; then
        msg UNKNOWN "记录的 PID=$SUPERVISOR 身份不匹配；不会擅自结束任何进程"
        return 2
    fi
    if kill -0 "$LOCK_PID" 2>/dev/null && kill -0 "$TOPO_PID" 2>/dev/null; then
        msg RUNNING "守护 PID=$SUPERVISOR 感知=$LOCK_PID 规划=$TOPO_PID 日志=$LOGPATH"
        return 0
    fi
    msg DEGRADED "有模块已退出；查看日志：$LOGPATH"
    return 3
}
stop() {
    if ! read_state; then msg STOPPED '没有已记录实例'; return 0; fi
    if ! is_ours "$SUPERVISOR" "$STARTED"; then
        msg ERROR "PID 身份不符或实例已不存在，禁止误杀；请检查 $STATE"
        return 2
    fi
    kill -TERM "$SUPERVISOR"
    for ((i=0;i<50;i++)); do
        if ! kill -0 "$SUPERVISOR" 2>/dev/null; then
            msg STOPPED '已通知本实例正常退出'; return 0
        fi
        sleep .1
    done
    msg ERROR "安全停止超时，未尝试强制杀死进程：$SUPERVISOR"
    return 3
}
ros_env() {
    [[ -r /opt/ros/humble/setup.bash ]] || { msg ERROR '缺少 /opt/ros/humble/setup.bash'; return 2; }
    # shellcheck disable=SC1091
    set +u
    source /opt/ros/humble/setup.bash
    set -u
    command -v python3 >/dev/null || { msg ERROR 'python3 不可用'; return 2; }
    command -v flock >/dev/null || { msg ERROR 'flock 不可用'; return 2; }
}
case "${1:-help}" in
    status) status ;;
    stop) stop ;;
    doctor)
        ros_env
        python3 "$STAGE/preflight_fixed_route.py" --mode inputs --timeout 10
        ;;
    logs)
        if read_state; then
            [[ -r "$LOGPATH/lock.log" ]] && tail -n 35 "$LOGPATH/lock.log" || :
            [[ -r "$LOGPATH/topology.log" ]] && tail -n 35 "$LOGPATH/topology.log" || :
        else
            msg INFO "历史日志在 $LOGDIR"
            ls -1dt "$LOGDIR"/* 2>/dev/null | head -n 5 || :
        fi
        ;;
    restart)
        shift; stop; exec "$STAGE/operate_lock.sh" start "$@"
        ;;
    start)
        shift
        EXPECTED=2 FROM=C1 TO=A3
        while (($#)); do
            case "$1" in
                --expected|--start|--goal)
                    (($#>=2)) || { msg ERROR "$1 缺少参数"; exit 2; }
                    case "$1" in
                        --expected) EXPECTED="$2";;
                        --start) FROM="$2";;
                        --goal) TO="$2";;
                    esac
                    shift 2 ;;
                *) msg ERROR "未知参数: $1"; exit 2 ;;
            esac
        done
        [[ "$EXPECTED" =~ ^[1-9][0-9]*$ ]] || { msg ERROR '--expected 必须是正整数'; exit 2; }
        [[ "$FROM" =~ ^[ABC][123]$ && "$TO" =~ ^[ABC][123]$ ]] || {
            msg ERROR '起点终点必须是 A1..C3'; exit 2;
        }
        init_dirs
        exec 9>"$LOCK"
        flock -n 9 || { msg ERROR '已有脚本管理实例正在运行，不重复启动'; exit 3; }
        if read_state; then
            if is_ours "$SUPERVISOR" "$STARTED"; then
                msg ERROR "已有实例 PID=$SUPERVISOR"; exit 3
            fi
            if kill -0 "$SUPERVISOR" 2>/dev/null; then
                msg ERROR "旧 PID=$SUPERVISOR 被占用但身份不符，人工核查 $STATE"; exit 3
            fi
            # Known-dead supervisor: this state belongs to the script, safe to retire.
            rm -f -- "$STATE"
        fi
        ros_env
        msg CHECK '先核对外部 /scan /map /tf，并拒绝已有同名节点'
        python3 "$STAGE/preflight_fixed_route.py" --mode inputs --timeout 10
        LOGPATH="$LOGDIR/$(date +%Y%m%d_%H%M%S)_$$"
        mkdir -p "$LOGPATH"
        LOCK_PID='' TOPO_PID=''
        cleanup() {
            local pid
            trap - EXIT INT TERM HUP
            for pid in "$TOPO_PID" "$LOCK_PID"; do
                [[ "$pid" =~ ^[0-9]+$ ]] && kill -TERM "$pid" 2>/dev/null || :
            done
            for pid in "$TOPO_PID" "$LOCK_PID"; do
                [[ "$pid" =~ ^[0-9]+$ ]] && wait "$pid" 2>/dev/null || :
            done
            if read_state && is_ours "$SUPERVISOR" "$STARTED" && [[ "$SUPERVISOR" == "$$" ]]; then
                rm -f -- "$STATE"
            fi
            msg INFO "仅本次两个子进程已停止；日志保留：$LOGPATH"
        }
        trap cleanup EXIT
        trap 'exit 130' INT
        trap 'exit 143' TERM
        trap 'exit 129' HUP
        "$STAGE/start_obstacle_lock.sh" --expected "$EXPECTED" \
            --snapshot "$LOGPATH/locked_snapshot.json" >"$LOGPATH/lock.log" 2>&1 & LOCK_PID=$!
        "$ROOT/topology_stage6/start_road_topology.sh" --expected-obstacles "$EXPECTED" \
            --start "$FROM" --goal "$TO" >"$LOGPATH/topology.log" 2>&1 & TOPO_PID=$!
        printf '%s %s %s %s %s\n' "$$" "$(start_ticks $$)" "$LOCK_PID" "$TOPO_PID" "$LOGPATH" > "$STATE"
        if ! python3 "$STAGE/preflight_fixed_route.py" --mode modules --timeout 12; then
            msg ERROR "启动未达到话题就绪；日志：$LOGPATH"; exit 4
        fi
        msg RUNNING '两个节点开始发布状态；尚未证明障碍物已锁定，不会启动真实底盘'
        msg INFO "可新开终端查询: $STAGE/operate_lock.sh status"
        set +e
        wait -n "$LOCK_PID" "$TOPO_PID"
        code=$?
        set -e
        msg ERROR "一个子模块提前退出（退出码=$code）；请查看 $LOGPATH"
        exit 5
        ;;
    *)
        echo "用法: $0 {start|stop|restart|status|doctor|logs} [--expected 2 --start C1 --goal A3]"
        exit 2;;
esac
