#!/usr/bin/env bash
# Desktop launches a background-managed service and retains a visible result.
set -Eeo pipefail
DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd -- "$DIR/.." && pwd)"
if bash "$ROOT/scripts/ai.sh" start; then
    rc=0
else
    rc=$?
fi
echo
echo "按回车关闭此窗口；后台服务不会随终端窗口关闭。"
echo "以后可在终端执行：$ROOT/scripts/ai.sh status / stop / logs"
if [[ -t 0 ]]; then
    read -r _ || true
fi
exit "$rc"
