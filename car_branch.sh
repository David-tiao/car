
#!/usr/bin/env bash
# 工创小车 car 项目：轻量分支管理
# 配合 main 专用的 sync_car.sh 使用
# Bash 4.4+ / Git 2.34+

set -Eeuo pipefail

ROOT="${CAR_DIR:-$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)}"
RETRIES="${CAR_PUSH_RETRIES:-3}"
WAIT="${CAR_PUSH_WAIT_SECONDS:-3}"
HTTP_VERSION="${CAR_GIT_HTTP_VERSION:-}"
TEMP=''
STAGE=''

say() { printf '%s\n' "$*"; }
warn() { printf '警告：%s\n' "$*" >&2; }
die() { printf '错误：%s\n' "$*" >&2; exit 1; }

help() {
  cat <<'HELP'
car_branch.sh 分支管理

不带参数                  打开分支菜单，回车默认 main
--main                    切换到 main
--create NAME             从本地 main 创建分支并进入
--switch NAME             切换到已有本地或远程分支
--merge-to-main NAME      将 NAME 合并到 main
--merge-main              将 main 合并到当前功能分支
--push                    推送当前功能分支
--list                    查看本地分支
--status                  查看状态
--help                    查看帮助

说明：
  main 使用 ./sync_car.sh 上传。
  功能分支使用 ./car_branch.sh --push 上传。
  合并先在临时工作区检查冲突。
  不强制推送，不自动删除分支。
HELP
}

[[ "$RETRIES" =~ ^[1-8]$ ]] ||
  die 'CAR_PUSH_RETRIES 必须为 1～8'

[[ "$WAIT" =~ ^[0-9]+$ ]] &&
  ((WAIT <= 60)) ||
  die 'CAR_PUSH_WAIT_SECONDS 必须为 0～60'

[[ -z "$HTTP_VERSION" ||
   "$HTTP_VERSION" == HTTP/1.1 ||
   "$HTTP_VERSION" == HTTP/2 ]] ||
  die 'HTTP 版本参数无效'

command -v git >/dev/null 2>&1 ||
  die '未安装 Git'

[[ -d "$ROOT" ]] ||
  die "仓库目录不存在：$ROOT"

cd "$ROOT"
ROOT="$(pwd -P)"

[[ -e .git ]] ||
  die '当前目录没有 Git 仓库'

[[ "$(git rev-parse --show-toplevel 2>/dev/null)" == "$ROOT" ]] ||
  die '必须在 car 仓库根目录执行'

GIT_DIR="$(git rev-parse --absolute-git-dir)"

cleanup() {
  if [[ -n "$STAGE" && -e "$STAGE/.git" ]]; then
    git worktree remove --force -- "$STAGE" \
      >/dev/null 2>&1 || true
  fi

  if [[ -n "$TEMP" && -d "$TEMP" ]]; then
    rm -rf -- "$TEMP"
  fi
}

trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

current_branch() {
  git symbolic-ref -q --short HEAD ||
    die '当前处于分离 HEAD 状态'
}

ensure_main_exists() {
  git show-ref --verify --quiet refs/heads/main ||
    die '本地 main 分支不存在'
}

valid_name() {
  local name="$1"

  [[ -n "$name" && "$name" != -* ]] ||
    die '分支名不能为空或以 - 开头'

  git check-ref-format --branch "$name" \
    >/dev/null 2>&1 ||
    die "非法分支名：$name"
}

no_unfinished_git() {
  [[ -z "$(git ls-files -u)" ]] ||
    die '存在尚未解决的冲突'

  [[ ! -f "$GIT_DIR/MERGE_HEAD" &&
     ! -d "$GIT_DIR/rebase-merge" &&
     ! -d "$GIT_DIR/rebase-apply" &&
     ! -f "$GIT_DIR/CHERRY_PICK_HEAD" ]] ||
    die '存在未完成的 Git 合并、变基或 cherry-pick'
}

require_clean() {
  no_unfinished_git

  [[ -z "$(git status --porcelain --untracked-files=normal)" ]] ||
    die '存在未提交修改或未跟踪文件，请先提交或妥善保存'
}

check_origin() {
  git remote get-url origin >/dev/null 2>&1 ||
    die '未配置 origin 远程仓库'
}

git_net() {
  if [[ -n "$HTTP_VERSION" ]]; then
    git -c "http.version=$HTTP_VERSION" "$@"
  else
    git "$@"
  fi
}

# ========== 切换 main ==========

use_main() {
  ensure_main_exists

  if [[ "$(current_branch)" == main ]]; then
    say '已经在 main，直接开发即可。'
    say '上传代码使用 ./sync_car.sh'
    return 0
  fi

  require_clean
  git switch main

  say '已切换到 main。'
  say '修改完成后执行 ./sync_car.sh'
}

# ========== 创建分支 ==========

create_branch() {
  local name="$1"

  valid_name "$name"

  if [[ "$name" == main ]]; then
    use_main
    return
  fi

  ensure_main_exists
  require_clean

  if git show-ref --verify --quiet "refs/heads/$name"; then
    die "分支已存在：$name，请使用 --switch"
  fi

  git switch -c "$name" main

  say "已从本地 main 创建并进入：$name"
  say '新分支基于本地 main。'
  say '需要最新远程代码时，请先同步 main。'
}

# ========== 切换分支 ==========

switch_branch() {
  local name="$1"
  local found

  valid_name "$name"

  if [[ "$(current_branch)" == "$name" ]]; then
    say "已经在 $name"
    return
  fi

  require_clean

  # 本地分支
  if git show-ref --verify --quiet "refs/heads/$name"; then
    git switch "$name"
    say "已进入本地分支：$name"
    return
  fi

  # 查询远程分支
  check_origin

  found="$(git_net ls-remote --heads origin "refs/heads/$name")" ||
    die '远程分支查询失败'

  [[ -n "$found" ]] ||
    die "找不到分支：$name"

  git_net fetch --no-tags origin \
    "refs/heads/$name:refs/remotes/origin/$name" ||
    die '远程分支拉取失败'

  git switch -c "$name" --track "origin/$name"

  say "已从远程创建本地跟踪分支：$name"
}

# ========== 安全合并 ==========

# 先在临时 worktree 中合并。
# 如果冲突，不修改当前主工作区或分支。
merge_safe() {
  local source="$1"
  local target="$2"
  local base
  local result

  valid_name "$source"
  valid_name "$target"

  [[ "$source" != "$target" ]] ||
    die '源分支和目标分支不能相同'

  git show-ref --verify --quiet "refs/heads/$source" ||
    die "源分支不存在：$source"

  git show-ref --verify --quiet "refs/heads/$target" ||
    die "目标分支不存在：$target"

  require_clean

  base="$(git rev-parse "refs/heads/$target")"

  # 已经合并过
  if git merge-base --is-ancestor \
    "refs/heads/$source" "refs/heads/$target"; then

    say "$source 已经包含在 $target 中，无需合并。"

    if [[ "$target" == main ]]; then
      use_main
    fi

    return 0
  fi

  TEMP="$(mktemp -d -t car-branch-XXXXXXXX)"
  STAGE="$TEMP/stage"

  git worktree add --detach --quiet \
    "$STAGE" "refs/heads/$target" ||
    die '无法创建临时工作区'

  say "正在检查合并：$source → $target"

  if ! git -C "$STAGE" merge --no-edit \
    "refs/heads/$source" \
    >"$TEMP/merge.log" 2>&1; then

    warn '检测到合并冲突，当前主工作区保持原样：'

    git -C "$STAGE" diff \
      --name-only --diff-filter=U >&2 || true

    tail -n 12 "$TEMP/merge.log" >&2 || true

    die '请人工解决冲突后再合并'
  fi

  result="$(git -C "$STAGE" rev-parse HEAD)"

  # 防止合并期间目标分支被其他操作改变
  [[ "$(git rev-parse "refs/heads/$target")" == "$base" ]] ||
    die '目标分支已发生变化，停止合并'

  if [[ "$(current_branch)" != "$target" ]]; then
    git switch "$target" ||
      die '无法切换目标分支'
  fi

  require_clean

  git merge --ff-only "$result" ||
    die '应用合并结果失败，未强制更新'

  say "合并成功（仅本地）：$source → $target"

  if [[ "$target" == main ]]; then
    say '请执行 ./sync_car.sh 上传 main。'
  else
    say '如需上传功能分支，执行 ./car_branch.sh --push'
  fi
}

# ========== 功能分支合并到 main ==========

merge_to_main() {
  local name="$1"

  valid_name "$name"

  [[ "$name" != main ]] ||
    die 'main 不能合并到自己'

  ensure_main_exists

  merge_safe "$name" main
}

# ========== main 合并到当前功能分支 ==========

merge_main_into_current() {
  local name

  name="$(current_branch)"

  if [[ "$name" == main ]]; then
    say '当前就是 main，无需合并。'
    return
  fi

  ensure_main_exists

  merge_safe main "$name"
}

# ========== 推送功能分支 ==========

remote_branch_sha() {
  local name="$1"
  local result

  result="$(git_net ls-remote origin "refs/heads/$name")" ||
    return 2

  [[ -n "$result" ]] || return 3

  printf '%s\n' "${result%%[[:space:]]*}"
}

transient_error() {
  grep -Eiq \
    'HTTP (408|429|500|502|503|504)|RPC failed|timed out|timeout|unexpected disconnect|connection reset|connection refused|network is unreachable|could not resolve host|SSL_read|gnutls|remote end hung up|early EOF' \
    "$1"
}

push_branch() {
  local name head remote rc tries
  local logdir logfile

  name="$(current_branch)"

  [[ "$name" != main ]] ||
    die 'main 请使用 ./sync_car.sh 上传'

  check_origin
  no_unfinished_git

  if [[ -n "$(git status --porcelain --untracked-files=normal)" ]]; then
    warn '存在未提交文件，只会推送已有 Git 提交。'
  fi

  head="$(git rev-parse HEAD)"

  logdir="$GIT_DIR/car-branch"
  mkdir -p "$logdir"
  logfile="$logdir/last-push.log"

  rc=0
  remote="$(remote_branch_sha "$name")" || rc=$?

  if ((rc == 0)) && [[ "$remote" == "$head" ]]; then
    say "远程分支已是最新提交：${head:0:12}"
    return
  fi

  if ((rc == 2)); then
    die '无法查询远程状态，暂不推送'
  fi

  for ((tries=1; tries<=RETRIES; tries++)); do
    say "推送 $name：第 $tries/$RETRIES 次"

    if git_net push -u origin "$name" \
      >"$logfile" 2>&1; then
      rc=0
    else
      rc=$?
    fi

    cat "$logfile"

    # 即使 Git 报错，也重新检查远程 SHA。
    remote="$(remote_branch_sha "$name")" || true

    if [[ "$remote" == "$head" ]]; then
      say "已核实 origin/$name 推送成功。"
      return
    fi

    if ((rc == 0)); then
      die '推送返回成功，但远程 SHA 暂时无法核实'
    fi

    if ! transient_error "$logfile"; then
      die "推送被拒绝或出现非网络错误：$logfile"
    fi

    if ((tries < RETRIES)); then
      sleep "$((WAIT * tries))"
    fi
  done

  die "推送多次失败，本地代码保留。日志：$logfile"
}

# ========== 状态 ==========

list_branches() {
  git branch -vv
}

status() {
  say "当前分支：$(current_branch)"
  git status -sb
}

# ========== 交互菜单 ==========

menu() {
  local choice name

  say ''
  say '========== car 分支管理 =========='
  say "当前分支：$(current_branch)"
  say ''
  say '  回车 / 0  使用 main（默认）'
  say '  1         创建分支并进入'
  say '  2         进入已有分支'
  say '  3         将分支合并到 main'
  say '  4         将 main 合并到当前分支'
  say '  5         推送当前功能分支'
  say '  6         查看分支列表'
  say '  7         查看状态'
  say '  q         退出'
  say ''

  IFS= read -r -p '请选择 [默认 0]：' choice ||
    die '未收到输入'

  case "$choice" in
    ''|0)
      use_main
      ;;
    1)
      IFS= read -r -p '新分支名称：' name ||
        die '未收到名称'
      create_branch "$name"
      ;;
    2)
      IFS= read -r -p '切换到分支：' name ||
        die '未收到名称'
      switch_branch "$name"
      ;;
    3)
      IFS= read -r -p '合并哪个分支到 main：' name ||
        die '未收到名称'
      merge_to_main "$name"
      ;;
    4)
      merge_main_into_current
      ;;
    5)
      push_branch
      ;;
    6)
      list_branches
      ;;
    7)
      status
      ;;
    q|Q)
      say '已退出，不作修改'
      ;;
    *)
      die '无效选项'
      ;;
  esac
}

# ========== 命令行参数 ==========

if (($# == 0)); then
  menu
  exit 0
fi

case "$1" in
  --main)
    (($# == 1)) || die '参数数量不正确'
    use_main
    ;;
  --create)
    (($# == 2)) || die '用法：--create 分支名'
    create_branch "$2"
    ;;
  --switch)
    (($# == 2)) || die '用法：--switch 分支名'
    switch_branch "$2"
    ;;
  --merge-to-main)
    (($# == 2)) || die '用法：--merge-to-main 分支名'
    merge_to_main "$2"
    ;;
  --merge-main)
    (($# == 1)) || die '参数数量不正确'
    merge_main_into_current
    ;;
  --push)
    (($# == 1)) || die '参数数量不正确'
    push_branch
    ;;
  --list)
    (($# == 1)) || die '参数数量不正确'
    list_branches
    ;;
  --status)
    (($# == 1)) || die '参数数量不正确'
    status
    ;;
  -h|--help)
    help
    ;;
  *)
    die "未知参数：$1"
    ;;
esac
