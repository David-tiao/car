
#!/usr/bin/env bash
# car 项目安全同步工具 v3.0
# Ubuntu/Linux, Bash 4.4+, Git 2.34+
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
REPO="${CAR_DIR:-$SCRIPT_DIR}"
REMOTE_URL="${CAR_REMOTE_URL:-https://github.com/David-tiao/car.git}"
STM32_URL="${CAR_STM32_URL:-https://github.com/Wangdachang888/GongChuangSai.git}"
STM32_BRANCH="${CAR_STM32_BRANCH:-refactor/architecture-v2}"
STM32_CACHE="${CAR_STM32_CACHE:-$HOME/.cache/car-sync/stm32-upstream.git}"
STATE_FILE='.car-sync/stm32-upstream.sha'
RETRIES="${CAR_PUSH_RETRIES:-3}"
WAIT="${CAR_PUSH_WAIT_SECONDS:-3}"
DELETE_LIMIT="${CAR_DELETE_WARN_THRESHOLD:-20}"
HTTP_VERSION="${CAR_GIT_HTTP_VERSION:-}"
MODE=sync
TITLE=''
DETAILS=''
REF=''
HISTORY=15
NESTED=false
DELETE_BUILD=false
DELETE_APPROVED=false
PROTECTED_APPROVED=false
TMP=''
STAGE=''
RULES=''
BASE=''
NEW_SHA=''
OLD_SHA=''
FETCH_HAS_REMOTE=false

say() { printf '%s\n' "$*"; }
warn() { printf '警告：%s\n' "$*" >&2; }
die() { printf '错误：%s\n' "$*" >&2; exit 1; }

usage() {
  cat <<'HELP'
用法：把脚本放在 car 根目录，执行 bash sync_car.sh [选项]
  (不带参数)                  提交本地更改、安全合并远程 main、推送并校验
  --message "标题" [--details "正文"]  非交互指定提交说明
  --preview                  查看本地变化及删除风险；不修改文件
  --doctor                   检查 Git/仓库/分支/远程配置；不修改文件
  --check-remote             检查本地 HEAD 与远程 main 的 SHA
  --retry-push               只补推已有提交（不会 add、commit、merge）
  --history [N]              查看最近 N 个提交（1～200）
  --show <版本>              查看提交详情与文件统计
  --rollback <版本>          在临时工作区准备回溯，审核后生成新提交
  --backups                  查看本地安全备份标签
  --include-nested           经确认后备份并移除嵌套 .git 元数据
  --delete-build             经确认后删除无被跟踪文件的构建目录
  --check-stm32              检查队友仓库的新提交
  --stm32-diff               查看队友自上次导入以来的代码差异
  --update-stm32             临时工作区中合并 STM32，成功后本地提交
  --help                     显示帮助
环境变量：
  CAR_DIR, CAR_REMOTE_URL, CAR_STM32_URL, CAR_STM32_BRANCH, CAR_STM32_CACHE
  CAR_PUSH_RETRIES=3, CAR_PUSH_WAIT_SECONDS=3, CAR_DELETE_WARN_THRESHOLD=20
  CAR_GIT_HTTP_VERSION=HTTP/1.1  # 仅对本次脚本的远程 Git 操作生效
安全原则：不 force-push；不 reset --hard；冲突不写入主工作区；
          删除关键目录文件需要确认；STM32 更新不会自动推送。
HELP
}

# 参数解析：禁止把两种操作意外混用。
choose_mode() {
  [[ "$MODE" == sync ]] || die '只能指定一种操作模式'
  [[ "$NESTED" == false && "$DELETE_BUILD" == false ]] || die '选项不能与其他操作模式组合'
  MODE="$1"
}
while (($#)); do
  case "$1" in
    --include-nested) [[ "$MODE" == sync ]] || die '参数冲突'; NESTED=true; shift ;;
    --delete-build) [[ "$MODE" == sync ]] || die '参数冲突'; DELETE_BUILD=true; shift ;;
    --message) (($# >= 2)) || die '--message 缺少标题'; TITLE="$2"; shift 2 ;;
    --details) (($# >= 2)) || die '--details 缺少正文'; DETAILS="$2"; shift 2 ;;
    --preview|--doctor|--check-remote|--retry-push|--backups|--check-stm32|--stm32-diff|--update-stm32)
      choose_mode "${1#--}"; shift ;;
    --history) choose_mode history; shift
      if (($#)) && [[ "$1" =~ ^[0-9]+$ ]]; then HISTORY="$1"; shift; fi ;;
    --show|--rollback)
      (($# >= 2)) || die "$1 缺少版本参数"
      opt="${1#--}"; choose_mode "$opt"; REF="$2"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) die "未知参数：$1" ;;
  esac
done
[[ "$MODE" == sync || ( -z "$TITLE" && -z "$DETAILS" ) ]] || die '提交说明只允许用于正常同步'
[[ -z "$DETAILS" || -n "$TITLE" ]] || die '--details 需要配合 --message'
[[ -z "$TITLE" || "$TITLE" =~ [^[:space:]] ]] || die '标题不能仅包含空白'
(( HISTORY >= 1 && HISTORY <= 200 )) || die '历史条数必须在 1～200'
[[ "$RETRIES" =~ ^[1-8]$ ]] || die 'CAR_PUSH_RETRIES 必须为 1～8'
[[ "$WAIT" =~ ^[0-9]+$ ]] && ((WAIT <= 60)) || die 'CAR_PUSH_WAIT_SECONDS 必须为 0～60'
[[ "$DELETE_LIMIT" =~ ^[1-9][0-9]*$ ]] && ((DELETE_LIMIT <= 100000)) || die '删除阈值必须在 1～100000'
[[ -z "$HTTP_VERSION" || "$HTTP_VERSION" == HTTP/1.1 || "$HTTP_VERSION" == HTTP/2 ]] || die 'HTTP 版本只允许 HTTP/1.1 或 HTTP/2'
command -v git >/dev/null || die '未安装 git'
command -v mktemp >/dev/null || die '缺少 mktemp'
[[ -d "$REPO" ]] || die "找不到仓库目录：$REPO"
cd "$REPO"
[[ -e .git ]] || die '未找到 .git：请在已初始化的 car 仓库中执行（不会自动覆盖/初始化）'
[[ "$(git rev-parse --show-toplevel 2>/dev/null)" == "$(pwd -P)" ]] || die '脚本须在主仓库根目录执行（CAR_DIR 可指定）'
ROOT="$(pwd -P)"
GIT_DIR="$(git rev-parse --absolute-git-dir)"
LOG_DIR="$GIT_DIR/car-sync"

cleanup() {
  if [[ -n "$STAGE" && -e "$STAGE/.git" ]]; then
    git -C "$ROOT" worktree remove --force -- "$STAGE" >/dev/null 2>&1 || true
  fi
  if [[ -n "$TMP" && -d "$TMP" ]]; then rm -rf -- "$TMP"; fi
  if [[ -n "$RULES" && -f "$RULES" ]]; then rm -f -- "$RULES"; fi
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

git_net() {
  if [[ -n "$HTTP_VERSION" ]]; then git -c "http.version=$HTTP_VERSION" "$@"
  else git "$@"; fi
}
require_main() {
  [[ "$(git symbolic-ref --quiet --short HEAD 2>/dev/null || true)" == main ]] || die '仅允许在 main 分支操作'
  [[ -z "$(git ls-files -u)" ]] || die '发现未解决的 Git 冲突，请先处理'
  [[ ! -f "$GIT_DIR/MERGE_HEAD" && ! -d "$GIT_DIR/rebase-merge" && ! -d "$GIT_DIR/rebase-apply" ]] || die '当前有未结束的合并/rebase'
  git rev-parse -q --verify HEAD >/dev/null || die '当前 main 尚无提交'
}
require_clean() {
  [[ -z "$(git status --porcelain --untracked-files=normal)" ]] || die '主工作区存在未提交或未跟踪文件；请先处理'
}
check_origin() {
  local actual
  actual="$(git remote get-url origin 2>/dev/null || true)"
  [[ -n "$actual" ]] || die '缺少 origin；请先配置远程仓库'
  if [[ "$actual" != "$REMOTE_URL" ]]; then
    # 默认 GitHub 仓库允许 SSH 与 HTTPS 两种等价写法，方便网络故障时切换协议。
    if [[ "$REMOTE_URL" == 'https://github.com/David-tiao/car.git' && "$actual" == 'git@github.com:David-tiao/car.git' ]]; then
      say '已识别 origin 为相同 GitHub 仓库的 SSH 地址'
    else
      die "origin 与预期不同：$actual（预期：$REMOTE_URL）"
    fi
  fi
}
# 允许 sha-1 与 sha-256 Git 仓库；空输出代表远程 main 尚未建立。
remote_sha() {
  local out
  out="$(git_net ls-remote origin refs/heads/main 2>/dev/null)" || return 2
  if [[ -z "$out" ]]; then return 3; fi
  printf '%s\n' "${out%%[[:space:]]*}"
}
check_remote() {
  local local_sha remote rc=0
  local_sha="$(git rev-parse HEAD)"
  remote="$(remote_sha)" || rc=$?
  if ((rc == 2)); then warn '无法连接远端或查询远端提交'; return 2; fi
  if ((rc == 3)); then warn '远程 main 尚不存在'; return 1; fi
  if [[ "$local_sha" == "$remote" ]]; then say "已核实同步：${local_sha:0:12}"; return 0; fi
  say "本地：$local_sha"
  say "远程：$remote"
  warn '远程和本地不一致'
  return 1
}

show_commit() {
  git --no-pager show --no-patch --date=iso-local \
    --pretty=format:'版本：%H%n日期：%ad%n作者：%an%n标题：%s%n%n%b%n' "$1"
  printf '\n文件统计：\n'
  git --no-pager show --stat --format= "$1"
}

# 保护的路径：任何单个删除均需专门确认。
protected_path() {
  case "$1" in
    STM32F4/*|algorithm/*|stm32_openmv_/*|pi/*|item/*|sync_car.sh|.gitignore|.car-sync/*) return 0 ;;
    *) return 1 ;;
  esac
}
# 只检查 "删除"；用 --no-renames 将改名的源路径也列入检查。
# stage: git 暂存区相对 HEAD；range: 某个提交相对另一个提交。
review_deletions() {
  local kind="$1" from="${2:-}" to="${3:-}" i path response count=0
  local -a removed=() critical=()
  if [[ "$kind" == stage ]]; then
    mapfile -d '' -t removed < <(git diff --cached --no-renames --diff-filter=D --name-only -z)
  elif [[ "$kind" == worktree ]]; then
    mapfile -d '' -t removed < <(git diff --no-renames --diff-filter=D --name-only -z HEAD)
  else
    mapfile -d '' -t removed < <(git diff --no-renames --diff-filter=D --name-only -z "$from" "$to")
  fi
  count=${#removed[@]}
  for path in "${removed[@]}"; do
    if protected_path "$path"; then critical+=("$path"); fi
  done
  if ((count == 0)); then return 0; fi
  if ((count < DELETE_LIMIT && ${#critical[@]} == 0)); then return 0; fi
  say ''
  warn "检测到 $count 个文件删除，其中关键路径 ${#critical[@]} 个。"
  for ((i=0; i<count && i<60; i++)); do printf '  - %s\n' "${removed[i]}"; done
  if ((count > 60)); then say "  ……还有 $((count-60)) 个"; fi
  if ((${#critical[@]})); then
    warn '包括 STM32F4、OpenMV、YDLidar/algorithm、pi、item、同步脚本等受保护路径！'
    if [[ "$PROTECTED_APPROVED" == false ]]; then
      [[ -t 0 ]] || die '非交互模式下拒绝删除关键路径；请在终端运行'
      IFS= read -r -p '确认关键路径删除？输入 CONFIRM PROTECTED DELETE：' response || die '未收到确认'
      [[ "$response" == 'CONFIRM PROTECTED DELETE' ]] || die '取消：未批准关键路径删除'
      PROTECTED_APPROVED=true
    fi
  fi
  if ((count >= DELETE_LIMIT)) && [[ "$DELETE_APPROVED" == false ]]; then
    [[ -t 0 ]] || die '非交互模式下拒绝大量删除；请在终端运行'
    IFS= read -r -p '确认大量删除？输入 CONFIRM DELETE：' response || die '未收到确认'
    [[ "$response" == 'CONFIRM DELETE' ]] || die '取消：未批准大量删除'
    DELETE_APPROVED=true
  fi
}

new_stage() {
  [[ -z "$TMP" ]] || die '临时工作区已在使用'
  TMP="$(mktemp -d -t car-sync-XXXXXXXX)"
  STAGE="$TMP/stage"
  BASE="$(git rev-parse HEAD)"
  git worktree add --detach --quiet "$STAGE" "$BASE" || die '无法建立临时工作区'
}
close_stage() {
  if [[ -n "$STAGE" && -e "$STAGE/.git" ]]; then
    git worktree remove --force -- "$STAGE" || die '无法清理临时 worktree'
  fi
  STAGE=''
  if [[ -n "$TMP" ]]; then rm -rf -- "$TMP"; fi
  TMP=''
}
promote_stage() {
  local commit="$1"
  [[ "$(git rev-parse HEAD)" == "$BASE" ]] || die '处理期间 main 被其他进程改变；拒绝写入'
  require_clean
  git merge --ff-only "$commit" || die '无法安全快进；保留原 HEAD，请手动检查'
}

# 读取远程：超时重试；失败不改变主工作区。
fetch_main() {
  local n remote rc2=0
  FETCH_HAS_REMOTE=false
  mkdir -p "$LOG_DIR"
  for ((n=1; n<=RETRIES; n++)); do
    rc2=0
    remote="$(remote_sha)" || rc2=$?
    if ((rc2 == 3)); then say '远端尚无 main；跳过远端合并'; return 0; fi
    if ((rc2 == 0)); then
      if git_net fetch --no-tags origin '+refs/heads/main:refs/remotes/origin/main' >"$LOG_DIR/last-fetch.log" 2>&1; then FETCH_HAS_REMOTE=true; return 0; fi
    fi
    warn "第 $n 次读取远程失败"
    [[ ! -s "$LOG_DIR/last-fetch.log" ]] || tail -n 6 "$LOG_DIR/last-fetch.log" >&2
    if ((n < RETRIES)); then sleep "$((WAIT*n))"; fi
  done
  die '远程 fetch 失败；本地 HEAD 保留，稍后重试'
}
# 不在主工作区直接 git merge；先在临时 worktree 测试。
safe_merge_remote() {
  fetch_main
  [[ "$FETCH_HAS_REMOTE" == true ]] || return 0
  if git merge-base --is-ancestor refs/remotes/origin/main HEAD; then
    say '远程提交已包含在本地历史中，无需合并'
    return 0
  fi
  require_clean
  new_stage
  mkdir -p "$LOG_DIR"
  if ! git -C "$STAGE" merge --no-edit --allow-unrelated-histories refs/remotes/origin/main >"$LOG_DIR/last-merge.log" 2>&1; then
    warn '远程合并发生冲突。以下是临时工作区的冲突文件：'
    git -C "$STAGE" diff --name-only --diff-filter=U >&2 || true
    warn "主工作区未执行 merge；详情：$LOG_DIR/last-merge.log"
    die '请手动对比双方修改，解决后再运行同步；不会自动覆盖'
  fi
  local merged
  merged="$(git -C "$STAGE" rev-parse HEAD)"
  promote_stage "$merged"
  close_stage
  say '远程变更已安全合并入 main'
}

# push 只重试可识别的临时网络错误；真实成功以远端 SHA 为准。
transient_push_error() {
  grep -Eiq 'HTTP (408|429|500|502|503|504)|timed out|timeout|RPC failed|unexpected disconnect|connection reset|connection refused|network is unreachable|could not resolve host|SSL_read|gnutls|early EOF|remote end hung up|TLS.*(error|fail)' "$1"
}
push_verified() {
  local n rc check_rc remote remote_rc=0
  mkdir -p "$LOG_DIR"
  if check_remote; then say '远端已是当前版本，无需再次推送'; return 0; fi
  # 保证远程分支未被其他人推进；补推时也适用。
  fetch_main
  if [[ "$FETCH_HAS_REMOTE" == true ]]; then
    if ! git merge-base --is-ancestor refs/remotes/origin/main HEAD; then
      die '远程 main 存在本地未合并的新提交；请先正常同步（不强推）'
    fi
    review_deletions range refs/remotes/origin/main HEAD
  fi
  for ((n=1; n<=RETRIES; n++)); do
    say "推送尝试 $n/$RETRIES …"
    if git_net push -u origin main >"$LOG_DIR/last-push.log" 2>&1; then rc=0; else rc=$?; fi
    cat "$LOG_DIR/last-push.log"
    # 即使 HTTP 408，也可能已在远程成功提交：先核实再决定是否重试。
    if check_remote; then
      ((rc == 0)) || say 'git push 曾报错，但远端 SHA 已匹配，确认成功'
      return 0
    fi
    if ((rc == 0)); then
      sleep 2
      if check_remote; then return 0; fi
      die 'push 返回成功，但远端 SHA 未能核实；停止操作，稍后运行 --check-remote'
    fi
    if ! transient_push_error "$LOG_DIR/last-push.log"; then
      die "不是可自动重试的网络错误；见 $LOG_DIR/last-push.log"
    fi
    if ((n < RETRIES)); then say "网络异常，等待 $((WAIT*n)) 秒重试"; sleep "$((WAIT*n))"; fi
  done
  die "多次推送仍未成功；本地提交安全保留。恢复后执行 --retry-push，日志：$LOG_DIR/last-push.log"
}

# .gitignore 只管理自己标记的段落，其他规则原样保留。
IGNORE_START='# >>> car-sync generated files >>>'
IGNORE_END='# <<< car-sync generated files <<<'
make_rules() {
  RULES="$(mktemp -t car-ignore-XXXXXXXX)"
  cat > "$RULES" <<'RULES_TEXT'
**/build/
**/install/
**/log/
**/devel/
**/Debug/
**/Release/
**/.pio/
**/CMakeFiles/
**/cmake-build-*/
**/__pycache__/
**/.pytest_cache/
**/.mypy_cache/
**/.ruff_cache/
*.pyc
*.pyo
*.o
*.obj
*.elf
*.map
RULES_TEXT
}
ensure_ignore() {
  local out=".gitignore.car-sync.tmp.$$"
  local old_start='# >>> car sync: generated files >>>'
  local old_end='# <<< car sync: generated files <<<'
  [[ ! -L .gitignore ]] || die '.gitignore 是符号链接，拒绝覆盖'
  if [[ -f .gitignore ]] && { grep -Fqx "$IGNORE_START" .gitignore || grep -Fqx "$IGNORE_END" .gitignore || grep -Fqx "$old_start" .gitignore || grep -Fqx "$old_end" .gitignore; }; then
    # 同时兼容旧版标记，迁移时只保留一个新规则区块。
    awk -v start="$IGNORE_START" -v end="$IGNORE_END" \
        -v old_start="$old_start" -v old_end="$old_end" -v rule_file="$RULES" '
      $0 == start || $0 == old_start {
        if (inside) {error=1; exit}
        inside=($0 == start ? 1 : 2)
        if (!written) {
          print start
          while ((getline line < rule_file) > 0) print line
          close(rule_file)
          print end
          written=1
        }
        next
      }
      $0 == end || $0 == old_end {
        if (!inside || (inside==1 && $0!=end) || (inside==2 && $0!=old_end)) {error=1; exit}
        inside=0
        next
      }
      !inside {print}
      END {if (inside || error) exit 3}
    ' .gitignore > "$out" || { rm -f -- "$out"; die '.gitignore 标记结构错误'; }
  else
    { if [[ -f .gitignore ]]; then cat .gitignore; printf '\n'; fi
      printf '%s\n' "$IGNORE_START"; cat "$RULES"; printf '%s\n' "$IGNORE_END"; } > "$out"
  fi
  if [[ -f .gitignore ]] && cmp -s "$out" .gitignore; then rm -f -- "$out"
  else mv -- "$out" .gitignore; fi
}

untrack_generated() {
  local -a paths=()
  mapfile -d '' -t paths < <(git ls-files -c -i --exclude-from="$RULES" -z)
  if ((${#paths[@]})); then
    say "从 Git 索引取消跟踪 ${#paths[@]} 个编译产物（本地文件保留）"
    printf '%s\0' "${paths[@]}" | xargs -0 -r git rm --cached -q --
  fi
}

# 删除构建目录前，拒绝删除其中任何已被跟踪的文件。
clean_build() {
  local -a dirs=() tracked=()
  local d answer
  mapfile -d '' -t dirs < <(find . -path './.git' -prune -o -type d \( \
    -name build -o -name install -o -name log -o -name devel -o -name Debug \
    -o -name Release -o -name .pio -o -name CMakeFiles -o -name __pycache__ \
    \) -print0 -prune)
  ((${#dirs[@]})) || { say '没有需要清理的构建目录'; return; }
  for d in "${dirs[@]}"; do
    [[ "$d" == ./* && "$d" != './.git' && -d "$d" && ! -L "$d" ]] || die "不安全目录：$d"
    mapfile -d '' -t tracked < <(git ls-files -z -- "${d#./}/")
    if ((${#tracked[@]})); then
      warn "拒绝删除 $d，因为其中有 ${#tracked[@]} 个被 Git 跟踪的文件（例：${tracked[0]}）"
      die '请手动检查目录；未删除任何构建目录'
    fi
  done
  say '将永久删除以下【没有被 Git 跟踪的文件】的目录：'
  printf '  %s\n' "${dirs[@]}"
  [[ -t 0 ]] || die '清理构建目录需要交互终端'
  IFS= read -r -p '输入 DELETE BUILD 确认永久删除：' answer || die '未收到确认'
  [[ "$answer" == 'DELETE BUILD' ]] || die '已取消构建目录清理'
  for d in "${dirs[@]}"; do rm -rf -- "$d"; done
  say '构建目录清理完成'
}

flatten_nested() {
  local -a nested=()
  local g answer backup rel
  mapfile -d '' -t nested < <(find . \( -path './.git' -o -type d \( \
    -name build -o -name install -o -name log -o -name Debug -o -name Release \
    \) \) -prune -o -name .git -print0)
  ((${#nested[@]})) || return 0
  say '发现嵌套 .git：'
  printf '  %s\n' "${nested[@]}"
  [[ "$NESTED" == true ]] || die '请先核查嵌套仓库；如确实需要合并使用 --include-nested'
  for g in "${nested[@]}"; do [[ -d "$g" && ! -L "$g" ]] || die "不处理链接或文件形式的 .git：$g"; done
  [[ -t 0 ]] || die '嵌套仓库处理必须交互确认'
  IFS= read -r -p '输入 FLATTEN NESTED 确认备份并移走嵌套 .git：' answer || die '未收到确认'
  [[ "$answer" == 'FLATTEN NESTED' ]] || die '取消嵌套仓库处理'
  backup="$HOME/car_git_metadata_backup_$(date '+%Y%m%d_%H%M%S')_$$"
  for g in "${nested[@]}"; do
    rel="${g#./}"; rel="${rel%/.git}"
    mkdir -p "$backup/$rel"
    mv -- "$g" "$backup/$rel/.git"
    git rm -r --cached --ignore-unmatch -- "$rel" >/dev/null || true
  done
  say "嵌套仓库元数据已备份：$backup"
}

write_commit() {
  local line
  if [[ -z "$TITLE" ]]; then
    [[ -t 0 ]] || die '存在未提交更改，请带 --message "标题" 使用'
    git --no-pager diff --cached --stat
    while :; do
      IFS= read -r -p '请输入本次提交标题：' TITLE || die '未收到标题'
      [[ "$TITLE" =~ [^[:space:]] ]] && break
    done
    say '可输入多行详细说明；单独输入 END 结束：'
    while IFS= read -r line; do
      [[ "$line" == END ]] && break
      DETAILS+="${DETAILS:+$'\n'}$line"
    done
  fi
  if [[ -n "$DETAILS" ]]; then git commit -m "$TITLE" -m "$DETAILS"
  else git commit -m "$TITLE"; fi
}

# STM32 独立裸仓库缓存：不会在 car 内嵌套 Git。
stm32_git() {
  if [[ -n "$HTTP_VERSION" ]]; then
    git -c "http.version=$HTTP_VERSION" --git-dir="$STM32_CACHE" "$@"
  else
    git --git-dir="$STM32_CACHE" "$@"
  fi
}
stm32_fetch() {
  mkdir -p "$(dirname "$STM32_CACHE")"
  if [[ ! -e "$STM32_CACHE" ]]; then
    git init --bare -q "$STM32_CACHE"
    stm32_git remote add origin "$STM32_URL"
  fi
  [[ "$(stm32_git rev-parse --is-bare-repository 2>/dev/null)" == true ]] || die 'STM32 缓存不是裸仓库'
  [[ "$(stm32_git remote get-url origin)" == "$STM32_URL" ]] || die 'STM32 缓存 origin 不匹配'
  stm32_git fetch --no-tags origin "+refs/heads/$STM32_BRANCH:refs/remotes/origin/$STM32_BRANCH" || die '队友分支下载失败'
  NEW_SHA="$(stm32_git rev-parse "refs/remotes/origin/${STM32_BRANCH}^{commit}")"
  OLD_SHA=''
  if [[ -f "$STATE_FILE" ]]; then
    IFS= read -r OLD_SHA < "$STATE_FILE" || true
    [[ "$OLD_SHA" =~ ^[0-9a-f]{40,64}$ ]] || die 'STM32 导入记录无效'
    stm32_git cat-file -e "${OLD_SHA}^{commit}" || die 'STM32 缓存缺少上次导入的提交'
    stm32_git merge-base --is-ancestor "$OLD_SHA" "$NEW_SHA" || die '队友分支历史已改写；拒绝自动合并'
  fi
}
stm32_report() {
  say "队友最新：$NEW_SHA"
  if [[ -z "$OLD_SHA" ]]; then
    say '首次导入：尚未记录上次版本；最近 12 次提交：'
    stm32_git --no-pager log -n 12 --format='%h %as %s' "$NEW_SHA"
  elif [[ "$OLD_SHA" == "$NEW_SHA" ]]; then
    say '已经是最新导入版本'
  else
    say "上次导入：$OLD_SHA"
    stm32_git --no-pager log --format='%h %as %s' "$OLD_SHA..$NEW_SHA"
  fi
}
stm32_check_tree() {
  if stm32_git ls-tree -r "$1" | awk '$1=="160000" || $1=="120000" {bad=1; print} END{exit !bad}'; then
    die '队友版本包含子模块/符号链接，自动导入不安全'
  fi
}
stm32_archive() {
  mkdir -p "$2"
  stm32_git archive --format=tar "$1" | tar -xf - -C "$2"
}
file_equal() {
  local a="$1" b="$2"
  if [[ ! -e "$a" && ! -e "$b" ]]; then return 0; fi
  [[ -f "$a" && -f "$b" && ! -L "$a" && ! -L "$b" ]] || return 1
  cmp -s -- "$a" "$b" || return 1
  if [[ -x "$a" ]]; then [[ -x "$b" ]]; else [[ ! -x "$b" ]]; fi
}
copy_file() {
  if [[ -f "$1" && ! -L "$1" ]]; then mkdir -p "$(dirname "$2")"; cp -p -- "$1" "$2"
  elif [[ ! -e "$1" ]]; then rm -f -- "$2"
  else die "非普通文件：$1"; fi
}
stm32_update() {
  local answer note commit tag rel old new current result rc n=0
  local -a paths=() conflicts=()
  require_main
  require_clean
  [[ ! -e STM32F4/.git ]] || die 'STM32F4 仍包含嵌套 .git'
  if git ls-files --stage -- STM32F4 | awk '$1=="160000" {bad=1} END{exit !bad}'; then
    die 'STM32F4 仍是 Git 子模块引用；请先人工展开或处理'
  fi
  if [[ -n "$OLD_SHA" && "$OLD_SHA" == "$NEW_SHA" ]]; then say '无新版本可导入'; return; fi
  stm32_check_tree "$NEW_SHA"
  [[ -z "$OLD_SHA" ]] || stm32_check_tree "$OLD_SHA"
  command -v tar >/dev/null || die '缺少 tar'
  new_stage
  stm32_archive "$NEW_SHA" "$TMP/upstream-new"
  if [[ -n "$OLD_SHA" ]]; then stm32_archive "$OLD_SHA" "$TMP/upstream-old"; fi
  mkdir -p "$STAGE/STM32F4"
  if [[ -z "$OLD_SHA" ]]; then
    say '首次导入：将用队友的【整个仓库快照】替换 STM32F4 的已跟踪文件（不只是固件目录）。'
    diff -qr --exclude=.git "$STAGE/STM32F4" "$TMP/upstream-new" 2>/dev/null | head -n 80 || true
    [[ -t 0 ]] || die '首次导入需要终端交互确认'
    IFS= read -r -p '确认首次整体替换？输入 ADOPT STM32：' answer || die '未收到确认'
    [[ "$answer" == 'ADOPT STM32' ]] || { say '已取消，主项目不变'; close_stage; return; }
    git -C "$STAGE" rm -r -q --ignore-unmatch -- STM32F4
    mkdir -p "$STAGE/STM32F4"
    cp -a "$TMP/upstream-new/." "$STAGE/STM32F4/"
  else
    mapfile -d '' -t paths < <(stm32_git diff --no-renames --name-only -z "$OLD_SHA" "$NEW_SHA")
    [[ -z "$(find "$STAGE/STM32F4" -type l -print -quit)" ]] || die 'STM32F4 内含符号链接，拒绝合并'
    for rel in "${paths[@]}"; do
      old="$TMP/upstream-old/$rel"; new="$TMP/upstream-new/$rel"; current="$STAGE/STM32F4/$rel"
      if [[ -L "$current" || ( -e "$current" && ! -f "$current" ) ]]; then
        conflicts+=("$rel：本地非普通文件")
      elif file_equal "$current" "$old"; then
        copy_file "$new" "$current"
      elif file_equal "$current" "$new"; then
        :
      elif [[ -f "$old" && -f "$new" && -f "$current" ]]; then
        n=$((n+1)); result="$TMP/merged-$n"; rc=0
        git merge-file -p -- "$current" "$old" "$new" > "$result" || rc=$?
        if ((rc == 0)); then
          chmod --reference="$current" "$result"
          copy_file "$result" "$current"
        else conflicts+=("$rel：文本冲突或二进制文件"); fi
      else conflicts+=("$rel：新增/删除冲突"); fi
    done
    if ((${#conflicts[@]})); then
      printf 'STM32 合并冲突（仅存在于临时工作区）：\n' >&2
      printf '  %s\n' "${conflicts[@]}" >&2
      die '主项目未被修改；请手动协调之后重试'
    fi
    git -C "$STAGE" --no-pager diff --stat -- STM32F4
    [[ -t 0 ]] || die 'STM32 合并需要终端交互确认'
    IFS= read -r -p '确认增量导入？输入 MERGE STM32：' answer || die '未收到确认'
    [[ "$answer" == 'MERGE STM32' ]] || { say '已取消，主项目不变'; close_stage; return; }
  fi
  mkdir -p "$STAGE/.car-sync"
  printf '%s\n' "$NEW_SHA" > "$STAGE/$STATE_FILE"
  git -C "$STAGE" add -A -- STM32F4 "$STATE_FILE"
  say '本次 STM32 变动：'
  git -C "$STAGE" --no-pager diff --cached --stat
  [[ -t 0 ]] || die '创建导入提交需要交互终端'
  IFS= read -r -p '导入备注（可留空）：' note || die '未收到备注'
  git -C "$STAGE" commit -m "stm32: integrate upstream ${NEW_SHA:0:12}" \
    -m "来源：$STM32_URL
分支：$STM32_BRANCH
提交：$NEW_SHA
上次导入：${OLD_SHA:-首次导入}
备注：$note" || die 'STM32 临时工作区提交失败'
  commit="$(git -C "$STAGE" rev-parse HEAD)"
  tag="backup/car-stm32-$(date +%Y%m%d-%H%M%S)-$$"
  git tag -a "$tag" "$BASE" -m 'before STM32 integration'
  promote_stage "$commit"
  close_stage
  say "STM32 本地导入成功：$NEW_SHA"
  say "备份标签：$tag（未推送 GitHub，请先检查编译和实机）"
}

# 只读命令先退出，避免无意修改仓库。
case "$MODE" in
  help) usage; exit 0 ;;
  doctor)
    say "Git：$(git --version)"
    say "仓库：$ROOT"
    say "分支：$(git branch --show-current)"
    say "HEAD：$(git rev-parse --short HEAD 2>/dev/null || echo 无提交)"
    say "origin：$(git remote get-url origin 2>/dev/null || echo 未配置)"
    git status -sb
    exit 0 ;;
  preview)
    say '=== 本地状态 ==='; git status -sb
    say '=== 已跟踪文件修改/删除 ==='; git --no-pager diff --stat HEAD || true
    say '=== 暂存区 ==='; git --no-pager diff --cached --stat || true
    say '=== 待删除的已跟踪文件 ==='
    git -c core.quotePath=false --no-pager diff --name-only --no-renames --diff-filter=D HEAD || true
    say '注意：未跟踪文件在 git status 中显示，不会出现在以上差异统计中。'
    exit 0 ;;
  history)
    git --no-pager log -n "$HISTORY" --date=short --pretty=format:'%h  %ad  %s%n%b%n----'
    printf '\n'; exit 0 ;;
  show)
    [[ "$REF" != -* ]] || die '非法版本参数'
    refsha="$(git rev-parse --verify --end-of-options "${REF}^{commit}" 2>/dev/null)" || die '找不到此版本'
    show_commit "$refsha"; exit 0 ;;
  backups)
    git for-each-ref --sort=-creatordate --format='%(refname:short) %(objectname:short)' 'refs/tags/backup/car-*'
    exit 0 ;;
esac

# 仅写操作要求主分支和严格 origin。
require_main
check_origin

case "$MODE" in
  check-remote) check_remote; exit $? ;;
  retry-push)
    [[ -z "$(git status --porcelain --untracked-files=normal)" ]] || warn '存在未提交文件；只推送已有提交'
    push_verified; say '补推完成'; exit 0 ;;
  check-stm32|stm32-diff|update-stm32)
    stm32_fetch; stm32_report
    case "$MODE" in
      check-stm32) exit 0 ;;
      stm32-diff)
        if [[ -n "$OLD_SHA" ]]; then stm32_git --no-pager diff "$OLD_SHA" "$NEW_SHA"
        else say '首次导入尚无比较基线'; stm32_git --no-pager ls-tree -r --name-only "$NEW_SHA" | head -n 100; fi
        exit 0 ;;
      update-stm32) stm32_update; exit 0 ;;
    esac ;;
  rollback)
    require_clean
    [[ "$REF" != -* ]] || die '非法版本参数'
    target="$(git rev-parse --verify --end-of-options "${REF}^{commit}" 2>/dev/null)" || die '找不到要回溯的版本'
    show_commit "$target"
    # 远程合并 + 历史恢复均只在临时 worktree 中操作，直到用户确认才更新 main。
    fetch_main
    new_stage
    if [[ "$FETCH_HAS_REMOTE" == true ]] && ! git merge-base --is-ancestor refs/remotes/origin/main "$BASE"; then
      mkdir -p "$LOG_DIR"
      if ! git -C "$STAGE" merge --no-edit --allow-unrelated-histories refs/remotes/origin/main >"$LOG_DIR/last-merge.log" 2>&1; then
        warn '远程合并冲突，回溯未触碰主工作区'
        git -C "$STAGE" diff --name-only --diff-filter=U >&2 || true
        die "详情：$LOG_DIR/last-merge.log"
      fi
    fi
    current_staged="$(git -C "$STAGE" rev-parse HEAD)"
    git merge-base --is-ancestor "$target" "$current_staged" || die '目标版本不属于当前 main 与远程合并后的历史'
    if git ls-tree -r "$target" | awk '$1=="160000" {bad=1} END{exit !bad}'; then die '目标版本含 Git 子模块，拒绝恢复'; fi
    review_deletions range "$current_staged" "$target"
    [[ -t 0 ]] || die '回溯需要终端确认'
    IFS= read -r -p '确认回溯？输入 ROLLBACK：' answer || die '未收到确认'
    [[ "$answer" == ROLLBACK ]] || { say '已取消回溯，主项目不变'; close_stage; exit 0; }
    git -C "$STAGE" restore --source="$target" --staged --worktree -- . || die '临时工作区恢复失败'
    (cd "$STAGE" && git add -A)
    if ! git -C "$STAGE" diff --cached --quiet; then
      git -C "$STAGE" commit -m "rollback: restore ${target:0:12}" -m "恢复历史快照：$target" || die '生成回溯提交失败'
    fi
    commit="$(git -C "$STAGE" rev-parse HEAD)"
    if [[ "$commit" == "$BASE" ]]; then
      say '当前版本已与目标一致，没有创建新提交'; close_stage; exit 0
    fi
    tag="backup/car-rollback-$(date +%Y%m%d-%H%M%S)-$$"
    git tag -a "$tag" "$BASE" -m "before rollback to $target"
    promote_stage "$commit"
    close_stage
    say "回溯已在本地创建新提交，原版本备份：$tag"
    push_verified
    say '回溯已推送并核实'; exit 0 ;;
esac

# 默认同步：先处理本地变更，然后在临时 worktree 合并远程。
flatten_nested
if [[ -n "$(git ls-files --stage | awk '$1=="160000" {print}')" ]]; then
  die '检测到未展开的 Git 子模块引用；请先人工处理'
fi
if [[ "$DELETE_BUILD" == true ]]; then clean_build; fi
review_deletions worktree
make_rules
ensure_ignore
untrack_generated
git add -A
if ! git diff --cached --quiet; then
  review_deletions stage
  write_commit
else
  say '没有新的本地更改'
fi
require_clean
safe_merge_remote
push_verified
say '同步完成：已核实本地 HEAD 与远端 main 一致'
git status -sb
