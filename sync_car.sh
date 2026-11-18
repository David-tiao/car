#!/usr/bin/env bash
# car 工程：交互式提交说明、安全同步、嵌套仓库合并、忽略编译产物和版本回溯
# 依赖：Ubuntu/Linux + Git 2.34+；默认 ~/Desktop/car / origin/main
set -Eeuo pipefail

REPO_DIR="${CAR_DIR:-$HOME/Desktop/car}"
REMOTE_URL="${CAR_REMOTE_URL:-https://github.com/David-tiao/car.git}"
MODE=sync
FLATTEN_NESTED=false
DELETE_BUILD=false
HISTORY_COUNT=15
ROLLBACK_REF=''
SHOW_REF=''
COMMIT_TITLE=''
COMMIT_DETAILS=''

usage() {
  cat <<'USAGE'
用法：
  bash sync_car.sh                         # 日常同步，有变更时让你填写提交标题和详细说明
  bash sync_car.sh --message "标题"          # 不进入交互，指定提交标题
  bash sync_car.sh --message "标题" --details "详细说明"  # 同时指定提交正文
  bash sync_car.sh --include-nested        # 首次合并嵌套 .git 到 car 主仓库
  bash sync_car.sh --delete-build          # 询问后删除本地编译目录再同步
  bash sync_car.sh --history [N]           # 查看最近 N 个提交的标题和详细说明
  bash sync_car.sh --show <提交ID>          # 查看某个版本的说明和变更文件统计
  bash sync_car.sh --backups               # 查看回溯时创建的安全备份标签
  bash sync_car.sh --rollback <提交ID>      # 按指定旧提交恢复项目，生成新提交并推送
  bash sync_car.sh --help

回溯示例：
  bash sync_car.sh --history 20
  bash sync_car.sh --rollback a1b2c3d
  bash sync_car.sh --show a1b2c3d
  bash sync_car.sh --rollback HEAD~2

说明：
  提交说明写入 Git commit 标题和正文，并随推送同步到 GitHub；--history 可查看。
  如有本地代码修改，交互模式下标题必填，详细说明可填写多行。
  回溯采用“恢复文件快照 + 新增回溯提交”，不改写 Git 历史，不强推。
  回溯前要求工作区干净；自动创建 backup/car-rollback-* 本地标签。
  默认只忽略编译产物；--delete-build 才会删除本地编译目录。
  可用 CAR_DIR、CAR_REMOTE_URL 环境变量覆盖路径/远程地址。
USAGE
}

die() { printf '错误：%s\n' "$*" >&2; exit 1; }

while (( $# > 0 )); do
  case "$1" in
    --include-nested)
      [[ "$MODE" == sync ]] || die '--include-nested 不能与查看/回溯操作组合'
      FLATTEN_NESTED=true; shift ;;
    --delete-build)
      [[ "$MODE" == sync ]] || die '--delete-build 不能与查看/回溯操作组合'
      DELETE_BUILD=true; shift ;;
    --message)
      (( $# >= 2 )) || die '--message 后需要填写标题'
      COMMIT_TITLE="$2"; shift 2 ;;
    --details)
      (( $# >= 2 )) || die '--details 后需要填写说明'
      COMMIT_DETAILS="$2"; shift 2 ;;
    --show)
      [[ "$MODE" == sync && "$FLATTEN_NESTED" == false && "$DELETE_BUILD" == false ]] || die '查看版本不能与其他操作组合'
      (( $# >= 2 )) || die '--show 后需要填写提交 ID'
      MODE=show; SHOW_REF="$2"; shift 2 ;;
    --history)
      [[ "$MODE" == sync && "$FLATTEN_NESTED" == false && "$DELETE_BUILD" == false ]] || die '查看历史不能与其他操作组合'
      MODE=history; shift
      if (( $# > 0 )) && [[ "$1" =~ ^[0-9]+$ ]]; then HISTORY_COUNT="$1"; shift; fi
      (( HISTORY_COUNT >= 1 && HISTORY_COUNT <= 200 )) || die '历史条数须在 1～200 之间' ;;
    --backups)
      [[ "$MODE" == sync && "$FLATTEN_NESTED" == false && "$DELETE_BUILD" == false ]] || die '查看备份不能与其他操作组合'
      MODE=backups; shift ;;
    --rollback)
      [[ "$MODE" == sync && "$FLATTEN_NESTED" == false && "$DELETE_BUILD" == false ]] || die '回溯不能与其他操作组合'
      (( $# >= 2 )) || die '--rollback 后需要填写提交 ID，例如 --rollback HEAD~1'
      MODE=rollback; ROLLBACK_REF="$2"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) die "不支持的参数：$1（执行 --help 查看帮助）" ;;
  esac
done

if [[ "$MODE" != sync && ( -n "$COMMIT_TITLE" || -n "$COMMIT_DETAILS" ) ]]; then
  die '--message / --details 只能用于日常同步'
fi
if [[ -n "$COMMIT_DETAILS" && -z "$COMMIT_TITLE" ]]; then
  die '--details 需与 --message 一起使用；交互模式直接按提示填写即可'
fi
if [[ -n "$COMMIT_TITLE" && ! "$COMMIT_TITLE" =~ [^[:space:]] ]]; then
  die '提交标题不能只有空白字符'
fi

command -v git >/dev/null 2>&1 || die '未安装 git'
[[ -d "$REPO_DIR" ]] || die "找不到项目目录：$REPO_DIR"
cd "$REPO_DIR"
if [[ ! -e .git ]]; then
  [[ "$MODE" == sync ]] || die '尚无 Git 仓库，无法查看或回溯版本'
  git init -b main
fi
ROOT="$(git rev-parse --show-toplevel 2>/dev/null)" || die '无法确定 Git 仓库根目录'
[[ "$ROOT" == "$(pwd -P)" ]] || die "当前目录不是仓库根目录，实际根目录是 $ROOT"

show_commit_description() {
  local ref="$1"
  git --no-pager show --no-patch --date=format:'%Y-%m-%d %H:%M:%S' \
    --pretty=format:'版本：%h%d%n时间：%ad%n作者：%an%n标题：%s%n%n%b%n' "$ref"
  printf '\n文件变动统计：\n'
  git --no-pager show --format= --stat "$ref"
  printf '\n'
}

if [[ "$MODE" == show ]]; then
  [[ "$SHOW_REF" != -* ]] || die '版本参数不能以 - 开头'
  SHA="$(git rev-parse --verify --end-of-options "${SHOW_REF}^{commit}" 2>/dev/null)" || die "未找到版本：$SHOW_REF"
  show_commit_description "$SHA"
  exit 0
fi

if [[ "$MODE" == history ]]; then
  git rev-parse --verify HEAD >/dev/null 2>&1 || die '仓库尚无提交历史'
  echo "仓库：$ROOT"
  echo "最近 ${HISTORY_COUNT} 条提交（含你填写的说明；版本 ID 可用于 --show / --rollback）："
  git --no-pager log -n "$HISTORY_COUNT" --date=format:'%Y-%m-%d %H:%M' \
    --pretty=format:'版本：%h%d%n时间：%ad%n标题：%s%n%b%n--------------------------------------------------'
  printf '\n\n可执行：bash sync_car.sh --show <提交ID> 查看文件变化，或 --rollback <提交ID> 恢复。\n'
  exit 0
fi

if [[ "$MODE" == backups ]]; then
  echo '回溯前的本地安全备份标签（最新在前）：'
  git for-each-ref --sort=-creatordate --format='%(refname:short)  %(objectname:short)  %(creatordate:short)' 'refs/tags/backup/car-rollback-*'
  echo '需要撤销某次回溯时，可使用：bash sync_car.sh --rollback <备份标签>'
  exit 0
fi

[[ -z "$(git ls-files -u)" ]] || die '发现未解决的合并冲突，请先修复'
[[ ! -f .git/MERGE_HEAD && ! -d .git/rebase-merge && ! -d .git/rebase-apply ]] || die '当前有未结束的合并或变基操作'
BRANCH="$(git symbolic-ref --quiet --short HEAD || true)"
[[ "$BRANCH" == main ]] || die "当前分支为 ${BRANCH:-分离 HEAD}；本脚本仅操作 main"

# 查看与回溯都无需更改远程地址；同步时仅在缺少 origin 时创建。
OLD_URL="$(git remote get-url origin 2>/dev/null || true)"
if [[ -z "$OLD_URL" ]]; then
  git remote add origin "$REMOTE_URL"
elif [[ "$OLD_URL" != "$REMOTE_URL" ]]; then
  die "origin 指向 $OLD_URL，与预期 $REMOTE_URL 不一致。请先核对远程仓库"
fi

# 回溯前先解析用户选中的版本（避免同步远端后 HEAD~N 含义变化）。
if [[ "$MODE" == rollback ]]; then
  git rev-parse --verify HEAD >/dev/null 2>&1 || die '当前仓库没有历史提交'
  [[ -z "$(git status --porcelain --untracked-files=normal)" ]] || die '工作区存在未提交或未跟踪文件：请先提交/备份（可先正常同步）'
  [[ "$ROLLBACK_REF" != -* ]] || die '版本参数不能以 - 开头'
  TARGET_SHA="$(git rev-parse --verify --end-of-options "${ROLLBACK_REF}^{commit}" 2>/dev/null)" || die "找不到提交版本：$ROLLBACK_REF"
fi

# 对回溯和普通同步使用同一套忽略规则。
IGNORE_BEGIN='# >>> car sync: generated files >>>'
IGNORE_END='# <<< car sync: generated files <<<' 
RULE_FILE="$(mktemp)"
trap 'rm -f -- "$RULE_FILE"' EXIT
cat > "$RULE_FILE" <<'RULES'
# ROS2 / colcon / catkin
**/build/
**/install/
**/log/
**/devel/
# STM32 / CMake / PlatformIO
**/Debug/
**/Release/
**/.pio/
**/CMakeFiles/
**/cmake-build-*/
# Python / OpenMV 缓存及编译中间文件
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
RULES

ensure_ignore_rules() {
  if ! grep -Fqx "$IGNORE_BEGIN" .gitignore 2>/dev/null; then
    {
      [[ ! -f .gitignore ]] || printf '\n'
      printf '%s\n' "$IGNORE_BEGIN"
      cat "$RULE_FILE"
      printf '%s\n' "$IGNORE_END"
    } >> .gitignore
    echo '已维护 .gitignore 中的编译产物忽略规则。'
  fi
}

untrack_generated() {
  local -a generated=()
  mapfile -d '' -t generated < <(git ls-files -c -i --exclude-from="$RULE_FILE" -z)
  if (( ${#generated[@]} > 0 )); then
    echo "从 Git 索引移除 ${#generated[@]} 个编译产物（不删除本地文件）。"
    printf '%s\0' "${generated[@]}" | xargs -0 -r git rm --cached -q --
  fi
}

merge_remote_main() {
  echo '读取远程 GitHub 最新提交...'
  git fetch origin
  if git show-ref --verify --quiet refs/remotes/origin/main; then
    if ! git merge --no-edit --allow-unrelated-histories origin/main; then
      die '远程合并发生冲突；已停止推送。请执行 git status，解决冲突后提交。'
    fi
  fi
}

if [[ "$MODE" == rollback ]]; then
  echo '你选中的历史版本信息：'
  show_commit_description "$TARGET_SHA"
  printf '\n当前版本与目标版本的差异统计：\n'
  git --no-pager diff --stat "$TARGET_SHA" HEAD
  echo '回溯会将当前跟踪文件恢复为该版本（保留忽略的本地编译目录），并生成一条新的提交。'
  read -r -p '确认回溯并推送 GitHub？请输入 ROLLBACK：' ANSWER
  [[ "$ANSWER" == ROLLBACK ]] || { echo '已取消回溯。'; exit 0; }

  merge_remote_main
  git merge-base --is-ancestor "$TARGET_SHA" HEAD || die '所选版本不属于当前 main 的历史；为安全起见拒绝回溯'
  # 禁止把历史 gitlink/子模块快照恢复为不完整的源码目录。
  if git ls-tree -r "$TARGET_SHA" | awk '$1 == "160000" {found=1} END {exit !found}'; then
    die '目标版本包含嵌套子仓库（gitlink），不能保证完整源码，请选择其他版本'
  fi
  [[ -z "$(git status --porcelain --untracked-files=normal)" ]] || die '远程合并后工作区不干净，已停止回溯'

  BACKUP_TAG="backup/car-rollback-$(date '+%Y%m%d-%H%M%S')-$$"
  git tag -a "$BACKUP_TAG" -m "car snapshot before rollback to $TARGET_SHA"
  echo "已为当前版本创建本地安全备份标签：$BACKUP_TAG"

  # --source 指定目标提交，--staged --worktree 恢复索引和工作目录。
  # 不用 reset --hard，不改写提交历史；忽略文件（例如 ROS2 install）保留。
  if ! git restore --source="$TARGET_SHA" --staged --worktree -- .; then
    die "恢复文件失败，原版本可通过标签 $BACKUP_TAG 找回"
  fi
  ensure_ignore_rules
  untrack_generated
  git add -A
  if git diff --cached --quiet; then
    echo '当前源码与目标版本一致，无需生成回溯提交。'
  else
    echo '可选：填写此次回溯原因，方便日后查看历史。'
    read -r -p '回溯原因（可直接回车）：' REASON
    if [[ -n "$REASON" ]]; then
      git commit -m "rollback: restore snapshot ${TARGET_SHA:0:12}" \
        -m "目标版本：${TARGET_SHA}
回溯原因：${REASON}"
    else
      git commit -m "rollback: restore snapshot ${TARGET_SHA:0:12}" \
        -m "目标版本：${TARGET_SHA}"
    fi
  fi
  echo '将回溯后的新提交推送到 GitHub（不强推）...'
  git push -u origin main
  echo
  echo "回溯完成。原版本备份：$BACKUP_TAG"
  echo "如需撤销回溯：bash sync_car.sh --rollback $BACKUP_TAG"
  git status -sb
  exit 0
fi

# 以下为原日常同步流程。
mapfile -d '' -t NESTED_GIT < <(
  find . \( -path './.git' -o -type d \( \
      -name build -o -name install -o -name log -o -name devel \
      -o -name Debug -o -name Release -o -name .pio \
    \) \) -prune -o -name '.git' -print0
)
if (( ${#NESTED_GIT[@]} > 0 )); then
  echo '检测到嵌套 Git 仓库：'
  for g in "${NESTED_GIT[@]}"; do echo "  ${g%/.git}"; done
  [[ "$FLATTEN_NESTED" == true ]] || die '首次需要 --include-nested 才能将嵌套仓库源码完整纳入主仓库'
  for g in "${NESTED_GIT[@]}"; do
    [[ -d "$g" ]] || die "发现 .git 文件（可能是 worktree / 子模块），为防止破坏引用停止：$g"
  done
  BACKUP="$HOME/car_git_metadata_backup_$(date +%Y%m%d_%H%M%S)_$$"
  echo "备份嵌套 Git 元数据至 $BACKUP"
  for g in "${NESTED_GIT[@]}"; do
    rel="${g#./}"
    project="${rel%/.git}"
    mkdir -p "$BACKUP/$project"
    mv -- "$g" "$BACKUP/$project/.git"
    git rm -r --cached --ignore-unmatch -- "$project" >/dev/null
  done
  echo '嵌套 Git 元数据已备份，源代码未删除。'
fi
GITLINKS="$(git ls-files --stage | awk '$1 == "160000" { print }')"
[[ -z "$GITLINKS" ]] || die "Git 索引仍有子仓库引用：$GITLINKS"

ensure_ignore_rules
if [[ "$DELETE_BUILD" == true ]]; then
  mapfile -d '' -t BUILD_DIRS < <(
    find . -path './.git' -prune -o -type d \( \
      -name build -o -name install -o -name log -o -name devel \
      -o -name Debug -o -name Release -o -name .pio \
      -o -name CMakeFiles -o -name '__pycache__' \
    \) -print0 -prune
  )
  if (( ${#BUILD_DIRS[@]} > 0 )); then
    echo '下列本地目录将被物理删除（请核查它们是否确实为构建目录）：'
    printf '  %s\n' "${BUILD_DIRS[@]}"
    read -r -p '确认永久删除这些目录？请输入 YES：' ANSWER
    [[ "$ANSWER" == YES ]] || { echo '已取消删除和同步。'; exit 0; }
    for d in "${BUILD_DIRS[@]}"; do
      [[ "$d" == ./* && "$d" != './.git' && -d "$d" && ! -L "$d" ]] || die "不安全的删除路径：$d"
      rm -rf -- "$d"
    done
    echo '本地编译目录已删除。'
  fi
fi

# 所有真实的本地代码变更都先要求你填写说明，再创建 Git 提交。
write_user_commit() {
  local line
  if [[ -z "$COMMIT_TITLE" ]]; then
    [[ -t 0 ]] || die '检测到需要提交的变更，但当前没有交互终端；请使用 --message "标题" [--details "说明"]'
    echo
    echo '检测到本地修改，请为此次版本填写说明（会保存到 GitHub 提交记录）。'
    echo '待提交文件统计：'
    git --no-pager diff --cached --stat
    while :; do
      IFS= read -r -p '本次修改标题（必填，例如：修复 OpenMV 串口协议）：' COMMIT_TITLE || die '读取提交标题失败'
      [[ "$COMMIT_TITLE" =~ [^[:space:]] ]] && break
      echo '标题不能为空，请重新输入。'
    done
    echo '填写详细修改说明（可多行，每行写一条变化）；单独输入 END 结束，直接输入 END 可跳过。'
    while IFS= read -r line; do
      [[ "$line" == END ]] && break
      if [[ -z "$COMMIT_DETAILS" ]]; then
        COMMIT_DETAILS="$line"
      else
        COMMIT_DETAILS+=$'\n'"$line"
      fi
    done
  fi
  [[ "$COMMIT_TITLE" =~ [^[:space:]] ]] || die '提交标题不能为空'
  if [[ -n "$COMMIT_DETAILS" ]]; then
    git commit -m "$COMMIT_TITLE" -m "$COMMIT_DETAILS"
  else
    git commit -m "$COMMIT_TITLE"
  fi
}

untrack_generated
git add -A
if ! git diff --cached --quiet; then
  write_user_commit
elif ! git rev-parse --verify HEAD >/dev/null 2>&1; then
  echo '仓库尚无提交，创建初始空提交。'
  git commit --allow-empty -m 'initialize car repository'
else
  echo '没有新的本地修改。'
fi

merge_remote_main
untrack_generated
if ! git diff --cached --quiet; then
  git commit -m 'chore: remove tracked build artifacts after merge'
fi

echo '推送到 GitHub...'
git push -u origin main
printf '\n同步完成！本地编译目录默认保留；--history 查看提交说明，--show 查看详情，--rollback 回溯。\n'
git status -sb
echo '注意：旧提交中的大文件不会自动从 Git 历史清除。'
