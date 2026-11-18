#!/usr/bin/env bash
# car 工程：提交说明、版本回溯、编译产物排除、手动检视/合并队友 STM32 固件
# 依赖：Ubuntu/Linux + Git 2.34+；默认以本脚本所在目录为仓库 / origin/main
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
REPO_DIR="${CAR_DIR:-$SCRIPT_DIR}"
REMOTE_URL="${CAR_REMOTE_URL:-https://github.com/David-tiao/car.git}"
MODE=sync
FLATTEN_NESTED=false
DELETE_BUILD=false
HISTORY_COUNT=15
ROLLBACK_REF=''
SHOW_REF=''
COMMIT_TITLE=''
COMMIT_DETAILS=''
STM32_URL="${CAR_STM32_URL:-https://github.com/Wangdachang888/GongChuangSai.git}"
STM32_BRANCH="${CAR_STM32_BRANCH:-refactor/architecture-v2}"
STM32_CACHE="${CAR_STM32_CACHE:-$HOME/.cache/car-sync/stm32-upstream.git}"
STM32_STATE_REL='.car-sync/stm32-upstream.sha'
STM32_NEW_SHA=''
STM32_OLD_SHA=''

usage() {
  cat <<'USAGE'
用法（脚本放在 car 根目录，先执行 cd ~/Desktop/car）：
  bash sync_car.sh                         # 日常同步，有变更时让你填写提交标题和详细说明
  bash sync_car.sh --message "标题"          # 不进入交互，指定提交标题
  bash sync_car.sh --message "标题" --details "详细说明"  # 同时指定提交正文
  bash sync_car.sh --include-nested        # 首次合并嵌套 .git 到 car 主仓库
  bash sync_car.sh --delete-build          # 询问后删除本地编译目录再同步
  bash sync_car.sh --history [N]           # 查看最近 N 个提交的标题和详细说明
  bash sync_car.sh --show <提交ID>          # 查看某个版本的说明和变更文件统计
  bash sync_car.sh --backups               # 查看回溯及 STM32 合并前的安全备份标签
  bash sync_car.sh --rollback <提交ID>      # 按指定旧提交恢复项目，生成新提交并推送
  bash sync_car.sh --check-stm32          # 手动 fetch 队友开发分支，列出相对上次导入的新提交
  bash sync_car.sh --stm32-diff           # 查看队友自上次导入以来的文件及代码差异
  bash sync_car.sh --update-stm32         # 安全三方合并到 car/STM32F4，本地提交但不自动 push
  bash sync_car.sh --help

也可在任意目录运行：bash ~/Desktop/car/sync_car.sh [参数]

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
  队友仓库固定默认 Wangdachang888/GongChuangSai 的 refactor/architecture-v2 分支。
  --check-stm32/--stm32-diff 只更新 HOME 下的独立缓存，不改 car/STM32F4。
  --update-stm32 要求 car/main 工作区干净；先在临时 worktree 合并，成功后生成新提交。
  首次导入不是三方合并：需要明确输入 ADOPT STM32，替换旧目录中被 Git 跟踪的文件。
  之后仅对队友上次导入以来变动的文件做三方合并；冲突时不改 car。
  STM32 合并后请自己检查编译、实机，再运行普通 sync_car.sh 上传；不自动烧录。
  可用 CAR_DIR、CAR_REMOTE_URL、CAR_STM32_URL、CAR_STM32_BRANCH、CAR_STM32_CACHE 自定义。
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
    --check-stm32|--stm32-diff|--update-stm32)
      [[ "$MODE" == sync && "$FLATTEN_NESTED" == false && "$DELETE_BUILD" == false ]] || die 'STM32 操作不能与同步/回溯参数组合'
      case "$1" in
        --check-stm32) MODE=check_stm32 ;;
        --stm32-diff) MODE=diff_stm32 ;;
        --update-stm32) MODE=update_stm32 ;;
      esac
      shift ;;
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
  echo '回溯和 STM32 合并前的本地安全备份标签（最新在前）：'
  git for-each-ref --sort=-creatordate --format='%(refname:short)  %(objectname:short)  %(creatordate:short)' 'refs/tags/backup/car-rollback-*' 'refs/tags/backup/car-stm32-*'
  echo '需要撤销某次回溯时，可使用：bash sync_car.sh --rollback <备份标签>'
  exit 0
fi

# ========== 队友 STM32 的手动检查、差异、受控合并（独立于日常同步） ==========
stm32_git() { git --git-dir="$STM32_CACHE" "$@"; }

stm32_fetch() {
  command -v tar >/dev/null || die '需要 tar 才能安全读取 STM32 代码快照'
  mkdir -p "$(dirname "$STM32_CACHE")"
  if [[ ! -d "$STM32_CACHE" ]]; then
    git init --bare --quiet "$STM32_CACHE"
    stm32_git remote add origin "$STM32_URL"
  fi
  [[ "$(stm32_git rev-parse --is-bare-repository 2>/dev/null)" == true ]] || die "缓存目录不是裸仓库：$STM32_CACHE"
  local actual
  actual="$(stm32_git remote get-url origin 2>/dev/null || true)"
  [[ "$actual" == "$STM32_URL" ]] || die "队友镜像 origin 与预期不一致：$actual"
  echo "手动检查队友仓库：$STM32_URL ($STM32_BRANCH)"
  stm32_git fetch --no-tags origin "+refs/heads/$STM32_BRANCH:refs/remotes/origin/$STM32_BRANCH" || die '下载队友提交失败；未修改 car 项目'
  STM32_NEW_SHA="$(stm32_git rev-parse --verify "refs/remotes/origin/${STM32_BRANCH}^{commit}")" || die '找不到队友目标分支'
  STM32_OLD_SHA=''
  if [[ -e "$STM32_STATE_REL" ]]; then
    IFS= read -r STM32_OLD_SHA < "$STM32_STATE_REL" || true
    [[ "$STM32_OLD_SHA" =~ ^[0-9a-f]{40}$ ]] || die "上次导入记录无效：$STM32_STATE_REL"
    stm32_git cat-file -e "${STM32_OLD_SHA}^{commit}" 2>/dev/null || die '缓存中缺少上次导入的提交；请保留原记录并找回历史，不要直接重设版本'
    if ! stm32_git merge-base --is-ancestor "$STM32_OLD_SHA" "$STM32_NEW_SHA"; then
      die '队友分支发生历史改写/非快进，自动合并不安全。请先人工核查'
    fi
  fi
}

stm32_report() {
  echo "队友最新 SHA：$STM32_NEW_SHA"
  if [[ -z "$STM32_OLD_SHA" ]]; then
    echo '你的 car 尚未记录首次导入；以下为队友最近 15 次提交：'
    stm32_git --no-pager log -n 15 --date=short --format='%h %ad %s' "$STM32_NEW_SHA"
    echo '注意：首次导入会整体替换 car/STM32F4 里原先由 Git 跟踪的文件，需明确确认。'
  elif [[ "$STM32_OLD_SHA" == "$STM32_NEW_SHA" ]]; then
    echo '目前队友开发分支没有比已导入版本更新的提交。'
  else
    echo "上次导入 SHA：$STM32_OLD_SHA"
    echo '尚未导入的新提交（时间、摘要）：'
    stm32_git --no-pager log --date=short --format='%h %ad %s' "$STM32_OLD_SHA..$STM32_NEW_SHA"
  fi
}

stm32_no_unsafe_entries() {
  local sha="$1"
  # 裸仓库中不应该含外部 submodule gitlink 或符号链接；拒绝不完整内容/目录逃逸。
  if stm32_git ls-tree -r "$sha" | awk '$1=="160000" || $1=="120000" {bad=1; print; exit} END{exit !bad}'; then
    die '队友代码包含 Git 子模块或符号链接，当前合并器拒绝自动导入（请人工处理）'
  fi
}

stm32_archive_into() {
  local sha="$1" path="$2"
  mkdir -p "$path"
  stm32_git archive --format=tar "$sha" | tar -xf - -C "$path"
}

# 比较路径内容 + 可执行属性（兼容新增、删除）
stm32_same_path() {
  local left="$1" right="$2"
  if [[ ! -e "$left" && ! -e "$right" ]]; then return 0; fi
  [[ -f "$left" && -f "$right" && ! -L "$left" && ! -L "$right" ]] || return 1
  cmp -s -- "$left" "$right" || return 1
  if [[ -x "$left" ]]; then [[ -x "$right" ]]; else [[ ! -x "$right" ]]; fi
}

stm32_copy_path() {
  local source="$1" target="$2"
  if [[ -f "$source" && ! -L "$source" ]]; then
    mkdir -p "$(dirname "$target")"
    cp -p -- "$source" "$target"
  elif [[ ! -e "$source" ]]; then
    rm -f -- "$target"
  else
    die "非普通文件，不能复制：$source"
  fi
}

stm32_cleanup() {
  if [[ -n "${STM32_WORKTREE:-}" && -d "$STM32_WORKTREE" ]]; then
    git -C "$ROOT" worktree remove --force "$STM32_WORKTREE" >/dev/null 2>&1 || true
  fi
  if [[ -n "${STM32_TEMP:-}" && -d "$STM32_TEMP" ]]; then
    rm -rf -- "$STM32_TEMP"
  fi
}

stm32_run() {
  local action="$1"
  stm32_fetch
  stm32_report
  if [[ "$action" == check_stm32 ]]; then
    echo '想看变更内容：bash sync_car.sh --stm32-diff'
    return 0
  fi
  if [[ "$action" == diff_stm32 ]]; then
    if [[ -n "$STM32_OLD_SHA" ]]; then
      [[ "$STM32_OLD_SHA" != "$STM32_NEW_SHA" ]] || return 0
      echo '本次队友修改的文件摘要：'
      stm32_git --no-pager diff --stat "$STM32_OLD_SHA" "$STM32_NEW_SHA"
      echo '完整代码差异（可在终端中滚动查看）：'
      stm32_git --no-pager diff --no-ext-diff "$STM32_OLD_SHA" "$STM32_NEW_SHA"
    else
      echo '首次导入：尚无上次导入版本。以下是队友仓库文件统计（非与你的 STM32F4 差分）：'
      local empty_tree
      empty_tree="$(stm32_git hash-object -t tree /dev/null)"
      stm32_git --no-pager diff --stat "$empty_tree" "$STM32_NEW_SHA"
      echo '首次导入请使用 --update-stm32 查看将被替换的本地文件后再决定。'
    fi
    return 0
  fi

  [[ "$action" == update_stm32 ]] || die '无效 STM32 操作'
  [[ "$(git symbolic-ref -q --short HEAD || true)" == main ]] || die '--update-stm32 仅允许 car/main'
  git rev-parse --verify HEAD >/dev/null 2>&1 || die 'car/main 尚无初始提交，请先完成首次普通同步'
  [[ -z "$(git ls-files -u)" ]] || die 'car 存在尚未解决的合并冲突'
  [[ ! -f .git/MERGE_HEAD && ! -d .git/rebase-merge && ! -d .git/rebase-apply ]] || die 'car 存在未结束的合并或 rebase'
  [[ -z "$(git status --porcelain --untracked-files=normal)" ]] || die 'car 工作区不干净：请先用正常 sync_car.sh 提交/备份，再执行 STM32 合并'
  [[ ! -e "$ROOT/STM32F4/.git" ]] || die 'STM32F4 内仍有嵌套 .git；请先用 --include-nested 处理'
  if git ls-files --stage -- STM32F4 | awk '$1=="160000" {found=1} END{exit !found}'; then
    die 'STM32F4 仍是 Git 子仓库引用；请先用 --include-nested 处理'
  fi
  if [[ -n "$STM32_OLD_SHA" && "$STM32_OLD_SHA" == "$STM32_NEW_SHA" ]]; then
    echo '已经是最新导入版本，无需合并。'
    return 0
  fi
  stm32_no_unsafe_entries "$STM32_NEW_SHA"
  [[ -z "$STM32_OLD_SHA" ]] || stm32_no_unsafe_entries "$STM32_OLD_SHA"

  local start_head note newcommit backup_tag
  start_head="$(git rev-parse HEAD)"
  STM32_TEMP="$(mktemp -d -t car-stm32-XXXXXXXX)"
  STM32_WORKTREE="$STM32_TEMP/car-staging"
  trap stm32_cleanup EXIT
  stm32_archive_into "$STM32_NEW_SHA" "$STM32_TEMP/upstream-new"
  if [[ -n "$STM32_OLD_SHA" ]]; then stm32_archive_into "$STM32_OLD_SHA" "$STM32_TEMP/upstream-old"; fi
  git worktree add --detach --quiet "$STM32_WORKTREE" "$start_head" || die '无法建立临时 Git 工作区，主项目不受影响'
  local target="$STM32_WORKTREE/STM32F4"
  mkdir -p "$target"

  if [[ -z "$STM32_OLD_SHA" ]]; then
    echo
    echo '【首次导入预览】新的 STM32F4 将以队友开发分支整体快照为准。'
    echo '原目录的被跟踪文件可能被删除/覆盖；备份标签会保存导入前版本。'
    echo '队友版本与 car/STM32F4 目前的文件差异（最多列出前 80 行）：'
    diff -qr --exclude=.git --exclude=build --exclude=install --exclude=log --exclude=Debug --exclude=Release \
      "$target" "$STM32_TEMP/upstream-new" 2>/dev/null | sed -n '1,80p' || true
    echo
    [[ -t 0 ]] || die '首次导入必须在交互终端核对并确认 ADOPT STM32'
    local answer
    IFS= read -r -p '确认以队友代码替换原 STM32F4 跟踪文件？输入 ADOPT STM32：' answer || die '未收到确认'
    [[ "$answer" == 'ADOPT STM32' ]] || { echo '取消首次导入；car 未改变。'; return 0; }
    # 初次整目录迁移，不进行无共同祖先的伪三方合并。
    git -C "$STM32_WORKTREE" rm -r -q --ignore-unmatch -- STM32F4
    mkdir -p "$target"
    cp -a "$STM32_TEMP/upstream-new/." "$target/"
  else
    echo
    echo '【增量三方合并】对每个变更文件，以「上次导入版 / car 现有版 / 队友新版」比较。'
    if find "$target" -type l -print -quit | grep -q .; then
      die 'car/STM32F4 存在符号链接；为了防止复制写入其他目录，停止自动合并'
    fi
    local -a changes=() conflicts=()
    mapfile -d '' -t changes < <(stm32_git diff --no-renames --name-only -z "$STM32_OLD_SHA" "$STM32_NEW_SHA")
    local rel old_file new_file current_file tempmerged index=0 rc
    for rel in "${changes[@]}"; do
      old_file="$STM32_TEMP/upstream-old/$rel"
      new_file="$STM32_TEMP/upstream-new/$rel"
      current_file="$target/$rel"
      # 防止目标目录里发生符号链接穿越；前面已禁止队友链接。
      if [[ -L "$current_file" || ( -e "$current_file" && ! -f "$current_file" ) ]]; then
        conflicts+=("$rel（本地不是普通文件）")
        continue
      fi
      if stm32_same_path "$current_file" "$old_file"; then
        stm32_copy_path "$new_file" "$current_file"
      elif stm32_same_path "$current_file" "$new_file"; then
        : # 已有相同内容（例如之前人工移植过此提交）
      elif [[ -f "$old_file" && -f "$new_file" && -f "$current_file" ]]; then
        index=$((index+1))
        tempmerged="$STM32_TEMP/merge-result-$index"
        rc=0
        git merge-file -p -- "$current_file" "$old_file" "$new_file" > "$tempmerged" || rc=$?
        if (( rc == 0 )); then
          chmod --reference="$current_file" "$tempmerged"
          stm32_copy_path "$tempmerged" "$current_file"
        else
          conflicts+=("$rel（三方合并冲突，git merge-file 返回 $rc）")
        fi
      else
        conflicts+=("$rel（新增/删除与本地改动冲突）")
      fi
    done
    if (( ${#conflicts[@]} > 0 )); then
      echo '发现以下冲突；本次只修改了临时工作区，car/STM32F4 未被触碰：' >&2
      printf '  %s\n' "${conflicts[@]}" >&2
      die '请先在 car/STM32F4 中手动协调这些文件，再重试 --update-stm32；切勿强制覆盖'
    fi
    echo "可以无冲突集成的队友变更文件数：${#changes[@]}"
    git -C "$STM32_WORKTREE" --no-pager diff --stat -- STM32F4
    [[ -t 0 ]] || die '增量导入必须在交互终端核对并确认 MERGE STM32'
    local answer
    IFS= read -r -p '确认导入上述修改并创建 car 本地提交？输入 MERGE STM32：' answer || die '未收到确认'
    [[ "$answer" == 'MERGE STM32' ]] || { echo '取消增量导入；car 未改变。'; return 0; }
  fi

  mkdir -p "$STM32_WORKTREE/.car-sync"
  printf '%s\n' "$STM32_NEW_SHA" > "$STM32_WORKTREE/$STM32_STATE_REL"
  git -C "$STM32_WORKTREE" add -A -- STM32F4 "$STM32_STATE_REL"
  echo '写入 car 的变更统计：'
  git -C "$STM32_WORKTREE" --no-pager diff --cached --stat
  [[ -t 0 ]] || die '合并需要交互终端填写本地说明/审核'
  IFS= read -r -p '可选：填写本次 STM32 导入说明（可回车）：' note || die '读取说明失败'
  local title="stm32: integrate upstream ${STM32_NEW_SHA:0:12}"
  local details
  printf -v details '来源：%s\n来源分支：%s\n来源提交：%s\n上次导入：%s' \
    "$STM32_URL" "$STM32_BRANCH" "$STM32_NEW_SHA" "${STM32_OLD_SHA:-首次导入}"
  if [[ -n "$note" ]]; then details+=$'\n'"说明：$note"; fi
  git -C "$STM32_WORKTREE" commit -m "$title" -m "$details" || die '暂存工作区提交失败；car 未改变'
  newcommit="$(git -C "$STM32_WORKTREE" rev-parse HEAD)"
  [[ "$(git rev-parse HEAD)" == "$start_head" ]] || die 'car/main 在合并过程中发生变化；未修改 car'
  [[ -z "$(git status --porcelain --untracked-files=normal)" ]] || die 'car 工作区在合并过程中发生变化；未修改 car'
  backup_tag="backup/car-stm32-$(date '+%Y%m%d-%H%M%S')-$$"
  git tag -a "$backup_tag" "$start_head" -m "car before STM32 integration of $STM32_NEW_SHA"
  echo "已创建本地备份标签：$backup_tag"
  git merge --ff-only "$newcommit" || die "本地快进合并失败；可用备份标签 $backup_tag 找回。未推送"
  echo
  echo 'STM32 合并成功：本地 car/main 已生成版本提交，尚未推送 GitHub！'
  echo "  队友提交：$STM32_NEW_SHA"
  echo "  car 新提交：$newcommit"
  echo "  本地备份：$backup_tag"
  echo '请在实际硬件上核对 Keil 构建、USART3/OpenMV 协议和电机参数。'
  echo "确认无误后，再运行：bash \"$REPO_DIR/sync_car.sh\"（普通同步，上传自己的 GitHub）"
}

if [[ "$MODE" == check_stm32 || "$MODE" == diff_stm32 || "$MODE" == update_stm32 ]]; then
  stm32_run "$MODE"
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
