#!/usr/bin/env bash
# 工创小车 car 项目一键同步 GitHub (Ubuntu / Linux)
# 首次合并嵌套 Git 仓库: bash ~/Desktop/sync_car.sh --include-nested
# 日常安全同步:        bash ~/Desktop/sync_car.sh
# 同步时同时删除本地编译目录（可选）: bash ~/Desktop/sync_car.sh --delete-build
# 默认只从 Git 跟踪和上传内容中排除编译产物，不删除电脑上的文件。
set -Eeuo pipefail

REPO_DIR="${CAR_DIR:-$HOME/Desktop/car}"
REMOTE_URL="${CAR_REMOTE_URL:-https://github.com/David-tiao/car.git}"
FLATTEN_NESTED=false
DELETE_BUILD=false

usage() {
  cat <<'USAGE'
用法: bash sync_car.sh [--include-nested] [--delete-build]

  无参数           日常同步：忽略编译产物，移除 Git 索引中的已跟踪产物，提交并推送
  --include-nested 首次使用：备份并移出嵌套仓库的 .git 元数据，把源码纳入 car 主仓库
  --delete-build   可选：同步前物理删除 car 内的常见编译目录（build/install/log 等）

默认不会删除本地编译产物。可通过 CAR_DIR、CAR_REMOTE_URL 环境变量更改配置。
USAGE
}

for arg in "$@"; do
  case "$arg" in
    --include-nested) FLATTEN_NESTED=true ;;
    --delete-build) DELETE_BUILD=true ;;
    -h|--help) usage; exit 0 ;;
    *) echo "不支持的参数: $arg" >&2; usage >&2; exit 2 ;;
  esac
done

command -v git >/dev/null || { echo '错误：未安装 git。' >&2; exit 1; }
[[ -d "$REPO_DIR" ]] || { echo "错误：找不到项目目录 $REPO_DIR" >&2; exit 1; }
cd "$REPO_DIR"

if [[ ! -e .git ]]; then
  git init -b main
fi

ROOT="$(git rev-parse --show-toplevel)"
if [[ "$ROOT" != "$(pwd -P)" ]]; then
  echo "错误：当前目录不是 Git 仓库根目录：$ROOT" >&2
  exit 1
fi

if [[ -n "$(git ls-files -u)" ]]; then
  echo '发现尚未解决的合并冲突。请先修复冲突并提交，再运行同步脚本。' >&2
  exit 1
fi

BRANCH="$(git symbolic-ref --quiet --short HEAD || true)"
if [[ "$BRANCH" != main ]]; then
  echo "错误：当前分支为 ${BRANCH:-（分离 HEAD）}；本脚本仅同步 main，不会擅自切换分支。" >&2
  exit 1
fi

OLD_URL="$(git remote get-url origin 2>/dev/null || true)"
if [[ -z "$OLD_URL" ]]; then
  git remote add origin "$REMOTE_URL"
elif [[ "$OLD_URL" != "$REMOTE_URL" ]]; then
  echo "错误：origin 已指向 $OLD_URL，不是 $REMOTE_URL。请手动核实。" >&2
  exit 1
fi

# 在生成目录以外检测真正嵌套的 Git 仓库；不扫描 .git 和编译目录。
mapfile -d '' -t NESTED_GIT < <(
  find . \( -path './.git' -o -type d \( \
      -name build -o -name install -o -name log -o -name devel \
      -o -name Debug -o -name Release -o -name .pio \
    \) \) -prune -o -name '.git' -print0
)
if (( ${#NESTED_GIT[@]} > 0 )); then
  echo '检测到嵌套 Git 仓库：'
  for g in "${NESTED_GIT[@]}"; do echo "  ${g%/.git}"; done
  if [[ "$FLATTEN_NESTED" != true ]]; then
    echo '尚未合并嵌套仓库。若要完整上传源码，请首次使用 --include-nested。' >&2
    exit 1
  fi
  for g in "${NESTED_GIT[@]}"; do
    if [[ ! -d "$g" ]]; then
      echo "发现 .git 文件（可能为 worktree / 子模块）: $g；为防止破坏引用，已停止。" >&2
      exit 1
    fi
  done
  BACKUP="$HOME/car_git_metadata_backup_$(date +%Y%m%d_%H%M%S)_$$"
  echo "备份子仓库 Git 元数据至 $BACKUP"
  for g in "${NESTED_GIT[@]}"; do
    rel="${g#./}"
    project="${rel%/.git}"
    mkdir -p "$BACKUP/$project"
    mv -- "$g" "$BACKUP/$project/.git"
    # 移除之前提交到主仓库的 gitlink，然后再把真正的源码添加进去。
    git rm -r --cached --ignore-unmatch -- "$project" >/dev/null
  done
  echo '嵌套 Git 历史已备份；源文件原样保留。'
fi

# 确保最终索引里没有指向嵌套 Git 仓库的引用。
GITLINKS="$(git ls-files --stage | awk '$1 == "160000" { print }')"
if [[ -n "$GITLINKS" ]]; then
  echo '错误：索引中仍有 gitlink，无法保证子目录源码完整上传：' >&2
  echo "$GITLINKS" >&2
  echo '请核实这些目录是否应以子模块管理，或手动解除 gitlink。' >&2
  exit 1
fi

# 维护忽略规则：只排除典型编译输出，不忽略 src、配置和源码。
# 这些规则会保存在 car/.gitignore，并随项目一同提交。
IGNORE_BEGIN='# >>> car sync: generated files >>>'
IGNORE_END='# <<< car sync: generated files <<<' 
RULE_FILE="$(mktemp)"
trap 'rm -f -- "$RULE_FILE"' EXIT
cat > "$RULE_FILE" <<'RULES'
# ROS 2 / colcon / catkin
**/build/
**/install/
**/log/
**/devel/
# STM32CubeIDE / PlatformIO / CMake
**/Debug/
**/Release/
**/.pio/
**/CMakeFiles/
**/cmake-build-*/
# Python / OpenMV cache and common intermediate files
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

if ! grep -Fqx "$IGNORE_BEGIN" .gitignore 2>/dev/null; then
  {
    if [[ -f .gitignore ]]; then printf '\n'; fi
    printf '%s\n' "$IGNORE_BEGIN"
    cat "$RULE_FILE"
    printf '%s\n' "$IGNORE_END"
  } >> .gitignore
  echo '已写入 car/.gitignore（自动忽略编译产物）。'
fi

# 可选：物理清除本地编译目录。默认不开启，避免造成不必要的重新编译。
if [[ "$DELETE_BUILD" == true ]]; then
  mapfile -d '' -t BUILD_DIRS < <(
    find . -path './.git' -prune -o -type d \( \
      -name build -o -name install -o -name log -o -name devel \
      -o -name Debug -o -name Release -o -name .pio \
      -o -name CMakeFiles -o -name '__pycache__' \
    \) -print0 -prune
  )
  if (( ${#BUILD_DIRS[@]} > 0 )); then
    echo '即将删除这些本地编译目录（源代码目录不受影响）：'
    printf '  %s\n' "${BUILD_DIRS[@]}"
    read -r -p '确认永久删除以上目录？输入 YES 继续：' ANSWER
    if [[ "$ANSWER" != YES ]]; then
      echo '已取消删除和同步。'; exit 1
    fi
    for d in "${BUILD_DIRS[@]}"; do
      [[ "$d" == ./* && "$d" != './.git' && -d "$d" && ! -L "$d" ]] || {
        echo "危险的删除路径，已停止: $d" >&2; exit 1;
      }
      rm -rf -- "$d"
    done
    echo '本地编译目录已删除。'
  fi
fi

# 已经被 Git 跟踪的生成文件，光加 .gitignore 不够：要从索引中移除。
# -ci 只选择规则文件内指定的构建产物；--cached 不删本地文件。
mapfile -d '' -t TRACKED_GENERATED < <(git ls-files -c -i --exclude-from="$RULE_FILE" -z)
if (( ${#TRACKED_GENERATED[@]} > 0 )); then
  echo "从 Git 跟踪中移除 ${#TRACKED_GENERATED[@]} 个编译产物（保留本地文件）："
  printf '  %s\n' "${TRACKED_GENERATED[@]:0:12}"
  if (( ${#TRACKED_GENERATED[@]} > 12 )); then echo '  ...'; fi
  printf '%s\0' "${TRACKED_GENERATED[@]}" | xargs -0 -r git rm --cached -q --
fi

# 正常提交、合并和推送。禁止强推，远端分支不会被覆盖。
git add -A
if ! git diff --cached --quiet; then
  git commit -m "sync car: $(date '+%Y-%m-%d %H:%M:%S')"
elif ! git rev-parse --verify HEAD >/dev/null 2>&1; then
  git commit --allow-empty -m 'initialize car repository'
else
  echo '本地没有新改动。'
fi

echo '获取 GitHub 上的最新提交...'
git fetch origin
if git show-ref --verify --quiet refs/remotes/origin/main; then
  echo '合并远程 main（不强制覆盖远程历史）...'
  if ! git merge --no-edit --allow-unrelated-histories origin/main; then
    echo '合并发生冲突，未推送。请用 git status 检查，解决冲突并提交后再运行。' >&2
    exit 1
  fi
fi

# 合并后远程可能重新带入旧编译产物，需再次移除并提交清理结果。
mapfile -d '' -t MERGED_GENERATED < <(git ls-files -c -i --exclude-from="$RULE_FILE" -z)
if (( ${#MERGED_GENERATED[@]} > 0 )); then
  printf '%s\0' "${MERGED_GENERATED[@]}" | xargs -0 -r git rm --cached -q --
  git commit -m 'chore: remove tracked build artifacts after merge'
fi

echo '推送到 GitHub...'
git push -u origin main
printf '\n同步完成！编译目录不会出现在新的上传内容中。\n'
git status -sb

echo '注意：如果旧提交历史已包含大型 build 文件，本脚本不会改写历史；旧大文件仍可能使首次推送超时。'
