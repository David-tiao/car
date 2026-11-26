#!/usr/bin/env bash
# PC -> Pi: immutable Git source snapshots, separate from installed runtimes.
set -Eeuo pipefail
usage() { cat <<'HELP'
Usage: bash sync_pi.sh [doctor|plan|sync] [--repo DIR] [--host USER@HOST]
                      [--ref GIT_REF] [--path algorithm/pi[/...]] [--dest HOME_RELATIVE_DIR]
Defaults: doctor, HEAD, algorithm/pi/tests, workspace/12_artifacts/car_source_sync.
Host: --host or CAR_PI_HOST. Repo: --repo or CAR_DIR, otherwise script directory
or current Git working directory. Only committed files; no fetch, build, install,
delete or runtime restart. sync creates a separate versioned source snapshot.
Exit 0 success, 1 error, 2 bad arguments, 124 timeout (or SSH/SCP failure code).
HELP
}
fail() { printf '[FAIL] %s\n' "$*" >&2; exit 1; }
bad() { usage >&2; exit 2; }
mode=doctor
if (($#)); then case "$1" in doctor|plan|sync) mode=$1; shift;; -h|--help) usage; exit 0;; esac; fi
repo=${CAR_DIR:-}; host=${CAR_PI_HOST:-}; ref=HEAD; path=algorithm/pi/tests
dest=workspace/12_artifacts/car_source_sync
while (($#)); do
  (($# >= 2)) || bad
  case "$1" in
    --repo) repo=$2;; --host) host=$2;; --ref) ref=$2;; --path) path=$2;; --dest) dest=$2;; *) bad;;
  esac; shift 2
done
[[ "$host" =~ ^[A-Za-z0-9_][A-Za-z0-9_.-]*@[A-Za-z0-9][A-Za-z0-9.-]*$ ]] || fail 'Set --host USER@HOST (DNS/IPv4) or CAR_PI_HOST'
[[ "$path" =~ ^algorithm/pi(/[A-Za-z0-9_.-]+)*$ && ! "$path/" =~ /\.\.?/ ]] || fail 'Only explicit algorithm/pi source paths are allowed'
[[ "$dest" =~ ^[A-Za-z0-9_-][A-Za-z0-9_.-]*(/[A-Za-z0-9_-][A-Za-z0-9_.-]*)*$ ]] || fail 'Destination must be a safe home-relative directory'
[[ "$dest" == workspace/12_artifacts/car_source_sync ]] || fail 'This version only permits the isolated car_source_sync destination'
for x in git tar sha256sum timeout mktemp find sort xargs cut ssh scp; do command -v "$x" >/dev/null || fail "Missing local tool: $x"; done
if [[ -z "$repo" ]]; then
  here=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)
  repo=$(git -C "$here" rev-parse --show-toplevel 2>/dev/null || git rev-parse --show-toplevel 2>/dev/null) || fail 'Specify --repo /path/to/car'
fi
repo=$(git -C "$repo" rev-parse --show-toplevel) || fail 'Invalid Git repository'
commit=$(git -C "$repo" rev-parse --verify --end-of-options "$ref^{commit}") || fail 'Ref not found locally; fetch on PC first'
entries=$(git -C "$repo" ls-tree -r "$commit" -- "$path") || fail 'Cannot inspect source tree'
[[ -n "$entries" ]] || fail 'Path does not exist in selected commit'
while IFS= read -r row; do
  [[ "$row" =~ ^100(644|755)[[:space:]]blob[[:space:]] ]] || fail 'Symlink/submodule or unsupported source entry'
done <<< "$entries"
printf '[INFO] PC: %s\n[INFO] ref=%s commit=%s\n[INFO] path=%s target=%s:~/%s\n' "$repo" "$ref" "$commit" "$path" "$host" "$dest"
if [[ "$mode" == plan ]]; then git -C "$repo" ls-tree -r --name-only "$commit" -- "$path"; exit 0; fi
opts=(-o ConnectTimeout=10 -o ServerAliveInterval=10 -o ServerAliveCountMax=3)
remote() { timeout --kill-after=5s 120s ssh "${opts[@]}" "$host" "$@"; }
remote 'bash -s' <<'REMOTE'
set -eu
printf '[INFO] Pi kernel: '; uname -m
if command -v dpkg >/dev/null; then printf '[INFO] native: '; dpkg --print-architecture; printf '[INFO] foreign: '; dpkg --print-foreign-architectures; fi
for x in tar sha256sum mktemp mkdir mv flock realpath cut cp rm rmdir; do command -v "$x" >/dev/null || { echo "[FAIL] Remote tool missing: $x" >&2; exit 1; }; done
printf '[PASS] SSH and remote prerequisites (no writes)\n'
REMOTE
[[ "$mode" == sync ]] || exit 0
work=$(mktemp -d)
trap 'rm -rf -- "$work"' EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
# git archive excludes uncommitted edits and all unrelated build/runtime files.
git -C "$repo" archive --format=tar "$commit" -- "$path" > "$work/source.tar"
mkdir "$work/extract"
tar -xf "$work/source.tar" -C "$work/extract"
(cd "$work/extract" && find . -type f -print0 | sort -z | xargs -0 -r sha256sum) > "$work/files.sha256"
[[ -s "$work/files.sha256" ]] || fail 'No files exported'
archive_hash=$(sha256sum "$work/source.tar" | cut -d' ' -f1)
manifest_hash=$(sha256sum "$work/files.sha256" | cut -d' ' -f1)
release="${commit:0:12}-${archive_hash:0:16}"
# Create a separate staging directory; never modify an existing published snapshot.
stage=$(remote "bash -s -- $dest $release $manifest_hash" <<'REMOTE'
set -eu
root="$HOME/$1"; release=$2; expected=$3
# Reject symlink path components before creating anything.
base="$HOME"
IFS=/ read -r -a parts <<< "$1"
for part in "${parts[@]}"; do
  base="$base/$part"
  [[ ! -L "$base" ]] || { echo '[FAIL] Symlink in destination' >&2; exit 1; }
  if [[ ! -d "$base" ]]; then mkdir -- "$base"; fi
done
[[ "$(realpath "$root")" == "$(realpath "$HOME")/$1" ]] || exit 1
[[ ! -L "$root/.sync.lock" ]] || exit 1
exec 9>>"$root/.sync.lock"; flock -w 10 9
if [[ -e "$root/$release" || -L "$root/$release" ]]; then
  [[ -d "$root/$release" && ! -L "$root/$release" ]] || exit 1
  cd "$root/$release"
  [[ "$(sha256sum .files.sha256 | cut -d' ' -f1)" == "$expected" ]]
  sha256sum -c .files.sha256 --quiet >&2
  printf 'REUSE\n'
else
  d=$(mktemp -d "$root/.incoming.XXXXXXXX")
  printf '%s\n' "${d##*/}"
fi
REMOTE
)
if [[ "$stage" == REUSE ]]; then
  printf '[PASS] Existing snapshot verified; no transfer\n[PATH] ~/%s/%s\n' "$dest" "$release"; exit 0
fi
[[ "$stage" =~ ^\.incoming\.[A-Za-z0-9]+$ ]] || fail 'Unexpected staging response'
printf '[INFO] Transfer source only to %s/%s\n' "$dest" "$stage"
timeout --kill-after=5s 120s scp "${opts[@]}" "$work/source.tar" "$work/files.sha256" "$host:$dest/$stage/"
remote "bash -s -- $dest $stage $release $archive_hash $manifest_hash" <<'REMOTE'
set -eu
root="$HOME/$1"; stage="$root/$2"; release="$root/$3"
[[ "$(realpath "$root")" == "$(realpath "$HOME")/$1" && -d "$stage" && ! -L "$stage" ]]
[[ ! -L "$root/.sync.lock" ]] || exit 1
exec 9>>"$root/.sync.lock"; flock -w 10 9
[[ ! -e "$release" && ! -L "$release" ]] || { echo '[FAIL] Concurrent publication; staging retained' >&2; exit 1; }
cd "$stage"
[[ "$(sha256sum source.tar | cut -d' ' -f1)" == "$4" ]]
[[ "$(sha256sum files.sha256 | cut -d' ' -f1)" == "$5" ]]
mkdir tree
tar --no-same-owner --no-same-permissions -xf source.tar -C tree
cp files.sha256 tree/.files.sha256
(cd tree && sha256sum -c .files.sha256 --quiet)
mv -T -- tree "$release"
# Only remove our own verified staging inputs, then the empty staging directory.
rm -- source.tar files.sha256
cd "$root"; rmdir -- "$stage"
printf '[PASS] Source snapshot verified and published\n[PATH] %s\n' "$release"
REMOTE
