#!/usr/bin/env bash
# PC-side auto sync + verified test-source publication. No auto execution on Pi.
set -Eeuo pipefail
umask 077
state="$HOME/.local/share/car-pi-auto"
units="$HOME/.config/systemd/user"
service=car-pi-auto.service
timer=car-pi-auto.timer
usage() { cat <<'HELP'
Usage: bash pi_auto.sh setup --repo DIR --host USER@HOST --branch NAME [--interval SECONDS]
       bash pi_auto.sh run|doctor|status|logs|auto-start|auto-stop
       bash pi_auto.sh rollback RELEASE
Setup installs user-only systemd timer; requires existing noninteractive SSH/Git credentials.
Scope: committed algorithm/pi/tests, verified publication at car_source_sync/current.
No sudo, package installation, native rebuild, auto program launch or motor control.
Rollback requires auto-stop. Exit 0 success/target offline skip; 1 failure; 2 usage.
HELP
}
fail(){ printf '[FAIL] %s\n' "$*" >&2; exit 1; }
bad(){ usage >&2; exit 2; }
valid_host(){ [[ "$1" =~ ^[A-Za-z0-9_][A-Za-z0-9_.-]*@[A-Za-z0-9][A-Za-z0-9.-]*$ ]]; }
valid_release(){ [[ "$1" =~ ^[0-9a-f]{12}-[0-9a-f]{16}$ ]]; }
ssh_run(){ timeout -k 5s 120s ssh -o BatchMode=yes -o StrictHostKeyChecking=yes -o ConnectTimeout=5 -o ServerAliveInterval=10 -o ServerAliveCountMax=2 "$host" "$@"; }
git_net(){ GIT_TERMINAL_PROMPT=0 GCM_INTERACTIVE=Never GIT_SSH_COMMAND='ssh -o BatchMode=yes -o StrictHostKeyChecking=yes' timeout -k 5s 90s git -C "$repo" "$@"; }
check_tools(){ for t in git bash timeout ssh scp systemctl flock python3; do command -v "$t" >/dev/null || fail "Missing $t"; done; }
mode=${1:-doctor}; if (($#)); then shift; fi
case "$mode" in -h|--help) usage; exit 0;; setup|run|doctor|status|logs|auto-start|auto-stop|rollback) ;; *) bad;; esac
check_tools
if [[ "$mode" == setup ]]; then
  repo= host= branch= interval=60
  while (($#)); do
    (($# >= 2)) || bad
    case "$1" in --repo) repo=$2;; --host) host=$2;; --branch) branch=$2;; --interval) interval=$2;; *) bad;; esac
    shift 2
  done
  [[ -n "$repo" ]] && valid_host "$host" && [[ "$branch" =~ ^[A-Za-z0-9][A-Za-z0-9_./-]*$ ]] || bad
  git check-ref-format --branch "$branch" >/dev/null || bad
  [[ "$interval" =~ ^[1-9][0-9]{1,3}$ ]] && ((interval >= 30 && interval <= 3600)) || bad
  repo=$(git -C "$repo" rev-parse --show-toplevel) || fail 'Invalid repository'
  systemctl --user show-environment >/dev/null || fail 'No user systemd session'
  ssh_run 'printf "[PASS] Noninteractive Pi SSH\n"' || fail 'Set up SSH key and known_hosts first (ssh-copy-id, then BatchMode test)'
  git_net ls-remote --exit-code origin "refs/heads/$branch" >/dev/null || fail 'Noninteractive GitHub access failed'
  git_net fetch --no-tags origin "refs/heads/$branch:refs/car-pi-auto/source"
  commit=$(git -C "$repo" rev-parse refs/car-pi-auto/source)
  # Install only from an explicitly fetched, selected bootstrap branch; no self-updates in run.
  tmp=$(mktemp -d); trap 'rm -rf -- "$tmp"' EXIT
  git -C "$repo" show "$commit:sync_pi.sh" > "$tmp/sync_pi.sh"
  git -C "$repo" show "$commit:scripts/pi_auto.sh" > "$tmp/pi_auto.sh"
  bash -n "$tmp/sync_pi.sh"; bash -n "$tmp/pi_auto.sh"
  mkdir -p "$state" "$units"
  [[ ! -L "$state" && ! -L "$units/$service" && ! -L "$units/$timer" ]] || fail 'Refuse symlink installation target'
  if [[ -e "$state/config" ]]; then
    fail 'Already configured; existing configuration preserved. Use auto-start/auto-stop, or review before reconfiguration.'
  fi
  [[ ! -e "$units/$service" && ! -e "$units/$timer" ]] || fail 'Existing systemd unit; review before setup'
  install -m 700 "$tmp/pi_auto.sh" "$tmp/sync_pi.sh" "$state/"
  printf 'repo=%q\nhost=%q\nbranch=%q\ninterval=%q\n' "$repo" "$host" "$branch" "$interval" > "$state/config"
  cat > "$units/$service" <<'UNIT'
[Unit]
Description=Car PC-to-Pi verified test-source deployment
[Service]
Type=oneshot
ExecStart=/bin/bash "%h/.local/share/car-pi-auto/pi_auto.sh" run
TimeoutStartSec=600
KillMode=control-group
UMask=0077
UNIT
  cat > "$units/$timer" <<UNIT
[Unit]
Description=Check Car Git and Pi connectivity periodically
[Timer]
OnStartupSec=15s
OnUnitInactiveSec=${interval}s
AccuracySec=1s
Unit=$service
[Install]
WantedBy=timers.target
UNIT
  systemctl --user daemon-reload
  # First deploy synchronously: failure leaves timer disabled; config remains for diagnosis.
  bash "$state/pi_auto.sh" run || fail 'Initial deploy failed; timer not enabled; fix and use auto-start'
  systemctl --user enable --now "$timer"
  echo '[PASS] Timer enabled (while user systemd session is running)'
  exit 0
fi
[[ -f "$state/config" && ! -L "$state/config" ]] || fail 'Run setup first'
# Trusted, user-private config created by setup, not fetched from remote.
# shellcheck disable=SC1090
source "$state/config"
valid_host "$host" || fail 'Invalid saved host'
[[ "$branch" =~ ^[A-Za-z0-9][A-Za-z0-9_./-]*$ ]] || fail 'Invalid saved branch'
if [[ "$mode" != rollback ]]; then (($# == 0)) || bad; fi
case "$mode" in
  status) systemctl --user status "$timer" "$service" --no-pager || true; [[ ! -f "$state/last_commit" ]] || cat "$state/last_commit"; ssh_run 'readlink "$HOME/workspace/12_artifacts/car_source_sync/current"' || true; exit 0;;
  logs) journalctl --user -u "$service" -n 80 --no-pager; exit $?;;
  auto-stop) systemctl --user disable --now "$timer"; systemctl --user stop "$service"; exit 0;;
  auto-start) bash "$state/pi_auto.sh" run; systemctl --user enable --now "$timer"; exit 0;;
  doctor) ssh_run 'uname -m; dpkg --print-architecture; dpkg --print-foreign-architectures; python3 --version; command -v rpicam-vid || command -v libcamera-vid || true; python3 -c "import cv2,numpy; print(cv2.__version__,numpy.__version__)"' || fail 'Pi diagnostic failed (no installation attempted)'; git_net ls-remote --exit-code origin "refs/heads/$branch" >/dev/null; exit 0;;
esac
exec 9>"$state/run.lock"
flock -n 9 || fail 'Another local deployment is running'
if [[ "$mode" == rollback ]]; then
  (($# == 1)) && valid_release "$1" || bad
  if systemctl --user is-active --quiet "$timer" || systemctl --user is-active --quiet "$service"; then fail 'Stop auto sync before rollback'; fi
  release=$1; manifest=rollback
else
  if ! ssh_run 'true'; then echo '[SKIP] Pi unavailable or SSH authentication failed; no deployment'; exit 0; fi
  git_net fetch --no-tags origin "refs/heads/$branch:refs/car-pi-auto/source"
  commit=$(git -C "$repo" rev-parse refs/car-pi-auto/source)
  if [[ -f "$state/last_commit" ]]; then
    old=$(cat "$state/last_commit")
    git -C "$repo" merge-base --is-ancestor "$old" "$commit" || fail 'Non-fast-forward source change; manual review required'
  fi
  out=$(CAR_PI_BATCH=1 bash "$state/sync_pi.sh" sync --repo "$repo" --host "$host" --ref "$commit" --path algorithm/pi/tests) || { printf '%s\n' "$out"; fail 'Source sync failed'; }
  printf '%s\n' "$out"
  record=$(printf '%s\n' "$out" | grep '^\[RELEASE\] ')
  read -r _ release manifest extra <<< "$record"
  valid_release "$release" && [[ "$manifest" =~ ^[0-9a-f]{64}$ && -z "$extra" ]] || fail 'Invalid sync result'
fi
# Validate static source, not execute fetched code. Publish only after success, lock on Pi.
ssh_run "bash -s -- $release $manifest" <<'REMOTE'
set -Eeuo pipefail
root="$HOME/workspace/12_artifacts/car_source_sync"
release=$1; expected=$2
[[ "$release" =~ ^[0-9a-f]{12}-[0-9a-f]{16}$ ]]
[[ -d "$root" && ! -L "$root" && ! -L "$root/.sync.lock" ]]
exec 9>>"$root/.sync.lock"; flock -w 10 9
cd "$root/$release"
[[ ! -L "$root/$release" && -f .files.sha256 && ! -L .files.sha256 ]]
if [[ "$expected" != rollback ]]; then [[ "$(sha256sum .files.sha256 | cut -d' ' -f1)" == "$expected" ]]; fi
sha256sum -c .files.sha256 --quiet
python3 - <<'PY'
import ast, pathlib, subprocess
root = pathlib.Path('algorithm/pi/tests')
assert root.is_dir()
for p in root.rglob('*'):
    if p.is_symlink(): raise RuntimeError('Unexpected symlink: ' + str(p))
    if p.is_file() and p.suffix == '.py': ast.parse(p.read_bytes(), filename=str(p))
    if p.is_file() and p.suffix == '.sh': subprocess.run(['bash', '-n', str(p)], check=True, timeout=10)
print('[PASS] Source syntax validation; runtime/hardware not tested')
PY
cd "$root"
if [[ -e current || -L current ]]; then
  [[ -L current && "$(readlink current)" =~ ^[0-9a-f]{12}-[0-9a-f]{16}$ ]] || { echo '[FAIL] Unknown current entry; preserved' >&2; exit 1; }
  if [[ "$(readlink current)" == "$release" ]]; then echo '[PASS] Current already published'; exit 0; fi
fi
stage=$(mktemp -d "$root/.activate.XXXXXXXX")
trap 'rm -f -- "$stage/current"; rmdir -- "$stage"' EXIT
ln -s -- "$release" "$stage/current"
mv -T -- "$stage/current" current
printf '[PASS] Published %s/current -> %s\n' "$root" "$release"
REMOTE
if [[ "$mode" == run ]]; then printf '%s\n' "$commit" > "$state/last_commit.tmp"; mv -T "$state/last_commit.tmp" "$state/last_commit"; fi
