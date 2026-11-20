#!/usr/bin/env bash
# One-command, zero-hardware, all-stage offline software acceptance.
set -Eeuo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../../.." && pwd -P)"
export PYTHONDONTWRITEBYTECODE=1
exec python3 "$ROOT/sim2real/offline_acceptance_suite.py" "$@"
