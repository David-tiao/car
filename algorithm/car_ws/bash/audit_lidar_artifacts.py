#!/usr/bin/env python3
"""Read-only Git audit of known historical generated lidar directories."""
import argparse
import collections
from pathlib import Path
import re
import subprocess

ROOT = Path(__file__).resolve().parents[3]
PREFIX = "algorithm/car_ws/"
KNOWN = re.compile(r"(?:build_lidar_repair_\d{8}_\d{6}_\d+|log_lidar_repair_\d{8}_\d{6}_\d+|install_oneclick_lidar(?:\.(?:bak|failed)_\d{8}_\d{6}_\d+)?)/")


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--repo", type=Path, default=ROOT)
    p.add_argument("--list", action="store_true")
    p.add_argument("--expect-empty", action="store_true", help="fail if historical generated items are tracked")
    a = p.parse_args()
    root = a.repo.resolve(strict=True)
    def git(*args):
        return subprocess.check_output(["git", "-C", str(root), *args])
    if Path(git("rev-parse", "--show-toplevel").decode().strip()).resolve() != root:
        p.error("--repo must be repository root")
    records = git("ls-files", "--stage", "-z").split(b"\0")
    groups = collections.Counter()
    links = missing = 0
    for record in records:
        if not record:
            continue
        meta, raw = record.split(b"\t", 1)
        mode, _, stage = meta.decode().split()
        path = raw.decode("utf-8", "surrogateescape")
        if not path.startswith(PREFIX):
            continue
        m = KNOWN.match(path[len(PREFIX):])
        if not m:
            continue
        if stage != "0":
            p.error("unmerged tracked generated file: " + path)
        groups[m.group()[:-1]] += 1
        if mode == "120000":
            links += 1
        f = root / path
        if not f.exists():
            missing += 1
        if a.list:
            print(mode, path)
    for name, count in sorted(groups.items()):
        print(f"{name}: {count}")
    print(f"TRACKED={sum(groups.values())} SYMLINKS={links} MISSING_OR_BROKEN={missing}")
    print("READ ONLY: no index/worktree changes.")
    if a.expect_empty and groups:
        p.exit(1, "FAIL: historical generated files remain tracked\n")
    if a.expect_empty:
        print("PASS: zero historical generated lidar items tracked")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
