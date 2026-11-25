#!/usr/bin/env python3
"""Back up local generated lidar artifacts BEFORE checking out the cleanup commit.
No extraction, Git index changes, hardware start or source deletion.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path, PurePosixPath
import re
import stat
import subprocess
import tarfile

PREFIX = "algorithm/car_ws"
KNOWN = re.compile(r"(?:build_lidar_repair_\d{8}_\d{6}_\d+|log_lidar_repair_\d{8}_\d{6}_\d+|install_oneclick_lidar(?:\.(?:bak|failed)_\d{8}_\d{6}_\d+)?)")


def git(root, *args):
    return subprocess.check_output(["git", "-C", str(root), *args])


def hash_stream(stream):
    h = hashlib.sha256()
    while True:
        chunk = stream.read(1024 * 1024)
        if not chunk:
            return h.hexdigest()
        h.update(chunk)


def snapshot(root):
    """Record directory, file content/mode, and literal symlink target without following links."""
    ws = root / PREFIX
    if ws.resolve(strict=True) != ws or not ws.is_dir():
        raise ValueError("workspace must be a real directory inside the repository")
    entries = {}

    def walk(path):
        key = path.relative_to(root).as_posix()
        st = path.lstat()
        mode = stat.S_IMODE(st.st_mode)
        if stat.S_ISLNK(st.st_mode):
            entries[key] = {"type": "symlink", "mode": mode, "target": os.readlink(path)}
        elif stat.S_ISDIR(st.st_mode):
            entries[key] = {"type": "dir", "mode": mode}
            for child in sorted(path.iterdir()):
                walk(child)
        elif stat.S_ISREG(st.st_mode):
            with path.open("rb") as fp:
                entries[key] = {"type": "file", "mode": mode, "size": st.st_size,
                                "sha256": hash_stream(fp)}
        else:
            raise ValueError(f"unsupported special file: {key}; stop relevant processes first")

    for child in sorted(ws.iterdir()):
        if KNOWN.fullmatch(child.name):
            walk(child)
    return entries


def archive_snapshot(archive):
    entries = {}
    with tarfile.open(archive, "r:gz") as tar:
        for item in tar:
            name = item.name
            parts = PurePosixPath(name).parts
            if (len(parts) < 3 or parts[:2] != ("algorithm", "car_ws")
                    or not KNOWN.fullmatch(parts[2]) or ".." in parts or name in entries):
                raise ValueError(f"unexpected archive entry: {name}")
            mode = item.mode & 0o7777
            if item.isdir():
                entries[name] = {"type": "dir", "mode": mode}
            elif item.issym():
                entries[name] = {"type": "symlink", "mode": mode, "target": item.linkname}
            elif item.isfile():
                with tar.extractfile(item) as fp:
                    entries[name] = {"type": "file", "mode": mode, "size": item.size,
                                     "sha256": hash_stream(fp)}
            else:
                raise ValueError(f"unsupported archive entry: {name}")
    return entries


def verify(folder):
    manifest = json.loads((folder / "manifest.json").read_text(encoding="utf-8"))
    archive = folder / "lidar-artifacts.tar.gz"
    with archive.open("rb") as fp:
        if hash_stream(fp) != manifest["archive_sha256"]:
            raise ValueError("archive SHA256 mismatch")
    entries = archive_snapshot(archive)
    if entries != manifest["entries"]:
        raise ValueError("archive content/mode/link verification failed")
    print(f"PASS BACKUP VERIFY: {len(entries)} entries; {archive}")
    return manifest


def backup(root, folder):
    if Path(git(root, "rev-parse", "--show-toplevel").decode().strip()).resolve() != root:
        raise ValueError("--repo must be repository root")
    if folder.is_symlink():
        raise ValueError("backup destination must not be a symlink")
    folder = folder.resolve()
    if folder == root or root in folder.parents:
        raise ValueError("backup destination must be outside repository")
    if folder.exists() or folder.is_symlink():
        raise ValueError("destination already exists; choose a new directory")
    # No concurrent writer is permitted. The second complete snapshot detects changes.
    before = snapshot(root)
    head = git(root, "rev-parse", "HEAD").decode().strip()
    status = git(root, "status", "--porcelain=v1", "-z").hex()
    folder.mkdir(mode=0o700, parents=False, exist_ok=False)
    archive = folder / "lidar-artifacts.tar.gz"
    with tarfile.open(archive, "x:gz", dereference=False) as tar:
        # Add each entry once and disable hardlink deduplication so hashes cover
        # regular files even if the local build reused an inode.
        for name in before:
            tar.inodes.clear()
            tar.add(root / name, arcname=name, recursive=False)
    if (before != snapshot(root) or head != git(root, "rev-parse", "HEAD").decode().strip()
            or status != git(root, "status", "--porcelain=v1", "-z").hex()):
        raise ValueError("workspace changed while backing up; archive retained but NOT verified")
    if archive_snapshot(archive) != before:
        raise ValueError("archive differs from original snapshot; NOT verified")
    with archive.open("rb") as fp:
        digest = hash_stream(fp)
    manifest = {"format": 1, "source_repo": str(root), "source_head": head,
                "git_status_porcelain_v1_z_hex": status,
                "archive_sha256": digest, "entries": before}
    with (folder / "manifest.json").open("x", encoding="utf-8") as fp:
        json.dump(manifest, fp, ensure_ascii=True, indent=2, sort_keys=True)
    verify(folder)
    print("READ ONLY SOURCE: no changes to Git or workspace.")
    print("Symlink targets are recorded, NOT dereferenced. External targets are NOT backed up.")
    print("Archive is for recovery/inspection, NOT a portable executable installation.")


def main():
    p = argparse.ArgumentParser(description=__doc__)
    sub = p.add_subparsers(dest="command", required=True)
    b = sub.add_parser("backup")
    b.add_argument("--repo", type=Path, required=True)
    b.add_argument("--output", type=Path, required=True, help="new directory outside repository; parent exists")
    v = sub.add_parser("verify")
    v.add_argument("--backup", type=Path, required=True)
    args = p.parse_args()
    try:
        if args.command == "backup":
            backup(args.repo.resolve(strict=True), args.output)
        else:
            verify(args.backup.resolve(strict=True))
    except (OSError, ValueError, KeyError, TypeError, tarfile.TarError, subprocess.CalledProcessError) as exc:
        p.exit(2, f"BACKUP FAIL: {exc}\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
