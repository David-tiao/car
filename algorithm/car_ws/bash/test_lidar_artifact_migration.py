#!/usr/bin/env python3
"""Exercise pre-update backup and checkout removal using an isolated Git repository."""
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import tarfile
import tempfile
import unittest

SCRIPT = Path(__file__).with_name("backup_lidar_artifacts.py")


class BackupTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory(prefix="lidar migration ")
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)
        self.repo = self.root / "old checkout"
        self.repo.mkdir()
        self.git("init", "-q")
        self.git("config", "user.name", "Test")
        self.git("config", "user.email", "test@example.invalid")
        self.ws = self.repo / "algorithm/car_ws"
        self.old = self.ws / "install_oneclick_lidar"
        self.old.mkdir(parents=True)
        self.source = self.ws / "src/source.cpp"
        self.source.parent.mkdir()
        self.source.write_text("source retained")
        self.local = self.old / "local.log"
        self.tracked = self.old / "node"
        self.tracked.write_text("old compiled file")
        self.tracked.chmod(0o755)
        self.outside = self.root / "external SDK"
        self.outside.write_text("external must not be copied")
        self.link = self.old / "link"
        self.link.symlink_to(self.outside)
        self.broken = self.old / "broken"
        self.broken.symlink_to("not-present")
        self.git("add", ".")
        self.git("commit", "-qm", "old tracked overlay")
        self.local.write_text("untracked local data")
        self.tracked.write_text("locally modified binary")
        self.dest = self.root / "verified archive"

    def git(self, *args):
        return subprocess.check_output(["git", "-C", str(self.repo), *args])

    def run_script(self, *args, ok=True):
        result = subprocess.run([sys.executable, str(SCRIPT), *map(str, args)],
                                cwd=self.root, capture_output=True, text=True, timeout=30)
        self.assertEqual(result.returncode == 0, ok, result.stdout + result.stderr)
        return result.stdout + result.stderr

    def backup(self):
        return self.run_script("backup", "--repo", self.repo, "--output", self.dest)

    def test_snapshot_and_verified_recovery_before_checkout(self):
        before = self.git("status", "--porcelain")
        self.assertIn("PASS BACKUP VERIFY", self.backup())
        self.assertEqual(before, self.git("status", "--porcelain"))
        self.run_script("verify", "--backup", self.dest)
        manifest = json.loads((self.dest / "manifest.json").read_text())
        prefix = "algorithm/car_ws/install_oneclick_lidar/"
        entries = manifest["entries"]
        self.assertEqual(entries[prefix+"local.log"]["sha256"], hashlib.sha256(self.local.read_bytes()).hexdigest())
        self.assertEqual(entries[prefix+"node"]["mode"], 0o755)
        self.assertEqual(entries[prefix+"link"]["target"], str(self.outside))
        self.assertEqual(entries[prefix+"broken"]["target"], "not-present")
        self.assertFalse(any("src/" in x for x in entries))
        # Simulate a cleanup commit; source and untracked local data are preserved.
        self.git("rm", "-r", "-f", "--", "algorithm/car_ws/install_oneclick_lidar")
        self.git("commit", "-qm", "untrack historical generated artifacts")
        self.assertEqual(self.source.read_text(), "source retained")
        self.assertEqual(self.local.read_text(), "untracked local data")
        with tarfile.open(self.dest / "lidar-artifacts.tar.gz") as tar:
            self.assertEqual(tar.extractfile(prefix+"node").read(), b"locally modified binary")
            self.assertTrue(tar.getmember(prefix+"link").issym())
        self.run_script("verify", "--backup", self.dest)

    def test_reject_overwrite_inside_repository_and_tampering(self):
        self.run_script("backup", "--repo", self.repo, "--output", self.repo / "bad", ok=False)
        self.backup()
        self.run_script("backup", "--repo", self.repo, "--output", self.dest, ok=False)
        with (self.dest / "lidar-artifacts.tar.gz").open("ab") as fp:
            fp.write(b"corrupted")
        self.assertIn("SHA256 mismatch", self.run_script("verify", "--backup", self.dest, ok=False))

    def test_empty_scope_is_explicit_and_special_file_rejected(self):
        os.mkfifo(self.old / "running-pipe")
        self.assertIn("unsupported special file", self.run_script("backup", "--repo", self.repo, "--output", self.dest, ok=False))
        self.assertFalse(self.dest.exists())


if __name__ == "__main__":
    unittest.main()
