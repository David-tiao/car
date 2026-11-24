#!/usr/bin/env python3
"""Hardware-free regression for relocated deployment entry points.

Only synthetic ROS/SDK discovery is exercised; no physical device is opened.
"""
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest

WS = Path(__file__).resolve().parents[1]


class PortabilityTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory(prefix="car portability ")
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)
        self.home = self.root / "another user"
        self.home.mkdir()
        self.ws = self.root / "arbitrary project" / "ros ws"
        self.ws.mkdir(parents=True)
        (self.ws / "src/ydlidar_ros2_driver").mkdir(parents=True)
        (self.ws / "src/ydlidar_ros2_driver/package.xml").write_text("<package/>")
        (self.ws / "src/ydlidar_ros2_driver/COLCON_IGNORE").touch()
        for directory in ("bash", "config", "behavior_trees"):
            shutil.copytree(WS / directory, self.ws / directory,
                            ignore=shutil.ignore_patterns("__pycache__", "*.pyc"))
        self.link = self.root / "link to workspace"
        self.link.symlink_to(self.ws, target_is_directory=True)
        self.cwd = self.root / "unrelated cwd"
        self.cwd.mkdir()
        self.ros = self.root / "ros custom prefix" / "setup.bash"
        self.ros.parent.mkdir()
        self.ros.write_text('export ROS_DISTRO=humble\n'
                            'export FAKE_ROS_TRACE="$UNDEFINED_ROS_TEST"\n')
        self.install = self.ws / "install"
        self.install.mkdir()
        (self.install / "setup.bash").write_text('export FAKE_OVERLAY=loaded\n')
        self.bin = self.root / "fake tools"
        self.bin.mkdir()
        # Ensure doctor can test discovery even on a machine without colcon.
        self.fake("colcon", "exit 99")
        self.env = dict(os.environ)
        for key in ("CAR_WS", "CAR_ROS_SETUP", "CAR_YDLIDAR_SDK_DIR",
                    "CAR_YDLIDAR_SDK_PREFIX", "CAR_LIDAR_CONFIG", "CONDA_PREFIX",
                    "VIRTUAL_ENV", "ROS_DISTRO", "PYTHONPATH", "PYTHONHOME",
                    "CMAKE_PREFIX_PATH"):
            self.env.pop(key, None)
        self.env.update(HOME=str(self.home), CAR_ROS_SETUP=str(self.ros),
                        PATH=str(self.bin) + os.pathsep + os.environ["PATH"])

    def fake(self, name, content):
        p = self.bin / name
        p.write_text("#!/usr/bin/env bash\n" + content + "\n")
        p.chmod(0o755)

    def run_cmd(self, *args, env=None, ok=True):
        result = subprocess.run([str(x) for x in args], cwd=self.cwd,
                                env=self.env if env is None else env,
                                text=True, capture_output=True, timeout=30)
        if ok:
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        else:
            self.assertNotEqual(result.returncode, 0, result.stdout + result.stderr)
        return result.stdout + result.stderr

    def test_default_workspace_and_symlink(self):
        out = self.run_cmd("bash", "-eu", "-c",
                           'source "$1"; car_check_workspace; printf "%s\\n" "$CAR_WS"',
                           "_", self.link / "bash/lib/common.sh")
        self.assertEqual(out.strip(), str(self.ws.resolve()))

    def test_explicit_workspace_and_nounset_restore(self):
        override = self.root / "other workspace"
        (override / "src").mkdir(parents=True)
        out = self.run_cmd("bash", "-eu", "-c",
                           'source "$1"; car_check_workspace; car_source_ros_base; '
                           '[[ $- == *u* ]]; printf "%s\\n" "$CAR_WS"',
                           "_", self.ws / "bash/lib/common.sh",
                           env=dict(self.env, CAR_WS=str(override)))
        self.assertEqual(out.strip(), str(override))

    def test_env_overlay_and_missing_override(self):
        out = self.run_cmd("bash", "-eu", "-c",
                           'source "$1"; [[ "$FAKE_OVERLAY" == loaded && $- == *u* ]]',
                           "_", self.link / "bash/env.sh")
        self.assertIn(str(self.ws), out)
        out = self.run_cmd("bash", "-eu", "-c",
                           'source "$1"; car_check_workspace',
                           "_", self.ws / "bash/lib/common.sh",
                           env=dict(self.env, CAR_WS=str(self.root / "missing")), ok=False)
        self.assertIn("missing src/", out)

    def test_reject_mixed_or_wrong_ros(self):
        for override in ({"CONDA_PREFIX": "/different/conda"},
                         {"VIRTUAL_ENV": "/different/venv"}):
            out = self.run_cmd("bash", "-eu", "-c",
                               'source "$1"; car_source_ros_base',
                               "_", self.ws / "bash/lib/common.sh",
                               env=dict(self.env, **override), ok=False)
            self.assertIn("clean ROS terminal", out)
        self.ros.write_text("export ROS_DISTRO=jazzy\n")
        out = self.run_cmd("bash", "-eu", "-c",
                           'source "$1"; car_source_ros_base',
                           "_", self.ws / "bash/lib/common.sh", ok=False)
        self.assertIn("Humble required", out)

    def test_sdk_explicit_directory_and_prefix(self):
        prefix = self.root / "external SDK"
        sdk = prefix / "lib/cmake/ydlidar_sdk"
        sdk.mkdir(parents=True)
        (sdk / "ydlidar_sdkConfig.cmake").touch()
        script = self.link / "bash/repair_car_ws_lidar.sh"
        for var, value in (("CAR_YDLIDAR_SDK_DIR", sdk),
                           ("CAR_YDLIDAR_SDK_PREFIX", prefix)):
            out = self.run_cmd("bash", script, "doctor",
                               env=dict(self.env, **{var: str(value)}))
            self.assertIn(str(sdk), out)
        out = self.run_cmd("bash", script, "doctor",
                           env=dict(self.env, CAR_YDLIDAR_SDK_DIR=str(prefix / "absent")),
                           ok=False)
        self.assertIn("CAR_YDLIDAR_SDK_DIR", out)
        self.assertTrue((self.ws / "src/ydlidar_ros2_driver/COLCON_IGNORE").exists())
        self.assertFalse((self.ws / "install_oneclick_lidar").exists())

    def test_lidar_rejects_invalid_install_before_device(self):
        prefix = self.root / "fake driver install"
        exe = prefix / "lib/ydlidar_ros2_driver/ydlidar_ros2_driver_node"
        exe.parent.mkdir(parents=True)
        exe.symlink_to(self.root / "missing executable")
        self.fake("ros2", 'if [[ "$1 $2" == "pkg prefix" ]]; then printf "%s\\n" "$FAKE_PREFIX"; '
                  'elif [[ "$1 $2" == "pkg executables" ]]; then '
                  'echo "ydlidar_ros2_driver ydlidar_ros2_driver_node"; else exit 98; fi')
        out = self.run_cmd("bash", self.link / "bash/lidar.sh", "doctor",
                           env=dict(self.env, FAKE_PREFIX=str(prefix)), ok=False)
        self.assertIn(str(prefix), out)
        self.assertIn("符号链接", out)

    def test_render_both_behavior_tree_configs(self):
        script = self.link / "bash/render_nav2_params.py"
        for name in ("nav2_params.yaml", "nav2_competition_params.yaml"):
            source = self.ws / "config" / name
            before = source.read_text()
            output = self.root / ("generated " + name)
            self.run_cmd(sys.executable, script, "--params", source, "--output", output)
            text = output.read_text()
            actual_tree = self.ws / "behavior_trees/competition_nav_to_pose.xml"
            expected = json.dumps(str(actual_tree), ensure_ascii=False)
            self.assertIn("default_nav_to_pose_bt_xml: " + expected, text)
            self.assertEqual(
                [x for x in text.splitlines() if "default_nav_to_pose_bt_xml:" not in x],
                [x for x in before.splitlines() if "default_nav_to_pose_bt_xml:" not in x])
            self.assertEqual(source.read_text(), before)
            self.run_cmd(sys.executable, script, "--params", source, "--output", output, ok=False)
            self.assertEqual(output.read_text(), text)

    def test_render_missing_tree_fails_without_output(self):
        output = self.root / "not created.yaml"
        self.run_cmd(sys.executable, self.link / "bash/render_nav2_params.py",
                     "--behavior-tree", self.root / "missing.xml", "--output", output,
                     ok=False)
        self.assertFalse(output.exists())

    def test_actual_web_diagnostics_from_link_and_other_cwd(self):
        # The full checkout remains unchanged except normal ignored Python cache.
        checkout_link = self.root / "source checkout"
        checkout_link.symlink_to(WS, target_is_directory=True)
        out = self.run_cmd("bash", checkout_link / "bash/web.sh", "doctor")
        self.assertIn("[PASS] Standalone Sim2Real", out)

    def test_deploy_plan_without_ros_and_arch_guard(self):
        packages = ("car_interfaces", "car_stm32_bridge", "car_debug", "car_bringup",
                    "car_serial_link", "car_vision", "car_mechanism", "car_mission",
                    "car_navigation", "car_web_sim", "car_web_real", "car_web_test", "car_gazebo")
        for name in packages:
            p = self.ws / "src" / name
            p.mkdir()
            (p / "package.xml").touch()
            (p / "CMakeLists.txt").touch()
        script = self.link / "bash/deploy.sh"
        for profile in ("pc", "pi5"):
            out = self.run_cmd("bash", script, "paths", profile)
            self.assertEqual(len(out.splitlines()), 12)
            self.assertNotIn("car_gazebo", out)
            self.assertIn(str(self.ws / "src/car_navigation"), out)
        out = self.run_cmd("bash", script, "paths", "sim")
        self.assertEqual(len(out.splitlines()), 2)
        self.assertIn("car_gazebo", out)
        self.fake("uname", "echo x86_64")
        out = self.run_cmd("bash", script, "build", "pi5", ok=False)
        self.assertIn("does not match native architecture", out)
        self.fake("uname", "echo aarch64")
        out = self.run_cmd("bash", script, "build", "pc", ok=False)
        self.assertIn("does not match native architecture", out)
        self.assertFalse((self.ws / "build").exists())

    def test_artifact_audit_is_read_only(self):
        repo = self.root / "git audit"
        repo.mkdir()
        self.run_cmd("git", "-C", repo, "init", "-q")
        target = repo / "algorithm/car_ws/install_oneclick_lidar/old_link"
        target.parent.mkdir(parents=True)
        target.symlink_to(self.root / "not present")
        source = repo / "algorithm/car_ws/src/source.cpp"
        source.parent.mkdir(parents=True)
        source.write_text("int main() {}")
        self.run_cmd("git", "-C", repo, "add", ".")
        before = self.run_cmd("git", "-C", repo, "status", "--porcelain")
        out = self.run_cmd(sys.executable, self.link / "bash/audit_lidar_artifacts.py",
                           "--repo", repo, "--list")
        self.assertIn("TRACKED=1 SYMLINKS=1 MISSING_OR_BROKEN=1", out)
        self.assertNotIn("source.cpp", out)
        self.assertEqual(before, self.run_cmd("git", "-C", repo, "status", "--porcelain"))

    def test_shell_syntax(self):
        scripts = sorted((WS / "bash").glob("*.sh")) + [WS / "bash/lib/common.sh"]
        for path in scripts:
            with self.subTest(path=path):
                self.run_cmd("bash", "-n", path)


if __name__ == "__main__":
    unittest.main()
