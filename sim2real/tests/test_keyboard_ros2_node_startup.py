"""Catch rclpy.Node reserved property conflicts before Ubuntu ROS2 startup.

These stdlib-only constructor smoke tests emulate read-only Node.publishers,
Node.subscriptions and Node.timers. They do NOT test DDS transport or hardware.
"""
from contextlib import contextmanager
import importlib
from pathlib import Path
import sys
import types
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))


class ReadOnlyNode:
    instances = []

    def __init__(self, name):
        self.name = name
        self._test_publishers = []
        self._test_subscriptions = []
        self._test_timers = []
        self.log = []
        ReadOnlyNode.instances.append(self)

    @property
    def publishers(self):
        return tuple(self._test_publishers)

    @property
    def subscriptions(self):
        return tuple(self._test_subscriptions)

    @property
    def timers(self):
        return tuple(self._test_timers)

    def create_publisher(self, msg_type, topic, qos):
        obj = types.SimpleNamespace(topic=topic, publish=lambda msg: None)
        self._test_publishers.append(obj)
        return obj

    def create_subscription(self, msg_type, topic, callback, qos):
        obj = types.SimpleNamespace(topic=topic, callback=callback)
        self._test_subscriptions.append(obj)
        return obj

    def create_timer(self, period, callback):
        obj = types.SimpleNamespace(period=period, callback=callback)
        self._test_timers.append(obj)
        return obj

    def get_logger(self):
        return types.SimpleNamespace(
            warning=lambda msg: self.log.append(("warning", msg)),
            info=lambda msg: self.log.append(("info", msg)))

    def destroy_node(self):
        pass


@contextmanager
def fake_rclpy(already_shutdown=False):
    rclpy = types.ModuleType("rclpy")
    node_mod = types.ModuleType("rclpy.node")
    node_mod.Node = ReadOnlyNode
    executor_mod = types.ModuleType("rclpy.executors")
    class ExternalShutdownException(Exception):
        pass
    executor_mod.ExternalShutdownException = ExternalShutdownException
    state = {"ok": True, "shutdown_calls": 0}
    def try_shutdown():
        if state["ok"]:
            state["ok"] = False
            state["shutdown_calls"] += 1
    rclpy.ok = lambda: state["ok"]
    rclpy.try_shutdown = try_shutdown
    time_mod = types.ModuleType("rclpy.time")
    time_mod.Time = type("Time", (), {"from_msg": staticmethod(lambda _: None)})
    rclpy.node = node_mod
    rclpy.time = time_mod
    rclpy.init = lambda: None
    rclpy.shutdown = lambda: None

    def spin(node):
        # Reaching spin means all constructors completed; ROS2 may already
        # have shut down on Ctrl+C when spin receives this exception.
        if already_shutdown:
            state["ok"] = False
            raise ExternalShutdownException()
        raise KeyboardInterrupt()
    rclpy.spin = spin

    geometry = types.ModuleType("geometry_msgs")
    geometry_msg = types.ModuleType("geometry_msgs.msg")
    geometry_msg.TwistStamped = type("TwistStamped", (), {})
    geometry.msg = geometry_msg
    std = types.ModuleType("std_msgs")
    std_msg = types.ModuleType("std_msgs.msg")
    std_msg.String = type("String", (), {})
    std.msg = std_msg

    mapping = {
        "rclpy": rclpy, "rclpy.node": node_mod, "rclpy.time": time_mod,
        "rclpy.executors": executor_mod,
        "geometry_msgs": geometry, "geometry_msgs.msg": geometry_msg,
        "std_msgs": std, "std_msgs.msg": std_msg,
    }
    with patch.dict(sys.modules, mapping), patch.object(
            sys, "argv", ["node.py"]):
        yield


class ROS2ConstructorSmokeTests(unittest.TestCase):
    def setUp(self):
        ReadOnlyNode.instances.clear()

    def test_readonly_rclpy_properties_reject_old_variable_names(self):
        node = ReadOnlyNode("check")
        with self.assertRaises(AttributeError):
            node.publishers = {}
        with self.assertRaises(AttributeError):
            node.subscriptions = []
        with self.assertRaises(AttributeError):
            node.timers = []

    def test_pc_keyboard_node_initializes_under_readonly_node(self):
        with fake_rclpy():
            import keyboard_ros2_pc
            keyboard_ros2_pc.main()
        node = ReadOnlyNode.instances[-1]
        self.assertEqual(node.name, "car_keyboard_pc_source")
        self.assertIn("/car/teleop/local_test_twist",
                      [x.topic for x in node.publishers])
        self.assertIn("/car/teleop/desired_twist",
                      [x.topic for x in node.publishers])
        self.assertIn("/car/teleop/local_test_diag",
                      [x.topic for x in node.publishers])
        self.assertEqual(len(node.timers), 1)

    def test_subscriber_web_feedback_node_initializes_under_readonly_node(self):
        with fake_rclpy():
            import keyboard_ros2_web_feedback
            keyboard_ros2_web_feedback.main()
        node = ReadOnlyNode.instances[-1]
        self.assertEqual(node.name, "car_keyboard_ros2_web_feedback")
        self.assertEqual({s.topic for s in node.subscriptions}, {
            "/car/teleop/local_test_twist",
            "/car/teleop/stm32_candidate"})

    def test_external_shutdown_does_not_double_shutdown_any_ros_node(self):
        # Reproduces the actual Humble Ctrl+C log:
        # ExternalShutdownException and a context already shut down.
        for module in ("keyboard_ros2_pc", "keyboard_ros2_web_feedback",
                       "keyboard_ros2_pi"):
            with self.subTest(module=module), fake_rclpy(already_shutdown=True):
                imported = importlib.import_module(module)
                imported.main()  # must NOT raise RCLError or shutdown twice

    def test_launcher_refuses_absent_web_api_before_spawning_ros_nodes(self):
        import subprocess
        path = (Path(__file__).resolve().parents[2] /
                "algorithm/car_ws/bash/start_keyboard_local_ros2.sh")
        result = subprocess.run(
            ["bash", str(path), "http://127.0.0.1:1"],
            capture_output=True, text=True, timeout=6)
        self.assertEqual(result.returncode, 2, result.stderr)
        self.assertIn("Web keyboard endpoint unavailable", result.stderr)
        self.assertIn("ROS2 nodes were NOT started", result.stderr)

    def test_pi_candidate_guard_node_initializes_under_readonly_node(self):
        with fake_rclpy():
            import keyboard_ros2_pi
            keyboard_ros2_pi.main()
        node = ReadOnlyNode.instances[-1]
        self.assertEqual(node.name, "car_keyboard_pi_guard")
        self.assertIn("/car/teleop/desired_twist",
                      [s.topic for s in node.subscriptions])
        self.assertIn("/car/teleop/stm32_candidate",
                      [p.topic for p in node.publishers])
        self.assertEqual(len(node.timers), 1)


if __name__ == "__main__":
    unittest.main(verbosity=2)
