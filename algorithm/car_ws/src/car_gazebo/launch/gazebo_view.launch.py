import os
import runpy
from ament_index_python.packages import get_package_prefix
from pathlib import Path
import xml.etree.ElementTree as ET

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context):
    world = Path(LaunchConfiguration("world").perform(context))
    partition = LaunchConfiguration("partition").perform(context)
    mode = LaunchConfiguration("mode").perform(context)
    if not world.is_absolute() or not world.is_file() or not partition:
        raise RuntimeError("Absolute generated SDF and nonempty partition required")
    root = ET.parse(world).getroot()
    actual_world = root.find("world").get("name")
    api = runpy.run_path(str(Path(get_package_prefix("car_gazebo")) / "lib/car_gazebo/gazebo_backend.py"))
    backend = LaunchConfiguration("backend").perform(context)
    if backend not in api["BACKENDS"]:
        raise RuntimeError("explicit fortress or harmonic backend required")
    expected = "ignition::gazebo::" if backend == "fortress" else "gz::sim::"
    plugins = root.findall(".//plugin")
    if not plugins or any(not p.get("name", "").startswith(expected) for p in plugins):
        raise RuntimeError("generated world plugin namespace does not match backend")
    if mode == "sim":
        api["select_backend"](backend)
    if mode not in ("view", "sim"):
        raise RuntimeError("mode must be view or sim")
    if mode == "view" and actual_world not in ("car_preview", "car_mirror"):
        raise RuntimeError("view mode accepts preview/mirror only")
    if mode == "sim" and (actual_world != "car_sim" or not partition.startswith("car_sim_") or
                          os.environ.get("ROS_DOMAIN_ID") != "84" or
                          os.environ.get("ROS_LOCALHOST_ONLY") != "1"):
        raise RuntimeError("SIM requires car_sim SDF, car_sim_* partition, ROS_DOMAIN_ID=84 and ROS_LOCALHOST_ONLY=1")
    env = api["partition_environment"](backend, partition)
    server_only = LaunchConfiguration("server_only").perform(context)
    if server_only not in ("true", "false"):
        raise RuntimeError("server_only must be true or false")
    gz_args = api["simulator_command"](backend) + ["-r"]
    if server_only == "true":
        gz_args.append("-s")
    gz = ExecuteProcess(cmd=gz_args + [str(world)], additional_env=env, output="screen")
    if mode == "view":
        return [gz]
    command_source = LaunchConfiguration("command_source").perform(context)
    if command_source not in ("manual", "nav2"):
        raise RuntimeError("command_source must be manual or nav2")
    urdf = (world.parent / "car_sim.urdf").read_text(encoding="utf-8")
    if ET.fromstring(urdf).get("name") != "car_sim":
        raise RuntimeError("SIM URDF mismatch")
    bridge = ExecuteProcess(
        cmd=["ros2", "run", "ros_gz_bridge", "parameter_bridge",
             *api["bridge_arguments"](backend)],
        additional_env=env, output="screen")
    adapter = Node(package="car_gazebo", executable="sim_adapter.py",
                   parameters=[{"use_sim_time": True, "command_source": command_source}], output="screen")
    rsp = Node(package="robot_state_publisher", executable="robot_state_publisher",
               parameters=[{"use_sim_time": True, "robot_description": urdf}], output="screen")
    processes = [gz, bridge, adapter, rsp]
    # VelocityControl retains its last command; stop this launch's children
    # if the bridge, watchdog, simulator or model publisher exits.
    handlers = [RegisterEventHandler(OnProcessExit(target_action=p,
                on_exit=[EmitEvent(event=Shutdown(reason="SIM process exited"))]))
                for p in processes]
    return handlers + processes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("world", description="Absolute generated SDF path"),
        DeclareLaunchArgument("partition", description="Unique Gazebo Transport partition"),
        DeclareLaunchArgument("backend", default_value="fortress"),
        DeclareLaunchArgument("mode", default_value="view", description="view or isolated sim"),
        DeclareLaunchArgument("command_source", default_value="manual"),
        DeclareLaunchArgument("server_only", default_value="false"),
        OpaqueFunction(function=setup),
    ])
