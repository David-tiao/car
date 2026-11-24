"""Composition only: reuse existing Gazebo and Nav2 launch files."""
import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context):
    if os.environ.get("ROS_DOMAIN_ID") != "84" or os.environ.get("ROS_LOCALHOST_ONLY") != "1":
        raise RuntimeError("SIM requires dedicated local domain 84")
    share = Path(get_package_share_directory("car_gazebo"))
    def arg(name):
        return LaunchConfiguration(name).perform(context)
    if arg("with_rviz") not in ("true", "false"):
        raise RuntimeError("with_rviz must be true or false")
    sim = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        str(share / "launch/gazebo_view.launch.py")), launch_arguments={
            "mode": "sim", "command_source": "nav2", "world": arg("world"),
            "partition": arg("partition"), "server_only": arg("server_only")}.items())
    nav = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        str(share / "launch/sim_nav.launch.py")), launch_arguments={
            "map_dir": arg("map_dir"), "start": arg("start"), "goal": arg("goal")}.items())
    actions = [sim, nav]
    if arg("with_rviz") == "true":
        actions.append(Node(package="rviz2", executable="rviz2", name="car_gazebo_rviz",
                            arguments=["-d", str(share / "config/sim.rviz")],
                            parameters=[{"use_sim_time": True}], output="screen"))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("world"),
        DeclareLaunchArgument("map_dir"),
        DeclareLaunchArgument("partition"),
        DeclareLaunchArgument("start", default_value="C3"),
        DeclareLaunchArgument("goal", default_value="A1"),
        DeclareLaunchArgument("server_only", default_value="false"),
        DeclareLaunchArgument("with_rviz", default_value="true"),
        OpaqueFunction(function=setup),
    ])
