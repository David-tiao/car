import os
from pathlib import Path
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, ExecuteProcess, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context):
    if os.environ.get("ROS_DOMAIN_ID") != "84" or os.environ.get("ROS_LOCALHOST_ONLY") != "1":
        raise RuntimeError("SIM Nav2 requires dedicated local ROS domain 84")
    directory = Path(LaunchConfiguration("map_dir").perform(context))
    topology = directory / "topology_stage6/road_topology.py"
    lock = directory / "simple_lock_stage/lock_obstacles.py"
    map_yaml = directory / "competition_navigation_map.yaml"
    mask = directory / "competition_keepout_mask.yaml"
    if not directory.is_absolute() or not all(p.is_file() for p in (topology, lock, map_yaml, mask)):
        raise RuntimeError("map_dir must be the existing absolute competition_v1 directory")
    start, goal = (LaunchConfiguration(n).perform(context) for n in ("start", "goal"))
    params = str(Path(get_package_share_directory("car_gazebo")) / "config/nav2_sim.yaml")
    map_node = Node(package="car_navigation", executable="car_map_publisher",
                    parameters=[{"use_sim_time": True, "map_yaml": str(map_yaml), "mask_yaml": str(mask)}],
                    remappings=[("/car/navigation/keepout_mask", "/keepout_filter_mask")], output="screen")
    # Existing scripts remain read-only and publish the same route and lock contracts.
    lock_node = ExecuteProcess(cmd=[sys.executable, str(lock), "--expected", "2"], output="screen")
    topology_node = ExecuteProcess(cmd=[sys.executable, str(topology), "--start", start,
                                        "--goal", goal, "--expected-obstacles", "2"], output="screen")
    controller = Node(package="nav2_controller", executable="controller_server",
                      name="controller_server", parameters=[params],
                      remappings=[("cmd_vel", "/car_gz/nav_cmd_vel")], output="screen")
    lifecycle = Node(package="nav2_lifecycle_manager", executable="lifecycle_manager",
                     name="lifecycle_manager_sim", parameters=[{"use_sim_time": True,
                         "autostart": True, "node_names": ["controller_server"]}], output="screen")
    route = Node(package="car_gazebo", executable="follow_locked_route.py",
                 parameters=[{"use_sim_time": True, "topology_file": str(topology)}], output="screen")
    processes = [map_node, lock_node, topology_node, controller, lifecycle, route]
    return [RegisterEventHandler(OnProcessExit(target_action=p,
               on_exit=[EmitEvent(event=Shutdown(reason="SIM navigation process exited"))]))
            for p in processes] + processes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("map_dir", description="Absolute competition_v1 directory"),
        DeclareLaunchArgument("start", default_value="C3"),
        DeclareLaunchArgument("goal", default_value="A1"),
        OpaqueFunction(function=setup),
    ])
