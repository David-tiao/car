from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Foreground-owned simulator only. No ROS /clock, /scan, /odom or /cmd_vel bridge.
    return LaunchDescription([
        DeclareLaunchArgument("world", description="Absolute generated SDF path"),
        DeclareLaunchArgument("partition", description="Unique Ignition Transport partition"),
        ExecuteProcess(
            cmd=["ign", "gazebo", "-r", LaunchConfiguration("world")],
            additional_env={"IGN_PARTITION": LaunchConfiguration("partition")},
            output="screen",
        ),
    ])
