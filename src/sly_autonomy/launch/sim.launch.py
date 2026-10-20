"""Start the disabled controller with isolated command output by default."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = str(Path(get_package_share_directory("sly_autonomy")) / "config" / "sim.yaml")
    return LaunchDescription([
        DeclareLaunchArgument("config", default_value=config),
        DeclareLaunchArgument("odom_topic", default_value="/sly/odom"),
        DeclareLaunchArgument("command_topic", default_value="/sly/sim/ackermann_cmd"),
        Node(package="sly_autonomy", executable="sim_controller", output="screen",
             parameters=[LaunchConfiguration("config")],
             remappings=[("/sly/odom", LaunchConfiguration("odom_topic")),
                         ("/sly/sim/ackermann_cmd", LaunchConfiguration("command_topic"))]),
    ])
