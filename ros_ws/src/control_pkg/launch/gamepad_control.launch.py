from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_config = str(
        Path(get_package_share_directory("control_pkg")) / "config" / "gamepad.yaml"
    )

    config = LaunchConfiguration("config")
    device_id = LaunchConfiguration("device_id")

    return LaunchDescription(
        [
            DeclareLaunchArgument("config", default_value=default_config),
            DeclareLaunchArgument("device_id", default_value="0"),
            Node(
                package="joy",
                executable="joy_node",
                name="joy_node",
                namespace="gamepad",
                parameters=[config, {"device_id": device_id}],
                output="screen",
            ),
            Node(
                package="control_pkg",
                executable="gamepad_reader",
                name="control",
                namespace="gamepad",
                parameters=[config],
                output="screen",
            ),
        ]
    )
