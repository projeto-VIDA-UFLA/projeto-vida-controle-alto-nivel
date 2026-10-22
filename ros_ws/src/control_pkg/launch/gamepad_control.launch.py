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
    steering_topic = LaunchConfiguration("steering_reference_topic")
    drive_topic = LaunchConfiguration("drive_reference_topic")

    return LaunchDescription(
        [
            DeclareLaunchArgument("config", default_value=default_config),
            DeclareLaunchArgument("device_id", default_value="0"),
            # A navegacao (vida_navigation navigation.launch.py) passa /gamepad/*.
            DeclareLaunchArgument("steering_reference_topic",
                                  default_value="/vida/control/steering/reference"),
            DeclareLaunchArgument("drive_reference_topic",
                                  default_value="/vida/control/drive/reference"),
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
                parameters=[config, {"steering_reference_topic": steering_topic,
                                     "drive_reference_topic": drive_topic}],
                output="screen",
            ),
        ]
    )
