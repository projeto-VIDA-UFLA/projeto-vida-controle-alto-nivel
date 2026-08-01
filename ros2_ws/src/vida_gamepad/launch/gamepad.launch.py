from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    package_share = Path(get_package_share_directory("vida_gamepad"))
    parameters_file = package_share / "config" / "gamepad.yaml"

    return LaunchDescription([
        Node(
            package="vida_gamepad",
            executable="gamepad_node",
            name="vida_gamepad_node",
            output="screen",
            parameters=[str(parameters_file)],
        )
    ])
