"""Navegacao no veiculo (notebook): gamepad + mode_manager + Stanley.

    ros2 launch vida_navigation navigation.launch.py path_file:=/caminho/pista.csv

O gamepad passa a publicar em /gamepad/* e o mode_manager e o unico que
publica as referencias do ESP32. O EKF roda no Odroid.
"""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def navigation_nodes(path_file, config):
    params = [config, {"path_file": path_file}]
    return [
        Node(package="vida_navigation", executable="stanley", name="stanley",
             parameters=params, output="screen"),
        Node(package="vida_navigation", executable="mode_manager", name="mode_manager",
             parameters=params, output="screen"),
        Node(package="vida_navigation", executable="path_publisher", name="path_publisher",
             parameters=[{"path_file": path_file}], output="screen"),
    ]


def generate_launch_description():
    share = Path(get_package_share_directory("vida_navigation"))
    path_file = LaunchConfiguration("path_file")
    # "nav_config", e nao "config": argumentos de launch sao compartilhados com os
    # launches incluidos, e o do gamepad usa "config" para o gamepad.yaml (com o
    # mesmo nome, o gamepad recebia o navigation.yaml e caia nos padroes do
    # codigo, 1 m/s; ensaio de 30/10).
    config = LaunchConfiguration("nav_config")
    gamepad = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(Path(get_package_share_directory("control_pkg")) / "launch" / "gamepad_control.launch.py")),
        launch_arguments={"device_id": LaunchConfiguration("device_id"),
                          "steering_reference_topic": "/gamepad/steering/reference",
                          "drive_reference_topic": "/gamepad/drive/reference"}.items(),
        condition=IfCondition(LaunchConfiguration("gamepad")))
    return LaunchDescription([
        DeclareLaunchArgument("path_file", description="caminho (CSV vida_path v1)"),
        DeclareLaunchArgument("nav_config", default_value=str(share / "config" / "navigation.yaml")),
        DeclareLaunchArgument("gamepad", default_value="true"),
        DeclareLaunchArgument("device_id", default_value="0"),
        gamepad,
        *navigation_nodes(path_file, config),
    ])
