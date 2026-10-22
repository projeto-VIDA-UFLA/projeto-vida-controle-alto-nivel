"""Navegacao com o simulador no lugar de ESP32 + GPS + EKF.

    ros2 launch vida_navigation simulation.launch.py
    ros2 launch vida_navigation simulation.launch.py gamepad:=true   # dirigir com o controle
    ros2 launch vida_navigation simulation.launch.py start_back_m:=5.0 encoder_scale_error:=0.07

Sem gamepad, os pedidos e as referencias manuais podem vir do sim_scenarios
ou de `ros2 topic pub` em /gamepad/* e /navigation/auto_request.
"""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory("vida_navigation"))
    path_file = LaunchConfiguration("path_file")
    sim_args = ["start_back_m", "start_left_m", "start_yaw_deg", "encoder_scale_error", "gps_noise_m",
                "gnss_scenario"]
    defaults = {"start_back_m": "0.0", "start_left_m": "0.0", "start_yaw_deg": "0.0",
                "encoder_scale_error": "0.07", "gps_noise_m": "0.02", "gnss_scenario": "fixed"}
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(share / "launch" / "navigation.launch.py")),
        launch_arguments={"path_file": path_file, "gamepad": LaunchConfiguration("gamepad")}.items())
    return LaunchDescription([
        DeclareLaunchArgument("path_file", default_value=str(share / "paths" / "exemplo_estadio.csv")),
        DeclareLaunchArgument("gamepad", default_value="false"),
        *[DeclareLaunchArgument(name, default_value=defaults[name]) for name in sim_args],
        Node(package="vida_navigation", executable="sim_vehicle", name="sim_vehicle", output="screen",
             parameters=[{"path_file": path_file,
                          **{name: LaunchConfiguration(name) for name in sim_args}}]),
        navigation,
    ])
