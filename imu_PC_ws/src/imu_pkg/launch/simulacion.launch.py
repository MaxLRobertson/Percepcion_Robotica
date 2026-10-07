"""Simulador de IMU + metodo 1.1 + RViz2."""
import os
import re

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _prefijo_sin_snap() -> str:
    """Quita las variables que VS Code (snap) deja en el entorno: con ellas
    RViz2 carga bibliotecas del snap y muere con 'symbol lookup error'."""
    patron = re.compile(r"snap|^GTK_|^GIO_|^GDK_PIXBUF|^GSETTINGS|^LOCPATH", re.IGNORECASE)
    vars_snap = sorted(v for v in os.environ if patron.search(v))
    if not vars_snap:
        return ""
    return "env " + " ".join(f"-u {v}" for v in vars_snap) + " "


def generate_launch_description() -> LaunchDescription:
    rviz_cfg = os.path.join(get_package_share_directory("imu_pkg"), "rviz", "imu.rviz")
    return LaunchDescription(
        [
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument("zupt", default_value="false"),
            DeclareLaunchArgument("pausas", default_value="false"),
            DeclareLaunchArgument("calibrar_acel", default_value="true"),
            DeclareLaunchArgument("kp", default_value="1.0"),
            DeclareLaunchArgument("ki", default_value="0.05"),
            Node(
                package="imu_pkg",
                executable="simulador_imu",
                output="screen",
                parameters=[{
                    "pausas": ParameterValue(LaunchConfiguration("pausas"), value_type=bool),
                }],
            ),
            Node(
                package="imu_pkg",
                executable="metodo_crudo",
                output="screen",
                parameters=[{
                    "zupt": ParameterValue(LaunchConfiguration("zupt"), value_type=bool),
                    "calibrar_acel": ParameterValue(
                        LaunchConfiguration("calibrar_acel"), value_type=bool),
                    "kp": ParameterValue(LaunchConfiguration("kp"), value_type=float),
                    "ki": ParameterValue(LaunchConfiguration("ki"), value_type=float),
                }],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                arguments=["-d", rviz_cfg],
                prefix=_prefijo_sin_snap(),
                condition=IfCondition(LaunchConfiguration("rviz")),
            ),
        ]
    )
