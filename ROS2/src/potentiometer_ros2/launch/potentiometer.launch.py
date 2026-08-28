"""Inicia el dashboard del potenciómetro."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "history_seconds",
                default_value="60.0",
                description="Segundos visibles en las gráficas",
            ),
            DeclareLaunchArgument(
                "csv_path",
                default_value="mediciones_potenciometro.csv",
                description="Archivo que usa el botón de grabación CSV",
            ),
            Node(
                package="potentiometer_ros2",
                executable="pot_dashboard",
                name="potentiometer_dashboard",
                output="screen",
                parameters=[
                    {
                        "history_seconds": ParameterValue(
                            LaunchConfiguration("history_seconds"), value_type=float
                        )
                    },
                    {"csv_path": LaunchConfiguration("csv_path")},
                ],
            ),
        ]
    )
