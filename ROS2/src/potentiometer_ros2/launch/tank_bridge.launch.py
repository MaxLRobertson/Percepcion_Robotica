"""Inicia el puente entre los sensores del ESP32 y el controlador del robot."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "encoder_angle_min_deg",
                default_value="0.0",
                description="Angulo del AS5600 (ya calibrado) que equivale a 0%",
            ),
            DeclareLaunchArgument(
                "encoder_angle_max_deg",
                default_value="360.0",
                description="Angulo del AS5600 (ya calibrado) que equivale a 100%",
            ),
            Node(
                package="potentiometer_ros2",
                executable="tank_bridge",
                name="tank_bridge",
                output="screen",
                parameters=[
                    {
                        "encoder_angle_min_deg": ParameterValue(
                            LaunchConfiguration("encoder_angle_min_deg"),
                            value_type=float,
                        )
                    },
                    {
                        "encoder_angle_max_deg": ParameterValue(
                            LaunchConfiguration("encoder_angle_max_deg"),
                            value_type=float,
                        )
                    },
                ],
            ),
        ]
    )
