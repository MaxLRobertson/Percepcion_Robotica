"""Tres aviones: gyro crudo, filtro Mahony colaborativo y salida DMP."""
from imu_vistas.launch_vistas import generar


def generate_launch_description():
    return generar('aviones')
