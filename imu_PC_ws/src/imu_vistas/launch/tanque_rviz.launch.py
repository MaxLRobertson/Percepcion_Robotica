"""Tanque fijo en RViz: orientación de /metodo1/odom (filtro colaborativo)."""
from imu_vistas.launch_vistas import generar


def generate_launch_description():
    return generar('tanque')
