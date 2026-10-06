"""Utilidades de rotaciones (reutilizadas del TP de microrotaciones)."""
import numpy as np


def euler_from_matrix(R):
    """Devuelve [roll, pitch, yaw] en grados para R = Rz·Ry·Rx.
    (Cerca de pitch = ±90° hay gimbal lock y la extracción es ambigua.)"""
    pitch = -np.arcsin(np.clip(R[2, 0], -1.0, 1.0))
    roll = np.arctan2(R[2, 1], R[2, 2])
    yaw = np.arctan2(R[1, 0], R[0, 0])
    return np.degrees([roll, pitch, yaw])


def wrap180(a):
    return (a + 180.0) % 360.0 - 180.0
