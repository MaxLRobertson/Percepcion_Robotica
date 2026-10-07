"""Conversion de la orientacion que entrega el DMP a una matriz de rotacion.

UNICO lugar que depende de la convencion de la cuaternion del chip. Se asume
la convencion estandar de ROS (x, y, z, w) y que rota del sensor al mundo.
Con el MPU-6050 real hay que verificarlo: las librerias del DMP suelen
entregar (w, x, y, z), a veces con otros ejes o signo. Si es asi, se corrige
aca (o en el firmware) y nada mas.
"""
import numpy as np
from scipy.spatial.transform import Rotation as Rot


def rotacion_desde_dmp(q_xyzw):
    return Rot.from_quat(q_xyzw).as_matrix()


def quitar_yaw_inicial(R_dmp0):
    """Rotacion Rz(-yaw0) que deja en 0 el rumbo que tenia el DMP al empezar.

    El DMP no tiene magnetometro: su yaw es relativo a donde arranco. Para que
    el mundo de ambos metodos coincida se toma el rumbo inicial como yaw = 0.
    """
    yaw0 = Rot.from_matrix(R_dmp0).as_euler("ZYX")[0]
    c, s = np.cos(-yaw0), np.sin(-yaw0)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
