"""Error de un estimador contra la verdad simulada (/verdad/odom)."""
import numpy as np
from scipy.spatial.transform import Rotation as Rot


def error_contra_verdad(R, pos, vel, verdad):
    """Devuelve [error de actitud en grados, de velocidad en m/s, de posicion en m]."""
    q = verdad.pose.pose.orientation
    r_ver = Rot.from_quat([q.x, q.y, q.z, q.w])
    err_ang = np.degrees((Rot.from_matrix(R) * r_ver.inv()).magnitude())
    p = verdad.pose.pose.position
    v = verdad.twist.twist.linear
    err_pos = np.linalg.norm(pos - np.array([p.x, p.y, p.z]))
    err_vel = np.linalg.norm(vel - np.array([v.x, v.y, v.z]))
    return [float(err_ang), float(err_vel), float(err_pos)]
