"""Adaptación de orientación para las vistas; no modifica los estimadores.

La rama compensada y DMP vienen de las odometrías originales. La rama sin
compensar integra únicamente el gyro de /imu_raw para la comparación visual.
El origen y el montaje se aplican después del cálculo, solo al modelo 3D.
"""
import numpy as np
from scipy.spatial.transform import Rotation as Rot

RAMAS = ('sin_compensar', 'compensado', 'dmp')


def rotacion(q):
    q = np.asarray(q, dtype=float)
    if q.shape != (4,) or not np.all(np.isfinite(q)):
        raise ValueError('Cuaternión no válido')
    norma = np.linalg.norm(q)
    if not np.isfinite(norma) or norma < 1e-12:
        raise ValueError('Cuaternión degenerado')
    return Rot.from_quat(q / norma)


def montaje_nivelado(inicial, eje_cabeceo_sensor):
    """Expresa los ejes del modelo en el sensor sin modificar su orientación.

    La nariz dibujada apunta hacia -X; girar sobre +Y la eleva. El montaje
    físico del TP4 usa +Z del MPU6050 para ese gesto. Para un módulo acostado
    se puede elegir +Y mediante el argumento eje_cabeceo_sensor del launch.
    """
    eje = np.asarray(eje_cabeceo_sensor, dtype=float)
    if eje.shape != (3,) or not np.all(np.isfinite(eje)) or np.linalg.norm(eje) < 1e-12:
        raise ValueError('El eje de cabeceo debe ser un vector no nulo de tres componentes')
    eje = inicial.apply(eje / np.linalg.norm(eje))
    if np.linalg.norm(eje[:2]) < .1:
        raise ValueError('Eje de cabeceo vertical: revisar el montaje o eje_cabeceo_sensor')
    giro = np.pi / 2 - np.arctan2(eje[1], eje[0])
    return (Rot.from_euler('z', giro) * inicial).inv()


class OrientacionesVista:
    def __init__(self, eje_cabeceo_sensor=(0., 0., 1.)):
        self.eje = eje_cabeceo_sensor
        self.montaje = None
        self.actuales, self.origenes = {}, {}
        self.t_raw = None

    def agregar(self, rama, q):
        r = rotacion(q)
        if rama == 'compensado' and self.montaje is None:
            self.montaje = montaje_nivelado(r, self.eje)
            # Igualar el instante de origen de las ramas que ya llegaron.
            self.origenes = self.actuales.copy()
        self.actuales[rama] = r
        if self.montaje is not None:
            self.origenes.setdefault(rama, r)

    def gyro_crudo(self, t, gyro, acel):
        g, a = np.asarray(gyro, dtype=float), np.asarray(acel, dtype=float)
        if (not np.isfinite(t) or t <= 0 or g.shape != (3,) or a.shape != (3,)
                or not np.all(np.isfinite(np.r_[g, a]))):
            return False
        if self.t_raw is not None and t <= self.t_raw:
            return False
        anterior, self.t_raw = self.t_raw, t
        if 'sin_compensar' not in self.actuales:
            if np.linalg.norm(a) < 1e-12:
                self.t_raw = None
                return False
            ax, ay, az = a
            r = Rot.from_euler('ZYX', [0., np.arctan2(-ax, np.hypot(ay, az)),
                                      np.arctan2(ay, az)])
        else:
            dt = t - anterior
            if dt > .2:
                return False
            # Misma composición de incrementos que el colaborativo, con gyro
            # crudo: sin restar sesgo ni corregir con acelerómetro.
            r = self.actuales['sin_compensar'] * Rot.from_rotvec(g * dt)
        self.agregar('sin_compensar', r.as_quat())
        return True

    def relativas(self):
        if self.montaje is None:
            return {}
        m = self.montaje
        return {k: m.inv() * self.origenes[k].inv() * r * m
                for k, r in self.actuales.items()}

    def fijar_origen(self):
        if self.montaje is None:
            return False
        self.origenes = self.actuales.copy()
        return True
