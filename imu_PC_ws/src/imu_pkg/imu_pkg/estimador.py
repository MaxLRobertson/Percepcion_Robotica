"""Metodo 1.1: actitud, velocidad y posicion solo con datos inerciales crudos.

- Actitud: filtro complementario tipo Mahony. El giroscopio se integra con
  microrotaciones exactas (exponencial de un vector de rotacion) y el
  acelerometro corrige roll y pitch usando la direccion de la gravedad.
  El yaw no es observable sin magnetometro: deriva con el sesgo residual.
- Velocidad y posicion: ver integrador.py.
"""
import numpy as np
from scipy.spatial.transform import Rotation as Rot

from imu_pkg.integrador import G, Integrador


class EstimadorCrudo:
    def __init__(self, kp=1.0, ki=0.05, tolerancia_g=0.5, zupt=False,
                 umbral_gyro=0.05, umbral_acel=0.3, ventana_zupt=0.2,
                 calibrar_acel=True):
        self.kp = kp                      # ganancia proporcional de la correccion
        self.ki = ki                      # ganancia integral (sesgo del giroscopio)
        self.tolerancia_g = tolerancia_g  # |a| debe estar en G ± tolerancia para corregir
        self.integrador = Integrador(zupt, umbral_gyro, umbral_acel,
                                     ventana_zupt, calibrar_acel)

        self.R = np.eye(3)
        self.sesgo_gyro = np.zeros(3)     # calibrado en reposo
        self.integral = np.zeros(3)       # estimacion adicional de sesgo (Mahony)
        self._calib = []
        self.calibrado = False

    @property
    def pos(self):
        return self.integrador.pos

    @property
    def vel(self):
        return self.integrador.vel

    # ---- calibracion en reposo -------------------------------------------
    def acumular_calibracion(self, gyro, acel):
        self._calib.append(np.concatenate([gyro, acel]))

    def finalizar_calibracion(self):
        """Sesgo del giroscopio y orientacion inicial a partir del reposo."""
        m = np.mean(self._calib, axis=0)
        self.sesgo_gyro = m[:3]
        ax, ay, az = m[3:]
        roll = np.arctan2(ay, az)
        pitch = np.arctan2(-ax, np.hypot(ay, az))
        self.R = Rot.from_euler("ZYX", [0.0, pitch, roll]).as_matrix()
        self.integrador.calibrar(m[3:])
        self.calibrado = True

    # ---- paso de estimacion ----------------------------------------------
    def actualizar(self, gyro, acel, dt):
        w = gyro - self.sesgo_gyro
        acel = self.integrador.corregir_acel(self.R, acel)

        # Correccion con la gravedad, solo si el acelerometro mide ~1 g.
        norma = np.linalg.norm(acel)
        if abs(norma - G) < self.tolerancia_g:
            arriba_medido = acel / norma
            arriba_estimado = self.R[2, :]        # R^T · z en ejes del sensor
            e = np.cross(arriba_medido, arriba_estimado)
            self.integral += self.ki * e * dt
            w = w + self.kp * e + self.integral

        self.R = self.R @ Rot.from_rotvec(w * dt).as_matrix()
        self.integrador.integrar(self.R, acel, dt, np.linalg.norm(w))

    def cuaternion(self):
        """(x, y, z, w)"""
        return Rot.from_matrix(self.R).as_quat()
