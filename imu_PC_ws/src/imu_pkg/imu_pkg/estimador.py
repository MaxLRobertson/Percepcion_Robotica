"""Metodo 1.1: actitud, velocidad y posicion solo con datos inerciales crudos.

- Actitud: filtro complementario tipo Mahony. El giroscopio se integra con
  microrotaciones exactas (exponencial de un vector de rotacion) y el
  acelerometro corrige roll y pitch usando la direccion de la gravedad.
  El yaw no es observable sin magnetometro: deriva con el sesgo residual.
- Velocidad y posicion: se rota la aceleracion al marco del mundo, se resta
  la gravedad e integra dos veces.
"""
import numpy as np
from scipy.spatial.transform import Rotation as Rot

G = 9.81
Z = np.array([0.0, 0.0, 1.0])


class EstimadorCrudo:
    def __init__(self, kp=1.0, ki=0.05, tolerancia_g=0.5, zupt=False,
                 umbral_gyro=0.05, umbral_acel=0.3, ventana_zupt=0.2,
                 calibrar_acel=True):
        self.kp = kp                      # ganancia proporcional de la correccion
        self.ki = ki                      # ganancia integral (sesgo del giroscopio)
        self.tolerancia_g = tolerancia_g  # |a| debe estar en G ± tolerancia para corregir
        self.zupt = zupt
        self.calibrar_acel = calibrar_acel  # corrige el sesgo vertical del acelerometro
        self.umbral_gyro = umbral_gyro    # [rad/s]
        self.umbral_acel = umbral_acel    # [m/s²]
        self.ventana_zupt = ventana_zupt  # [s] que debe sostenerse el reposo

        self.R = np.eye(3)
        self.pos = np.zeros(3)
        self.vel = np.zeros(3)
        self.sesgo_gyro = np.zeros(3)     # calibrado en reposo
        self.integral = np.zeros(3)       # estimacion adicional de sesgo (Mahony)
        self.sesgo_acel_vert = 0.0        # [m/s²] sesgo a lo largo de la gravedad
        self._calib = []
        self._t_quieto = 0.0
        self.calibrado = False

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
        # En reposo el modulo del acelerometro deberia ser G. Lo que sobra o
        # falta es el sesgo en la direccion vertical (el unico observable aca).
        if self.calibrar_acel:
            self.sesgo_acel_vert = float(np.linalg.norm(m[3:]) - G)
        self.calibrado = True

    # ---- paso de estimacion ----------------------------------------------
    def actualizar(self, gyro, acel, dt):
        w = gyro - self.sesgo_gyro
        # El sesgo vertical se resta a lo largo de la vertical estimada (en ejes del sensor).
        acel = acel - self.sesgo_acel_vert * self.R[2, :]

        # Correccion con la gravedad, solo si el acelerometro mide ~1 g.
        norma = np.linalg.norm(acel)
        if abs(norma - G) < self.tolerancia_g:
            arriba_medido = acel / norma
            arriba_estimado = self.R[2, :]        # R^T · z en ejes del sensor
            e = np.cross(arriba_medido, arriba_estimado)
            self.integral += self.ki * e * dt
            w = w + self.kp * e + self.integral

        self.R = self.R @ Rot.from_rotvec(w * dt).as_matrix()

        # Doble integracion de la aceleracion en el marco del mundo.
        a_mundo = self.R @ acel - G * Z
        self.vel += a_mundo * dt
        self.pos += self.vel * dt

        if self.zupt:
            self._aplicar_zupt(w, norma, dt)

    def _aplicar_zupt(self, w, norma_acel, dt):
        quieto = (np.linalg.norm(w) < self.umbral_gyro
                  and abs(norma_acel - G) < self.umbral_acel)
        self._t_quieto = self._t_quieto + dt if quieto else 0.0
        if self._t_quieto >= self.ventana_zupt:
            self.vel[:] = 0.0

    def cuaternion(self):
        """(x, y, z, w)"""
        return Rot.from_matrix(self.R).as_quat()
