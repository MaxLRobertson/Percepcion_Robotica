"""Integracion de aceleracion -> velocidad y posicion, compartida por 1.1 y 1.2.

La actitud (R: sensor -> mundo) la pone quien llama: el filtro propio en el
metodo 1.1 o el DMP del chip en el 1.2.
"""
import numpy as np

G = 9.81
Z = np.array([0.0, 0.0, 1.0])


class Integrador:
    def __init__(self, zupt=False, umbral_gyro=0.05, umbral_acel=0.3,
                 ventana_zupt=0.2, calibrar_acel=True):
        self.zupt = zupt
        self.umbral_gyro = umbral_gyro    # [rad/s]
        self.umbral_acel = umbral_acel    # [m/s²]
        self.ventana_zupt = ventana_zupt  # [s] que debe sostenerse el reposo
        self.calibrar_acel = calibrar_acel
        self.pos = np.zeros(3)
        self.vel = np.zeros(3)
        self.sesgo_acel_vert = 0.0        # [m/s²] sesgo a lo largo de la gravedad
        self._t_quieto = 0.0

    def calibrar(self, acel_media_reposo):
        """En reposo |a| deberia ser G: lo que sobra es el sesgo vertical."""
        if self.calibrar_acel:
            self.sesgo_acel_vert = float(np.linalg.norm(acel_media_reposo) - G)

    def corregir_acel(self, R, acel):
        """Resta el sesgo vertical a lo largo de la vertical (ejes del sensor)."""
        return acel - self.sesgo_acel_vert * R[2, :]

    def integrar(self, R, acel_corregida, dt, w_norm):
        """R: actitud actual; w_norm: |velocidad angular| sin sesgo [rad/s]."""
        a_mundo = R @ acel_corregida - G * Z
        self.vel += a_mundo * dt
        self.pos += self.vel * dt
        if self.zupt:
            quieto = (w_norm < self.umbral_gyro
                      and abs(np.linalg.norm(acel_corregida) - G) < self.umbral_acel)
            self._t_quieto = self._t_quieto + dt if quieto else 0.0
            if self._t_quieto >= self.ventana_zupt:
                self.vel[:] = 0.0
