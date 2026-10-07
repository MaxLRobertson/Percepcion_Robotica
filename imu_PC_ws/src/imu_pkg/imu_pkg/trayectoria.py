"""Trayectoria de referencia analitica para simular un MPU-6050.

Actitud: R = Rz(yaw)·Ry(pitch)·Rx(roll), cada ángulo senoidal (parte de 0).
Posición: vaivén recto sobre el eje X del mundo, parte del reposo en el origen.
Los primeros T_REPOSO segundos el sensor está quieto (para calibrar).
Con la verdad conocida se puede medir el error de cualquier estimador.
"""
import numpy as np

G = 9.81  # m/s²

# Amplitud [rad] y frecuencia [Hz] de cada ángulo, y vaivén en X.
AMP_RPY = np.radians([30.0, 20.0, 45.0])
FREC_RPY = np.array([0.20, 0.15, 0.10])
VAIVEN_L = 0.5      # recorrido total [m]
VAIVEN_F = 0.5      # frecuencia [Hz]
T_REPOSO = 2.0      # el sensor arranca quieto este tiempo [s]
# Modo con pausas: MOV_S debe ser entero para que cada parada caiga cuando la
# velocidad del vaivén es cero (su semiperíodo es 1 s).
MOV_S = 4.0         # tramo en movimiento [s]
PAUSA_S = 3.0       # tramo quieto [s]


def _rot(roll, pitch, yaw):
    sr, cr = np.sin(roll), np.cos(roll)
    sp, cp = np.sin(pitch), np.cos(pitch)
    sy, cy = np.sin(yaw), np.cos(yaw)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp,     cp * sr,                cp * cr],
    ])


def estado_verdadero(t, pausas=False):
    """Devuelve (R, pos, vel, gyro_cuerpo, fuerza_especifica_cuerpo) en t.

    pausas=True: tras el reposo inicial alterna MOV_S de movimiento con PAUSA_S
    quieto (la trayectoria "se congela" y gyro, velocidad y aceleración valen 0).

    gyro_cuerpo: velocidad angular [rad/s] en ejes del sensor.
    fuerza_especifica_cuerpo: lo que mide un acelerómetro ideal [m/s²],
    es decir R^T (a_mundo + g·z).
    """
    if t < T_REPOSO:
        return np.eye(3), np.zeros(3), np.zeros(3), np.zeros(3), np.array([0.0, 0.0, G])
    t = t - T_REPOSO
    quieto = False
    if pausas:
        ciclo = MOV_S + PAUSA_S
        n, resto = divmod(t, ciclo)
        quieto = resto >= MOV_S
        t = n * MOV_S + min(resto, MOV_S)   # tiempo de trayectoria "efectivo"
    w = 2.0 * np.pi * FREC_RPY
    rpy = AMP_RPY * np.sin(w * t)
    rpy_p = AMP_RPY * w * np.cos(w * t)
    roll, pitch, yaw = rpy
    dr, dp, dy = rpy_p
    R = _rot(roll, pitch, yaw)

    sr, cr = np.sin(roll), np.cos(roll)
    sp, cp = np.sin(pitch), np.cos(pitch)
    gyro = np.array([
        dr - dy * sp,
        dp * cr + dy * cp * sr,
        -dp * sr + dy * cp * cr,
    ])

    wv = 2.0 * np.pi * VAIVEN_F
    pos = np.array([0.5 * VAIVEN_L * (1.0 - np.cos(wv * t)), 0.0, 0.0])
    vel = np.array([0.5 * VAIVEN_L * wv * np.sin(wv * t), 0.0, 0.0])
    acc = np.array([0.5 * VAIVEN_L * wv ** 2 * np.cos(wv * t), 0.0, 0.0])

    if quieto:
        gyro, vel, acc = np.zeros(3), np.zeros(3), np.zeros(3)

    fuerza = R.T @ (acc + np.array([0.0, 0.0, G]))
    return R, pos, vel, gyro, fuerza
