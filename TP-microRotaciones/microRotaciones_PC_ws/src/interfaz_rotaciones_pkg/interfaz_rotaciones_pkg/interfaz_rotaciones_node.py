#!/usr/bin/env python3
"""
Nodo ROS2 (Jazzy) + GUI PyQt5/Matplotlib para comparar rotación directa
vs. microrotaciones calculadas en una ESP32-S3 (micro-ROS).

- Servicio cliente : rotacion_srv   (rot_interfaces/srv/Rotacion)
- Subscripciones   : mat_orient        (Float32MultiArray, 9 elem, row-major)
                     mat_orient_micro  (Float32MultiArray, 9 elem, row-major)

Convención: R = Rz(yaw) @ Ry(pitch) @ Rx(roll)
"""
import sys
import threading

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float32MultiArray

# from rot_interfaces.srv import Rotacion

from PyQt5 import QtWidgets, QtCore
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure


# ----------------------------------------------------------------------------
# Utilidades matemáticas
# ----------------------------------------------------------------------------
def euler_from_matrix(R):
    """Devuelve [roll, pitch, yaw] en grados para R = Rz·Ry·Rx.
    (Cerca de pitch = ±90° hay gimbal lock y la extracción es ambigua.)"""
    pitch = -np.arcsin(np.clip(R[2, 0], -1.0, 1.0))
    roll = np.arctan2(R[2, 1], R[2, 2])
    yaw = np.arctan2(R[1, 0], R[0, 0])
    return np.degrees([roll, pitch, yaw])


def wrap180(a):
    return (a + 180.0) % 360.0 - 180.0


# ----------------------------------------------------------------------------
# Señales Qt (puente entre el hilo de ROS y el hilo de la GUI)
# ----------------------------------------------------------------------------
class Senales(QtCore.QObject):
    mat_directa = QtCore.pyqtSignal(object)
    mat_micro = QtCore.pyqtSignal(object)
    # respuesta = QtCore.pyqtSignal(bool, str)
    estado = QtCore.pyqtSignal(str)


# ----------------------------------------------------------------------------
# Nodo ROS2
# ----------------------------------------------------------------------------
class RotNode(Node):
    def __init__(self, senales):
        super().__init__('rot_pc_node')
        self.senales = senales

        # self.cli = self.create_client(Rotacion, 'rotacion_srv')

        self.create_subscription(
            Float32MultiArray, 'mat_orient',
            self.cb_directa, qos_profile_sensor_data)
        self.create_subscription(
            Float32MultiArray, 'mat_orient_micro',
            self.cb_micro, qos_profile_sensor_data)

        self.pub_cmd = self.create_publisher(Float32MultiArray, 'rot_cmd', 10)

    # def enviar(self, vec, ang_deg, pasos):
    #     msg = Float32MultiArray()
    #     msg.data = [*map(float, vec), *map(float, ang_deg), float(pasos)]
    #     self.pub_cmd.publish(msg)

    def enviar(self, vec, ang_deg, pasos):
        if self.pub_cmd.get_subscription_count() == 0:
            self.senales.estado.emit('Nadie escucha en rot_cmd (¿está conectada la ESP32?)')
            return False
        msg = Float32MultiArray()
        msg.data = [*map(float, vec), *map(float, ang_deg), float(pasos)]
        self.pub_cmd.publish(msg)
        self.senales.estado.emit('Comando enviado, esperando matrices de la ESP32...')
        return True

    @staticmethod
    def _a_matriz(msg):
        if len(msg.data) != 9:
            return None
        return np.array(msg.data, dtype=float).reshape(3, 3)

    def cb_directa(self, msg):
        R = self._a_matriz(msg)
        if R is not None:
            self.senales.mat_directa.emit(R)

    def cb_micro(self, msg):
        R = self._a_matriz(msg)
        if R is not None:
            self.senales.mat_micro.emit(R)

    # def enviar(self, vec, ang_deg, pasos):
    #     if not self.cli.service_is_ready():
    #         self.senales.estado.emit('Servicio rotacion_srv no disponible (¿está corriendo micro-ROS?)')
    #         return False
    #     req = Rotacion.Request()
    #     req.vector.x, req.vector.y, req.vector.z = map(float, vec)
    #     req.angulos.x, req.angulos.y, req.angulos.z = map(float, ang_deg)
    #     req.pasos = int(pasos)
    #     fut = self.cli.call_async(req)
    #     fut.add_done_callback(self._cb_resp)
    #     self.senales.estado.emit('Solicitud enviada, esperando a la ESP32...')
    #     return True

    # def _cb_resp(self, fut):
    #     try:
    #         r = fut.result()
    #         self.senales.respuesta.emit(r.success, r.message)
    #     except Exception as e:  # noqa
    #         self.senales.respuesta.emit(False, f'Error en el servicio: {e}')


# ----------------------------------------------------------------------------
# Widget de un gráfico 3D con etiqueta debajo
# ----------------------------------------------------------------------------
class Plot3D(QtWidgets.QWidget):
    def __init__(self, titulo):
        super().__init__()
        self.titulo = titulo
        self.fig = Figure(figsize=(4, 4))
        self.canvas = FigureCanvas(self.fig)
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.label = QtWidgets.QLabel(' ')
        self.label.setAlignment(QtCore.Qt.AlignCenter)
        self.label.setWordWrap(True)

        lay = QtWidgets.QVBoxLayout(self)
        lay.addWidget(self.canvas)
        lay.addWidget(self.label)
        self.dibujar(None, 1.0)

    def dibujar(self, v, lim, color='k'):
        ax = self.ax
        ax.cla()
        ax.set_xlim(-lim, lim)
        ax.set_ylim(-lim, lim)
        ax.set_zlim(-lim, lim)
        ax.set_box_aspect((1, 1, 1))
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        ax.set_title(self.titulo)
        # Ejes cartesianos de referencia
        for i, c in enumerate(('r', 'g', 'b')):
            e = np.zeros(3)
            e[i] = lim
            ax.quiver(0, 0, 0, *e, color=c, alpha=0.35,
                      arrow_length_ratio=0.08, linewidth=1)
        if v is not None:
            ax.quiver(0, 0, 0, *v, color=color, linewidth=2.5,
                      arrow_length_ratio=0.12)
        self.canvas.draw_idle()

    def texto(self, t):
        self.label.setText(t)


# ----------------------------------------------------------------------------
# Ventana principal
# ----------------------------------------------------------------------------
class Ventana(QtWidgets.QWidget):
    def __init__(self, nodo, senales):
        super().__init__()
        self.nodo = nodo
        self.setWindowTitle('Rotación directa vs. microrotaciones (ESP32-S3)')
        self.resize(1300, 750)

        self.v = np.array([1.0, 0.0, 0.0])
        self.ang = np.array([0.0, 0.0, 90.0])
        self.R_dir = None
        self.R_mic = None
        self.lim = 1.0

        # ---- Entradas ----
        def spin(val, lo=-1000.0, hi=1000.0, dec=3):
            s = QtWidgets.QDoubleSpinBox()
            s.setRange(lo, hi)
            s.setDecimals(dec)
            s.setValue(val)
            return s

        self.sp_v = [spin(1.0), spin(0.0), spin(0.0)]
        self.sp_a = [spin(0.0, -360, 360), spin(0.0, -360, 360), spin(90.0, -360, 360)]
        self.sp_pasos = QtWidgets.QSpinBox()
        self.sp_pasos.setRange(1, 100000)
        self.sp_pasos.setValue(10)
        self.btn = QtWidgets.QPushButton('Enviar a la ESP32')
        self.btn.clicked.connect(self.enviar)

        form = QtWidgets.QHBoxLayout()
        for nombre, spins in (('Vector', self.sp_v), ('Ángulos [°]', self.sp_a)):
            form.addWidget(QtWidgets.QLabel(f'<b>{nombre}</b>'))
            for eje, s in zip(('x', 'y', 'z'), spins):
                form.addWidget(QtWidgets.QLabel(eje))
                form.addWidget(s)
            form.addSpacing(20)
        form.addWidget(QtWidgets.QLabel('<b>Pasos</b>'))
        form.addWidget(self.sp_pasos)
        form.addSpacing(20)
        form.addWidget(self.btn)
        form.addStretch()

        # ---- Gráficos ----
        self.p1 = Plot3D('Original')
        self.p2 = Plot3D('Rotación directa')
        self.p3 = Plot3D('Microrotaciones')
        graficos = QtWidgets.QHBoxLayout()
        for p in (self.p1, self.p2, self.p3):
            graficos.addWidget(p)

        self.lbl_error = QtWidgets.QLabel('Error de microrotación vs. directa: -')
        self.lbl_error.setAlignment(QtCore.Qt.AlignCenter)
        self.lbl_error.setStyleSheet('font-size: 14px; font-weight: bold;')
        self.lbl_estado = QtWidgets.QLabel('Listo.')

        lay = QtWidgets.QVBoxLayout(self)
        lay.addLayout(form)
        lay.addLayout(graficos)
        lay.addWidget(self.lbl_error)
        lay.addWidget(self.lbl_estado)

        # ---- Conexiones ROS -> GUI ----
        senales.mat_directa.connect(self.on_directa)
        senales.mat_micro.connect(self.on_micro)
        # senales.respuesta.connect(self.on_respuesta)
        senales.estado.connect(self.lbl_estado.setText)

        self.refrescar()

    # -- Acciones --
    def enviar(self):
        self.v = np.array([s.value() for s in self.sp_v])
        self.ang = np.array([s.value() for s in self.sp_a])
        pasos = self.sp_pasos.value()
        self.R_dir = None
        self.R_mic = None
        self.lim = max(np.linalg.norm(self.v), 1e-3) * 1.2
        self.refrescar()
        self.nodo.enviar(self.v, self.ang, pasos)

    # def on_respuesta(self, ok, msg):
    #     self.lbl_estado.setText(f"Respuesta ESP32: {'OK' if ok else 'ERROR'} - {msg}")

    def on_directa(self, R):
        self.R_dir = R
        self.refrescar()

    def on_micro(self, R):
        self.R_mic = R
        self.refrescar()

    # -- Dibujo --
    def refrescar(self):
        a = self.ang
        self.p1.dibujar(self.v, self.lim, 'k')
        self.p1.texto('Sin rotación<br>Rx=0°  Ry=0°  Rz=0°')

        ang_dir = ang_mic = None

        if self.R_dir is not None:
            self.p2.dibujar(self.R_dir @ self.v, self.lim, 'tab:blue')
            ang_dir = euler_from_matrix(self.R_dir)
            self.p2.texto('Rx={:.3f}°  Ry={:.3f}°  Rz={:.3f}°'.format(*ang_dir))
        else:
            self.p2.dibujar(None, self.lim)
            self.p2.texto('Esperando mat_orient... (pedido: '
                          'Rx={:.1f}° Ry={:.1f}° Rz={:.1f}°)'.format(*a))

        if self.R_mic is not None:
            self.p3.dibujar(self.R_mic @ self.v, self.lim, 'tab:red')
            ang_mic = euler_from_matrix(self.R_mic)
            self.p3.texto('Rx={:.3f}°  Ry={:.3f}°  Rz={:.3f}°'.format(*ang_mic))
        else:
            self.p3.dibujar(None, self.lim)
            self.p3.texto('Esperando mat_orient_micro...')

        if ang_dir is not None and ang_mic is not None:
            partes = []
            for nombre, d, m in zip(('Rx', 'Ry', 'Rz'), ang_dir, ang_mic):
                dif = abs(wrap180(m - d))
                if abs(d) > 1e-6:
                    partes.append(f'{nombre}: {dif / abs(d) * 100:.4f} %')
                else:
                    partes.append(f'{nombre}: ref=0° (err. abs. {dif:.4f}°)')
            self.lbl_error.setText('Error microrotación vs. directa  |  ' + '   '.join(partes))
        else:
            self.lbl_error.setText('Error de microrotación vs. directa: -')


# ----------------------------------------------------------------------------
def main():
    rclpy.init()
    app = QtWidgets.QApplication(sys.argv)

    senales = Senales()
    nodo = RotNode(senales)
    ventana = Ventana(nodo, senales)

    def spin():
        try:
            rclpy.spin(nodo)
        except Exception:
            pass

    hilo = threading.Thread(target=spin, daemon=True)
    hilo.start()

    ventana.show()
    codigo = app.exec_()

    nodo.destroy_node()
    rclpy.shutdown()
    sys.exit(codigo)


if __name__ == '__main__':
    main()
