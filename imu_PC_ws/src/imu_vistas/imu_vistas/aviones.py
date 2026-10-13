"""Tres aviones centrados que solo muestran orientación de la placa."""
import signal
import sys
import threading
import time
import warnings

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from rclpy.qos import qos_profile_sensor_data
from rclpy.node import Node
from scipy.spatial.transform import Rotation as Rot
from PyQt5 import QtCore, QtWidgets
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg
from matplotlib.figure import Figure
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

from imu_vistas.avion import triangulos
from imu_vistas.orientaciones import RAMAS, OrientacionesVista

NOMBRES = ('Sin compensar', 'Compensado', 'DMP interno')
DETALLES = ('Giroscopio sin quitar sesgo', 'Filtro Mahony del trabajo colaborativo', 'Cuaternión del MPU6050')
COLORES = ('#ed7a24', '#b54b19', '#7a553e')
GIRO_MODELO = Rot.from_euler('z', 180, degrees=True)


class DatosAviones(Node):
    def __init__(self):
        super().__init__('aviones_imu_colaborativo')
        self.declare_parameter('eje_cabeceo_sensor', [0., 0., 1.])
        self.lock = threading.RLock()
        self.modelo = OrientacionesVista(self.get_parameter('eje_cabeceo_sensor').value)
        self.recibidas = {}
        self.aviso = ''
        self.create_subscription(Imu, '/imu_raw', self.raw, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/metodo1/odom',
                                 lambda m: self.recibir('compensado', m), 10)
        self.create_subscription(Odometry, '/metodo2/odom',
                                 lambda m: self.recibir('dmp', m), 10)
        self.hilo = threading.Thread(target=self.trabajar, daemon=True)
        self.hilo.start()

    def trabajar(self):
        from rclpy.executors import ExternalShutdownException
        try:
            rclpy.spin(self)
        except ExternalShutdownException:
            pass

    def raw(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        g, a = msg.angular_velocity, msg.linear_acceleration
        with self.lock:
            if self.modelo.gyro_crudo(t, [g.x, g.y, g.z], [a.x, a.y, a.z]):
                self.recibidas['sin_compensar'] = time.monotonic()

    def recibir(self, k, msg):
        q = msg.pose.pose.orientation
        with self.lock:
            try:
                self.modelo.agregar(k, [q.x, q.y, q.z, q.w])
            except ValueError as error:
                self.aviso = str(error)
                return
            self.recibidas[k] = time.monotonic()

    def snapshot(self):
        with self.lock:
            fase = ('Rotación relativa al origen · filtro colaborativo Mahony'
                    if self.modelo.montaje is not None else
                    'Esperando orientación del filtro colaborativo · placa quieta al iniciar')
            return self.modelo.relativas(), self.recibidas.copy(), {'fase': fase}, self.aviso

    def solicitar(self, comando):
        if comando == 'reset':
            with self.lock:
                self.aviso = ('Origen angular de los aviones fijado'
                              if self.modelo.fijar_origen() else 'Todavía no hay orientación filtrada')

    def cerrar(self):
        rclpy.try_shutdown()
        self.hilo.join(timeout=3)
        self.destroy_node()


class VentanaAviones(QtWidgets.QMainWindow):
    def __init__(self, nodo):
        super().__init__()
        self.nodo = nodo
        self.setWindowTitle('IMU · Comparación de rotación')
        area = QtWidgets.QApplication.primaryScreen().availableGeometry()
        self.resize(min(1440, area.width()-24), min(850, area.height()-48))
        self.setStyleSheet('''QWidget{background:#fffaf5;color:#35271e;font:12px Sans Serif}
            QPushButton{background:white;border:1px solid #eadbcc;border-radius:5px;padding:4px 8px}
            QPushButton:hover{background:#ffead8} QLabel#titulo{font-size:20px;font-weight:600}
            QLabel#detalle{color:#806957;font-size:11px}''')
        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        layout = QtWidgets.QVBoxLayout(central)
        layout.setContentsMargins(10, 8, 10, 8)
        layout.setSpacing(5)
        encabezado = QtWidgets.QHBoxLayout()
        titulo = QtWidgets.QLabel('Rotación de la placa'); titulo.setObjectName('titulo')
        encabezado.addWidget(titulo); encabezado.addStretch()
        for texto, comando in (('Fijar origen', 'reset'),):
            b = QtWidgets.QPushButton(texto)
            b.clicked.connect(lambda _checked=False, c=comando: nodo.solicitar(c))
            encabezado.addWidget(b)
        centrar = QtWidgets.QPushButton('Centrar cámaras'); centrar.clicked.connect(self.centrar)
        encabezado.addWidget(centrar)
        self.vincular = QtWidgets.QCheckBox('Vincular cámaras'); self.vincular.setChecked(True)
        encabezado.addWidget(self.vincular)
        layout.addLayout(encabezado)
        paneles = QtWidgets.QHBoxLayout()
        self.ejes, self.canvases, self.aviones, self.lecturas = {}, {}, {}, {}
        self.arrastre = None
        for k, nombre, detalle, color in zip(RAMAS, NOMBRES, DETALLES, COLORES):
            columna = QtWidgets.QVBoxLayout()
            etiqueta = QtWidgets.QLabel(nombre); etiqueta.setAlignment(QtCore.Qt.AlignCenter)
            etiqueta.setStyleSheet('font-size:16px;font-weight:600;color:'+color)
            columna.addWidget(etiqueta)
            sub = QtWidgets.QLabel(detalle); sub.setObjectName('detalle'); sub.setAlignment(QtCore.Qt.AlignCenter)
            sub.setWordWrap(True); columna.addWidget(sub)
            fig = Figure(facecolor='white')
            canvas = FigureCanvasQTAgg(fig)
            canvas.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
            ax = fig.add_subplot(111, projection='3d')
            fig.subplots_adjust(left=.02, right=.98, bottom=.03, top=.99)
            ax.set_facecolor('white'); ax.set_box_aspect((1, 1, 1))
            ax.set_axis_off()
            avion = Poly3DCollection([], facecolor=color, edgecolor='#674332', linewidth=.7)
            ax.add_collection3d(avion)
            # Líneas fijas que permiten apreciar la rotación sin trayectoria.
            ax.plot([-.6, .6], [0, 0], [0, 0], color='#dbc5b1', lw=.8)
            ax.plot([0, 0], [-.6, .6], [0, 0], color='#dbc5b1', lw=.8)
            columna.addWidget(canvas, 1)
            lectura = QtWidgets.QLabel('Esperando datos…'); lectura.setAlignment(QtCore.Qt.AlignCenter)
            lectura.setWordWrap(True); columna.addWidget(lectura)
            paneles.addLayout(columna, 1)
            self.ejes[k], self.canvases[k], self.aviones[k], self.lecturas[k] = ax, canvas, avion, lectura
            canvas.mpl_connect('button_press_event', lambda e, k=k: self.iniciar_arrastre(k, e))
            canvas.mpl_connect('motion_notify_event', lambda e, k=k: self.orbitar(k, e))
            canvas.mpl_connect('button_release_event', lambda e, k=k: self.soltar(k, e))
            canvas.mpl_connect('scroll_event', lambda e, k=k: self.zoom(k, e))
        layout.addLayout(paneles, 1)
        self.estado = QtWidgets.QLabel('Esperando sensor · dejá la placa quieta al iniciar')
        self.estado.setObjectName('detalle'); self.estado.setWordWrap(True); layout.addWidget(self.estado)
        self.centrar()
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.actualizar); self.timer.start(70)

    def centrar(self):
        for k, ax in self.ejes.items():
            ax.set_xlim(-.55, .55); ax.set_ylim(-.55, .55); ax.set_zlim(-.55, .55)
            ax.view_init(elev=22, azim=90, roll=0)
            self.canvases[k].draw_idle()

    def sincronizar(self, k):
        origen = self.ejes[k]
        if not self.vincular.isChecked():
            return
        for otro, ax in self.ejes.items():
            if otro == k:
                continue
            ax.view_init(elev=origen.elev, azim=origen.azim, roll=origen.roll)
            ax.set_xlim(origen.get_xlim()); ax.set_ylim(origen.get_ylim()); ax.set_zlim(origen.get_zlim())
            self.canvases[otro].draw_idle()

    def iniciar_arrastre(self, k, e):
        if e.inaxes == self.ejes[k]:
            self.arrastre = k

    def orbitar(self, k, _e):
        if self.arrastre == k:
            self.sincronizar(k)

    def soltar(self, k, _e):
        if self.arrastre == k:
            self.sincronizar(k); self.arrastre = None

    def zoom(self, k, e):
        ax = self.ejes[k]
        factor = .85 if e.button == 'up' else 1.18
        for get, setter in ((ax.get_xlim, ax.set_xlim), (ax.get_ylim, ax.set_ylim), (ax.get_zlim, ax.set_zlim)):
            a, b = get(); h = (b-a)*factor/2
            setter(-h, h)
        self.sincronizar(k); self.canvases[k].draw_idle()

    def actualizar(self):
        if not rclpy.ok():
            QtWidgets.QApplication.instance().quit()
            return
        poses, recibidas, estado, aviso = self.nodo.snapshot()
        for k in RAMAS:
            if k not in poses:
                self.aviones[k].set_verts([])
                self.lecturas[k].setText('Esperando datos…')
            else:
                r = poses[k]
                vertices = (r*GIRO_MODELO).apply(triangulos().reshape(-1, 3)*.35).reshape(-1, 3, 3)
                self.aviones[k].set_verts(vertices)
                with warnings.catch_warnings():
                    warnings.simplefilter('ignore', UserWarning)
                    roll, pitch, yaw = r.as_euler('xyz', degrees=True)
                fresco = time.monotonic()-recibidas[k] < 1
                self.aviones[k].set_alpha(1. if fresco else .35)
                self.lecturas[k].setText(f'Alabeo {roll:+.1f}° · Cabeceo {pitch:+.1f}°\nGiro {yaw:+.1f}°'
                                        + ('' if fresco else '\nSin datos recientes'))
            self.canvases[k].draw_idle()
        self.estado.setText(estado.get('fase', '') + (' · '+aviso if aviso else ''))

    def closeEvent(self, event):
        self.timer.stop(); event.accept()


def main():
    from rclpy.signals import SignalHandlerOptions
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    app = QtWidgets.QApplication(sys.argv[:1])
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: app.quit())
    nodo = DatosAviones()
    ventana = VentanaAviones(nodo); ventana.show()
    try:
        app.exec_()
    finally:
        nodo.cerrar()
