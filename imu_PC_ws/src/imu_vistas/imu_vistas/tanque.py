"""Tanque TP3 fijo: torreta y cañón con la orientación del filtro colaborativo."""
import time
import warnings

import rclpy

import numpy as np
from scipy.spatial.transform import Rotation as Rot
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import ColorRGBA
from imu_vistas.orientaciones import OrientacionesVista
from visualization_msgs.msg import Marker, MarkerArray

ESCALA = .02
BASE = Rot.from_euler('z', -90, degrees=True)
RAMA_RVIZ = 'compensado'


def poses_tanque(orientacion, centro):
    """Reproduce los pivotes y orígenes de las mallas del URDF del TP3.

    Malla original apunta -Y; BASE la orienta hacia -X. El cabeceo del
    modelo sobre +Y se convierte en giro negativo sobre el eje X del cañón.
    El tanque tiene dos grados de libertad: no representa el alabeo.
    """
    with warnings.catch_warnings():
        warnings.simplefilter('ignore', UserWarning)
        yaw, pitch, _roll = orientacion.as_euler('ZYX')
    cabeza = Rot.from_euler('z', yaw)*BASE
    canon = cabeza*Rot.from_euler('x', -pitch)
    pivote_cabeza = np.asarray(centro)+np.array([0., 0., .5])
    pivote_canon = pivote_cabeza+cabeza.apply([0., -.38, .1])
    return ((BASE, np.asarray(centro)),
            (cabeza, pivote_cabeza+cabeza.apply([0., 0., -.5])),
            (canon, pivote_canon+canon.apply([0., .38, -.6])))


class Tanque(Node):
    def __init__(self):
        super().__init__('tanque_imu_colaborativo')
        self.declare_parameter('eje_cabeceo_sensor', [0., 0., 1.])
        self.modelo = OrientacionesVista(self.get_parameter('eje_cabeceo_sensor').value)
        self.poses = {}
        self.pub = self.create_publisher(MarkerArray, '/imu_vistas/tanque', 10)
        self.create_subscription(Odometry, '/metodo1/odom', self.recibir, 10)
        self.create_timer(.05, self.publicar)

    def recibir(self, msg):
        q = msg.pose.pose.orientation
        try:
            self.modelo.agregar('compensado', [q.x, q.y, q.z, q.w])
        except ValueError as error:
            self.get_logger().warning(str(error), throttle_duration_sec=5.)
            return
        self.poses[RAMA_RVIZ] = (self.modelo.relativas()['compensado'], time.monotonic())

    def marker(self, namespace, numero, tipo, r, p, color, stamp):
        m = Marker()
        m.header.frame_id, m.header.stamp = 'odom', stamp
        m.ns, m.id, m.type, m.action = namespace, numero, tipo, Marker.ADD
        m.pose.position.x, m.pose.position.y, m.pose.position.z = map(float, p)
        m.pose.orientation.x, m.pose.orientation.y, m.pose.orientation.z, m.pose.orientation.w = map(float, r.as_quat())
        m.color.r, m.color.g, m.color.b, m.color.a = color
        return m

    def publicar(self):
        stamp = self.get_clock().now().to_msg()
        ahora = time.monotonic()
        array = MarkerArray()
        k, nombre = RAMA_RVIZ, 'Filtro colaborativo · Mahony'
        centro = np.zeros(3)
        r, t = self.poses.get(k, (Rot.identity(), 0.))
        fresca = 0 <= ahora-t < 1
        color = (.9, .39, .08, 1. if fresca else .3)
        for j, ((pieza_r, pieza_p), archivo) in enumerate(zip(
                poses_tanque(r, centro), ('tank_body_max.stl', 'tank_head_max.stl', 'tank_gun_max.stl'))):
            m = self.marker(k, j, Marker.MESH_RESOURCE, pieza_r, pieza_p, color, stamp)
            m.mesh_resource = 'package://imu_vistas/meshes/'+archivo
            m.mesh_use_embedded_materials = False
            m.scale.x = m.scale.y = m.scale.z = ESCALA
            if j == 0:
                m.color.r, m.color.g, m.color.b = .48, .45, .41
            array.markers.append(m)
        texto = self.marker(k, 3, Marker.TEXT_VIEW_FACING, Rot.identity(),
                            centro+[0., 0., 1.5], (.25, .17, .1, 1.), stamp)
        texto.text = nombre+('' if fresca else ' · esperando datos')
        texto.scale.z = .18
        array.markers.append(texto)
        # Flecha indica dónde apunta la nariz; la terna también muestra
        # el alabeo que las dos articulaciones no pueden representar.
        m = self.marker(k, 4, Marker.ARROW, Rot.identity(), np.zeros(3), color, stamp)
        p0 = centro+[0., 0., 1.]
        p1 = p0+r.apply([-.8, 0., 0.])
        m.points = [Point(x=float(p[0]), y=float(p[1]), z=float(p[2])) for p in (p0, p1)]
        m.scale.x, m.scale.y, m.scale.z = .025, .06, .09
        array.markers.append(m)
        ejes = self.marker(k, 5, Marker.LINE_LIST, r, centro+[0., 0., 1.], (1., 1., 1., 1.), stamp)
        ejes.scale.x = .015
        for eje, rgb in zip(np.eye(3)*.35, ((1., 0., 0.), (0., .7, 0.), (0., 0., 1.))):
            ejes.points.extend([Point(), Point(x=float(eje[0]), y=float(eje[1]), z=float(eje[2]))])
            c = ColorRGBA(r=rgb[0], g=rgb[1], b=rgb[2], a=color[3])
            ejes.colors.extend([c, c])
        array.markers.append(ejes)
        self.pub.publish(array)


def main():
    from rclpy.executors import ExternalShutdownException
    rclpy.init()
    nodo = Tanque()
    try:
        rclpy.spin(nodo)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        nodo.destroy_node()
        rclpy.try_shutdown()
