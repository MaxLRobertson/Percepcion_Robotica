"""Nodo del metodo 1.2: actitud del DMP + integracion de la aceleracion cruda.

Entradas: /imu/raw (aceleracion y giroscopio) y /imu/dmp (orientacion del chip).
"""
import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy._rclpy_pybind11 import RCLError
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from scipy.spatial.transform import Rotation as Rot
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32MultiArray
from tf2_ros import TransformBroadcaster

from imu_pkg.dmp import quitar_yaw_inicial, rotacion_desde_dmp
from imu_pkg.integrador import Integrador
from imu_pkg.metricas import error_contra_verdad


def _vec(v):
    return np.array([v.x, v.y, v.z])


class MetodoDmp(Node):
    def __init__(self):
        super().__init__("metodo_dmp")
        self.declare_parameter("zupt", False)
        self.declare_parameter("calibrar_acel", True)
        self.declare_parameter("calibracion_s", 1.5)

        self.integ = Integrador(
            zupt=self.get_parameter("zupt").value,
            calibrar_acel=self.get_parameter("calibrar_acel").value,
        )
        self.t_calib = self.get_parameter("calibracion_s").value
        self.R_dmp = None          # ultima orientacion del DMP (sensor -> mundo del chip)
        self.R_yaw = np.eye(3)     # para dejar el yaw inicial en 0
        self.R = np.eye(3)
        self.sesgo_gyro = np.zeros(3)
        self._calib = []
        self.calibrado = False
        self.t0 = None
        self.t_prev = None
        self.verdad = None

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Imu, "/imu/dmp", self.cb_dmp, qos)
        self.create_subscription(Imu, "/imu/raw", self.cb_raw, qos)
        self.create_subscription(Odometry, "/verdad/odom", self.cb_verdad, 10)
        self.pub_odom = self.create_publisher(Odometry, "/metodo2/odom", 10)
        self.pub_error = self.create_publisher(Float32MultiArray, "/metodo2/error", 10)
        self.tf = TransformBroadcaster(self)
        self.get_logger().info(
            f"Esperando /imu/raw y /imu/dmp. Calibrando {self.t_calib:.1f} s con el sensor quieto...")

    def cb_verdad(self, msg):
        self.verdad = msg

    def cb_dmp(self, msg):
        q = msg.orientation
        self.R_dmp = rotacion_desde_dmp([q.x, q.y, q.z, q.w])

    def cb_raw(self, msg):
        if self.R_dmp is None:
            return
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        gyro = _vec(msg.angular_velocity)
        acel = _vec(msg.linear_acceleration)

        if not self.calibrado:
            if self.t0 is None:
                self.t0 = t
            self._calib.append(np.concatenate([gyro, acel]))
            if t - self.t0 >= self.t_calib:
                m = np.mean(self._calib, axis=0)
                self.sesgo_gyro = m[:3]
                self.integ.calibrar(m[3:])
                self.R_yaw = quitar_yaw_inicial(self.R_dmp)
                self.t_prev = t
                self.calibrado = True
                self.get_logger().info("Calibrado. Integrando con la actitud del DMP.")
            return

        dt = t - self.t_prev
        self.t_prev = t
        if dt <= 0.0:
            return
        self.R = self.R_yaw @ self.R_dmp
        acel_c = self.integ.corregir_acel(self.R, acel)
        self.integ.integrar(self.R, acel_c, dt, np.linalg.norm(gyro - self.sesgo_gyro))
        self.publicar(msg.header.stamp)

    def publicar(self, stamp):
        qx, qy, qz, qw = Rot.from_matrix(self.R).as_quat()
        od = Odometry()
        od.header.stamp = stamp
        od.header.frame_id = "odom"
        od.child_frame_id = "imu_metodo2"
        od.pose.pose.position.x, od.pose.pose.position.y, od.pose.pose.position.z = map(float, self.integ.pos)
        od.pose.pose.orientation.x = float(qx)
        od.pose.pose.orientation.y = float(qy)
        od.pose.pose.orientation.z = float(qz)
        od.pose.pose.orientation.w = float(qw)
        od.twist.twist.linear.x, od.twist.twist.linear.y, od.twist.twist.linear.z = map(float, self.integ.vel)
        self.pub_odom.publish(od)

        tf = TransformStamped()
        tf.header = od.header
        tf.child_frame_id = od.child_frame_id
        tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z = map(float, self.integ.pos)
        tf.transform.rotation = od.pose.pose.orientation
        self.tf.sendTransform(tf)

        if self.verdad is not None:
            self.pub_error.publish(Float32MultiArray(data=error_contra_verdad(
                self.R, self.integ.pos, self.integ.vel, self.verdad)))


def main():
    rclpy.init()
    nodo = MetodoDmp()
    try:
        rclpy.spin(nodo)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RCLError:
        if rclpy.ok():   # solo se ignora si el contexto ya estaba apagado
            raise
    finally:
        nodo.destroy_node()
        rclpy.try_shutdown()
