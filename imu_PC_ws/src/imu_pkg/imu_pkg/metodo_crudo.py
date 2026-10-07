"""Nodo del metodo 1.1: estima actitud, velocidad y posicion desde /imu/raw."""
import numpy as np
import rclpy
from rclpy._rclpy_pybind11 import RCLError
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from scipy.spatial.transform import Rotation as Rot
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32MultiArray
from tf2_ros import TransformBroadcaster

from imu_pkg.estimador import EstimadorCrudo


def _vec(v):
    return np.array([v.x, v.y, v.z])


class MetodoCrudo(Node):
    def __init__(self):
        super().__init__("metodo_crudo")
        self.declare_parameter("kp", 1.0)
        self.declare_parameter("ki", 0.05)
        self.declare_parameter("tolerancia_g", 0.5)
        self.declare_parameter("zupt", False)
        self.declare_parameter("calibrar_acel", True)
        self.declare_parameter("calibracion_s", 1.5)

        self.est = EstimadorCrudo(
            kp=self.get_parameter("kp").value,
            ki=self.get_parameter("ki").value,
            tolerancia_g=self.get_parameter("tolerancia_g").value,
            zupt=self.get_parameter("zupt").value,
            calibrar_acel=self.get_parameter("calibrar_acel").value,
        )
        self.t_calib = self.get_parameter("calibracion_s").value
        self.t0 = None
        self.t_prev = None
        self.verdad = None

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Imu, "/imu/raw", self.cb_imu, qos)
        self.create_subscription(Odometry, "/verdad/odom", self.cb_verdad, 10)
        self.pub_odom = self.create_publisher(Odometry, "/metodo1/odom", 10)
        self.pub_error = self.create_publisher(Float32MultiArray, "/metodo1/error", 10)
        self.tf = TransformBroadcaster(self)
        self.get_logger().info(
            f"Esperando /imu/raw. Calibrando {self.t_calib:.1f} s con el sensor quieto...")

    def cb_verdad(self, msg):
        self.verdad = msg

    def cb_imu(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        gyro = _vec(msg.angular_velocity)
        acel = _vec(msg.linear_acceleration)

        if not self.est.calibrado:
            if self.t0 is None:
                self.t0 = t
            self.est.acumular_calibracion(gyro, acel)
            if t - self.t0 >= self.t_calib:
                self.est.finalizar_calibracion()
                self.t_prev = t
                self.get_logger().info("Calibrado. Estimando actitud, velocidad y posicion.")
            return

        dt = t - self.t_prev
        self.t_prev = t
        if dt <= 0.0:
            return
        self.est.actualizar(gyro, acel, dt)
        self.publicar(msg.header.stamp)

    def publicar(self, stamp):
        qx, qy, qz, qw = self.est.cuaternion()
        od = Odometry()
        od.header.stamp = stamp
        od.header.frame_id = "odom"
        od.child_frame_id = "imu_metodo1"
        od.pose.pose.position.x, od.pose.pose.position.y, od.pose.pose.position.z = map(float, self.est.pos)
        od.pose.pose.orientation.x = float(qx)
        od.pose.pose.orientation.y = float(qy)
        od.pose.pose.orientation.z = float(qz)
        od.pose.pose.orientation.w = float(qw)
        od.twist.twist.linear.x, od.twist.twist.linear.y, od.twist.twist.linear.z = map(float, self.est.vel)
        self.pub_odom.publish(od)

        tf = TransformStamped()
        tf.header = od.header
        tf.child_frame_id = od.child_frame_id
        tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z = map(float, self.est.pos)
        tf.transform.rotation = od.pose.pose.orientation
        self.tf.sendTransform(tf)

        if self.verdad is not None:
            v = self.verdad
            q = v.pose.pose.orientation
            r_ver = Rot.from_quat([q.x, q.y, q.z, q.w])
            err_ang = np.degrees((Rot.from_matrix(self.est.R) * r_ver.inv()).magnitude())
            p = v.pose.pose.position
            err_pos = np.linalg.norm(self.est.pos - np.array([p.x, p.y, p.z]))
            err_vel = np.linalg.norm(self.est.vel - _vec(v.twist.twist.linear))
            self.pub_error.publish(Float32MultiArray(
                data=[float(err_ang), float(err_vel), float(err_pos)]))


def main():
    rclpy.init()
    nodo = MetodoCrudo()
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
