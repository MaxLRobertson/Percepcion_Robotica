"""Simula un MPU-6050: publica /imu/raw y la verdad en /verdad/odom."""
import numpy as np
import rclpy
from rclpy._rclpy_pybind11 import RCLError
from rclpy.executors import ExternalShutdownException
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from scipy.spatial.transform import Rotation as Rot
from sensor_msgs.msg import Imu

from imu_pkg.trayectoria import estado_verdadero


class SimuladorImu(Node):
    def __init__(self):
        super().__init__("simulador_imu")
        self.declare_parameter("frecuencia_hz", 100.0)
        self.declare_parameter("ruido_gyro", 0.005)      # [rad/s] desvío estándar
        self.declare_parameter("ruido_acel", 0.05)       # [m/s²] desvío estándar
        self.declare_parameter("sesgo_gyro", [0.01, -0.01, 0.005])  # [rad/s]
        self.declare_parameter("sesgo_acel", [0.05, -0.03, 0.08])   # [m/s²]
        self.declare_parameter("semilla", 0)

        self.dt = 1.0 / self.get_parameter("frecuencia_hz").value
        self.ruido_g = self.get_parameter("ruido_gyro").value
        self.ruido_a = self.get_parameter("ruido_acel").value
        self.sesgo_g = np.array(self.get_parameter("sesgo_gyro").value)
        self.sesgo_a = np.array(self.get_parameter("sesgo_acel").value)
        self.rng = np.random.default_rng(self.get_parameter("semilla").value)

        # Igual que micro-ROS en la ESP32: best-effort.
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.pub_imu = self.create_publisher(Imu, "/imu/raw", qos)
        self.pub_verdad = self.create_publisher(Odometry, "/verdad/odom", 10)

        self.n = 0
        self.create_timer(self.dt, self.paso)

    def paso(self):
        # La trayectoria arranca recien cuando alguien escucha /imu/raw, para
        # que el reposo inicial (calibracion) no se pierda.
        if self.n == 0 and self.pub_imu.get_subscription_count() == 0:
            t = 0.0
        else:
            t = self.n * self.dt
            self.n += 1
        R, pos, vel, gyro, fuerza = estado_verdadero(t)
        ahora = self.get_clock().now().to_msg()

        g = gyro + self.sesgo_g + self.rng.normal(0.0, self.ruido_g, 3)
        a = fuerza + self.sesgo_a + self.rng.normal(0.0, self.ruido_a, 3)

        imu = Imu()
        imu.header.stamp = ahora
        imu.header.frame_id = "imu_link"
        imu.orientation_covariance[0] = -1.0   # el MPU-6050 crudo no da orientación
        imu.angular_velocity.x, imu.angular_velocity.y, imu.angular_velocity.z = map(float, g)
        imu.linear_acceleration.x, imu.linear_acceleration.y, imu.linear_acceleration.z = map(float, a)
        self.pub_imu.publish(imu)

        qx, qy, qz, qw = Rot.from_matrix(R).as_quat()
        od = Odometry()
        od.header.stamp = ahora
        od.header.frame_id = "odom"
        od.child_frame_id = "imu_verdad"
        od.pose.pose.position.x, od.pose.pose.position.y, od.pose.pose.position.z = map(float, pos)
        od.pose.pose.orientation.x = float(qx)
        od.pose.pose.orientation.y = float(qy)
        od.pose.pose.orientation.z = float(qz)
        od.pose.pose.orientation.w = float(qw)
        od.twist.twist.linear.x, od.twist.twist.linear.y, od.twist.twist.linear.z = map(float, vel)
        self.pub_verdad.publish(od)


def main():
    rclpy.init()
    nodo = SimuladorImu()
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
