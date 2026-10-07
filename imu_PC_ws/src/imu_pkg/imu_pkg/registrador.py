"""Registra en CSV la pose/velocidad de varios topicos nav_msgs/Odometry.

Formato largo: una fila por mensaje, con la columna 'topico'. Sirve igual para
la verdad simulada, el metodo 1.1 y (mas adelante) el metodo 1.2 con el DMP.
"""
import csv
import os
from datetime import datetime

import rclpy
from nav_msgs.msg import Odometry
from rclpy._rclpy_pybind11 import RCLError
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from scipy.spatial.transform import Rotation as Rot

CABECERA = ["t", "topico", "x", "y", "z", "vx", "vy", "vz", "roll", "pitch", "yaw"]


class Registrador(Node):
    def __init__(self):
        super().__init__("registrador")
        self.declare_parameter("topicos", ["/verdad/odom", "/metodo1/odom", "/metodo2/odom"])
        self.declare_parameter("archivo", "")

        archivo = self.get_parameter("archivo").value
        if not archivo:
            archivo = datetime.now().strftime("imu_%Y%m%d_%H%M%S.csv")
        self.archivo = os.path.abspath(archivo)
        self.f = open(self.archivo, "w", newline="")
        self.w = csv.writer(self.f)
        self.w.writerow(CABECERA)
        self.t0 = None

        for topico in self.get_parameter("topicos").value:
            self.create_subscription(
                Odometry, topico, lambda m, tp=topico: self.cb(tp, m), 10)
        self.get_logger().info(f"Registrando en {self.archivo}")

    def cb(self, topico, m):
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        if self.t0 is None:
            self.t0 = t
        p = m.pose.pose.position
        q = m.pose.pose.orientation
        v = m.twist.twist.linear
        # secuencia 'xyz' extrinseca = roll, pitch, yaw con R = Rz·Ry·Rx
        roll, pitch, yaw = Rot.from_quat([q.x, q.y, q.z, q.w]).as_euler("xyz", degrees=True)
        self.w.writerow([f"{t - self.t0:.4f}", topico, p.x, p.y, p.z,
                         v.x, v.y, v.z, roll, pitch, yaw])

    def cerrar(self):
        self.f.close()
        self.get_logger().info(f"CSV guardado: {self.archivo}")


def main():
    rclpy.init()
    nodo = Registrador()
    try:
        rclpy.spin(nodo)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RCLError:
        if rclpy.ok():   # solo se ignora si el contexto ya estaba apagado
            raise
    finally:
        nodo.cerrar()
        nodo.destroy_node()
        rclpy.try_shutdown()
