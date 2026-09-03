#!/usr/bin/env python3
import rclpy
import math
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import JointState

# =============================================================================
# CONFIGURACIÓN DE LÍMITES DE ROTACIÓN (En radianes. Ej: 3.14 = 180 grados)
# =============================================================================
TANK_HEAD_MIN = math.radians(-120)  # Ángulo en radianes equivalente al 0% de la torreta
TANK_HEAD_MAX = math.radians(120)  # Ángulo en radianes equivalente al 100% de la torreta

TANK_GUN_MIN  = math.radians(-15)  # Ángulo en radianes equivalente al 0% del cañón (~ -45°)
TANK_GUN_MAX  = math.radians(45)  # Ángulo en radianes equivalente al 100% del cañón (~ 45°)
# =============================================================================

class TankJointController(Node):

    def __init__(self):
        super().__init__('tank_joint_controller')

        # Suscriptor al tópico que recibe el array de 2 floats [porcentaje_head, porcentaje_gun]
        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/tank/joint_percentages',
            self.percentage_callback,
            10
        )

        # Publicador en el tópico estándar de estados de articulaciones
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)

        # Estado inicial de los ángulos (en radianes)
        self.current_head_angle = 0.0
        self.current_gun_angle = 0.0

        # Timer para publicar continuamente a 30Hz
        # robot_state_publisher necesita recibir /joint_states de forma constante
        self.timer = self.create_timer(1.0 / 30.0, self.publish_joints)
        
        self.get_logger().info('Nodo controlador del tanque iniciado correctamente.')

    def map_percentage_to_angle(self, percentage, min_angle, max_angle):
        # Restringir el porcentaje de entrada estrictamente entre 0.0 y 100.0
        clamped_percentage = max(0.0, min(100.0, float(percentage)))
        # Interpolación lineal básica: mapea [0, 100] -> [min_angle, max_angle]
        return min_angle + (clamped_percentage / 100.0) * (max_angle - min_angle)

    def percentage_callback(self, msg):
        # Validar que el mensaje tenga por lo menos los 2 elementos solicitados
        if len(msg.data) < 2:
            self.get_logger().warn('Se recibió un array con menos de 2 elementos. Mensaje ignorado.')
            return

        percentage_head = msg.data[0]
        percentage_gun = msg.data[1]

        # Calcular los ángulos equivalentes usando los límites preseteados
        self.current_head_angle = self.map_percentage_to_angle(percentage_head, TANK_HEAD_MIN, TANK_HEAD_MAX)
        self.current_gun_angle = self.map_percentage_to_angle(percentage_gun, TANK_GUN_MIN, TANK_GUN_MAX)

    def publish_joints(self):
        msg = JointState()
        
        # Sello de tiempo de ROS 2 (Crucial para que TF no se rompa)
        msg.header.stamp = self.get_clock().now().to_msg()
        
        # Coincide exactamente con los nombres de los joints de tu URDF
        msg.name = ['tank_head_joint', 'tank_gun_joint']
        
        # Posiciones calculadas en radianes
        msg.position = [self.current_head_angle, self.current_gun_angle]

        # Publicar el mensaje hacia robot_state_publisher
        self.joint_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TankJointController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
