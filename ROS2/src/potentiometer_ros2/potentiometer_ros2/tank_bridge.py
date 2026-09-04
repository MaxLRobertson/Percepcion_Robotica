"""Traduce los sensores del ESP32 al formato que espera tank_controller_pkg (RViz2)."""

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float32, Float32MultiArray


def _a_porcentaje(valor: float, minimo: float, maximo: float) -> float:
    rango = maximo - minimo
    if rango <= 0.0:
        return 0.0
    porcentaje = (valor - minimo) / rango * 100.0
    return max(0.0, min(100.0, porcentaje))


class TankBridgeNode(Node):
    """Junta /pot/position (potenciometro) y /encoder/angle (AS5600) en un
    unico Float32MultiArray [porcentaje_head, porcentaje_gun], que es lo
    que espera /tank/joint_percentages en tank_controller_pkg."""

    def __init__(self) -> None:
        super().__init__("tank_bridge")
        self.declare_parameter("encoder_angle_min_deg", 0.0)
        self.declare_parameter("encoder_angle_max_deg", 360.0)
        self.declare_parameter("publish_rate_hz", 30.0)

        self._encoder_min = float(self.get_parameter("encoder_angle_min_deg").value)
        self._encoder_max = float(self.get_parameter("encoder_angle_max_deg").value)
        publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)

        self._porcentaje_head = 0.0
        self._porcentaje_gun = 0.0

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.create_subscription(Float32, "/pot/position", self._al_recibir_pot, qos)
        self.create_subscription(
            Float32, "/encoder/angle", self._al_recibir_encoder, qos
        )
        self._publicador = self.create_publisher(
            Float32MultiArray, "/tank/joint_percentages", 10
        )
        self.create_timer(1.0 / publish_rate_hz, self._publicar)

        self.get_logger().info(
            "tank_bridge activo: /pot/position -> head, /encoder/angle -> gun "
            f"(rango encoder {self._encoder_min:.1f}-{self._encoder_max:.1f} deg)"
        )

    def _al_recibir_pot(self, mensaje: Float32) -> None:
        self._porcentaje_head = max(0.0, min(100.0, mensaje.data))

    def _al_recibir_encoder(self, mensaje: Float32) -> None:
        self._porcentaje_gun = _a_porcentaje(
            mensaje.data, self._encoder_min, self._encoder_max
        )

    def _publicar(self) -> None:
        mensaje = Float32MultiArray()
        mensaje.data = [self._porcentaje_head, self._porcentaje_gun]
        self._publicador.publish(mensaje)


def main(args=None) -> None:
    rclpy.init(args=args)
    nodo = TankBridgeNode()
    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        pass
    finally:
        nodo.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
