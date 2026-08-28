"""Dashboard ROS 2 para visualizar y registrar las mediciones del TP2."""

from collections import deque
from pathlib import Path
import signal
import sys
import time
from typing import Deque, Dict, Optional

from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg
from matplotlib.figure import Figure
from potentiometer_ros2.measurement import (
    CsvRecorder,
    HistoryBuffer,
    Measurement,
    SampleAssembler,
)
from PyQt5.QtCore import QTimer
from PyQt5.QtGui import QCloseEvent, QFont
from PyQt5.QtWidgets import (
    QApplication,
    QFrame,
    QGridLayout,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QPushButton,
    QVBoxLayout,
    QWidget,
)
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import Float32


class PotentiometerNode(Node):
    """Recibe los cinco tópicos y entrega únicamente muestras completas."""

    def __init__(self) -> None:
        super().__init__("potentiometer_dashboard")
        self.declare_parameter("history_seconds", 60.0)
        self.declare_parameter("csv_path", "mediciones_potenciometro.csv")

        self.history_seconds = float(self.get_parameter("history_seconds").value)
        self.csv_path = Path(str(self.get_parameter("csv_path").value))
        self._assembler = SampleAssembler()
        self._pending_samples: Deque[Measurement] = deque()
        self._start_ns = self.get_clock().now().nanoseconds
        self.last_complete_sample_monotonic: Optional[float] = None

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        topics = {
            "direct": "/pot/direct_voltage",
            "buffer": "/pot/buffer_voltage",
            "amplified": "/pot/amplified_voltage",
            "position": "/pot/position",
            "encoder_angle": "/encoder/angle",
        }
        self._subscriptions = [
            self.create_subscription(
                Float32,
                topic,
                lambda message, name=signal: self._receive(name, message),
                qos,
            )
            for signal, topic in topics.items()
        ]

        self.get_logger().info(
            "Dashboard suscripto a los cuatro tópicos /pot y a /encoder/angle"
        )

    def _receive(self, signal: str, message: Float32) -> None:
        now_s = (
            self.get_clock().now().nanoseconds - self._start_ns
        ) / 1_000_000_000.0
        sample = self._assembler.update(signal, message.data, now_s)
        if sample is not None:
            self._pending_samples.append(sample)
            self.last_complete_sample_monotonic = time.monotonic()

    def take_pending_samples(self) -> Deque[Measurement]:
        samples = self._pending_samples
        self._pending_samples = deque()
        return samples

    def data_is_recent(self, timeout_s: float = 1.0) -> bool:
        if self.last_complete_sample_monotonic is None:
            return False
        return time.monotonic() - self.last_complete_sample_monotonic <= timeout_s


class ValueCard(QFrame):
    """Tarjeta reutilizable para mostrar un valor actual."""

    def __init__(self, title: str, unit: str, color: str) -> None:
        super().__init__()
        self.setObjectName("valueCard")
        layout = QVBoxLayout(self)

        title_label = QLabel(title)
        title_label.setObjectName("cardTitle")
        self.value_label = QLabel(f"-- {unit}")
        self.value_label.setStyleSheet(f"color: {color};")
        self.value_label.setFont(QFont("Sans Serif", 22, QFont.Bold))

        layout.addWidget(title_label)
        layout.addWidget(self.value_label)

    def set_value(self, value: float, unit: str, decimals: int) -> None:
        self.value_label.setText(f"{value:.{decimals}f} {unit}")


class DashboardWindow(QMainWindow):
    """Interfaz gráfica principal."""

    COLORS = {
        "direct": "#38bdf8",
        "buffer": "#4ade80",
        "amplified": "#fb923c",
        "position": "#e879f9",
        "encoder_angle": "#facc15",
    }

    def __init__(self, node: PotentiometerNode) -> None:
        super().__init__()
        self.node = node
        self.history = HistoryBuffer(node.history_seconds)
        self.recorder = CsvRecorder(node.csv_path)

        self.setWindowTitle("TP2 · Potenciómetro y micro-ROS")
        self.resize(1180, 900)
        self._build_ui()
        self._configure_style()

        self.spin_timer = QTimer(self)
        self.spin_timer.timeout.connect(self._process_ros)
        self.spin_timer.start(20)

        self.status_timer = QTimer(self)
        self.status_timer.timeout.connect(self._update_connection_status)
        self.status_timer.start(250)

    def _build_ui(self) -> None:
        central = QWidget()
        main_layout = QVBoxLayout(central)
        main_layout.setContentsMargins(20, 16, 20, 18)
        main_layout.setSpacing(14)

        header = QHBoxLayout()
        title_box = QVBoxLayout()
        title = QLabel("Lectura analógica del potenciómetro")
        title.setObjectName("mainTitle")
        subtitle = QLabel("ESP32 · ADC1 · micro-ROS · ROS 2 Jazzy")
        subtitle.setObjectName("subtitle")
        title_box.addWidget(title)
        title_box.addWidget(subtitle)
        header.addLayout(title_box)
        header.addStretch()

        self.status_label = QLabel("● Esperando datos")
        self.status_label.setObjectName("statusWaiting")
        self.record_button = QPushButton("Iniciar CSV")
        self.record_button.setCheckable(True)
        self.record_button.clicked.connect(self._toggle_recording)
        self.clear_button = QPushButton("Limpiar gráfica")
        self.clear_button.clicked.connect(self._clear_history)
        header.addWidget(self.status_label)
        header.addWidget(self.record_button)
        header.addWidget(self.clear_button)
        main_layout.addLayout(header)

        cards_layout = QGridLayout()
        self.cards: Dict[str, ValueCard] = {
            "direct": ValueCard("Pote directo", "V", self.COLORS["direct"]),
            "buffer": ValueCard("Salida buffer", "V", self.COLORS["buffer"]),
            "amplified": ValueCard(
                "Salida amplificada", "V", self.COLORS["amplified"]
            ),
            "position": ValueCard("Posición", "%", self.COLORS["position"]),
            "encoder_angle": ValueCard(
                "Ángulo AS5600", "°", self.COLORS["encoder_angle"]
            ),
        }
        for column, card in enumerate(self.cards.values()):
            cards_layout.addWidget(card, 0, column)
        main_layout.addLayout(cards_layout)

        self.figure = Figure(facecolor="#0f172a", constrained_layout=True)
        self.canvas = FigureCanvasQTAgg(self.figure)
        self.voltage_axes, self.position_axes, self.angle_axes = (
            self.figure.subplots(3, 1)
        )
        self._configure_axes()
        main_layout.addWidget(self.canvas, stretch=1)

        self.csv_label = QLabel(f"CSV: {self.recorder.path.resolve()}")
        self.csv_label.setObjectName("footer")
        main_layout.addWidget(self.csv_label)
        self.setCentralWidget(central)

    def _configure_axes(self) -> None:
        for axes in (self.voltage_axes, self.position_axes, self.angle_axes):
            axes.set_facecolor("#111827")
            axes.tick_params(colors="#cbd5e1")
            axes.grid(True, color="#334155", alpha=0.5)
            for spine in axes.spines.values():
                spine.set_color("#475569")

        self.voltage_axes.set_ylabel("Tensión (V)", color="#e2e8f0")
        self.voltage_axes.set_ylim(0.0, 3.5)
        self.position_axes.set_ylabel("Posición (%)", color="#e2e8f0")
        self.position_axes.set_ylim(-2.0, 102.0)
        self.angle_axes.set_ylabel("Ángulo AS5600 (°)", color="#e2e8f0")
        self.angle_axes.set_ylim(-5.0, 365.0)
        self.angle_axes.set_xlabel("Tiempo respecto de ahora (s)", color="#e2e8f0")

        (self.direct_line,) = self.voltage_axes.plot(
            [], [], label="Directo", color=self.COLORS["direct"], linewidth=2
        )
        (self.buffer_line,) = self.voltage_axes.plot(
            [], [], label="Buffer", color=self.COLORS["buffer"], linewidth=2
        )
        (self.amplified_line,) = self.voltage_axes.plot(
            [], [], label="Amplificado", color=self.COLORS["amplified"], linewidth=2
        )
        (self.position_line,) = self.position_axes.plot(
            [], [], label="Posición", color=self.COLORS["position"], linewidth=2
        )
        (self.encoder_angle_line,) = self.angle_axes.plot(
            [], [], label="Ángulo AS5600", color=self.COLORS["encoder_angle"],
            linewidth=2
        )

        legend = self.voltage_axes.legend(
            loc="upper left", facecolor="#1e293b", edgecolor="#475569"
        )
        for text in legend.get_texts():
            text.set_color("#e2e8f0")

    def _configure_style(self) -> None:
        self.setStyleSheet(
            """
            QMainWindow, QWidget { background: #0f172a; color: #e2e8f0; }
            QLabel#mainTitle { font-size: 24px; font-weight: 700; }
            QLabel#subtitle, QLabel#footer { color: #94a3b8; }
            QLabel#cardTitle { color: #cbd5e1; font-size: 13px; }
            QLabel#statusWaiting { color: #fbbf24; font-weight: 700; padding: 8px; }
            QLabel#statusConnected { color: #4ade80; font-weight: 700; padding: 8px; }
            QFrame#valueCard {
                background: #1e293b; border: 1px solid #334155;
                border-radius: 10px; padding: 8px;
            }
            QPushButton {
                background: #334155; border: 1px solid #475569;
                border-radius: 7px; padding: 9px 13px; font-weight: 600;
            }
            QPushButton:hover { background: #475569; }
            QPushButton:checked { background: #9f1239; border-color: #fb7185; }
            """
        )

    def _process_ros(self) -> None:
        if not rclpy.ok():
            self.close()
            return
        rclpy.spin_once(self.node, timeout_sec=0.0)
        samples = self.node.take_pending_samples()
        if not samples:
            return

        for sample in samples:
            self.history.append(sample)
            self.recorder.write(sample)
        self._update_values(samples[-1])
        self._update_plot()

    def _update_values(self, sample: Measurement) -> None:
        self.cards["direct"].set_value(sample.direct_voltage_v, "V", 5)
        self.cards["buffer"].set_value(sample.buffer_voltage_v, "V", 5)
        self.cards["amplified"].set_value(sample.amplified_voltage_v, "V", 5)
        self.cards["position"].set_value(sample.position_percent, "%", 2)
        self.cards["encoder_angle"].set_value(sample.encoder_angle_deg, "°", 2)

    def _update_plot(self) -> None:
        if not self.history.samples:
            for line in (
                self.direct_line,
                self.buffer_line,
                self.amplified_line,
                self.position_line,
                self.encoder_angle_line,
            ):
                line.set_data([], [])
            self.canvas.draw_idle()
            return

        samples = list(self.history.samples)
        latest_time = samples[-1].timestamp_s
        relative_time = [sample.timestamp_s - latest_time for sample in samples]

        self.direct_line.set_data(
            relative_time, [sample.direct_voltage_v for sample in samples]
        )
        self.buffer_line.set_data(
            relative_time, [sample.buffer_voltage_v for sample in samples]
        )
        self.amplified_line.set_data(
            relative_time, [sample.amplified_voltage_v for sample in samples]
        )
        self.position_line.set_data(
            relative_time, [sample.position_percent for sample in samples]
        )
        self.encoder_angle_line.set_data(
            relative_time, [sample.encoder_angle_deg for sample in samples]
        )

        left_limit = -self.history.history_seconds
        self.voltage_axes.set_xlim(left_limit, 0.0)
        self.position_axes.set_xlim(left_limit, 0.0)
        self.angle_axes.set_xlim(left_limit, 0.0)
        self.canvas.draw_idle()

    def _update_connection_status(self) -> None:
        if self.node.data_is_recent():
            self.status_label.setText("● Recibiendo 10 Hz")
            self.status_label.setObjectName("statusConnected")
        else:
            self.status_label.setText("● Esperando datos")
            self.status_label.setObjectName("statusWaiting")
        self.status_label.style().unpolish(self.status_label)
        self.status_label.style().polish(self.status_label)

    def _toggle_recording(self, checked: bool) -> None:
        if checked:
            try:
                self.recorder.start()
            except OSError as error:
                self.node.get_logger().error(f"No se pudo abrir el CSV: {error}")
                self.record_button.setChecked(False)
                return
            self.record_button.setText("Detener CSV")
            self.csv_label.setText(f"Grabando CSV: {self.recorder.path.resolve()}")
        else:
            self.recorder.stop()
            self.record_button.setText("Iniciar CSV")
            self.csv_label.setText(f"CSV: {self.recorder.path.resolve()}")

    def _clear_history(self) -> None:
        self.history.clear()
        self._update_plot()

    def closeEvent(self, event: QCloseEvent) -> None:  # noqa: N802 (API de Qt)
        self.spin_timer.stop()
        self.status_timer.stop()
        self.recorder.stop()
        event.accept()


def main(args=None) -> None:
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    app = QApplication.instance() or QApplication(sys.argv[:1])
    node = PotentiometerNode()
    window = DashboardWindow(node)
    previous_sigint_handler = signal.signal(
        signal.SIGINT, lambda _signal, _frame: app.quit()
    )
    window.show()

    try:
        app.exec_()
    finally:
        signal.signal(signal.SIGINT, previous_sigint_handler)
        window.recorder.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
