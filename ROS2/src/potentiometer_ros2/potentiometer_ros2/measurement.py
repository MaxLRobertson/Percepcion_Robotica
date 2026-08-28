"""Agrupación, historial y registro de las mediciones del potenciómetro."""

from collections import deque
import csv
from dataclasses import dataclass
import math
from pathlib import Path
from typing import Deque, Dict, Optional, Set, TextIO


SIGNALS = ("direct", "buffer", "amplified", "position", "encoder_angle")


@dataclass(frozen=True)
class Measurement:
    """Una muestra completa recibida desde los cinco tópicos."""

    timestamp_s: float
    direct_voltage_v: float
    buffer_voltage_v: float
    amplified_voltage_v: float
    position_percent: float
    encoder_angle_deg: float


class SampleAssembler:
    """Agrupa el último valor nuevo de cada uno de los cuatro tópicos."""

    def __init__(self) -> None:
        self._values: Dict[str, float] = {}
        self._updated: Set[str] = set()
        self._latest_timestamp_s = 0.0

    def update(
        self, signal: str, value: float, timestamp_s: float
    ) -> Optional[Measurement]:
        """Devuelve una muestra solamente cuando las cinco señales se renovaron."""
        if signal not in SIGNALS:
            raise ValueError(f"Señal desconocida: {signal}")
        if not math.isfinite(value) or not math.isfinite(timestamp_s):
            return None

        self._values[signal] = float(value)
        self._updated.add(signal)
        self._latest_timestamp_s = max(self._latest_timestamp_s, timestamp_s)

        if self._updated != set(SIGNALS):
            return None

        sample = Measurement(
            timestamp_s=self._latest_timestamp_s,
            direct_voltage_v=self._values["direct"],
            buffer_voltage_v=self._values["buffer"],
            amplified_voltage_v=self._values["amplified"],
            position_percent=self._values["position"],
            encoder_angle_deg=self._values["encoder_angle"],
        )
        self._updated.clear()
        return sample


class HistoryBuffer:
    """Mantiene sólo las muestras dentro de una ventana temporal."""

    def __init__(self, history_seconds: float) -> None:
        if history_seconds <= 0.0:
            raise ValueError("history_seconds debe ser mayor que cero")
        self.history_seconds = float(history_seconds)
        self.samples: Deque[Measurement] = deque()

    def append(self, sample: Measurement) -> None:
        self.samples.append(sample)
        minimum_time = sample.timestamp_s - self.history_seconds
        while self.samples and self.samples[0].timestamp_s < minimum_time:
            self.samples.popleft()

    def clear(self) -> None:
        self.samples.clear()


class CsvRecorder:
    """Escribe muestras completas en un archivo CSV."""

    HEADER = (
        "timestamp_s",
        "direct_voltage_v",
        "buffer_voltage_v",
        "amplified_voltage_v",
        "position_percent",
        "encoder_angle_deg",
    )

    def __init__(self, path: Path) -> None:
        self.path = path.expanduser()
        self._file: Optional[TextIO] = None
        self._writer = None

    @property
    def is_recording(self) -> bool:
        return self._file is not None

    def start(self) -> None:
        if self.is_recording:
            return

        self.path.parent.mkdir(parents=True, exist_ok=True)
        file_exists = self.path.exists() and self.path.stat().st_size > 0
        self._file = self.path.open("a", newline="", encoding="utf-8")
        self._writer = csv.writer(self._file)
        if not file_exists:
            self._writer.writerow(self.HEADER)
            self._file.flush()

    def write(self, sample: Measurement) -> None:
        if not self.is_recording or self._writer is None or self._file is None:
            return

        self._writer.writerow(
            [
                f"{sample.timestamp_s:.3f}",
                f"{sample.direct_voltage_v:.6f}",
                f"{sample.buffer_voltage_v:.6f}",
                f"{sample.amplified_voltage_v:.6f}",
                f"{sample.position_percent:.2f}",
                f"{sample.encoder_angle_deg:.2f}",
            ]
        )
        self._file.flush()

    def stop(self) -> None:
        if self._file is not None:
            self._file.flush()
            self._file.close()
        self._file = None
        self._writer = None
