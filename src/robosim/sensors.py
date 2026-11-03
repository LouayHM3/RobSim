from __future__ import annotations

import random
from dataclasses import dataclass


@dataclass
class Sensor:
    noise_std: float = 0.0
    bias: float = 0.0
    dropout: bool = False
    rng: random.Random | None = None
    drift_per_second: float = 0.0
    stuck_value: float | None = None
    latency: float = 0.0
    communication_interruption: bool = False
    quantization: float | None = None
    quantization_step: float | None = None
    valid_min: float | None = None
    valid_max: float | None = None
    invalid: bool = False
    _history: list[tuple[float, float]] | None = None

    def read(self, value: float, time: float = 0.0) -> tuple[float | None, float]:
        if self.dropout or self.communication_interruption or self.invalid:
            return None, 0.0
        if self._history is None:
            self._history = []
        self._history.append((time, value))
        source_time = time - self.latency
        delayed = next((v for t, v in reversed(self._history) if t <= source_time), value)
        if self.stuck_value is not None:
            delayed = self.stuck_value
        rng = self.rng or random.Random(0)
        measured = (delayed + self.bias + self.drift_per_second * time
                    + (rng.gauss(0.0, self.noise_std) if self.noise_std else 0.0))
        step = self.quantization_step if self.quantization_step is not None else self.quantization
        if step and step > 0:
            measured = round(measured / step) * step
        if ((self.valid_min is not None and measured < self.valid_min)
                or (self.valid_max is not None and measured > self.valid_max)):
            return None, max(0.0, time - source_time)
        return measured, max(0.0, time - source_time)
