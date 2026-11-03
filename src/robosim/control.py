from dataclasses import dataclass


@dataclass
class PID:
    kp: float = 4.0
    ki: float = 0.2
    kd: float = 0.5
    output_min: float = -10.0
    output_max: float = 10.0
    integral: float = 0.0
    previous_error: float | None = None

    def reset(self) -> None:
        self.integral = 0.0
        self.previous_error = None

    def update(self, setpoint: float, measurement: float, dt: float) -> float:
        error = setpoint - measurement
        candidate = self.integral + error * dt
        derivative = 0.0 if self.previous_error is None else (error - self.previous_error) / dt
        output = self.kp * error + self.ki * candidate + self.kd * derivative
        clamped = max(self.output_min, min(self.output_max, output))
        if output == clamped:
            self.integral = candidate
        self.previous_error = error
        return clamped
