from __future__ import annotations

from dataclasses import dataclass, field


@dataclass
class Plant:
    position: float = 0.0
    velocity: float = 0.0
    mass: float = 1.0
    damping: float = 0.2
    limit: float = 10.0
    velocity_limit: float | None = None
    max_velocity: float | None = None
    acceleration_limit: float | None = None
    acceleration: float = 0.0
    acceleration_limit_violation: bool = False

    def step(self, force: float, dt: float) -> None:
        acceleration = (force - self.damping * self.velocity) / self.mass
        self.acceleration_limit_violation = (
            self.acceleration_limit is not None and abs(acceleration) > self.acceleration_limit)
        if self.acceleration_limit is not None:
            acceleration = max(-self.acceleration_limit,
                               min(self.acceleration_limit, acceleration))
        self.acceleration = acceleration
        self.velocity += acceleration * dt
        velocity_limit = self.velocity_limit if self.velocity_limit is not None else self.max_velocity
        if velocity_limit is not None:
            self.velocity = max(-velocity_limit, min(velocity_limit, self.velocity))
        self.position += self.velocity * dt
        self.position = max(-self.limit, min(self.limit, self.position))
        if abs(self.position) >= self.limit:
            self.velocity = 0.0

    def state(self) -> dict[str, float]:
        return {"position": self.position, "velocity": self.velocity}


@dataclass
class Command:
    target: float = 0.0


@dataclass
class Sample:
    time: float
    target: float
    position: float
    velocity: float
    command: float
    measured_position: float
    fault: str | None
    safe: bool
    requested_command: float = 0.0
    emergency_stop: bool = False
    measurement_age: float = 0.0
    measurement_valid: bool = True
    position_limit_violation: bool = False
    velocity_limit_violation: bool = False
    actuator_limit_violation: bool = False
    actuator_velocity_violation: bool = False
    actuator_acceleration_violation: bool = False
    acceleration_limit_violation: bool = False
    actuator_velocity: float = 0.0
    actuator_acceleration: float = 0.0
    feedback_delayed: bool = False
    sensor_unchanged: bool = False


@dataclass
class Diagnostic:
    level: str
    code: str
    message: str
    time: float


@dataclass
class SimulationResult:
    scenario: str
    samples: list[Sample] = field(default_factory=list)
    diagnostics: list[Diagnostic] = field(default_factory=list)
    passed: bool = False
    reason: str = ""
    events: list[Diagnostic] = field(default_factory=list)

    def metrics(self) -> dict[str, object]:
        if not self.samples:
            return {"scenario": self.scenario, "passed": self.passed, "samples": 0}
        errors = [s.target - s.position for s in self.samples]
        absolute = [abs(value) for value in errors]
        dt = self.samples[1].time - self.samples[0].time if len(self.samples) > 1 else 0.0
        target = self.samples[0].target
        initial = self.samples[0].position
        band = 0.02 * max(abs(target - initial), 1.0)
        settling_index = next((i for i, error in enumerate(absolute)
                               if error <= band and all(later <= band for later in absolute[i:])), None)
        settled = self.samples[settling_index].time if settling_index is not None else None
        # Overshoot is excursion beyond the target after first crossing, not peak error.
        if target >= initial:
            crossed = [s.position - target for s in self.samples if s.position > target]
        else:
            crossed = [target - s.position for s in self.samples if s.position < target]
        overshoot = max(crossed, default=0.0)
        sat = sum(abs(s.command) >= 0.999999 * max(abs(s.requested_command), 1.0)
                  and abs(s.command) > 0 for s in self.samples)
        return {
            "scenario": self.scenario,
            "passed": self.passed,
            "reason": self.reason,
            "samples": len(self.samples),
            "duration": self.samples[-1].time,
            "max_error": max(absolute),
            "peak_error": max(absolute),
            "rmse": (sum(value * value for value in errors) / len(errors)) ** 0.5,
            "final_error": absolute[-1],
            "steady_state_error": sum(absolute[-max(1, len(absolute) // 10):]) /
            max(1, len(absolute) // 10),
            "mean_absolute_error": sum(absolute) / len(absolute),
            "integral_absolute_error": sum(absolute) * dt,
            "integral_squared_error": sum(value * value for value in errors) * dt,
            "overshoot": overshoot,
            "settling_time": settled,
            "rise_time": next((s.time for s in self.samples if abs(s.position) >= 0.9 * abs(target)), None),
            "max_command": max(abs(s.command) for s in self.samples),
            "control_effort": sum(abs(s.command) for s in self.samples) * dt,
            "saturation_fraction": sat / len(self.samples),
            "max_velocity": max(abs(s.velocity) for s in self.samples),
            "position_limit_violations": sum(s.position_limit_violation for s in self.samples),
            "velocity_limit_violations": sum(s.velocity_limit_violation for s in self.samples),
            "command_limit_violations": sum(s.actuator_limit_violation for s in self.samples),
            "actuator_velocity_violations": sum(s.actuator_velocity_violation for s in self.samples),
            "actuator_acceleration_violations": sum(s.actuator_acceleration_violation for s in self.samples),
            "acceleration_limit_violations": sum(s.acceleration_limit_violation for s in self.samples),
            "mean_measurement_age": sum(s.measurement_age for s in self.samples) / len(self.samples),
            "safe_state_time": next((s.time for s in self.samples if s.safe), None),
            "fault_count": sum(s.fault is not None for s in self.samples),
            "diagnostic_count": len(self.diagnostics) + len(self.events),
            "safe_state": any(s.safe for s in self.samples),
            "emergency_stop": any(s.emergency_stop for s in self.samples),
            "invalid_readings": sum(not s.measurement_valid for s in self.samples),
            "fault_detection_latency": self._fault_detection_latency(),
            "feedback_delay_diagnostics": sum(s.feedback_delayed for s in self.samples),
            "sensor_unchanged_diagnostics": sum(s.sensor_unchanged for s in self.samples),
        }

    def _fault_detection_latency(self) -> float | None:
        starts = [event.time for event in self.events if event.code not in {"SAFE_STATE"}]
        detections = [event.time for event in self.diagnostics if event.code != "SAFE_STATE"]
        if not starts:
            return None
        if not starts or not detections:
            safe = [event.time for event in self.diagnostics if event.code == "SAFE_STATE"]
            return (safe[0] - starts[0]) if safe else None
        return max(0.0, min(detections) - min(starts))
