from dataclasses import dataclass


@dataclass
class Actuator:
    maximum: float = 10.0
    slew_rate: float | None = None
    max_velocity: float | None = None
    max_acceleration: float | None = None
    velocity_limit: float | None = None
    acceleration_limit: float | None = None
    output: float = 0.0
    stuck: bool = False
    velocity: float = 0.0
    acceleration: float = 0.0
    velocity_violation: bool = False
    acceleration_violation: bool = False

    def apply(self, request: float, dt: float) -> float:
        max_velocity = self.max_velocity if self.max_velocity is not None else self.velocity_limit
        max_acceleration = (self.max_acceleration if self.max_acceleration is not None
                            else self.acceleration_limit)
        self.velocity_violation = False
        self.acceleration_violation = False
        if self.stuck:
            self.velocity = 0.0
            self.acceleration = 0.0
            return self.output
        desired = max(-self.maximum, min(self.maximum, request))
        if self.slew_rate is not None:
            delta = desired - self.output
            desired = self.output + max(-self.slew_rate * dt, min(self.slew_rate * dt, delta))
        requested_velocity = (desired - self.output) / dt if dt else 0.0
        if max_velocity is not None and abs(requested_velocity) > max_velocity:
            self.velocity_violation = True
            requested_velocity = max(-max_velocity, min(max_velocity, requested_velocity))
        requested_acceleration = (requested_velocity - self.velocity) / dt if dt else 0.0
        if max_acceleration is not None and abs(requested_acceleration) > max_acceleration:
            self.acceleration_violation = True
            requested_acceleration = max(-max_acceleration,
                                         min(max_acceleration, requested_acceleration))
        self.acceleration = requested_acceleration
        self.velocity += self.acceleration * dt
        if max_velocity is not None:
            self.velocity = max(-max_velocity, min(max_velocity, self.velocity))
        self.output += self.velocity * dt
        self.output = max(-self.maximum, min(self.maximum, self.output))
        return self.output
