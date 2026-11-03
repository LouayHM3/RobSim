from dataclasses import dataclass


@dataclass
class SafetyMonitor:
    error_limit: float = 5.0
    command_limit: float = 10.0
    dropout_limit: int = 3
    safe: bool = False
    reason: str = ""
    dropouts: int = 0

    def check(self, error: float, command: float, measurement_valid: bool) -> bool:
        if not measurement_valid:
            self.dropouts += 1
        else:
            self.dropouts = 0
        if abs(error) > self.error_limit:
            self.safe, self.reason = True, "position error limit exceeded"
        elif abs(command) > self.command_limit + 1e-9:
            self.safe, self.reason = True, "command limit exceeded"
        elif self.dropouts >= self.dropout_limit:
            self.safe, self.reason = True, "sensor dropout limit exceeded"
        return self.safe
