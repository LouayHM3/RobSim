from __future__ import annotations

import itertools
import random
from dataclasses import asdict
from typing import Any

from .actuators import Actuator
from .control import PID
from .diagnostics import SafetyMonitor
from .models import Diagnostic, Plant, Sample, SimulationResult
from .scenarios import load_scenario
from .sensors import Sensor


def target_at(config: dict[str, Any], time: float) -> float:
    trajectory = config.get("trajectory")
    if not trajectory:
        return float(config.get("target", 0.0))
    kind = str(trajectory.get("type", "fixed")).lower()
    if kind == "fixed":
        return float(trajectory.get("value", config.get("target", 0.0)))
    if kind == "step":
        return float(trajectory.get("before", trajectory.get("initial", config.get("target", 0.0)))) \
            if time < float(trajectory.get("time", trajectory.get("at",
                                                                   trajectory.get("step_time", 0.0)))) \
            else float(trajectory.get("after", trajectory.get("final",
                                                               trajectory.get("value", config.get("target", 0.0)))))
    if kind == "ramp":
        start = float(trajectory.get("start", trajectory.get("from", 0.0)))
        end = float(trajectory.get("end", trajectory.get("to", config.get("target", 0.0))))
        begin = float(trajectory.get("start_time", 0.0))
        duration = float(trajectory.get("duration", config.get("duration", 1.0)))
        fraction = max(0.0, min(1.0, (time - begin) / duration)) if duration else 1.0
        return start + (end - start) * fraction
    if kind in {"multi-point", "multipoint", "multi_point"}:
        points = trajectory.get("points", [])
        parsed = [(float(p["time"]), float(p["value"])) if isinstance(p, dict)
                  else (float(p[0]), float(p[1])) for p in points]
        if not parsed:
            return float(config.get("target", 0.0))
        if time <= parsed[0][0]:
            return parsed[0][1]
        for (t0, v0), (t1, v1) in itertools.pairwise(parsed):
            if time <= t1:
                ratio = (time - t0) / (t1 - t0) if t1 != t0 else 1.0
                return v0 + (v1 - v0) * ratio
        return parsed[-1][1]
    raise ValueError(f"unknown trajectory type: {kind}")


class Simulation:
    def __init__(self, scenario: dict[str, Any]) -> None:
        self.config = scenario
        seed = int(scenario.get("seed", 1))
        plant = Plant(**scenario["plant"])
        self.plant = plant
        self.pid = PID(**scenario["pid"])
        self.actuator = Actuator(**scenario["actuator"])
        sensor_cfg = scenario["sensor"]
        self.sensor = Sensor(**sensor_cfg, rng=random.Random(seed))
        self._sensor_dropout_config = bool(sensor_cfg.get("dropout", False))
        self.safety = SafetyMonitor(**scenario["safety"])

    def run(self) -> SimulationResult:
        cfg = self.config
        dt, duration = float(cfg["dt"]), float(cfg["duration"])
        result = SimulationResult(str(cfg.get("name", "scenario")))
        schedules = cfg.get("faults", [])
        seen: set[str] = set()
        previous_measurement: float | None = None
        unchanged_count = 0
        diag_cfg = cfg.get("diagnostics", {})
        delay_limit = float(diag_cfg.get("feedback_latency_limit", diag_cfg.get("max_feedback_delay", dt)))
        unchanged_limit = int(diag_cfg.get("unchanged_samples", 3))
        steps = round(duration / dt)
        for index in range(steps + 1):
            time = index * dt
            target = target_at(cfg, time)
            active = [fault for fault in schedules if float(fault.get("start", 0)) <= time
                      <= float(fault.get("end", duration))]
            names = {str(fault.get("type", "unknown")) for fault in active}
            if self._sensor_dropout_config:
                names.add("sensor_dropout")
            self.sensor.dropout = self._sensor_dropout_config or "sensor_dropout" in names
            self.sensor.communication_interruption = "communication_interruption" in names
            self.sensor.invalid = ("sensor_invalid" in names or "invalid_reading" in names)
            if "sensor_stuck" in names and self.sensor.stuck_value is None:
                self.sensor.stuck_value = self.plant.position
            elif "sensor_stuck" not in names:
                self.sensor.stuck_value = None
            if "sensor_drift" in names:
                self.sensor.drift_per_second = float(next(f for f in active if f.get("type") == "sensor_drift").get("rate", 0.1))
            else:
                self.sensor.drift_per_second = 0.0
            emergency = "emergency_stop" in names
            self.actuator.stuck = "actuator_stuck" in names
            if emergency:
                self.safety.safe = True
                self.safety.reason = "emergency stop requested"
            if "sensor_stuck" in names:
                self.safety.safe = True
                self.safety.reason = "sensor stuck fault detected"
            for name in sorted(names):
                if name not in seen:
                    result.events.append(Diagnostic("WARNING", name.upper(), f"fault active: {name}", time))
                    seen.add(name)
            measurement, age = self.sensor.read(self.plant.position, time)
            valid = measurement is not None
            measured = self.plant.position if measurement is None else measurement
            request = 0.0 if self.safety.safe else self.pid.update(target, measured, dt)
            requested = request
            force = self.actuator.apply(request, dt)
            safe = self.safety.check(target - self.plant.position, force, valid)
            self.plant.step(0.0 if safe else force, dt)
            fault = ",".join(sorted(names)) if names else None
            if not valid:
                fault = fault or ("sensor_invalid" if self.sensor.invalid else "sensor_dropout")
            feedback_delayed = valid and age > delay_limit and abs(request) > 1e-9
            sensor_unchanged = False
            if valid and previous_measurement is not None and abs(measured - previous_measurement) <= 1e-12 \
                    and abs(request) > 1e-9:
                unchanged_count += 1
            else:
                unchanged_count = 0
            if unchanged_count >= unchanged_limit:
                sensor_unchanged = True
            if feedback_delayed and "FEEDBACK_DELAY" not in seen:
                result.diagnostics.append(Diagnostic("WARNING", "FEEDBACK_DELAY",
                                                     "feedback measurement is delayed during commanded motion", time))
                seen.add("FEEDBACK_DELAY")
            if sensor_unchanged and "SENSOR_UNCHANGED" not in seen:
                result.diagnostics.append(Diagnostic("WARNING", "SENSOR_UNCHANGED",
                                                     "sensor value is unchanged during commanded motion", time))
                seen.add("SENSOR_UNCHANGED")
            previous_measurement = measured if valid else previous_measurement
            result.samples.append(Sample(time=time, target=target, position=self.plant.position,
                                         velocity=self.plant.velocity, command=force,
                                         measured_position=measured, fault=fault, safe=safe,
                                         requested_command=requested, emergency_stop=emergency,
                                         measurement_age=age, measurement_valid=valid,
                                         position_limit_violation=abs(self.plant.position) >= self.plant.limit,
                                         velocity_limit_violation=(
                                             (self.plant.velocity_limit is not None
                                              and abs(self.plant.velocity) >= self.plant.velocity_limit)
                                             or (self.plant.max_velocity is not None
                                                 and abs(self.plant.velocity) >= self.plant.max_velocity)),
                                         actuator_limit_violation=abs(requested) > self.actuator.maximum))
            result.samples[-1].actuator_velocity_violation = self.actuator.velocity_violation
            result.samples[-1].actuator_acceleration_violation = self.actuator.acceleration_violation
            result.samples[-1].acceleration_limit_violation = self.plant.acceleration_limit_violation
            result.samples[-1].actuator_velocity = self.actuator.velocity
            result.samples[-1].actuator_acceleration = self.actuator.acceleration
            result.samples[-1].feedback_delayed = feedback_delayed
            result.samples[-1].sensor_unchanged = sensor_unchanged
            if safe and not result.diagnostics:
                result.diagnostics.append(Diagnostic("ERROR", "SAFE_STATE", self.safety.reason, time))
            if safe and result.diagnostics and not any(d.code == "SAFE_STATE" for d in result.diagnostics):
                result.diagnostics.append(Diagnostic("ERROR", "SAFE_STATE", self.safety.reason, time))
        acceptance = cfg.get("acceptance", {})
        expected_safe = bool(acceptance.get("expected_safe", False))
        expected_stop = bool(acceptance.get("expected_emergency_stop", False))
        safe_ok = bool(self.safety.safe) == expected_safe
        stop_ok = (not expected_stop) or bool(result.metrics()["emergency_stop"])
        result.passed = safe_ok and stop_ok and (
            expected_safe or float(str(result.metrics()["final_error"])) <= float(
                str(acceptance.get("final_error", 0.1))))
        result.reason = "pass" if result.passed else (self.safety.reason or "acceptance threshold failed")
        return result


def run_file(path: str) -> SimulationResult:
    return Simulation(load_scenario(path)).run()


def result_dict(result: SimulationResult) -> dict[str, Any]:
    return {"metrics": result.metrics(), "samples": [asdict(x) for x in result.samples],
            "diagnostics": [asdict(x) for x in result.diagnostics],
            "events": [asdict(x) for x in result.events]}
