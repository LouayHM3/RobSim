# Requirements traceability

| ID | Requirement | Implementation | Verification |
|---|---|---|---|
| RS-01 | Fixed-step deterministic simulation | `robosim.engine.Simulation` | `test_nominal_is_deterministic` |
| RS-02 | Bounded actuator and plant limits | `robosim.actuators`, `robosim.models.Plant` | `test_pid_output_is_bounded` |
| RS-03 | PID feedback with anti-windup | `robosim.control.PID` | engine integration tests |
| RS-04 | Configurable sensor imperfections | `robosim.sensors.Sensor` | dropout and scenario tests |
| RS-05 | Scheduled faults and diagnostics | `robosim.engine`, `robosim.diagnostics` | dropout and output tests |
| RS-06 | Safe-state behavior | `SafetyMonitor` | `test_dropout_reaches_safe_state` |
| RS-07 | Reproducible metrics and reports | `robosim.metrics`, `robosim.reporting` | `test_output_bundle` |
