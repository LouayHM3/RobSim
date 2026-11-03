# Architecture

`Simulation` composes a fixed-step `Plant`, `PID`, `Actuator`, `Sensor`, and
`SafetyMonitor`. Each step samples the sensor, computes a bounded command, checks
diagnostics, then advances the plant. `SimulationResult` contains the complete trace,
diagnostics, and acceptance metrics.
