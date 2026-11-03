# Requirements traceability

Test groups are printed by the native executable and defined in
[`tests/test_simulation.cpp`](../tests/test_simulation.cpp).

| ID | Requirement | Implementation | Tests |
| --- | --- | --- | --- |
| RS-01 | Deterministic fixed-step runs | `src/engine.cpp` | nominal parity; seeded noise |
| RS-02 | Plant/actuator limits | `src/components.cpp` | saturation/slew; velocity/acceleration; plant limits |
| RS-03 | PID anti-windup | `PID` | saturation anti-windup and reset |
| RS-04 | Sensor imperfections | `Sensor` | latency/bias/quantization/range; configured drift/stuck |
| RS-05 | Scheduled faults/diagnostics | `src/engine.cpp` | all eight scenarios; delay/unchanged feedback |
| RS-06 | Latched safe state | `SafetyMonitor`, engine | dropout timing/recovery; emergency stop |
| RS-07 | Metrics/reports | `src/metrics.cpp`, `src/reporting.cpp` | known metrics; bundle schema/repeatability |
| RS-08 | Trajectories | `target_at` | fixed/step/ramp/interpolation |
| RS-09 | Configuration validation | `src/scenario.cpp`, CLI | invalid settings; YAML syntax/duplicates; CLI errors |
