# Scenarios and faults

Scenarios are YAML mappings. `duration`, `dt`, `target`, and `seed` are top-level;
`plant`, `pid`, `actuator`, `sensor`, and `safety` contain component parameters.
Use `acceptance.final_error` to define the verification threshold. A fault schedule is
an array of `{type, start, end}` entries. Supported scheduled faults are
`sensor_dropout`, `sensor_drift` (with `rate`), `sensor_stuck`,
`communication_interruption`, and `emergency_stop`. Sensor configuration also supports
noise, bias, drift, stuck values, and latency. Actuator maximum and slew rate model
saturation and rate limiting. Set `acceptance.expected_safe: true` for scenarios whose
verified outcome is a safe state. Safety transitions and fault starts are recorded in
`events.json`.

The repository includes nominal, noise, dropout, drift, stuck, communication,
saturation, and emergency-stop scenarios.
