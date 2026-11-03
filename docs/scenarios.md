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

Additional scheduled faults are `sensor_invalid` (alias `invalid_reading`) and
`actuator_stuck`. Fault intervals include both endpoints. `seed` must be a 32-bit
unsigned integer; `dt` and `duration` must be positive and finite. Configurations
with more than one million simulation steps are rejected. Unknown settings,
unknown faults, and duplicate YAML keys are errors.

Replace `target` with `trajectory` to use a dynamic command:

```yaml
# Step: before the transition time, use `before`; then use `after`.
trajectory: {type: step, time: 0.5, before: 0.0, after: 1.0}
```

```yaml
# Ramp clamps to the endpoint after its duration.
trajectory: {type: ramp, start: 0.0, end: 1.0, start_time: 0.0, duration: 1.0}
```

```yaml
trajectory:
  type: multi-point
  points:
    - {time: 0.0, value: 0.0}
    - {time: 1.0, value: 1.0}
    - {time: 2.0, value: 0.5}
```

Multi-point times must be nonnegative and strictly increasing. Values are linearly
interpolated, with the first/last point held outside the specified interval. A
fixed trajectory can use `{type: fixed, value: 1.0}`.

Optional `plant` limits include `velocity_limit` (alias `max_velocity`) and
`acceleration_limit`. `actuator` supports `max_velocity`/`max_acceleration` with
`velocity_limit`/`acceleration_limit` aliases. Sensor settings include `noise_std`,
`bias`, `drift_per_second`, `stuck_value`, `latency`, `quantization_step` (alias
`quantization`), `valid_min`, `valid_max`, `dropout`, `invalid`, and
`communication_interruption`. The `diagnostics` block supports
`feedback_latency_limit` and `unchanged_samples`.

The repository includes nominal, noise, dropout, drift, stuck, communication,
saturation, and emergency-stop scenarios. See [migration notes](migration.md)
for differences from the Python implementation.
