# Simulation model

The simulation is deterministic and advances only on a configured fixed timestep.
The actuator applies a saturated command (and optional slew-rate limit) to a
one-degree-of-freedom mass-damper plant. Position is clamped to the configured
physical limit and velocity is cleared at a limit.

The PID controller computes proportional, integral, and derivative terms. The
integrator is updated only when the output is not saturated, providing simple
anti-windup. Sensor values can include seeded Gaussian noise, bias, drift,
quantization-compatible configuration, dropout, stuck-at behavior, and latency.
These are educational models and are not validated medical-device models.
