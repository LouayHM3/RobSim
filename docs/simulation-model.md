# Simulation model

The one-dimensional mass-damper plant uses fixed-step semi-implicit Euler:

```text
acceleration = (force - damping * velocity) / mass
velocity += acceleration * dt
position += velocity * dt
```

Acceleration/velocity constraints clamp the state. Position is clamped to
`plant.limit` and velocity becomes zero at the boundary. Time indices run from
zero through `round(duration / dt)`, with a post-update position at each index.
Choose a duration divisible by `dt` for an exact final timestamp.

PID integration updates only when output is not saturated, providing anti-windup.
The actuator models output magnitude, slew, velocity, acceleration, and stuck faults.
The sensor supports Gaussian noise, bias, drift, quantization, dropout, stuck
readings, valid ranges, and latency. MT19937 and an explicit Box-Muller transform
produce reproducible noise for the same seed/build, with possible small platform
math differences. The sequence differs from Python's original random generator.

Latency reports the configured delay; the current position is a startup fallback
when no historical reading exists yet. Fault intervals are inclusive. Safe state
removes force and remains latched after faults end. Scheduled sensor-stuck and
emergency-stop faults trigger safety directly, preserving the original model.
This educational model does not establish physical hardware safety.
