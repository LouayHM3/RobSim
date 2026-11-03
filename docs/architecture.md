# Architecture

The CLI calls the reusable `robosim_core` C++17 library. Public declarations live
in [`include/robosim/simulation.hpp`](../include/robosim/simulation.hpp).

| File | Responsibility |
| --- | --- |
| `src/scenario.cpp` | YAML loading, defaults, validation, target interpolation |
| `src/components.cpp` | PID, plant, actuator, sensor, and safety monitor |
| `src/engine.cpp` | Fixed-step simulation, faults, diagnostics, acceptance |
| `src/metrics.cpp` | Metrics and JSON serialization |
| `src/reporting.cpp` | JSON/CSV/Markdown bundles and native SVG plots |
| `src/main.cpp` | `run`, `verify`, and `benchmark` commands |

Each time index activates scheduled faults, reads the pre-step plant position,
computes the PID request, constrains actuator output, checks safety, and advances
the plant. Samples record the post-step plant state and its pre-step measurement
at the current time index. This preserves Python's update order and step at zero.

Safe state latches and removes applied plant force; existing velocity decays with
damping. Recorded `command` is actuator output before the safe-state override.
Each run owns its state, random generator, and sensor history. yaml-cpp handles
YAML and nlohmann/json handles structured data. SVG plots use only standard C++.
