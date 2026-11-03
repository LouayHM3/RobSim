# Python to C++ migration

Version 0.2.0 replaces Python with a C++17 library and CLI. Original source and
reports remain in Git history.

| Area | C++ behavior |
| --- | --- |
| Installation | CMake instead of pip; no Python runtime |
| Commands | `run`, `verify`, `benchmark`, JSON/CSV and bundles retained |
| Scenarios | All eight existing YAML files unchanged |
| Plots | SVG replaces PNG/matplotlib output |
| Noise | MT19937/Box-Muller; numerical traces differ from Python |
| Validation | Reject unknown keys/faults, invalid ranges/timing, duplicate YAML keys, excessive steps |
| Single-file verification | Optional `--output` now writes a bundle |
| Directory verification | Adds `.yml`; rejects empty directories and duplicate stems |
| Benchmark | Validates repeats; excludes configuration loading from timing |
| Sensor configuration | Drift/stuck/interruption/invalid settings persist outside scheduled faults |

Fixed-step order, default component values, safety latching, acceptance semantics,
and JSON sample/metric fields are retained. Tests compare noiseless nominal final
error against the Python reference with a tolerance. Cross-language noise parity
is not promised. `command` retains actuator output before the safe-state zero-force
override. See [verification outputs](verification.md) for metric limitations.

Offline builds can use installed yaml-cpp/nlohmann_json CMake packages with
`-DROBOSIM_USE_SYSTEM_DEPS=ON`. Otherwise pinned source archives are downloaded
during the first configure and cached under the build directory.
