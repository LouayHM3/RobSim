# Verification plan

`robosim_tests` uses explicit checks active in Release builds. It covers deterministic
runs, noiseless Python reference parity, all scenarios, fault timing and safety
latching, PID anti-windup, actuator/plant limits, sensors, trajectories, known metric
values, invalid configuration, and report artifacts.

CTest also exercises directory verification, CLI help, invalid repeats, and empty
directory rejection. GitHub Actions builds/tests Linux and Windows and runs Linux
AddressSanitizer/UndefinedBehaviorSanitizer. Scenario bundles are workflow artifacts.

See [CONTRIBUTING.md](../CONTRIBUTING.md) for commands. Tracked `results/all`
illustrates current C++ simulation behavior, rather than hardware validation.
