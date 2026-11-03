# Verification outputs

`robosim run scenario.yaml --output results/name` writes `result.json`,
`metrics.json`, `timeseries.csv`, `events.json`, `report.md`, `trajectory.svg`, and
`tracking_error.svg`. Events list fault starts followed by diagnostic events,
each with a timestamp. Plots are generated natively without Python or a GUI.

`robosim verify scenarios --output results/verification` reads `.yaml`/`.yml`
files in filename order, writes one bundle per stem, and adds `summary.md` and
`summary.json`. Empty directories and duplicate stems are errors. Single-file
verification prints metrics; `--output` optionally writes its bundle.

Exit codes: `verify`/`benchmark` return 0 on acceptance success, 1 on failure.
`run` returns 0 after completion regardless of acceptance. All commands return 2
for invalid input or I/O errors.

Metrics include RMSE, MAE, IAE, ISE, final error, overshoot, rise/settling times,
control effort, constraint counts, measurement age, fault/diagnostic counts,
safe-state time, and emergency-stop status. Rise time, overshoot, and settling
band use the initial target, making them primarily fixed-target measures.
`fault_count` counts samples with faults, not distinct events. `saturation_fraction`
retains the original command/request comparison heuristic.

Acceptance checks the expected safe-state outcome and, unless safe state is
expected, final absolute error. `expected_emergency_stop: true` also requires a
stop sample. Other metrics are diagnostic, without implicit acceptance thresholds.
A passing fault case can have large tracking error because safe state was expected.
