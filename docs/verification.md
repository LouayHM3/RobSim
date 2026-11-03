# Verification outputs

`robosim run scenario.yaml --output results/name` writes `result.json`, a complete
`timeseries.csv`, `events.json`, `report.md`, and `plot.png` when matplotlib is
installed. `robosim verify scenarios` executes every YAML file and writes one bundle
per scenario. Metrics include tracking error, MAE, IAE, ISE, overshoot, rise and
settling time, control effort, saturation fraction, fault/diagnostic counts, safe
state, and emergency-stop status.
