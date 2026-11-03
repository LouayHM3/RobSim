# Test results

Run:

```console
python -m pytest
python -m robosim verify scenarios --output results/verification
```

The test suite validates deterministic nominal tracking, dropout safe-state
behavior, bounded control output, and generated report artifacts. The scenario
command produces one independently inspectable bundle per YAML configuration.
All numeric results are generated at execution time rather than embedded claims.
