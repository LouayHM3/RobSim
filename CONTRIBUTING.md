# Contributing

Create a focused branch, add or update tests for behavior changes, and run the
full local checks before opening a pull request:

```console
python -m pip install -e ".[dev,plot]"
pytest
ruff check .
mypy src
python -m robosim verify scenarios --output results/all
```

Keep scenario files deterministic (use a seed), preserve existing scenarios,
and include a short description of validation results in the pull request.
