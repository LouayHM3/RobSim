from pathlib import Path

from robosim.engine import run_file
from robosim.reporting import write_output_bundle


def test_output_bundle_contains_required_artifacts(tmp_path: Path) -> None:
    result = run_file("scenarios/nominal.yaml")
    write_output_bundle(result, tmp_path)
    for name in ("result.json", "events.json", "timeseries.csv", "report.md"):
        assert (tmp_path / name).exists()


def test_emergency_stop_is_detected() -> None:
    result = run_file("scenarios/emergency_stop.yaml")
    assert result.passed
    assert result.metrics()["emergency_stop"]


def test_all_scenarios_are_loadable() -> None:
    scenarios = list(Path("scenarios").glob("*.yaml"))
    assert len(scenarios) >= 8
    for path in scenarios:
        result = run_file(str(path))
        assert result.samples
