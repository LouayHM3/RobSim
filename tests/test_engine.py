from robosim.engine import Simulation, run_file


def test_nominal_is_deterministic() -> None:
    first = run_file("scenarios/nominal.yaml")
    second = run_file("scenarios/nominal.yaml")
    assert first.metrics() == second.metrics()
    assert first.passed


def test_dropout_reaches_safe_state() -> None:
    result = run_file("scenarios/sensor_dropout.yaml")
    assert result.passed
    assert any(sample.safe for sample in result.samples)


def test_pid_output_is_bounded() -> None:
    result = Simulation({"duration": 0.1, "dt": 0.01, "target": 100, "plant": {}, "pid": {},
                         "actuator": {}, "sensor": {}, "safety": {"error_limit": 1000}}).run()
    assert max(abs(sample.command) for sample in result.samples) <= 10
