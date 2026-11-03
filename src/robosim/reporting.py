from __future__ import annotations

import csv
import json
from pathlib import Path

from .engine import result_dict
from .models import SimulationResult


def write_json(result: SimulationResult, path: str | Path) -> None:
    target = Path(path)
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text(json.dumps(result_dict(result), indent=2), encoding="utf-8")


def write_metrics(result: SimulationResult, path: str | Path) -> None:
    target = Path(path)
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text(json.dumps(result.metrics(), indent=2), encoding="utf-8")


def write_csv(result: SimulationResult, path: str | Path) -> None:
    target = Path(path)
    target.parent.mkdir(parents=True, exist_ok=True)
    if not result.samples:
        target.write_text("", encoding="utf-8")
        return
    with target.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(result_dict(result)["samples"][0]))
        writer.writeheader()
        writer.writerows(result_dict(result)["samples"])


def plot(result: SimulationResult, path: str | Path) -> None:
    import matplotlib.pyplot as plt
    times = [s.time for s in result.samples]
    plt.plot(times, [s.target for s in result.samples], label="target")
    plt.plot(times, [s.position for s in result.samples], label="position")
    plt.xlabel("time (s)"); plt.ylabel("position"); plt.legend(); plt.tight_layout()
    Path(path).parent.mkdir(parents=True, exist_ok=True); plt.savefig(path); plt.close()


def plot_tracking_error(result: SimulationResult, path: str | Path) -> None:
    import matplotlib.pyplot as plt
    times = [sample.time for sample in result.samples]
    errors = [sample.target - sample.position for sample in result.samples]
    plt.plot(times, errors, label="tracking error")
    plt.axhline(0.0, color="black", linewidth=0.7)
    plt.xlabel("time (s)")
    plt.ylabel("error (position units)")
    plt.legend()
    plt.tight_layout()
    Path(path).parent.mkdir(parents=True, exist_ok=True)
    plt.savefig(path)
    plt.close()


def write_events(result: SimulationResult, path: str | Path) -> None:
    target = Path(path)
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text(json.dumps([event.__dict__ for event in result.events + result.diagnostics],
                                 indent=2), encoding="utf-8")


def write_report(result: SimulationResult, path: str | Path) -> None:
    target = Path(path)
    target.parent.mkdir(parents=True, exist_ok=True)
    metrics = result.metrics()
    lines = [f"# RoboSim report: {result.scenario}", "", f"Status: **{'PASS' if result.passed else 'FAIL'}**",
             "", "## Metrics", ""]
    lines.extend(f"- **{key}**: {value}" for key, value in metrics.items())
    lines.extend(["", "## Diagnostics", ""])
    lines.extend(f"- `{event.time:.3f}s` {event.level} {event.code}: {event.message}"
                 for event in result.events + result.diagnostics)
    target.write_text("\n".join(lines) + "\n", encoding="utf-8")


def write_output_bundle(result: SimulationResult, directory: str | Path) -> None:
    output = Path(directory)
    output.mkdir(parents=True, exist_ok=True)
    write_json(result, output / "result.json")
    write_metrics(result, output / "metrics.json")
    write_csv(result, output / "timeseries.csv")
    write_events(result, output / "events.json")
    write_report(result, output / "report.md")
    try:
        plot(result, output / "trajectory.png")
        plot_tracking_error(result, output / "tracking_error.png")
    except ImportError:
        # Keep the bundle contract even in minimal installations.
        pixel = bytes.fromhex(
            "89504e470d0a1a0a0000000d49484452000000010000000108060000001f"
            "15c4890000000d49444154789c6360000000020001e221bc330000000049454e44ae426082")
        (output / "trajectory.png").write_bytes(pixel)
        (output / "tracking_error.png").write_bytes(pixel)


def write_summary(results: list[SimulationResult], directory: str | Path) -> None:
    """Write one machine-readable and one human-readable report for a scenario run."""
    output = Path(directory)
    output.mkdir(parents=True, exist_ok=True)
    rows = [result.metrics() for result in results]
    (output / "summary.json").write_text(json.dumps(rows, indent=2), encoding="utf-8")
    columns = ["scenario", "passed", "final_error", "max_error", "duration",
               "safe_state", "fault_detection_latency"]
    lines = ["# RoboSim all-scenarios summary", "", "| " + " | ".join(columns) + " |",
             "| " + " | ".join("---" for _ in columns) + " |"]
    for row in rows:
        lines.append("| " + " | ".join(str(row.get(column, "")) for column in columns) + " |")
    (output / "summary.md").write_text("\n".join(lines) + "\n", encoding="utf-8")
