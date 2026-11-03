"""Convenience metric calculations for simulation traces."""

from __future__ import annotations

from typing import cast

from .models import SimulationResult


def summarize(result: SimulationResult) -> dict[str, object]:
    """Return the acceptance and performance summary for a result."""
    return result.metrics()


def root_mean_square_error(result: SimulationResult) -> float:
    if not result.samples:
        return 0.0
    value = (sum((sample.target - sample.position) ** 2 for sample in result.samples)
             / len(result.samples)) ** 0.5
    return cast(float, value)
