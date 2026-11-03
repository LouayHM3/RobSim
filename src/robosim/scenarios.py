from __future__ import annotations

from pathlib import Path
from typing import Any

import yaml

DEFAULT_SCENARIO: dict[str, Any] = {
    "name": "nominal", "duration": 5.0, "dt": 0.01, "seed": 1,
    "target": 1.0, "trajectory": None, "plant": {}, "pid": {}, "actuator": {}, "sensor": {},
    "safety": {}, "diagnostics": {},
}


def load_scenario(path: str | Path) -> dict[str, Any]:
    with Path(path).open(encoding="utf-8") as handle:
        data = yaml.safe_load(handle) or {}
    merged = dict(DEFAULT_SCENARIO)
    merged.update(data)
    for key in ("plant", "pid", "actuator", "sensor", "safety", "diagnostics"):
        nested = dict(DEFAULT_SCENARIO[key])
        nested.update(data.get(key, {}))
        merged[key] = nested
    return merged
