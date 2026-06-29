from __future__ import annotations

import csv
import json
from pathlib import Path
from typing import Any

import numpy as np

from ddird.data.dataset import TrajectoryRecord


DEFAULT_DATA_ROOT = Path("data/libero_ee_trajectories")
DEFAULT_OUTPUT_ROOT = Path("outputs/libero_ddird")


def ensure_dir(path: str | Path) -> Path:
    path = Path(path)
    path.mkdir(parents=True, exist_ok=True)
    return path


def write_json(path: str | Path, payload: dict[str, Any]) -> Path:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return path


def write_csv(path: str | Path, rows: list[dict[str, Any]]) -> Path:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    if not rows:
        path.write_text("", encoding="utf-8")
        return path

    fieldnames: list[str] = []
    for row in rows:
        for key in row:
            if key not in fieldnames:
                fieldnames.append(key)

    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            writer.writerow(row)
    return path


def read_json(path: str | Path) -> dict[str, Any]:
    return json.loads(Path(path).read_text(encoding="utf-8"))


def filter_records(
    records: list[TrajectoryRecord],
    suite: str | None = None,
    max_trajectories: int | None = None,
    max_waypoints_per_trajectory: int | None = None,
    seed: int = 7,
) -> list[TrajectoryRecord]:
    selected = [record for record in records if suite is None or record.suite == suite]
    if max_trajectories is not None and max_trajectories > 0 and len(selected) > max_trajectories:
        rng = np.random.default_rng(seed)
        indexes = sorted(rng.choice(len(selected), size=max_trajectories, replace=False).tolist())
        selected = [selected[index] for index in indexes]
    selected = [record.downsample(max_waypoints_per_trajectory) for record in selected]
    return selected
