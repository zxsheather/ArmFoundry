from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import numpy as np

from ddird.data.dataset import TrajectoryRecord
from ddird.eval.evaluator import EvaluationConfig
from ddird.optim.objectives import ObjectiveWeights, evaluate_design_objective
from ddird.robots.simple_chain import SimpleChainRobot


@dataclass(frozen=True)
class SearchBounds:
    base_x: tuple[float, float] = (-0.35, 0.45)
    base_y: tuple[float, float] = (-0.45, 0.45)
    base_z: tuple[float, float] = (0.0, 0.16)
    base_yaw: tuple[float, float] = (-np.pi, np.pi)
    link_1: tuple[float, float] = (0.22, 0.42)
    link_2: tuple[float, float] = (0.24, 0.50)
    link_3: tuple[float, float] = (0.20, 0.48)
    link_4: tuple[float, float] = (0.10, 0.24)


@dataclass
class SearchResult:
    best_robot: SimpleChainRobot
    best_loss: float
    rows: list[dict[str, Any]]

    @property
    def best_row(self) -> dict[str, Any]:
        return min(self.rows, key=lambda row: float(row["objective_loss"]))


def _uniform(rng: np.random.Generator, bounds: tuple[float, float]) -> float:
    return float(rng.uniform(bounds[0], bounds[1]))


def sample_design(
    base_robot: SimpleChainRobot,
    rng: np.random.Generator,
    bounds: SearchBounds,
    optimize_links: bool,
    name: str,
) -> SimpleChainRobot:
    base_xyz = np.array(
        [
            _uniform(rng, bounds.base_x),
            _uniform(rng, bounds.base_y),
            _uniform(rng, bounds.base_z),
        ],
        dtype=float,
    )
    base_yaw = _uniform(rng, bounds.base_yaw)
    if optimize_links:
        link_lengths = np.array(
            [
                _uniform(rng, bounds.link_1),
                _uniform(rng, bounds.link_2),
                _uniform(rng, bounds.link_3),
                _uniform(rng, bounds.link_4),
            ],
            dtype=float,
        )
    else:
        link_lengths = base_robot.link_lengths
    return base_robot.with_design(base_xyz=base_xyz, base_yaw=base_yaw, link_lengths=link_lengths, name=name)


def random_search(
    base_robot: SimpleChainRobot,
    records: list[TrajectoryRecord],
    iterations: int = 48,
    optimize_links: bool = False,
    seed: int = 7,
    bounds: SearchBounds | None = None,
    config: EvaluationConfig | None = None,
    weights: ObjectiveWeights | None = None,
    num_workers: int = 1,
) -> SearchResult:
    bounds = bounds or SearchBounds()
    rng = np.random.default_rng(seed)
    rows: list[dict[str, Any]] = []

    candidates = [base_robot.with_design(name=f"{base_robot.name}_initial")]
    for index in range(iterations):
        candidates.append(sample_design(base_robot, rng, bounds, optimize_links, name=f"{base_robot.name}_candidate_{index:04d}"))

    best_robot = candidates[0]
    best_loss = float("inf")
    for index, candidate in enumerate(candidates):
        loss, row = evaluate_design_objective(candidate, records, config=config, weights=weights, num_workers=num_workers)
        row["iteration"] = index
        row["search_mode"] = "base_plus_links" if optimize_links else "base_only"
        rows.append(row)
        if loss < best_loss:
            best_loss = loss
            best_robot = candidate

    best_named = best_robot.with_design(name="simple_optimized_links" if optimize_links else "simple_optimized_base")
    return SearchResult(best_robot=best_named, best_loss=best_loss, rows=rows)
