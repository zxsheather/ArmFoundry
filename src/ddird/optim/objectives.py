from __future__ import annotations

from dataclasses import dataclass
from typing import Any

from ddird.data.dataset import TrajectoryRecord
from ddird.eval.evaluator import EvaluationConfig, EvaluationResult, evaluate_robot
from ddird.robots.simple_chain import SimpleChainRobot


@dataclass(frozen=True)
class ObjectiveWeights:
    ik_failure: float = 10.0
    joint_margin: float = 1.0
    manipulability: float = 0.001
    collision: float = 2.0
    smoothness: float = 0.05
    hardware_cost: float = 0.05


def hardware_objective(
    evaluation: EvaluationResult,
    weights: ObjectiveWeights | None = None,
    joint_margin_target: float = 0.12,
) -> float:
    weights = weights or ObjectiveWeights()
    aggregate = evaluation.aggregate
    ik_loss = 1.0 - float(aggregate["ik_success_rate"])
    joint_loss = max(0.0, joint_margin_target - float(aggregate["mean_joint_margin"]))
    manipulability_loss = 1.0 / (1e-6 + max(0.0, float(aggregate["mean_manipulability"])))
    return float(
        weights.ik_failure * ik_loss
        + weights.joint_margin * joint_loss
        + weights.manipulability * manipulability_loss
        + weights.collision * float(aggregate["collision_proxy"])
        + weights.smoothness * float(aggregate["smoothness_cost"])
        + weights.hardware_cost * float(aggregate["hardware_cost"])
    )


def evaluate_design_objective(
    robot: SimpleChainRobot,
    records: list[TrajectoryRecord],
    config: EvaluationConfig | None = None,
    weights: ObjectiveWeights | None = None,
    num_workers: int = 1,
) -> tuple[float, dict[str, Any]]:
    result = evaluate_robot(robot, records, config=config, num_workers=num_workers)
    loss = hardware_objective(result, weights=weights)
    row = dict(result.aggregate)
    row["objective_loss"] = round(loss, 8)
    row["base_x"] = round(float(robot.base_xyz[0]), 6)
    row["base_y"] = round(float(robot.base_xyz[1]), 6)
    row["base_z"] = round(float(robot.base_xyz[2]), 6)
    row["base_yaw"] = round(float(robot.base_yaw), 6)
    for index, value in enumerate(robot.link_lengths, start=1):
        row[f"link_{index}"] = round(float(value), 6)
    return loss, row
