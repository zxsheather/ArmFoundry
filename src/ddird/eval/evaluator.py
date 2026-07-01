from __future__ import annotations

from concurrent.futures import ProcessPoolExecutor
from dataclasses import dataclass
from typing import Any

import numpy as np

from ddird.data.dataset import TrajectoryRecord
from ddird.eval.ik import solve_pose_ik_multiseed, solve_position_ik_multiseed
from ddird.eval.metrics import TrajectoryMetrics, aggregate_metric_dicts, safe_mean, safe_min
from ddird.eval.orientation import ORIENTATION_FORMATS, record_orientation_matrices
from ddird.eval.tool_frame_mapping import TOOL_FRAME_MAPPING_MODES, apply_tool_frame_mapping
from ddird.robots.simple_chain import SimpleChainRobot

LIBERO_SOURCE_BASE_BY_SUITE = {
    "libero_goal": ((-0.66, 0.0, 0.912), 0.0),
    "libero_object": ((-0.6, 0.0, 0.0), 0.0),
    "libero_spatial": ((-0.66, 0.0, 0.912), 0.0),
}


@dataclass(frozen=True)
class EvaluationWeights:
    ik_failure: float = 10.0
    joint_margin: float = 1.0
    manipulability: float = 0.3
    collision: float = 2.0
    smoothness: float = 0.05
    hardware_cost: float = 0.05


@dataclass(frozen=True)
class EvaluationConfig:
    ik_tolerance: float = 2e-3
    max_iters: int = 120
    manipulability_threshold: float = 0.004
    joint_margin_threshold: float = 0.12
    base_pose_mode: str = "fixed"
    evaluation_mode: str = "position"
    orientation_tolerance: float = 0.10
    orientation_format: str = "auto"
    orientation_weight: float = 1.0
    tool_frame_mapping: str = "identity"
    weights: EvaluationWeights = EvaluationWeights()

    def __post_init__(self) -> None:
        if self.base_pose_mode not in {"fixed", "source"}:
            raise ValueError("base_pose_mode must be 'fixed' or 'source'")
        if self.evaluation_mode not in {"position", "pose"}:
            raise ValueError("evaluation_mode must be 'position' or 'pose'")
        if self.orientation_tolerance <= 0.0:
            raise ValueError("orientation_tolerance must be positive")
        if self.orientation_format not in ORIENTATION_FORMATS:
            raise ValueError(f"orientation_format must be one of {ORIENTATION_FORMATS}")
        if self.orientation_weight <= 0.0:
            raise ValueError("orientation_weight must be positive")
        if self.tool_frame_mapping not in TOOL_FRAME_MAPPING_MODES:
            raise ValueError(f"tool_frame_mapping must be one of {TOOL_FRAME_MAPPING_MODES}")


@dataclass
class EvaluationResult:
    robot: SimpleChainRobot
    trajectory_rows: list[dict[str, Any]]
    aggregate: dict[str, Any]
    details: list[dict[str, Any]]

    @property
    def objective_loss(self) -> float:
        ik_loss = 1.0 - float(self.aggregate["ik_success_rate"])
        joint_loss = max(0.0, 0.12 - float(self.aggregate["mean_joint_margin"]))
        manip_loss = 1.0 / (1e-6 + float(self.aggregate["mean_manipulability"]))
        collision = float(self.aggregate["collision_proxy"])
        smoothness = float(self.aggregate["smoothness_cost"])
        hardware = float(self.aggregate["hardware_cost"])
        return ik_loss * 10.0 + joint_loss + 0.001 * manip_loss + 2.0 * collision + 0.05 * smoothness + 0.05 * hardware


def _smoothness(q_values: np.ndarray, successes: np.ndarray) -> float:
    if len(q_values) < 2:
        return 0.0
    diffs = []
    for index in range(len(q_values) - 1):
        if successes[index] and successes[index + 1]:
            delta = q_values[index + 1] - q_values[index]
            diffs.append(float(np.dot(delta, delta)))
    if not diffs:
        return 0.0
    return float(np.mean(diffs))


def _record_source_base(record: TrajectoryRecord) -> tuple[np.ndarray, float]:
    base_xyz = record.metadata.get("source_robot_base_xyz")
    if base_xyz is not None:
        return np.asarray(base_xyz, dtype=float).reshape(3), float(record.metadata.get("source_robot_base_yaw", 0.0))
    if record.suite in LIBERO_SOURCE_BASE_BY_SUITE:
        fallback_xyz, fallback_yaw = LIBERO_SOURCE_BASE_BY_SUITE[record.suite]
        return np.asarray(fallback_xyz, dtype=float).reshape(3), float(fallback_yaw)
    raise ValueError(
        f"Record {record.suite}/{record.task}/{record.episode_id} does not contain source_robot_base_xyz metadata"
    )


def _robot_for_record(robot: SimpleChainRobot, record: TrajectoryRecord, config: EvaluationConfig) -> SimpleChainRobot:
    if config.base_pose_mode == "fixed":
        return robot
    base_xyz, base_yaw = _record_source_base(record)
    return robot.with_base(base_xyz=base_xyz, base_yaw=base_yaw)


def evaluate_trajectory(
    robot: SimpleChainRobot,
    record: TrajectoryRecord,
    config: EvaluationConfig | None = None,
    return_details: bool = False,
) -> tuple[TrajectoryMetrics, dict[str, Any] | None]:
    config = config or EvaluationConfig()
    robot = _robot_for_record(robot, record, config)
    q_prev = robot.neutral_q.copy()
    q_values = []
    successes = []
    errors = []
    margins = []
    manip = []
    collisions = []
    achieved = []
    achieved_rotations = []
    orientation_errors = []
    pose_successes = []
    target_rotations = None
    if config.evaluation_mode == "pose":
        target_rotations = record_orientation_matrices(record, config.orientation_format)
        if target_rotations is None:
            raise ValueError(
                f"Pose-aware evaluation requested, but {record.path} does not contain orientation data"
            )
        if len(target_rotations) != record.num_waypoints:
            raise ValueError(
                f"Pose-aware evaluation requested, but {record.path} has {len(target_rotations)} orientations "
                f"for {record.num_waypoints} waypoints"
            )
        target_rotations = apply_tool_frame_mapping(target_rotations, robot, config.tool_frame_mapping)

    for index, target in enumerate(record.ee_pos):
        seeds = [q_prev, robot.neutral_q, -robot.neutral_q]
        if target_rotations is None:
            ik = solve_position_ik_multiseed(
                robot,
                target,
                seeds=seeds,
                max_iters=config.max_iters,
                tolerance=config.ik_tolerance,
            )
        else:
            ik = solve_pose_ik_multiseed(
                robot,
                target,
                target_rotations[index],
                seeds=seeds,
                max_iters=config.max_iters,
                position_tolerance=config.ik_tolerance,
                orientation_tolerance=config.orientation_tolerance,
                orientation_weight=config.orientation_weight,
            )
        q = ik.q
        q_prev = q.copy()
        q_values.append(q)
        successes.append(ik.success)
        errors.append(ik.position_error_norm if ik.position_error_norm is not None else ik.error_norm)
        if ik.orientation_error_norm is not None:
            orientation_errors.append(ik.orientation_error_norm)
            pose_successes.append(ik.success)
        margins.append(float(np.min(robot.joint_margin(q))))
        manip.append(robot.manipulability(q))
        collisions.append(robot.collision_proxy(q))
        achieved.append(robot.end_effector_position(q))
        if target_rotations is not None:
            achieved_rotations.append(robot.end_effector_rotation(q))

    q_array = np.asarray(q_values, dtype=float)
    success_array = np.asarray(successes, dtype=bool)
    errors_array = np.asarray(errors, dtype=float)
    margins_array = np.asarray(margins, dtype=float)
    manip_array = np.asarray(manip, dtype=float)
    collision_array = np.asarray(collisions, dtype=float)
    orientation_error_array = np.asarray(orientation_errors, dtype=float)
    pose_success_array = np.asarray(pose_successes, dtype=bool)

    metrics = TrajectoryMetrics(
        robot_name=robot.name,
        suite=record.suite,
        task=record.task,
        episode_id=record.episode_id,
        num_waypoints=record.num_waypoints,
        ik_success_rate=float(np.mean(success_array)) if len(success_array) else 0.0,
        trajectory_success=bool(np.all(success_array)) if len(success_array) else False,
        mean_position_error=safe_mean(errors_array),
        max_position_error=float(np.max(errors_array)) if len(errors_array) else 0.0,
        mean_joint_margin=safe_mean(margins_array),
        min_joint_margin=safe_min(margins_array),
        mean_manipulability=safe_mean(manip_array),
        min_manipulability=safe_min(manip_array),
        manipulability_below_threshold=float(np.mean(manip_array < config.manipulability_threshold)) if len(manip_array) else 0.0,
        smoothness_cost=_smoothness(q_array, success_array),
        collision_proxy=safe_mean(collision_array),
        hardware_cost=robot.reach_proxy,
        orientation_tolerance_rad=config.orientation_tolerance if target_rotations is not None else None,
        mean_orientation_error_rad=safe_mean(orientation_error_array) if target_rotations is not None else None,
        max_orientation_error_rad=float(np.max(orientation_error_array)) if len(orientation_error_array) else None,
        pose_success_rate=float(np.mean(pose_success_array)) if len(pose_success_array) else None,
        trajectory_pose_success=bool(np.all(pose_success_array)) if len(pose_success_array) else None,
    )

    details = None
    if return_details:
        details = {
            "target_pos": record.ee_pos,
            "achieved_pos": np.asarray(achieved, dtype=float),
            "q": q_array,
            "success": success_array,
            "position_error": errors_array,
            "joint_margin": margins_array,
            "manipulability": manip_array,
        }
        if target_rotations is not None:
            details.update(
                {
                    "target_rotation": target_rotations,
                    "achieved_rotation": np.asarray(achieved_rotations, dtype=float),
                    "orientation_error": orientation_error_array,
                    "pose_success": pose_success_array,
                }
            )
    return metrics, details


def _evaluate_trajectory_worker(args: tuple[SimpleChainRobot, TrajectoryRecord, EvaluationConfig | None, bool]) -> tuple[dict[str, Any], dict[str, Any] | None]:
    robot, record, config, return_details = args
    metrics, trajectory_details = evaluate_trajectory(robot, record, config=config, return_details=return_details)
    detail_payload = None
    if trajectory_details is not None:
        detail_payload = {
            "suite": record.suite,
            "task": record.task,
            "episode_id": record.episode_id,
            **trajectory_details,
        }
    return metrics.to_dict(), detail_payload


def evaluate_robot(
    robot: SimpleChainRobot,
    records: list[TrajectoryRecord],
    config: EvaluationConfig | None = None,
    return_details: bool = False,
    num_workers: int = 1,
) -> EvaluationResult:
    if num_workers > 1 and len(records) > 1:
        args = [(robot, record, config, return_details) for record in records]
        with ProcessPoolExecutor(max_workers=num_workers) as executor:
            results = list(executor.map(_evaluate_trajectory_worker, args))
        rows = [row for row, _ in results]
        details = [detail for _, detail in results if detail is not None]
    else:
        rows = []
        details = []
        for record in records:
            row, detail = _evaluate_trajectory_worker((robot, record, config, return_details))
            rows.append(row)
            if detail is not None:
                details.append(detail)
    aggregate = aggregate_metric_dicts(rows, group_name=robot.name)
    return EvaluationResult(robot=robot, trajectory_rows=rows, aggregate=aggregate, details=details)


def evaluate_robot_by_suite(
    robot: SimpleChainRobot,
    records: list[TrajectoryRecord],
    config: EvaluationConfig | None = None,
    num_workers: int = 1,
) -> list[dict[str, Any]]:
    rows = evaluate_robot(robot, records, config=config, num_workers=num_workers).trajectory_rows
    suites = sorted({row["suite"] for row in rows})
    suite_rows = []
    for suite in suites:
        suite_metrics = [row for row in rows if row["suite"] == suite]
        aggregate = aggregate_metric_dicts(suite_metrics, group_name=robot.name)
        aggregate["suite"] = suite
        suite_rows.append(aggregate)
    return suite_rows
