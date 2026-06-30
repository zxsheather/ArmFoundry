from __future__ import annotations

import argparse
import warnings

from ddird.data.dataset import load_dataset
from ddird.eval.evaluator import EvaluationConfig, evaluate_robot
from ddird.eval.metrics import aggregate_metric_dicts
from ddird.experiments.common import (
    DEFAULT_DATA_ROOT,
    DEFAULT_OUTPUT_ROOT,
    filter_records,
    write_csv,
    write_json,
)
from ddird.robots.robot_registry import baseline_robots
from ddird.robots.tool_frames import tool_frame_metadata


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Evaluate robot-arm baselines on processed EE trajectories.")
    parser.add_argument("--data", default=DEFAULT_DATA_ROOT)
    parser.add_argument("--outputs", default=DEFAULT_OUTPUT_ROOT)
    parser.add_argument("--max-iters", type=int, default=80)
    parser.add_argument("--suite", default=None, help="Optional suite filter, e.g. libero_spatial.")
    parser.add_argument("--max-trajectories", type=int, default=None, help="Optional cap for quick real-data runs.")
    parser.add_argument("--max-waypoints-per-trajectory", type=int, default=None, help="Evenly downsample each trajectory before IK.")
    parser.add_argument("--num-workers", type=int, default=1, help="Parallel worker processes per robot.")
    parser.add_argument(
        "--base-pose-mode",
        choices=("auto", "fixed", "source"),
        default="auto",
        help=(
            "Use fixed robot baselines, each trajectory's source robot base pose, or auto-select source for "
            "LIBERO world-frame records and fixed for synthetic records."
        ),
    )
    parser.add_argument("--include-true-models", action="store_true", help="Also evaluate available true robot models such as panda_true.")
    parser.add_argument("--seed", type=int, default=7)
    return parser


def _record_is_libero_world(record) -> bool:
    source = str(record.metadata.get("source", "")).lower()
    frame = str(record.metadata.get("coordinate_frame", "")).lower()
    return source == "libero" and frame == "world"


def _resolve_base_pose_mode(requested: str, records) -> str:
    if requested == "auto":
        return "source" if any(_record_is_libero_world(record) for record in records) else "fixed"
    if requested == "fixed" and any(_record_is_libero_world(record) for record in records):
        warnings.warn(
            "Using fixed base pose for LIBERO world-frame records. This is diagnostic only; "
            "source-base mode is the recommended evaluation mode.",
            stacklevel=2,
        )
    return requested


def main() -> None:
    args = build_parser().parse_args()
    records = filter_records(
        load_dataset(args.data),
        suite=args.suite,
        max_trajectories=args.max_trajectories,
        max_waypoints_per_trajectory=args.max_waypoints_per_trajectory,
        seed=args.seed,
    )
    if not records:
        raise SystemExit("No trajectories selected for evaluation")
    base_pose_mode = _resolve_base_pose_mode(args.base_pose_mode, records)
    config = EvaluationConfig(max_iters=args.max_iters, base_pose_mode=base_pose_mode)

    aggregate_rows = []
    suite_rows = []
    trajectory_rows = []
    robots = baseline_robots(include_true_models=args.include_true_models)
    for robot in robots:
        result = evaluate_robot(robot, records, config=config, num_workers=args.num_workers)
        aggregate_rows.append(result.aggregate)
        for suite in sorted({row["suite"] for row in result.trajectory_rows}):
            aggregate = aggregate_metric_dicts(
                [row for row in result.trajectory_rows if row["suite"] == suite],
                group_name=robot.name,
            )
            aggregate["suite"] = suite
            suite_rows.append(aggregate)
        trajectory_rows.extend(result.trajectory_rows)

    write_csv(f"{args.outputs}/baseline_results.csv", aggregate_rows)
    write_csv(f"{args.outputs}/baseline_results_by_suite.csv", suite_rows)
    write_csv(f"{args.outputs}/baseline_trajectory_results.csv", trajectory_rows)
    write_json(
        f"{args.outputs}/tool_frame_metadata.json",
        {
            "base_pose_mode": base_pose_mode,
            "max_iters": args.max_iters,
            "robots": [
                tool_frame_metadata(
                    robot,
                    base_body="robot0_base" if robot.name == "panda_true" else None,
                    target_site="gripper0_grip_site" if robot.name == "panda_true" else None,
                    target_body="gripper0_eef" if robot.name == "panda_true" else None,
                )
                for robot in robots
            ],
        },
    )
    print(
        f"Evaluated {len(aggregate_rows)} baseline robots on {len(records)} trajectories "
        f"({sum(record.num_waypoints for record in records)} waypoints, workers={args.num_workers}, "
        f"base_pose_mode={base_pose_mode})"
    )


if __name__ == "__main__":
    main()
