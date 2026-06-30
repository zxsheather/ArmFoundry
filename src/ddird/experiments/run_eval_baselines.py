from __future__ import annotations

import argparse

from ddird.data.dataset import load_dataset
from ddird.eval.evaluator import EvaluationConfig, evaluate_robot
from ddird.eval.metrics import aggregate_metric_dicts
from ddird.experiments.common import DEFAULT_DATA_ROOT, DEFAULT_OUTPUT_ROOT, filter_records, write_csv
from ddird.robots.robot_registry import baseline_robots


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Evaluate baseline robot-arm proxies on processed EE trajectories.")
    parser.add_argument("--data", default=DEFAULT_DATA_ROOT)
    parser.add_argument("--outputs", default=DEFAULT_OUTPUT_ROOT)
    parser.add_argument("--max-iters", type=int, default=80)
    parser.add_argument("--suite", default=None, help="Optional suite filter, e.g. libero_spatial.")
    parser.add_argument("--max-trajectories", type=int, default=None, help="Optional cap for quick real-data runs.")
    parser.add_argument("--max-waypoints-per-trajectory", type=int, default=None, help="Evenly downsample each trajectory before IK.")
    parser.add_argument("--num-workers", type=int, default=1, help="Parallel worker processes per robot.")
    parser.add_argument(
        "--base-pose-mode",
        choices=("fixed", "source"),
        default="fixed",
        help="Use fixed robot baselines or each trajectory's extracted source robot base pose.",
    )
    parser.add_argument("--seed", type=int, default=7)
    return parser


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
    config = EvaluationConfig(max_iters=args.max_iters, base_pose_mode=args.base_pose_mode)

    aggregate_rows = []
    suite_rows = []
    trajectory_rows = []
    for robot in baseline_robots():
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
    print(
        f"Evaluated {len(aggregate_rows)} baseline robots on {len(records)} trajectories "
        f"({sum(record.num_waypoints for record in records)} waypoints, workers={args.num_workers}, "
        f"base_pose_mode={args.base_pose_mode})"
    )


if __name__ == "__main__":
    main()
