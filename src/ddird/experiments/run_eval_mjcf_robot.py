from __future__ import annotations

import argparse
from pathlib import Path

from ddird.data.dataset import load_dataset
from ddird.eval.evaluator import EvaluationConfig, evaluate_robot
from ddird.eval.metrics import aggregate_metric_dicts
from ddird.eval.orientation import ORIENTATION_FORMATS
from ddird.eval.tool_frame_mapping import TOOL_FRAME_MAPPING_MODES, tool_frame_mapping_metadata
from ddird.experiments.common import (
    DEFAULT_DATA_ROOT,
    DEFAULT_OUTPUT_ROOT,
    filter_records,
    write_csv,
    write_json,
)
from ddird.robots.mjcf_chain import serial_robot_from_mjcf_xml
from ddird.robots.tool_frames import tool_frame_metadata


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Evaluate one external MJCF serial robot on processed EE trajectories.")
    parser.add_argument("--mjcf", required=True, help="Path to a MuJoCo XML model containing a serial robot chain.")
    parser.add_argument("--robot-name", required=True, help="Name to use in output CSVs, e.g. ur5_true or xarm6_true.")
    parser.add_argument("--base-body", required=True, help="Body name for the robot base in the MJCF.")
    parser.add_argument("--target-site", default="tool0", help="Preferred target site / TCP name.")
    parser.add_argument("--target-body", default="tool0", help="Fallback target body / TCP name.")
    parser.add_argument("--data", default=DEFAULT_DATA_ROOT)
    parser.add_argument("--outputs", default=DEFAULT_OUTPUT_ROOT)
    parser.add_argument("--max-iters", type=int, default=80)
    parser.add_argument(
        "--evaluation-mode",
        choices=("position", "pose"),
        default="position",
        help="Use the current position-only IK path or pose-aware IK with position and orientation constraints.",
    )
    parser.add_argument(
        "--orientation-tolerance",
        type=float,
        default=0.10,
        help="Pose-aware orientation tolerance in radians.",
    )
    parser.add_argument(
        "--orientation-format",
        choices=ORIENTATION_FORMATS,
        default="auto",
        help="How to decode trajectory orientation arrays in pose-aware mode.",
    )
    parser.add_argument(
        "--orientation-weight",
        type=float,
        default=1.0,
        help="Weight applied to angular error inside the pose IK solve.",
    )
    parser.add_argument(
        "--tool-frame-mapping",
        choices=TOOL_FRAME_MAPPING_MODES,
        default="identity",
        help=(
            "How to map source orientation frames to the target robot TCP. "
            "Use canonical_tool only for robots with an explicit canonical mapping."
        ),
    )
    parser.add_argument("--suite", default=None)
    parser.add_argument("--max-trajectories", type=int, default=None)
    parser.add_argument("--max-waypoints-per-trajectory", type=int, default=None)
    parser.add_argument("--num-workers", type=int, default=1)
    parser.add_argument(
        "--base-pose-mode",
        choices=("fixed", "source"),
        default="source",
        help=(
            "Use fixed MJCF base pose or place the external robot at each LIBERO source workcell base pose. "
            "For cross-robot LIBERO reachability comparisons, source is the documented default."
        ),
    )
    parser.add_argument("--seed", type=int, default=7)
    return parser


def main() -> None:
    args = build_parser().parse_args()
    robot = serial_robot_from_mjcf_xml(
        Path(args.mjcf).read_text(encoding="utf-8"),
        name=args.robot_name,
        base_body_name=args.base_body,
        target_site=args.target_site,
        target_body=args.target_body,
    )
    records = filter_records(
        load_dataset(args.data),
        suite=args.suite,
        max_trajectories=args.max_trajectories,
        max_waypoints_per_trajectory=args.max_waypoints_per_trajectory,
        seed=args.seed,
    )
    if not records:
        raise SystemExit("No trajectories selected for evaluation")

    result = evaluate_robot(
        robot,
        records,
        config=EvaluationConfig(
            max_iters=args.max_iters,
            base_pose_mode=args.base_pose_mode,
            evaluation_mode=args.evaluation_mode,
            orientation_tolerance=args.orientation_tolerance,
            orientation_format=args.orientation_format,
            orientation_weight=args.orientation_weight,
            tool_frame_mapping=args.tool_frame_mapping,
        ),
        num_workers=args.num_workers,
    )
    suite_rows = []
    for suite in sorted({row["suite"] for row in result.trajectory_rows}):
        aggregate = aggregate_metric_dicts(
            [row for row in result.trajectory_rows if row["suite"] == suite],
            group_name=robot.name,
        )
        aggregate["suite"] = suite
        suite_rows.append(aggregate)

    outputs = Path(args.outputs)
    write_csv(outputs / "baseline_results.csv", [result.aggregate])
    write_csv(outputs / "baseline_results_by_suite.csv", suite_rows)
    write_csv(outputs / "baseline_trajectory_results.csv", result.trajectory_rows)
    write_json(
        outputs / "tool_frame_metadata.json",
        {
            "base_pose_mode": args.base_pose_mode,
            "evaluation_mode": args.evaluation_mode,
            "max_iters": args.max_iters,
            "orientation_format": args.orientation_format,
            "orientation_tolerance_rad": args.orientation_tolerance,
            "orientation_weight": args.orientation_weight,
            "tool_frame_mapping": args.tool_frame_mapping,
            "robots": [
                tool_frame_metadata(
                    robot,
                    base_body=args.base_body,
                    target_site=args.target_site,
                    target_body=args.target_body,
                    model_path=args.mjcf,
                )
            ],
            "tool_frame_mappings": [tool_frame_mapping_metadata(robot, args.tool_frame_mapping)],
        },
    )
    print(
        f"Evaluated {robot.name} on {len(records)} trajectories "
        f"({sum(record.num_waypoints for record in records)} waypoints, workers={args.num_workers}, "
        f"base_pose_mode={args.base_pose_mode}, evaluation_mode={args.evaluation_mode})"
    )


if __name__ == "__main__":
    main()
