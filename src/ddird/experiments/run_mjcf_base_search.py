from __future__ import annotations

import argparse
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Sequence

import numpy as np

from ddird.data.dataset import TrajectoryRecord, load_dataset, split_trajectories
from ddird.eval.evaluator import EvaluationConfig, _record_source_base, evaluate_robot
from ddird.eval.orientation import ORIENTATION_FORMATS
from ddird.eval.tool_frame_mapping import TOOL_FRAME_MAPPING_MODES, tool_frame_mapping_metadata
from ddird.experiments.common import DEFAULT_DATA_ROOT, DEFAULT_OUTPUT_ROOT, filter_records, write_csv, write_json
from ddird.robots.mjcf_chain import MJCFSerialRobot, serial_robot_from_mjcf_xml
from ddird.robots.tool_frames import tool_frame_metadata


@dataclass(frozen=True)
class BaseSearchBounds:
    dx: float = 0.35
    dy: float = 0.35
    dz: float = 0.20
    yaw: float = 0.70
    min_z: float = 0.0


@dataclass(frozen=True)
class BaseCandidate:
    group: str
    candidate_index: int
    candidate_kind: str
    reference_xyz: np.ndarray
    reference_yaw: float
    base_xyz: np.ndarray
    base_yaw: float

    def row(self) -> dict[str, Any]:
        return {
            "group": self.group,
            "candidate_index": self.candidate_index,
            "candidate_kind": self.candidate_kind,
            "reference_base_x": round(float(self.reference_xyz[0]), 6),
            "reference_base_y": round(float(self.reference_xyz[1]), 6),
            "reference_base_z": round(float(self.reference_xyz[2]), 6),
            "reference_base_yaw": round(float(self.reference_yaw), 6),
            "base_x": round(float(self.base_xyz[0]), 6),
            "base_y": round(float(self.base_xyz[1]), 6),
            "base_z": round(float(self.base_xyz[2]), 6),
            "base_yaw": round(float(self.base_yaw), 6),
            "dx_from_reference": round(float(self.base_xyz[0] - self.reference_xyz[0]), 6),
            "dy_from_reference": round(float(self.base_xyz[1] - self.reference_xyz[1]), 6),
            "dz_from_reference": round(float(self.base_xyz[2] - self.reference_xyz[2]), 6),
            "yaw_delta_from_reference": round(float(self.base_yaw - self.reference_yaw), 6),
        }


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Search MJCF robot base placements with train/test validation.")
    parser.add_argument("--mjcf", required=True, help="Path to a MuJoCo XML model containing a serial robot chain.")
    parser.add_argument("--robot-name", required=True, help="Name to use in output CSVs.")
    parser.add_argument("--base-body", required=True, help="Body name for the robot base in the MJCF.")
    parser.add_argument("--target-site", default="tool0", help="Preferred target site / TCP name.")
    parser.add_argument("--target-body", default="tool0", help="Fallback target body / TCP name.")
    parser.add_argument("--data", default=DEFAULT_DATA_ROOT)
    parser.add_argument("--outputs", default=DEFAULT_OUTPUT_ROOT)
    parser.add_argument("--suite", default=None, help="Optional suite filter before grouping.")
    parser.add_argument("--search-scope", choices=("suite", "global"), default="suite")
    parser.add_argument("--split-by", choices=("task", "episode"), default="task")
    parser.add_argument("--train-fraction", type=float, default=0.7)
    parser.add_argument("--num-candidates", type=int, default=24, help="Random candidates per group; reference base is added separately.")
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument("--max-iters", type=int, default=80)
    parser.add_argument("--max-trajectories", type=int, default=None, help="Optional cap before train/test splitting.")
    parser.add_argument(
        "--max-search-trajectories",
        type=int,
        default=None,
        help="Optional cap on train trajectories used to rank candidate bases.",
    )
    parser.add_argument(
        "--max-eval-trajectories",
        type=int,
        default=None,
        help="Optional cap on train/test trajectories used for final validation.",
    )
    parser.add_argument("--max-waypoints-per-trajectory", type=int, default=None)
    parser.add_argument("--num-workers", type=int, default=1)
    parser.add_argument("--evaluation-mode", choices=("position", "pose"), default="position")
    parser.add_argument("--orientation-tolerance", type=float, default=0.10)
    parser.add_argument("--orientation-format", choices=ORIENTATION_FORMATS, default="auto")
    parser.add_argument("--orientation-weight", type=float, default=1.0)
    parser.add_argument("--tool-frame-mapping", choices=TOOL_FRAME_MAPPING_MODES, default="identity")
    parser.add_argument("--dx-bound", type=float, default=0.35)
    parser.add_argument("--dy-bound", type=float, default=0.35)
    parser.add_argument("--dz-bound", type=float, default=0.20)
    parser.add_argument("--yaw-bound", type=float, default=0.70)
    parser.add_argument("--min-base-z", type=float, default=0.0)
    return parser


def _group_records(records: list[TrajectoryRecord], search_scope: str) -> dict[str, list[TrajectoryRecord]]:
    if search_scope == "global":
        return {"global": records}
    if search_scope != "suite":
        raise ValueError("search_scope must be 'suite' or 'global'")
    grouped: dict[str, list[TrajectoryRecord]] = {}
    for record in records:
        grouped.setdefault(record.suite, []).append(record)
    return dict(sorted(grouped.items()))


def _reference_base(records: list[TrajectoryRecord]) -> tuple[np.ndarray, float]:
    bases = [_record_source_base(record) for record in records]
    xyz = np.mean(np.asarray([base_xyz for base_xyz, _ in bases], dtype=float), axis=0)
    yaws = np.asarray([yaw for _, yaw in bases], dtype=float)
    yaw = float(np.arctan2(np.mean(np.sin(yaws)), np.mean(np.cos(yaws))))
    return xyz, yaw


def _sample_candidates(
    group: str,
    records: list[TrajectoryRecord],
    num_candidates: int,
    seed: int,
    bounds: BaseSearchBounds,
) -> list[BaseCandidate]:
    reference_xyz, reference_yaw = _reference_base(records)
    candidates = [
        BaseCandidate(
            group=group,
            candidate_index=0,
            candidate_kind="reference_base",
            reference_xyz=reference_xyz,
            reference_yaw=reference_yaw,
            base_xyz=reference_xyz,
            base_yaw=reference_yaw,
        )
    ]
    rng = np.random.default_rng(seed)
    for index in range(1, num_candidates + 1):
        delta = np.array(
            [
                float(rng.uniform(-bounds.dx, bounds.dx)),
                float(rng.uniform(-bounds.dy, bounds.dy)),
                float(rng.uniform(-bounds.dz, bounds.dz)),
            ],
            dtype=float,
        )
        base_xyz = reference_xyz + delta
        base_xyz[2] = max(bounds.min_z, float(base_xyz[2]))
        base_yaw = reference_yaw + float(rng.uniform(-bounds.yaw, bounds.yaw))
        candidates.append(
            BaseCandidate(
                group=group,
                candidate_index=index,
                candidate_kind=f"random_{index - 1:02d}",
                reference_xyz=reference_xyz,
                reference_yaw=reference_yaw,
                base_xyz=base_xyz,
                base_yaw=base_yaw,
            )
        )
    return candidates


def _rank_key(row: dict[str, Any]) -> tuple[float, float, float, float]:
    trajectory_key = "trajectory_pose_success_rate" if "trajectory_pose_success_rate" in row else "trajectory_success_rate"
    waypoint_key = "pose_success_rate" if "pose_success_rate" in row else "ik_success_rate"
    orientation_error = float(row.get("mean_orientation_error_rad", 0.0) or 0.0)
    return (
        float(row[trajectory_key]),
        float(row[waypoint_key]),
        -float(row["mean_position_error"]),
        -orientation_error,
    )


def _downsample_records(
    records: list[TrajectoryRecord],
    max_trajectories: int | None,
    max_waypoints_per_trajectory: int | None,
    seed: int,
) -> list[TrajectoryRecord]:
    return filter_records(
        records,
        max_trajectories=max_trajectories,
        max_waypoints_per_trajectory=max_waypoints_per_trajectory,
        seed=seed,
    )


def _evaluate_candidate(
    base_robot: MJCFSerialRobot,
    candidate: BaseCandidate,
    records: list[TrajectoryRecord],
    config: EvaluationConfig,
    num_workers: int,
    stage: str,
    split: str,
) -> dict[str, Any]:
    robot = base_robot.with_base(base_xyz=candidate.base_xyz, base_yaw=candidate.base_yaw)
    result = evaluate_robot(robot, records, config=config, num_workers=num_workers)
    row = dict(result.aggregate)
    row.update(candidate.row())
    row["stage"] = stage
    row["split"] = split
    return row


def _write_report(
    outputs: Path,
    *,
    search_scope: str,
    split_by: str,
    train_fraction: float,
    best_rows: list[dict[str, Any]],
    comparison_rows: list[dict[str, Any]],
) -> Path:
    lines = [
        "# MJCF Base Search Train/Test Report",
        "",
        "## Summary",
        "",
        (
            f"This run searched `{search_scope}` base placements with a `{split_by}` split "
            f"and train fraction `{train_fraction}`."
        ),
        "",
        "Best train-selected bases:",
        "",
        "| Group | Base xyz | Base yaw | Train traj | Test traj | Train pose | Test pose |",
        "| --- | --- | ---: | ---: | ---: | ---: | ---: |",
    ]
    by_group_split = {(row["group"], row["split"]): row for row in comparison_rows if row["base_label"] == "best_train_base"}
    for best in best_rows:
        group = str(best["group"])
        train = by_group_split[(group, "train")]
        test = by_group_split[(group, "test")]
        train_traj = train.get("trajectory_pose_success_rate", train.get("trajectory_success_rate"))
        test_traj = test.get("trajectory_pose_success_rate", test.get("trajectory_success_rate"))
        train_pose = train.get("pose_success_rate", train.get("ik_success_rate"))
        test_pose = test.get("pose_success_rate", test.get("ik_success_rate"))
        lines.append(
            "| "
            f"`{group}` | "
            f"`({float(best['base_x']):.6f}, {float(best['base_y']):.6f}, {float(best['base_z']):.6f})` | "
            f"{float(best['base_yaw']):.6f} | "
            f"{float(train_traj):.6f} | {float(test_traj):.6f} | "
            f"{float(train_pose):.6f} | {float(test_pose):.6f} |"
        )
    lines.extend(
        [
            "",
            "## Caveats",
            "",
            "- The search is stochastic and local around each group's reference base.",
            "- The selected base is optimized on the train split; test rows estimate generalization.",
            "- This remains a kinematic IK benchmark, not a full manipulation simulation.",
            "",
            "## Artifacts",
            "",
            "- `search_candidates.csv`",
            "- `best_train_base.csv`",
            "- `train_test_results.csv`",
            "- `train_test_comparison.csv`",
            "- `train_test_aggregate_comparison.csv`",
            "- `metadata.json`",
        ]
    )
    report_path = outputs / "analysis_report.md"
    report_path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return report_path


def _aggregate_validation_rows(rows: list[dict[str, Any]]) -> list[dict[str, Any]]:
    aggregate_rows = []
    for split in sorted({str(row["split"]) for row in rows}):
        for label in sorted({str(row["base_label"]) for row in rows}):
            selected = [row for row in rows if row["split"] == split and row["base_label"] == label]
            if not selected:
                continue
            trajectories = sum(int(row["num_trajectories"]) for row in selected)
            waypoints = sum(int(row["num_waypoints"]) for row in selected)

            def waypoint_mean(key: str) -> float:
                return sum(float(row[key]) * int(row["num_waypoints"]) for row in selected) / waypoints

            def trajectory_mean(key: str) -> float:
                return sum(float(row[key]) * int(row["num_trajectories"]) for row in selected) / trajectories

            row: dict[str, Any] = {
                "base_label": label,
                "split": split,
                "num_trajectories": trajectories,
                "num_waypoints": waypoints,
                "ik_success_rate": round(waypoint_mean("ik_success_rate"), 6),
                "trajectory_success_rate": round(trajectory_mean("trajectory_success_rate"), 6),
                "mean_position_error": round(waypoint_mean("mean_position_error"), 6),
                "max_position_error": round(max(float(item["max_position_error"]) for item in selected), 6),
            }
            if "pose_success_rate" in selected[0]:
                row.update(
                    {
                        "pose_success_rate": round(waypoint_mean("pose_success_rate"), 6),
                        "trajectory_pose_success_rate": round(trajectory_mean("trajectory_pose_success_rate"), 6),
                        "mean_orientation_error_rad": round(waypoint_mean("mean_orientation_error_rad"), 6),
                        "max_orientation_error_rad": round(
                            max(float(item["max_orientation_error_rad"]) for item in selected),
                            6,
                        ),
                    }
                )
            aggregate_rows.append(row)
    return aggregate_rows


def run_base_search(args: argparse.Namespace) -> dict[str, Any]:
    outputs = Path(args.outputs)
    base_robot = serial_robot_from_mjcf_xml(
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
        max_waypoints_per_trajectory=None,
        seed=args.seed,
    )
    if not records:
        raise SystemExit("No trajectories selected for base search")

    config = EvaluationConfig(
        max_iters=args.max_iters,
        base_pose_mode="fixed",
        evaluation_mode=args.evaluation_mode,
        orientation_tolerance=args.orientation_tolerance,
        orientation_format=args.orientation_format,
        orientation_weight=args.orientation_weight,
        tool_frame_mapping=args.tool_frame_mapping,
    )
    bounds = BaseSearchBounds(
        dx=args.dx_bound,
        dy=args.dy_bound,
        dz=args.dz_bound,
        yaw=args.yaw_bound,
        min_z=args.min_base_z,
    )

    search_rows: list[dict[str, Any]] = []
    best_rows: list[dict[str, Any]] = []
    comparison_rows: list[dict[str, Any]] = []
    split_metadata: dict[str, Any] = {}

    for group_index, (group, group_records) in enumerate(_group_records(records, args.search_scope).items()):
        if len(group_records) < 2:
            raise SystemExit(f"Group {group!r} has fewer than two trajectories")
        train_records, test_records = split_trajectories(
            group_records,
            train_fraction=args.train_fraction,
            split_by=args.split_by,
            seed=args.seed + group_index,
        )
        if not test_records:
            raise SystemExit(f"Group {group!r} produced an empty test split")
        split_metadata[group] = {
            "num_group_trajectories": len(group_records),
            "num_train_trajectories": len(train_records),
            "num_test_trajectories": len(test_records),
            "train_tasks": sorted({record.task for record in train_records}),
            "test_tasks": sorted({record.task for record in test_records}),
        }
        search_records = _downsample_records(
            train_records,
            max_trajectories=args.max_search_trajectories,
            max_waypoints_per_trajectory=args.max_waypoints_per_trajectory,
            seed=args.seed + group_index,
        )
        train_eval_records = _downsample_records(
            train_records,
            max_trajectories=args.max_eval_trajectories,
            max_waypoints_per_trajectory=args.max_waypoints_per_trajectory,
            seed=args.seed + group_index,
        )
        test_eval_records = _downsample_records(
            test_records,
            max_trajectories=args.max_eval_trajectories,
            max_waypoints_per_trajectory=args.max_waypoints_per_trajectory,
            seed=args.seed + group_index + 1000,
        )
        candidates = _sample_candidates(
            group=group,
            records=group_records,
            num_candidates=args.num_candidates,
            seed=args.seed + group_index,
            bounds=bounds,
        )
        group_search_rows = []
        print(
            f"[{group}] train={len(train_records)} test={len(test_records)} "
            f"search={len(search_records)} candidates={len(candidates)}",
            flush=True,
        )
        for candidate in candidates:
            row = _evaluate_candidate(
                base_robot,
                candidate,
                search_records,
                config=config,
                num_workers=args.num_workers,
                stage="search",
                split="train_search",
            )
            group_search_rows.append(row)
            search_rows.append(row)
            trajectory_key = "trajectory_pose_success_rate" if "trajectory_pose_success_rate" in row else "trajectory_success_rate"
            waypoint_key = "pose_success_rate" if "pose_success_rate" in row else "ik_success_rate"
            print(
                f"[{group}] candidate={candidate.candidate_index:02d} "
                f"traj={row[trajectory_key]} waypoint={row[waypoint_key]}",
                flush=True,
            )
        best_search_row = max(group_search_rows, key=_rank_key)
        best_candidate = candidates[int(best_search_row["candidate_index"])]
        best_row = dict(best_search_row)
        best_row["stage"] = "best_train_base"
        best_rows.append(best_row)

        reference_candidate = candidates[0]
        for label, candidate in [("reference_base", reference_candidate), ("best_train_base", best_candidate)]:
            for split_name, split_records in [("train", train_eval_records), ("test", test_eval_records)]:
                row = _evaluate_candidate(
                    base_robot,
                    candidate,
                    split_records,
                    config=config,
                    num_workers=args.num_workers,
                    stage="validation",
                    split=split_name,
                )
                row["base_label"] = label
                comparison_rows.append(row)
                trajectory_key = "trajectory_pose_success_rate" if "trajectory_pose_success_rate" in row else "trajectory_success_rate"
                print(f"[{group}] {label} {split_name} traj={row[trajectory_key]}", flush=True)

    write_csv(outputs / "search_candidates.csv", search_rows)
    write_csv(outputs / "best_train_base.csv", best_rows)
    write_csv(outputs / "train_test_results.csv", comparison_rows)

    comparison_summary = []
    for group in sorted({row["group"] for row in comparison_rows}):
        group_rows = [row for row in comparison_rows if row["group"] == group]
        by_label_split = {(row["base_label"], row["split"]): row for row in group_rows}
        reference_test = by_label_split[("reference_base", "test")]
        best_test = by_label_split[("best_train_base", "test")]
        trajectory_key = "trajectory_pose_success_rate" if "trajectory_pose_success_rate" in best_test else "trajectory_success_rate"
        waypoint_key = "pose_success_rate" if "pose_success_rate" in best_test else "ik_success_rate"
        comparison_summary.append(
            {
                "group": group,
                "reference_test_trajectory_success": reference_test[trajectory_key],
                "best_test_trajectory_success": best_test[trajectory_key],
                "test_trajectory_success_delta": round(float(best_test[trajectory_key]) - float(reference_test[trajectory_key]), 6),
                "reference_test_waypoint_success": reference_test[waypoint_key],
                "best_test_waypoint_success": best_test[waypoint_key],
                "test_waypoint_success_delta": round(float(best_test[waypoint_key]) - float(reference_test[waypoint_key]), 6),
                "best_base_x": best_test["base_x"],
                "best_base_y": best_test["base_y"],
                "best_base_z": best_test["base_z"],
                "best_base_yaw": best_test["base_yaw"],
            }
        )
    write_csv(outputs / "train_test_comparison.csv", comparison_summary)
    aggregate_rows = _aggregate_validation_rows(comparison_rows)
    write_csv(outputs / "train_test_aggregate_comparison.csv", aggregate_rows)

    metadata = {
        "experiment": "mjcf_base_search",
        "mjcf": args.mjcf,
        "robot_name": args.robot_name,
        "base_body": args.base_body,
        "target_site": args.target_site,
        "target_body": args.target_body,
        "search_scope": args.search_scope,
        "split_by": args.split_by,
        "train_fraction": args.train_fraction,
        "seed": args.seed,
        "num_candidates": args.num_candidates,
        "max_search_trajectories": args.max_search_trajectories,
        "max_eval_trajectories": args.max_eval_trajectories,
        "max_waypoints_per_trajectory": args.max_waypoints_per_trajectory,
        "num_workers": args.num_workers,
        "evaluation_config": {
            "max_iters": args.max_iters,
            "base_pose_mode": "fixed",
            "evaluation_mode": args.evaluation_mode,
            "orientation_format": args.orientation_format,
            "orientation_tolerance_rad": args.orientation_tolerance,
            "orientation_weight": args.orientation_weight,
            "tool_frame_mapping": args.tool_frame_mapping,
        },
        "bounds": {
            "dx": bounds.dx,
            "dy": bounds.dy,
            "dz": bounds.dz,
            "yaw": bounds.yaw,
            "min_z": bounds.min_z,
        },
        "split": split_metadata,
        "tool_frame": tool_frame_metadata(
            base_robot,
            base_body=args.base_body,
            target_site=args.target_site,
            target_body=args.target_body,
            model_path=args.mjcf,
        ),
        "tool_frame_mapping_metadata": tool_frame_mapping_metadata(base_robot, args.tool_frame_mapping),
    }
    write_json(outputs / "metadata.json", metadata)
    _write_report(
        outputs,
        search_scope=args.search_scope,
        split_by=args.split_by,
        train_fraction=args.train_fraction,
        best_rows=best_rows,
        comparison_rows=comparison_rows,
    )
    return {
        "search_rows": search_rows,
        "best_rows": best_rows,
        "comparison_rows": comparison_rows,
        "comparison_summary": comparison_summary,
        "aggregate_rows": aggregate_rows,
        "metadata": metadata,
    }


def main(argv: Sequence[str] | None = None) -> None:
    args = build_parser().parse_args(argv)
    result = run_base_search(args)
    print(
        f"Wrote MJCF base search for {len(result['best_rows'])} group(s) "
        f"and {len(result['search_rows'])} candidate evaluations"
    )


if __name__ == "__main__":
    main()
