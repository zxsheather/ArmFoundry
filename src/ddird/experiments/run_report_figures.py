from __future__ import annotations

import argparse
from pathlib import Path

from ddird.data.dataset import load_dataset
from ddird.eval.evaluator import EvaluationConfig, evaluate_robot
from ddird.experiments.common import DEFAULT_DATA_ROOT, DEFAULT_OUTPUT_ROOT, filter_records, read_json, write_csv
from ddird.robots.robot_registry import baseline_robots, create_robot
from ddird.viz.plots import (
    plot_ee_point_cloud,
    plot_ik_success_bar,
    plot_joint_margin_hist,
    plot_manipulability_box,
    plot_reachability,
    plot_robot_geometry,
)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Create report figures and markdown summary.")
    parser.add_argument("--data", default=DEFAULT_DATA_ROOT)
    parser.add_argument("--outputs", default=DEFAULT_OUTPUT_ROOT)
    parser.add_argument("--max-iters", type=int, default=80)
    parser.add_argument("--suite", default=None, help="Optional suite filter, e.g. libero_spatial.")
    parser.add_argument("--max-trajectories", type=int, default=None, help="Optional cap for quick real-data runs.")
    parser.add_argument("--max-waypoints-per-trajectory", type=int, default=None, help="Evenly downsample each trajectory before IK.")
    parser.add_argument("--num-workers", type=int, default=1, help="Parallel worker processes per robot.")
    parser.add_argument("--seed", type=int, default=7)
    return parser


def _optimized_robot(outputs: Path):
    best_path = outputs / "best_design.json"
    if not best_path.exists():
        return None
    best = read_json(best_path)
    payload = best["best_robot"]
    robot = create_robot("simple_default")
    return robot.with_design(
        base_xyz=payload["base_xyz"],
        base_yaw=payload["base_yaw"],
        link_lengths=payload["link_lengths"],
        name=payload["name"],
    )


def _write_report(
    outputs: Path,
    baseline_rows: list[dict],
    optimized_row: dict | None,
    figure_paths: list[Path],
    position_only: bool = True,
) -> Path:
    lines = [
        "# LIBERO DDIRD Prototype Report",
        "",
        "## Summary",
        "",
        "This prototype treats demonstration end-effector trajectories as a task-space requirement distribution and evaluates candidate robot-arm hardware with a fixed position-IK pipeline.",
        "",
        "LIBERO can validate the structure of a DDIRD prototype, but it is not real industrial operation data.",
        "",
        "We optimized candidate hardware for reachability, IK feasibility, and kinematic quality under a task-space trajectory distribution.",
        "",
    ]
    if position_only:
        lines.extend(
            [
                "## Simplification",
                "",
                "The current evaluator is position-only. Orientation and gripper traces are preserved when present, but orientation feasibility is not yet included in IK or manipulability scoring.",
                "",
            ]
        )

    lines.extend(
        [
            "## Baseline Comparison",
            "",
            "`*_proxy` rows are simplified serial-chain proxy models, not real commercial robot performance. `*_true` rows are explicitly loaded real kinematic models when available.",
            "",
        ]
    )
    lines.append("| Robot | IK success | Trajectory success | Mean joint margin | Mean manipulability | Hardware cost |")
    lines.append("|---|---:|---:|---:|---:|---:|")
    for row in baseline_rows:
        lines.append(
            f"| {row['robot_name']} | {float(row['ik_success_rate']):.3f} | {float(row['trajectory_success_rate']):.3f} | "
            f"{float(row['mean_joint_margin']):.3f} | {float(row['mean_manipulability']):.5f} | {float(row['hardware_cost']):.3f} |"
        )
    if optimized_row is not None:
        lines.append(
            f"| {optimized_row['robot_name']} | {float(optimized_row['ik_success_rate']):.3f} | {float(optimized_row['trajectory_success_rate']):.3f} | "
            f"{float(optimized_row['mean_joint_margin']):.3f} | {float(optimized_row['mean_manipulability']):.5f} | {float(optimized_row['hardware_cost']):.3f} |"
        )
    lines.extend(
        [
            "",
            "## Viability Assessment",
            "",
            "The prototype is viable if optimized hardware improves held-out IK success, joint-limit margin, or manipulability relative to the initial simple model while staying clear about whether a row is a proxy or a true kinematic model.",
            "",
            "Failure cases concentrate where target points fall outside the arm's reachable shell, near table/workspace boundaries, or around low-manipulability postures.",
            "",
            "For real DDIRD, the next data needed is factory operation logs with task trajectories, failures, cycle-time pressure, torque/stress traces, maintenance events, and real mounting constraints.",
            "",
            "## Figures",
            "",
        ]
    )
    for path in figure_paths:
        rel = path.relative_to(outputs)
        lines.append(f"- [{rel}]({rel})")
    report_path = outputs / "report.md"
    report_path.write_text("\n".join(lines).rstrip() + "\n", encoding="utf-8")
    return report_path


def main() -> None:
    args = build_parser().parse_args()
    outputs = Path(args.outputs)
    figures = outputs / "figures"
    figures.mkdir(parents=True, exist_ok=True)
    records = filter_records(
        load_dataset(args.data),
        suite=args.suite,
        max_trajectories=args.max_trajectories,
        max_waypoints_per_trajectory=args.max_waypoints_per_trajectory,
        seed=args.seed,
    )
    if not records:
        raise SystemExit("No trajectories selected for report generation")
    config = EvaluationConfig(max_iters=args.max_iters)

    robots = baseline_robots()
    optimized = _optimized_robot(outputs)
    if optimized is not None:
        robots.append(optimized)

    results = [evaluate_robot(robot, records, config=config, return_details=True, num_workers=args.num_workers) for robot in robots]
    aggregate_rows = [result.aggregate for result in results]
    write_csv(outputs / "report_results.csv", aggregate_rows)

    figure_paths = [
        plot_ee_point_cloud(records, figures / "ee_point_cloud.png"),
        plot_reachability(results[-1], figures / "reachable_unreachable_points.png"),
        plot_ik_success_bar(aggregate_rows, figures / "ik_success_by_robot.png"),
        plot_joint_margin_hist(results, figures / "joint_margin_histogram.png"),
        plot_manipulability_box(results, figures / "manipulability_boxplot.png"),
    ]
    if optimized is not None:
        figure_paths.append(plot_robot_geometry(create_robot("simple_default"), optimized, figures / "robot_geometry_before_after.png"))

    optimized_row = results[-1].aggregate if optimized is not None else None
    baseline_rows = [result.aggregate for result in results if result.robot.source != "parameterized" or result.robot.name == "simple_default"]
    report_path = _write_report(outputs, baseline_rows, optimized_row, figure_paths)
    print(f"Wrote report to {report_path}")


if __name__ == "__main__":
    main()
