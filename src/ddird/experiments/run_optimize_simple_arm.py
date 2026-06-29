from __future__ import annotations

import argparse

from ddird.data.dataset import load_dataset, split_trajectories
from ddird.eval.evaluator import EvaluationConfig, evaluate_robot
from ddird.experiments.common import DEFAULT_DATA_ROOT, DEFAULT_OUTPUT_ROOT, filter_records, write_csv, write_json
from ddird.optim.search import random_search
from ddird.robots.robot_registry import create_robot


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Optimize simple-arm base pose and link lengths.")
    parser.add_argument("--data", default=DEFAULT_DATA_ROOT)
    parser.add_argument("--outputs", default=DEFAULT_OUTPUT_ROOT)
    parser.add_argument("--iterations", type=int, default=14, help="Random candidates per optimization stage.")
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument("--max-iters", type=int, default=80)
    parser.add_argument("--train-fraction", type=float, default=0.7)
    parser.add_argument("--split-by", choices=["episode", "task"], default="episode")
    parser.add_argument("--suite", default=None, help="Optional suite filter, e.g. libero_spatial.")
    parser.add_argument("--max-trajectories", type=int, default=None, help="Optional cap for quick real-data runs.")
    parser.add_argument("--max-waypoints-per-trajectory", type=int, default=None, help="Evenly downsample each trajectory before IK.")
    parser.add_argument("--num-workers", type=int, default=1, help="Parallel worker processes per candidate. Use conservatively during search.")
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
    if len(records) < 2:
        raise SystemExit("At least two trajectories are required for train/test optimization")
    train, test = split_trajectories(records, train_fraction=args.train_fraction, split_by=args.split_by, seed=args.seed)
    config = EvaluationConfig(max_iters=args.max_iters)
    base_robot = create_robot("simple_default")

    base_search = random_search(
        base_robot,
        train,
        iterations=args.iterations,
        optimize_links=False,
        seed=args.seed,
        config=config,
        num_workers=args.num_workers,
    )
    link_search = random_search(
        base_search.best_robot,
        train,
        iterations=args.iterations,
        optimize_links=True,
        seed=args.seed + 1,
        config=config,
        num_workers=args.num_workers,
    )

    all_rows = base_search.rows + link_search.rows
    write_csv(f"{args.outputs}/optimization_results.csv", all_rows)

    initial_test = evaluate_robot(base_robot, test, config=config, num_workers=args.num_workers)
    best_test = evaluate_robot(link_search.best_robot, test, config=config, num_workers=args.num_workers)
    best_design = {
        "split": {
            "split_by": args.split_by,
            "train_fraction": args.train_fraction,
            "num_train_trajectories": len(train),
            "num_test_trajectories": len(test),
            "train_waypoints": sum(record.num_waypoints for record in train),
            "test_waypoints": sum(record.num_waypoints for record in test),
            "max_waypoints_per_trajectory": args.max_waypoints_per_trajectory,
            "num_workers": args.num_workers,
        },
        "best_train_objective_loss": round(float(link_search.best_loss), 8),
        "best_robot": link_search.best_robot.to_dict(),
        "initial_test_metrics": initial_test.aggregate,
        "best_test_metrics": best_test.aggregate,
        "position_only_evaluation": True,
        "orientation_note": "Orientation is stored when present but not optimized in this first-stage IK evaluator.",
    }
    write_json(f"{args.outputs}/best_design.json", best_design)
    write_csv(f"{args.outputs}/optimization_test_comparison.csv", [initial_test.aggregate, best_test.aggregate])
    print(f"Wrote optimization results for {len(all_rows)} candidates")
    print(f"Best design: {link_search.best_robot.to_dict()}")


if __name__ == "__main__":
    main()
