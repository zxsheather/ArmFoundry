from __future__ import annotations

from ddird.data.dataset import create_synthetic_dataset
from ddird.eval.evaluator import EvaluationConfig, evaluate_robot
from ddird.robots.robot_registry import create_robot


def test_evaluator_returns_aggregate_metrics(tmp_path):
    root = tmp_path / "data"
    records = create_synthetic_dataset(root, episodes_per_task=1, timesteps=6, seed=2)
    robot = create_robot("simple_default")

    result = evaluate_robot(robot, records[:1], config=EvaluationConfig(max_iters=30))

    assert result.aggregate["robot_name"] == "simple_default"
    assert result.aggregate["num_trajectories"] == 1
    assert 0.0 <= result.aggregate["ik_success_rate"] <= 1.0


def test_parallel_evaluator_matches_sequential(tmp_path):
    root = tmp_path / "data"
    records = create_synthetic_dataset(root, episodes_per_task=1, timesteps=5, seed=3)[:2]
    robot = create_robot("simple_default")
    config = EvaluationConfig(max_iters=20)

    sequential = evaluate_robot(robot, records, config=config, num_workers=1)
    parallel = evaluate_robot(robot, records, config=config, num_workers=2)

    assert parallel.aggregate == sequential.aggregate
    assert parallel.trajectory_rows == sequential.trajectory_rows
