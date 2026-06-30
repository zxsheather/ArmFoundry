from __future__ import annotations

import numpy as np
import pytest

from ddird.data.dataset import TrajectoryRecord, create_synthetic_dataset
from ddird.eval.evaluator import EvaluationConfig, evaluate_robot
from ddird.experiments.run_eval_baselines import _resolve_base_pose_mode
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


def test_source_base_pose_mode_uses_record_metadata(tmp_path):
    robot = create_robot("simple_default")
    source_base = np.array([0.4, -0.1, 0.2])
    target = robot.with_base(base_xyz=source_base).end_effector_position(robot.neutral_q)
    record = TrajectoryRecord(
        suite="suite",
        task="task",
        episode_id="episode",
        path=tmp_path / "episode.npz",
        ee_pos=target.reshape(1, 3),
        metadata={"source_robot_base_xyz": source_base.tolist(), "source_robot_base_yaw": 0.0},
    )

    result = evaluate_robot(robot, [record], config=EvaluationConfig(max_iters=1, base_pose_mode="source"))

    assert result.aggregate["ik_success_rate"] == 1.0


def test_source_base_pose_mode_falls_back_to_libero_suite_base(tmp_path):
    robot = create_robot("simple_default")
    source_base = np.array([-0.6, 0.0, 0.0])
    target = robot.with_base(base_xyz=source_base).end_effector_position(robot.neutral_q)
    record = TrajectoryRecord(
        suite="libero_object",
        task="task",
        episode_id="episode",
        path=tmp_path / "episode.npz",
        ee_pos=target.reshape(1, 3),
    )

    result = evaluate_robot(robot, [record], config=EvaluationConfig(max_iters=1, base_pose_mode="source"))

    assert result.aggregate["ik_success_rate"] == 1.0


def test_auto_base_pose_mode_selects_source_for_libero_world(tmp_path):
    record = TrajectoryRecord(
        suite="libero_goal",
        task="task",
        episode_id="episode",
        path=tmp_path / "episode.npz",
        ee_pos=np.zeros((1, 3)),
        metadata={"source": "LIBERO", "coordinate_frame": "world"},
    )

    assert _resolve_base_pose_mode("auto", [record]) == "source"


def test_auto_base_pose_mode_keeps_fixed_for_synthetic_data(tmp_path):
    record = TrajectoryRecord(
        suite="synthetic",
        task="task",
        episode_id="episode",
        path=tmp_path / "episode.npz",
        ee_pos=np.zeros((1, 3)),
        metadata={"source": "synthetic", "coordinate_frame": "robot"},
    )

    assert _resolve_base_pose_mode("auto", [record]) == "fixed"


def test_fixed_base_pose_mode_warns_for_libero_world(tmp_path):
    record = TrajectoryRecord(
        suite="libero_goal",
        task="task",
        episode_id="episode",
        path=tmp_path / "episode.npz",
        ee_pos=np.zeros((1, 3)),
        metadata={"source": "LIBERO", "coordinate_frame": "world"},
    )

    with pytest.warns(UserWarning, match="Using fixed base pose"):
        assert _resolve_base_pose_mode("fixed", [record]) == "fixed"
