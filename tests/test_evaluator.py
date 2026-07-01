from __future__ import annotations

import numpy as np
import pytest

from ddird.data.dataset import TrajectoryRecord, create_synthetic_dataset
from ddird.eval.evaluator import EvaluationConfig, evaluate_robot
from ddird.eval.orientation import matrix_to_rotvec, rotvec_to_matrix
from ddird.experiments.run_eval_baselines import _resolve_base_pose_mode, build_parser
from ddird.experiments.run_eval_mjcf_robot import build_parser as build_mjcf_parser
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


def test_pose_evaluator_reports_orientation_metrics_for_successful_pose(tmp_path):
    robot = create_robot("simple_default")
    q_target = robot.neutral_q
    fk = robot.forward_kinematics(q_target)
    record = TrajectoryRecord(
        suite="suite",
        task="task",
        episode_id="episode",
        path=tmp_path / "episode.npz",
        ee_pos=fk["position"].reshape(1, 3),
        ee_ori=matrix_to_rotvec(fk["rotation"]).reshape(1, 3),
    )

    result = evaluate_robot(
        robot,
        [record],
        config=EvaluationConfig(
            max_iters=1,
            evaluation_mode="pose",
            orientation_format="rotvec",
            orientation_tolerance=0.01,
        ),
    )

    assert result.aggregate["ik_success_rate"] == 1.0
    assert result.aggregate["pose_success_rate"] == 1.0
    assert result.aggregate["trajectory_pose_success_rate"] == 1.0
    assert result.aggregate["mean_orientation_error_rad"] == 0.0


def test_pose_evaluator_marks_orientation_failure(tmp_path):
    robot = create_robot("simple_default")
    q_target = robot.neutral_q
    fk = robot.forward_kinematics(q_target)
    target_rotation = fk["rotation"] @ rotvec_to_matrix(np.array([0.0, 0.0, np.pi / 2.0]))
    record = TrajectoryRecord(
        suite="suite",
        task="task",
        episode_id="episode",
        path=tmp_path / "episode.npz",
        ee_pos=fk["position"].reshape(1, 3),
        ee_ori=matrix_to_rotvec(target_rotation).reshape(1, 3),
    )

    result = evaluate_robot(
        robot,
        [record],
        config=EvaluationConfig(
            max_iters=1,
            evaluation_mode="pose",
            orientation_format="rotvec",
            orientation_tolerance=0.01,
        ),
    )

    assert result.aggregate["ik_success_rate"] == 0.0
    assert result.aggregate["pose_success_rate"] == 0.0
    assert result.aggregate["trajectory_pose_success_rate"] == 0.0
    assert result.aggregate["mean_orientation_error_rad"] > 0.01


def test_position_only_evaluator_keeps_pose_metrics_out_of_default_rows(tmp_path):
    robot = create_robot("simple_default")
    q_target = robot.neutral_q
    fk = robot.forward_kinematics(q_target)
    record = TrajectoryRecord(
        suite="suite",
        task="task",
        episode_id="episode",
        path=tmp_path / "episode.npz",
        ee_pos=fk["position"].reshape(1, 3),
        ee_ori=matrix_to_rotvec(fk["rotation"]).reshape(1, 3),
    )

    result = evaluate_robot(robot, [record], config=EvaluationConfig(max_iters=1))

    assert result.aggregate["ik_success_rate"] == 1.0
    assert "pose_success_rate" not in result.aggregate
    assert "mean_orientation_error_rad" not in result.trajectory_rows[0]


def test_baseline_cli_exposes_pose_evaluation_options():
    args = build_parser().parse_args(
        [
            "--evaluation-mode",
            "pose",
            "--orientation-format",
            "rotvec",
            "--orientation-tolerance",
            "0.05",
            "--orientation-weight",
            "0.5",
            "--tool-frame-mapping",
            "canonical_tool",
        ]
    )

    assert args.evaluation_mode == "pose"
    assert args.orientation_format == "rotvec"
    assert args.orientation_tolerance == 0.05
    assert args.orientation_weight == 0.5
    assert args.tool_frame_mapping == "canonical_tool"


def test_baseline_cli_accepts_explicit_robot_subset():
    args = build_parser().parse_args(["--robots", "panda_true", "simple_default"])

    assert args.robots == ["panda_true", "simple_default"]


def test_mjcf_cli_exposes_pose_evaluation_options():
    args = build_mjcf_parser().parse_args(
        [
            "--mjcf",
            "robot.xml",
            "--robot-name",
            "ur5e_true",
            "--base-body",
            "base",
            "--evaluation-mode",
            "pose",
            "--orientation-format",
            "rotvec",
            "--tool-frame-mapping",
            "canonical_tool",
        ]
    )

    assert args.evaluation_mode == "pose"
    assert args.orientation_format == "rotvec"
    assert args.tool_frame_mapping == "canonical_tool"


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
