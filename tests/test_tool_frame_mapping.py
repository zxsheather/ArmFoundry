from __future__ import annotations

from types import SimpleNamespace

import numpy as np
import pytest

from ddird.data.dataset import TrajectoryRecord
from ddird.eval.evaluator import EvaluationConfig, evaluate_robot
from ddird.eval.orientation import matrix_to_rotvec, quat_wxyz_to_matrix, rotvec_to_matrix
from ddird.eval.tool_frame_mapping import (
    CanonicalToolFrameMapping,
    apply_tool_frame_mapping,
    canonical_tool_frame_mapping,
    tool_frame_mapping_metadata,
)
from ddird.robots.robot_registry import create_robot


def test_canonical_mapping_composes_source_to_target_rotation():
    quarter_turn_z = (np.sqrt(0.5), 0.0, 0.0, np.sqrt(0.5))
    mapping = CanonicalToolFrameMapping(
        robot_name="test",
        target_frame="tcp",
        source_frame="source_tcp",
        source_to_canonical_quat_wxyz=quarter_turn_z,
    )

    assert np.allclose(mapping.source_to_target_rotation, quat_wxyz_to_matrix(np.asarray(quarter_turn_z)))


def test_canonical_mapping_keeps_registered_tcp_frames_identity_mapped():
    robot = SimpleNamespace(name="xarm6_true", target_frame="link_tcp")
    rotations = np.asarray([rotvec_to_matrix(np.array([0.1, -0.2, 0.3]))])

    mapped = apply_tool_frame_mapping(rotations, robot, "canonical_tool")

    assert np.allclose(mapped, rotations)
    metadata = tool_frame_mapping_metadata(robot, "canonical_tool")
    assert metadata["eligible"] is True
    assert metadata["status"] == "mapped_tcp_offset_only"


def test_canonical_mapping_rejects_unmapped_or_wrong_target_frame():
    with pytest.raises(ValueError, match="has no canonical_tool frame mapping"):
        canonical_tool_frame_mapping(SimpleNamespace(name="ur5_proxy", target_frame="end_effector"))

    with pytest.raises(ValueError, match="does not match canonical mapping target frame"):
        canonical_tool_frame_mapping(SimpleNamespace(name="xarm6_true", target_frame="link6"))


def test_pose_evaluator_rejects_canonical_mapping_for_proxy_robot(tmp_path):
    robot = create_robot("simple_default")
    fk = robot.forward_kinematics(robot.neutral_q)
    record = TrajectoryRecord(
        suite="suite",
        task="task",
        episode_id="episode",
        path=tmp_path / "episode.npz",
        ee_pos=fk["position"].reshape(1, 3),
        ee_ori=matrix_to_rotvec(fk["rotation"]).reshape(1, 3),
    )

    with pytest.raises(ValueError, match="has no canonical_tool frame mapping"):
        evaluate_robot(
            robot,
            [record],
            config=EvaluationConfig(
                max_iters=1,
                evaluation_mode="pose",
                orientation_format="rotvec",
                tool_frame_mapping="canonical_tool",
            ),
        )
