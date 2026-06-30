from __future__ import annotations

import numpy as np

from ddird.eval.ik import solve_pose_ik, solve_position_ik
from ddird.eval.orientation import matrix_to_rotvec, orientation_error_vector, rotvec_to_matrix
from ddird.robots.robot_registry import create_robot


def test_forward_kinematics_and_jacobian_shapes():
    robot = create_robot("simple_default")
    q = robot.neutral_q
    fk = robot.forward_kinematics(q)
    jac = robot.position_jacobian(q)

    assert fk["position"].shape == (3,)
    assert fk["rotation"].shape == (3, 3)
    assert fk["joint_positions"].shape == (robot.dof + 1, 3)
    assert jac.shape == (3, robot.dof)
    assert robot.orientation_jacobian(q).shape == (3, robot.dof)


def test_position_ik_reaches_known_forward_kinematics_target():
    robot = create_robot("simple_default")
    q_target = np.array([0.2, -0.45, 0.55, 0.1, -0.2, 0.3])
    target = robot.end_effector_position(q_target)

    result = solve_position_ik(robot, target, q0=robot.neutral_q, max_iters=100)

    assert result.success
    assert result.error_norm < 2e-3


def test_pose_ik_accepts_known_forward_kinematics_pose():
    robot = create_robot("simple_default")
    q_target = np.array([0.2, -0.45, 0.55, 0.1, -0.2, 0.3])
    fk = robot.forward_kinematics(q_target)

    result = solve_pose_ik(
        robot,
        fk["position"],
        fk["rotation"],
        q0=robot.neutral_q,
        max_iters=100,
        position_tolerance=2e-3,
        orientation_tolerance=0.01,
    )

    assert result.success
    assert result.position_error_norm is not None and result.position_error_norm < 2e-3
    assert result.orientation_error_norm is not None and result.orientation_error_norm < 0.01


def test_orientation_error_rotvec_round_trip():
    robot = create_robot("simple_default")
    rotation = robot.end_effector_rotation(np.array([0.1, -0.2, 0.3, -0.4, 0.2, -0.1]))

    rotvec = matrix_to_rotvec(rotation)
    reconstructed = rotvec_to_matrix(rotvec)

    assert np.linalg.norm(orientation_error_vector(rotation, rotation)) < 1e-9
    assert np.allclose(reconstructed, rotation)
