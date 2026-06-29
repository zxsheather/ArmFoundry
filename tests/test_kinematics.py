from __future__ import annotations

import numpy as np

from ddird.eval.ik import solve_position_ik
from ddird.robots.robot_registry import create_robot


def test_forward_kinematics_and_jacobian_shapes():
    robot = create_robot("simple_default")
    q = robot.neutral_q
    fk = robot.forward_kinematics(q)
    jac = robot.position_jacobian(q)

    assert fk["position"].shape == (3,)
    assert fk["joint_positions"].shape == (robot.dof + 1, 3)
    assert jac.shape == (3, robot.dof)


def test_position_ik_reaches_known_forward_kinematics_target():
    robot = create_robot("simple_default")
    q_target = np.array([0.2, -0.45, 0.55, 0.1, -0.2, 0.3])
    target = robot.end_effector_position(q_target)

    result = solve_position_ik(robot, target, q0=robot.neutral_q, max_iters=100)

    assert result.success
    assert result.error_norm < 2e-3
