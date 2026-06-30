from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from ddird.eval.orientation import orientation_error_vector
from ddird.robots.simple_chain import SimpleChainRobot


@dataclass(frozen=True)
class IKResult:
    success: bool
    q: np.ndarray
    error_norm: float
    iterations: int
    position_error_norm: float | None = None
    orientation_error_norm: float | None = None


def solve_position_ik(
    robot: SimpleChainRobot,
    target_pos: np.ndarray,
    q0: np.ndarray | None = None,
    max_iters: int = 120,
    tolerance: float = 2e-3,
    damping: float = 2e-2,
    step_limit: float = 0.18,
) -> IKResult:
    target_pos = np.asarray(target_pos, dtype=float).reshape(3)
    q = robot.clip_to_limits(q0 if q0 is not None else robot.neutral_q)
    best_q = q.copy()
    best_error = float("inf")

    for iteration in range(1, max_iters + 1):
        current = robot.end_effector_position(q)
        error = target_pos - current
        error_norm = float(np.linalg.norm(error))
        if error_norm < best_error:
            best_error = error_norm
            best_q = q.copy()
        if error_norm <= tolerance and robot.within_limits(q):
            return IKResult(True, q.copy(), error_norm, iteration, position_error_norm=error_norm)

        jac = robot.position_jacobian(q)
        lhs = jac @ jac.T + (damping**2) * np.eye(3)
        try:
            step = jac.T @ np.linalg.solve(lhs, error)
        except np.linalg.LinAlgError:
            step = jac.T @ np.linalg.pinv(lhs) @ error
        step_norm = float(np.linalg.norm(step))
        if step_norm > step_limit:
            step = step * (step_limit / step_norm)

        q = robot.clip_to_limits(q + step)

    return IKResult(False, best_q, best_error, max_iters, position_error_norm=best_error)


def solve_position_ik_multiseed(
    robot: SimpleChainRobot,
    target_pos: np.ndarray,
    seeds: list[np.ndarray] | None = None,
    max_iters: int = 120,
    tolerance: float = 2e-3,
    damping: float = 2e-2,
) -> IKResult:
    candidate_seeds = seeds or [robot.neutral_q]
    best: IKResult | None = None
    for seed in candidate_seeds:
        result = solve_position_ik(
            robot,
            target_pos,
            q0=seed,
            max_iters=max_iters,
            tolerance=tolerance,
            damping=damping,
        )
        if result.success:
            return result
        if best is None or result.error_norm < best.error_norm:
            best = result
    if best is None:
        return solve_position_ik(robot, target_pos, max_iters=max_iters, tolerance=tolerance, damping=damping)
    return best


def solve_pose_ik(
    robot: SimpleChainRobot,
    target_pos: np.ndarray,
    target_rotation: np.ndarray,
    q0: np.ndarray | None = None,
    max_iters: int = 120,
    position_tolerance: float = 2e-3,
    orientation_tolerance: float = 0.10,
    damping: float = 2e-2,
    step_limit: float = 0.18,
    orientation_weight: float = 1.0,
) -> IKResult:
    target_pos = np.asarray(target_pos, dtype=float).reshape(3)
    target_rotation = np.asarray(target_rotation, dtype=float).reshape(3, 3)
    q = robot.clip_to_limits(q0 if q0 is not None else robot.neutral_q)
    best_q = q.copy()
    best_score = float("inf")
    best_position_error = float("inf")
    best_orientation_error = float("inf")

    for iteration in range(1, max_iters + 1):
        fk = robot.forward_kinematics(q)
        current_pos = np.asarray(fk["position"], dtype=float)
        current_rotation = np.asarray(fk["rotation"], dtype=float)
        position_error = target_pos - current_pos
        orientation_error = orientation_error_vector(target_rotation, current_rotation)
        position_error_norm = float(np.linalg.norm(position_error))
        orientation_error_norm = float(np.linalg.norm(orientation_error))
        score = np.sqrt(
            (position_error_norm / max(position_tolerance, 1e-12)) ** 2
            + (orientation_error_norm / max(orientation_tolerance, 1e-12)) ** 2
        )
        if score < best_score:
            best_score = score
            best_q = q.copy()
            best_position_error = position_error_norm
            best_orientation_error = orientation_error_norm
        if (
            position_error_norm <= position_tolerance
            and orientation_error_norm <= orientation_tolerance
            and robot.within_limits(q)
        ):
            return IKResult(
                True,
                q.copy(),
                score,
                iteration,
                position_error_norm=position_error_norm,
                orientation_error_norm=orientation_error_norm,
            )

        jac = np.vstack([robot.position_jacobian(q), orientation_weight * robot.orientation_jacobian(q)])
        error = np.concatenate([position_error, orientation_weight * orientation_error])
        lhs = jac @ jac.T + (damping**2) * np.eye(6)
        try:
            step = jac.T @ np.linalg.solve(lhs, error)
        except np.linalg.LinAlgError:
            step = jac.T @ np.linalg.pinv(lhs) @ error
        step_norm = float(np.linalg.norm(step))
        if step_norm > step_limit:
            step = step * (step_limit / step_norm)

        q = robot.clip_to_limits(q + step)

    return IKResult(
        False,
        best_q,
        best_score,
        max_iters,
        position_error_norm=best_position_error,
        orientation_error_norm=best_orientation_error,
    )


def solve_pose_ik_multiseed(
    robot: SimpleChainRobot,
    target_pos: np.ndarray,
    target_rotation: np.ndarray,
    seeds: list[np.ndarray] | None = None,
    max_iters: int = 120,
    position_tolerance: float = 2e-3,
    orientation_tolerance: float = 0.10,
    damping: float = 2e-2,
    orientation_weight: float = 1.0,
) -> IKResult:
    candidate_seeds = seeds or [robot.neutral_q]
    best: IKResult | None = None
    for seed in candidate_seeds:
        result = solve_pose_ik(
            robot,
            target_pos,
            target_rotation,
            q0=seed,
            max_iters=max_iters,
            position_tolerance=position_tolerance,
            orientation_tolerance=orientation_tolerance,
            damping=damping,
            orientation_weight=orientation_weight,
        )
        if result.success:
            return result
        if best is None or result.error_norm < best.error_norm:
            best = result
    if best is None:
        return solve_pose_ik(
            robot,
            target_pos,
            target_rotation,
            max_iters=max_iters,
            position_tolerance=position_tolerance,
            orientation_tolerance=orientation_tolerance,
            damping=damping,
            orientation_weight=orientation_weight,
        )
    return best
