from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from ddird.robots.simple_chain import SimpleChainRobot


@dataclass(frozen=True)
class IKResult:
    success: bool
    q: np.ndarray
    error_norm: float
    iterations: int


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
            return IKResult(True, q.copy(), error_norm, iteration)

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

    return IKResult(False, best_q, best_error, max_iters)


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
