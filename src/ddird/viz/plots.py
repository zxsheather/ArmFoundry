from __future__ import annotations

from pathlib import Path
from typing import Any

import numpy as np

from ddird.data.dataset import TrajectoryRecord, trajectory_point_cloud
from ddird.eval.evaluator import EvaluationResult
from ddird.robots.simple_chain import SimpleChainRobot


def _plt():
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    return plt


def _prepare(path: str | Path) -> Path:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    return path


def plot_ee_point_cloud(records: list[TrajectoryRecord], path: str | Path) -> Path:
    plt = _plt()
    path = _prepare(path)
    points = trajectory_point_cloud(records)
    fig = plt.figure(figsize=(8, 6))
    ax = fig.add_subplot(111, projection="3d")
    if len(points):
        ax.scatter(points[:, 0], points[:, 1], points[:, 2], s=5, c="#6b7280", alpha=0.45)
    ax.set_title("End-effector trajectory point cloud")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_zlabel("z [m]")
    fig.tight_layout()
    fig.savefig(path, dpi=180)
    plt.close(fig)
    return path


def _collect_detail_points(result: EvaluationResult) -> tuple[np.ndarray, np.ndarray]:
    targets = []
    successes = []
    for detail in result.details:
        targets.append(np.asarray(detail["target_pos"], dtype=float))
        successes.append(np.asarray(detail["success"], dtype=bool))
    if not targets:
        return np.zeros((0, 3)), np.zeros((0,), dtype=bool)
    return np.concatenate(targets, axis=0), np.concatenate(successes, axis=0)


def plot_reachability(result: EvaluationResult, path: str | Path) -> Path:
    plt = _plt()
    path = _prepare(path)
    points, successes = _collect_detail_points(result)
    fig = plt.figure(figsize=(8, 6))
    ax = fig.add_subplot(111, projection="3d")
    if len(points):
        reachable = points[successes]
        unreachable = points[~successes]
        if len(reachable):
            ax.scatter(reachable[:, 0], reachable[:, 1], reachable[:, 2], s=6, c="#16a34a", alpha=0.55, label="reachable")
        if len(unreachable):
            ax.scatter(unreachable[:, 0], unreachable[:, 1], unreachable[:, 2], s=10, c="#dc2626", alpha=0.75, label="unreachable")
    ax.set_title(f"Reachability: {result.robot.name}")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_zlabel("z [m]")
    ax.legend(loc="best")
    fig.tight_layout()
    fig.savefig(path, dpi=180)
    plt.close(fig)
    return path


def plot_robot_geometry(before: SimpleChainRobot, after: SimpleChainRobot, path: str | Path) -> Path:
    plt = _plt()
    path = _prepare(path)
    fig = plt.figure(figsize=(8, 6))
    ax = fig.add_subplot(111, projection="3d")
    for robot, color, label in [(before, "#2563eb", before.name), (after, "#dc2626", after.name)]:
        points = robot.forward_kinematics(robot.neutral_q)["joint_positions"]
        ax.plot(points[:, 0], points[:, 1], points[:, 2], marker="o", color=color, linewidth=2, label=label)
        ax.scatter([robot.base_xyz[0]], [robot.base_xyz[1]], [robot.base_xyz[2]], c=color, marker="s", s=50)
    ax.set_title("Robot geometry before and after optimization")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_zlabel("z [m]")
    ax.legend(loc="best")
    fig.tight_layout()
    fig.savefig(path, dpi=180)
    plt.close(fig)
    return path


def plot_ik_success_bar(rows: list[dict[str, Any]], path: str | Path) -> Path:
    plt = _plt()
    path = _prepare(path)
    names = [str(row["robot_name"]) for row in rows]
    values = [float(row["ik_success_rate"]) for row in rows]
    fig, ax = plt.subplots(figsize=(8, 4.5))
    ax.bar(names, values, color="#2563eb")
    ax.set_ylim(0.0, 1.05)
    ax.set_ylabel("IK success rate")
    ax.set_title("IK success by robot model")
    ax.tick_params(axis="x", rotation=20)
    fig.tight_layout()
    fig.savefig(path, dpi=180)
    plt.close(fig)
    return path


def _collect_named_values(results: list[EvaluationResult], field: str) -> list[tuple[str, np.ndarray]]:
    named = []
    for result in results:
        values = []
        for detail in result.details:
            values.append(np.asarray(detail[field], dtype=float))
        if values:
            named.append((result.robot.name, np.concatenate(values)))
    return named


def plot_joint_margin_hist(results: list[EvaluationResult], path: str | Path) -> Path:
    plt = _plt()
    path = _prepare(path)
    fig, ax = plt.subplots(figsize=(8, 4.5))
    for name, values in _collect_named_values(results, "joint_margin"):
        ax.hist(values, bins=30, alpha=0.42, label=name)
    ax.set_xlabel("Minimum joint-limit margin [rad]")
    ax.set_ylabel("Waypoint count")
    ax.set_title("Joint-limit margin distribution")
    ax.legend(loc="best")
    fig.tight_layout()
    fig.savefig(path, dpi=180)
    plt.close(fig)
    return path


def plot_manipulability_box(results: list[EvaluationResult], path: str | Path) -> Path:
    plt = _plt()
    path = _prepare(path)
    named = _collect_named_values(results, "manipulability")
    labels = [name for name, _ in named]
    values = [data for _, data in named]
    fig, ax = plt.subplots(figsize=(8, 4.5))
    if values:
        ax.boxplot(values, tick_labels=labels, showfliers=False)
    ax.set_ylabel("Yoshikawa manipulability proxy")
    ax.set_title("Manipulability distribution")
    ax.tick_params(axis="x", rotation=20)
    fig.tight_layout()
    fig.savefig(path, dpi=180)
    plt.close(fig)
    return path
