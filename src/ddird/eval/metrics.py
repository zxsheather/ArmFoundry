from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import numpy as np


@dataclass(frozen=True)
class TrajectoryMetrics:
    robot_name: str
    suite: str
    task: str
    episode_id: str
    num_waypoints: int
    ik_success_rate: float
    trajectory_success: bool
    mean_position_error: float
    max_position_error: float
    mean_joint_margin: float
    min_joint_margin: float
    mean_manipulability: float
    min_manipulability: float
    manipulability_below_threshold: float
    smoothness_cost: float
    collision_proxy: float
    hardware_cost: float

    def to_dict(self) -> dict[str, Any]:
        return {
            "robot_name": self.robot_name,
            "suite": self.suite,
            "task": self.task,
            "episode_id": self.episode_id,
            "num_waypoints": self.num_waypoints,
            "ik_success_rate": round(float(self.ik_success_rate), 6),
            "trajectory_success": int(self.trajectory_success),
            "mean_position_error": round(float(self.mean_position_error), 6),
            "max_position_error": round(float(self.max_position_error), 6),
            "mean_joint_margin": round(float(self.mean_joint_margin), 6),
            "min_joint_margin": round(float(self.min_joint_margin), 6),
            "mean_manipulability": round(float(self.mean_manipulability), 8),
            "min_manipulability": round(float(self.min_manipulability), 8),
            "manipulability_below_threshold": round(float(self.manipulability_below_threshold), 6),
            "smoothness_cost": round(float(self.smoothness_cost), 8),
            "collision_proxy": round(float(self.collision_proxy), 8),
            "hardware_cost": round(float(self.hardware_cost), 6),
        }


def safe_mean(values: np.ndarray, default: float = 0.0) -> float:
    values = np.asarray(values, dtype=float)
    values = values[np.isfinite(values)]
    if values.size == 0:
        return default
    return float(np.mean(values))


def safe_min(values: np.ndarray, default: float = 0.0) -> float:
    values = np.asarray(values, dtype=float)
    values = values[np.isfinite(values)]
    if values.size == 0:
        return default
    return float(np.min(values))


def aggregate_metric_dicts(rows: list[dict[str, Any]], group_name: str = "overall") -> dict[str, Any]:
    if not rows:
        return {
            "robot_name": group_name,
            "num_trajectories": 0,
            "num_waypoints": 0,
            "ik_success_rate": 0.0,
            "trajectory_success_rate": 0.0,
            "mean_joint_margin": 0.0,
            "min_joint_margin": 0.0,
            "mean_manipulability": 0.0,
            "min_manipulability": 0.0,
            "smoothness_cost": 0.0,
            "collision_proxy": 0.0,
            "hardware_cost": 0.0,
        }

    total_waypoints = sum(int(row["num_waypoints"]) for row in rows)
    weighted_success = sum(float(row["ik_success_rate"]) * int(row["num_waypoints"]) for row in rows)
    return {
        "robot_name": group_name,
        "num_trajectories": len(rows),
        "num_waypoints": total_waypoints,
        "ik_success_rate": round(weighted_success / max(1, total_waypoints), 6),
        "trajectory_success_rate": round(float(np.mean([float(row["trajectory_success"]) for row in rows])), 6),
        "mean_position_error": round(float(np.mean([float(row["mean_position_error"]) for row in rows])), 6),
        "max_position_error": round(float(np.max([float(row["max_position_error"]) for row in rows])), 6),
        "mean_joint_margin": round(float(np.mean([float(row["mean_joint_margin"]) for row in rows])), 6),
        "min_joint_margin": round(float(np.min([float(row["min_joint_margin"]) for row in rows])), 6),
        "mean_manipulability": round(float(np.mean([float(row["mean_manipulability"]) for row in rows])), 8),
        "min_manipulability": round(float(np.min([float(row["min_manipulability"]) for row in rows])), 8),
        "manipulability_below_threshold": round(float(np.mean([float(row["manipulability_below_threshold"]) for row in rows])), 6),
        "smoothness_cost": round(float(np.mean([float(row["smoothness_cost"]) for row in rows])), 8),
        "collision_proxy": round(float(np.mean([float(row["collision_proxy"]) for row in rows])), 8),
        "hardware_cost": round(float(np.mean([float(row["hardware_cost"]) for row in rows])), 6),
    }
