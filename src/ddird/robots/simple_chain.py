from __future__ import annotations

from dataclasses import dataclass, replace
from typing import Any

import numpy as np


def rot_x(angle: float) -> np.ndarray:
    c = np.cos(angle)
    s = np.sin(angle)
    return np.array(
        [
            [1.0, 0.0, 0.0],
            [0.0, c, -s],
            [0.0, s, c],
        ],
        dtype=float,
    )


def rot_y(angle: float) -> np.ndarray:
    c = np.cos(angle)
    s = np.sin(angle)
    return np.array(
        [
            [c, 0.0, s],
            [0.0, 1.0, 0.0],
            [-s, 0.0, c],
        ],
        dtype=float,
    )


def rot_z(angle: float) -> np.ndarray:
    c = np.cos(angle)
    s = np.sin(angle)
    return np.array(
        [
            [c, -s, 0.0],
            [s, c, 0.0],
            [0.0, 0.0, 1.0],
        ],
        dtype=float,
    )


def rotation_matrix(axis: str, angle: float) -> np.ndarray:
    if axis == "x":
        return rot_x(angle)
    if axis == "y":
        return rot_y(angle)
    if axis == "z":
        return rot_z(angle)
    raise ValueError(f"Unsupported axis {axis!r}")


def axis_vector(axis: str) -> np.ndarray:
    if axis == "x":
        return np.array([1.0, 0.0, 0.0])
    if axis == "y":
        return np.array([0.0, 1.0, 0.0])
    if axis == "z":
        return np.array([0.0, 0.0, 1.0])
    raise ValueError(f"Unsupported axis {axis!r}")


def wrap_to_pi(values: np.ndarray) -> np.ndarray:
    return (values + np.pi) % (2.0 * np.pi) - np.pi


@dataclass(frozen=True)
class SimpleChainRobot:
    name: str
    base_xyz: np.ndarray
    base_yaw: float
    link_lengths: np.ndarray
    joint_limits: np.ndarray
    joint_axes: tuple[str, ...] = ("z", "y", "y", "x", "y", "z")
    table_height: float = 0.0
    workspace_bounds: np.ndarray | None = None
    source: str = "parameterized"

    def __post_init__(self) -> None:
        object.__setattr__(self, "base_xyz", np.asarray(self.base_xyz, dtype=float).reshape(3))
        object.__setattr__(self, "link_lengths", np.asarray(self.link_lengths, dtype=float).reshape(-1))
        object.__setattr__(self, "joint_limits", np.asarray(self.joint_limits, dtype=float))
        if len(self.link_lengths) != 4:
            raise ValueError("link_lengths must contain four main link lengths")
        if self.joint_limits.shape != (self.dof, 2):
            raise ValueError(f"joint_limits must have shape ({self.dof}, 2)")
        if self.workspace_bounds is not None:
            object.__setattr__(self, "workspace_bounds", np.asarray(self.workspace_bounds, dtype=float))

    @property
    def dof(self) -> int:
        return len(self.joint_axes)

    @property
    def reach_proxy(self) -> float:
        return float(np.sum(np.abs(self.link_lengths)))

    @property
    def neutral_q(self) -> np.ndarray:
        return self.joint_limits.mean(axis=1)

    def with_base(self, base_xyz: np.ndarray | None = None, base_yaw: float | None = None) -> "SimpleChainRobot":
        return replace(
            self,
            base_xyz=np.asarray(base_xyz, dtype=float).reshape(3) if base_xyz is not None else self.base_xyz,
            base_yaw=float(base_yaw) if base_yaw is not None else self.base_yaw,
        )

    def with_design(
        self,
        base_xyz: np.ndarray | None = None,
        base_yaw: float | None = None,
        link_lengths: np.ndarray | None = None,
        name: str | None = None,
    ) -> "SimpleChainRobot":
        return replace(
            self,
            name=name or self.name,
            base_xyz=np.asarray(base_xyz, dtype=float).reshape(3) if base_xyz is not None else self.base_xyz,
            base_yaw=float(base_yaw) if base_yaw is not None else self.base_yaw,
            link_lengths=np.asarray(link_lengths, dtype=float).reshape(4) if link_lengths is not None else self.link_lengths,
        )

    def _link_vectors(self) -> list[np.ndarray]:
        l1, l2, l3, l4 = self.link_lengths
        return [
            np.array([0.0, 0.0, l1], dtype=float),
            np.array([l2, 0.0, 0.0], dtype=float),
            np.array([l3, 0.0, 0.0], dtype=float),
            np.array([0.0, 0.0, 0.15 * l4], dtype=float),
            np.array([0.85 * l4, 0.0, 0.0], dtype=float),
            np.array([0.0, 0.0, 0.0], dtype=float),
        ]

    def forward_kinematics(self, q: np.ndarray) -> dict[str, Any]:
        q = np.asarray(q, dtype=float).reshape(self.dof)
        position = self.base_xyz.astype(float).copy()
        rotation = rot_z(self.base_yaw)
        origins: list[np.ndarray] = []
        axes_world: list[np.ndarray] = []
        joint_positions: list[np.ndarray] = [position.copy()]

        for axis, angle, link_vector in zip(self.joint_axes, q, self._link_vectors(), strict=True):
            origins.append(position.copy())
            axes_world.append(rotation @ axis_vector(axis))
            rotation = rotation @ rotation_matrix(axis, angle)
            position = position + rotation @ link_vector
            joint_positions.append(position.copy())

        return {
            "position": position,
            "rotation": rotation,
            "origins": np.asarray(origins),
            "axes": np.asarray(axes_world),
            "joint_positions": np.asarray(joint_positions),
        }

    def end_effector_position(self, q: np.ndarray) -> np.ndarray:
        return np.asarray(self.forward_kinematics(q)["position"], dtype=float)

    def end_effector_rotation(self, q: np.ndarray) -> np.ndarray:
        return np.asarray(self.forward_kinematics(q)["rotation"], dtype=float)

    def position_jacobian(self, q: np.ndarray) -> np.ndarray:
        fk = self.forward_kinematics(q)
        end = fk["position"]
        jac = np.zeros((3, self.dof), dtype=float)
        for index, (axis, origin) in enumerate(zip(fk["axes"], fk["origins"], strict=True)):
            jac[:, index] = np.cross(axis, end - origin)
        return jac

    def orientation_jacobian(self, q: np.ndarray) -> np.ndarray:
        fk = self.forward_kinematics(q)
        return np.asarray(fk["axes"], dtype=float).T

    def joint_margin(self, q: np.ndarray) -> np.ndarray:
        q = np.asarray(q, dtype=float).reshape(self.dof)
        return np.minimum(q - self.joint_limits[:, 0], self.joint_limits[:, 1] - q)

    def clip_to_limits(self, q: np.ndarray) -> np.ndarray:
        q = wrap_to_pi(np.asarray(q, dtype=float).reshape(self.dof))
        return np.clip(q, self.joint_limits[:, 0], self.joint_limits[:, 1])

    def within_limits(self, q: np.ndarray, tolerance: float = 1e-8) -> bool:
        q = np.asarray(q, dtype=float).reshape(self.dof)
        return bool(np.all(q >= self.joint_limits[:, 0] - tolerance) and np.all(q <= self.joint_limits[:, 1] + tolerance))

    def manipulability(self, q: np.ndarray) -> float:
        jac = self.position_jacobian(q)
        gram = jac @ jac.T
        return float(np.sqrt(max(0.0, np.linalg.det(gram))))

    def collision_proxy(self, q: np.ndarray) -> float:
        points = np.asarray(self.forward_kinematics(q)["joint_positions"], dtype=float)
        penalty = float(np.sum(np.maximum(0.0, self.table_height - points[:, 2]) ** 2))
        if self.workspace_bounds is not None:
            lower = self.workspace_bounds[:, 0]
            upper = self.workspace_bounds[:, 1]
            below = np.maximum(0.0, lower - points)
            above = np.maximum(0.0, points - upper)
            penalty += float(np.sum(below**2 + above**2))
        return penalty

    def to_dict(self) -> dict[str, Any]:
        return {
            "name": self.name,
            "base_xyz": self.base_xyz.round(6).tolist(),
            "base_yaw": round(float(self.base_yaw), 6),
            "link_lengths": self.link_lengths.round(6).tolist(),
            "joint_limits": self.joint_limits.round(6).tolist(),
            "reach_proxy": round(self.reach_proxy, 6),
            "source": self.source,
        }


def default_joint_limits(dof: int = 6) -> np.ndarray:
    limits = np.tile(np.array([-np.pi, np.pi], dtype=float), (dof, 1))
    limits[1] = [-2.6, 2.6]
    limits[2] = [-2.8, 2.8]
    limits[3] = [-2.9, 2.9]
    limits[4] = [-2.5, 2.5]
    return limits


def make_simple_arm(
    name: str = "simple_default",
    base_xyz: tuple[float, float, float] | np.ndarray = (0.0, 0.0, 0.0),
    base_yaw: float = 0.0,
    link_lengths: tuple[float, float, float, float] | np.ndarray = (0.30, 0.34, 0.28, 0.16),
    source: str = "parameterized",
) -> SimpleChainRobot:
    return SimpleChainRobot(
        name=name,
        base_xyz=np.asarray(base_xyz, dtype=float),
        base_yaw=base_yaw,
        link_lengths=np.asarray(link_lengths, dtype=float),
        joint_limits=default_joint_limits(6),
        workspace_bounds=np.array([[-1.2, 1.2], [-1.2, 1.2], [-0.02, 1.4]], dtype=float),
        source=source,
    )
