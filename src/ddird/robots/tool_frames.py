from __future__ import annotations

from pathlib import Path
from typing import Any

import numpy as np


def _rounded_vector(values: Any, size: int = 3) -> list[float]:
    array = np.asarray(values, dtype=float).reshape(size)
    return [round(float(value), 8) for value in array]


def _target_offset(robot: Any) -> np.ndarray:
    return np.asarray(getattr(robot, "target_offset", np.zeros(3, dtype=float)), dtype=float).reshape(3)


def _target_quat(robot: Any) -> np.ndarray:
    return np.asarray(getattr(robot, "target_quat_wxyz", np.array([1.0, 0.0, 0.0, 0.0])), dtype=float).reshape(4)


def _source(robot: Any) -> str:
    return str(getattr(robot, "source", "unknown"))


def _tool_modeling(robot: Any, target_site: str | None) -> tuple[str, bool, bool, str]:
    name = str(getattr(robot, "name", "")).lower()
    source = _source(robot)
    target_frame = str(getattr(robot, "target_frame", target_site or ""))
    has_target_offset = float(np.linalg.norm(_target_offset(robot))) > 1e-9

    if name == "panda_true":
        return (
            "source_gripper_chain",
            True,
            True,
            "LIBERO source Panda gripper/eef kinematic chain is included up to the recorded gripper site.",
        )
    if name == "xarm6_true" and has_target_offset:
        return (
            "tcp_offset_only",
            True,
            False,
            "Official xArm gripper TCP offset is included; detailed gripper geometry and actuation are not modeled.",
        )
    if name == "xarm6_true":
        return (
            "wrist_origin_diagnostic",
            False,
            False,
            "xArm6 is evaluated at link6/wrist origin; this is diagnostic and does not represent gripper TCP reachability.",
        )
    if target_frame == "attachment_site":
        return (
            "attachment_site_only",
            has_target_offset,
            False,
            "UR attachment site is used as the TCP; no concrete gripper tip is modeled.",
        )
    if name.startswith("ur5") and has_target_offset:
        return (
            "tcp_offset_only",
            True,
            False,
            "Explicit UR TCP offset is included; detailed gripper geometry and actuation are not modeled.",
        )
    if source == "real_kinematics":
        return (
            "target_site_only",
            has_target_offset,
            False,
            "External true-kinematics model is evaluated at the requested target site/body.",
        )
    return (
        "proxy_endpoint",
        False,
        False,
        "Simplified proxy chain endpoint; no concrete tool or gripper is modeled.",
    )


def tool_frame_metadata(
    robot: Any,
    *,
    base_body: str | None = None,
    target_site: str | None = None,
    target_body: str | None = None,
    model_path: str | Path | None = None,
) -> dict[str, Any]:
    target_frame = str(getattr(robot, "target_frame", target_site or target_body or "end_effector"))
    tool_modeling, tcp_offset_included, concrete_gripper_modeled, semantics = _tool_modeling(robot, target_site)
    metadata: dict[str, Any] = {
        "robot_name": str(getattr(robot, "name", "unknown")),
        "source": _source(robot),
        "base_body": base_body,
        "target_site": target_site,
        "target_body": target_body,
        "target_frame": target_frame,
        "target_offset_xyz": _rounded_vector(_target_offset(robot)),
        "target_quat_wxyz": _rounded_vector(_target_quat(robot), size=4),
        "tool_modeling": tool_modeling,
        "tcp_offset_included": bool(tcp_offset_included),
        "concrete_gripper_modeled": bool(concrete_gripper_modeled),
        "semantics": semantics,
    }
    if model_path is not None:
        metadata["model_path"] = str(model_path)
    return metadata
