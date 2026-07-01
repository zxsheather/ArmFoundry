from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import numpy as np

from ddird.eval.orientation import quat_wxyz_to_matrix

TOOL_FRAME_MAPPING_MODES = ("identity", "canonical_tool")

IDENTITY_QUAT_WXYZ = (1.0, 0.0, 0.0, 0.0)


@dataclass(frozen=True)
class CanonicalToolFrameMapping:
    robot_name: str
    target_frame: str
    source_frame: str
    source_to_canonical_quat_wxyz: tuple[float, float, float, float] = IDENTITY_QUAT_WXYZ
    canonical_to_target_quat_wxyz: tuple[float, float, float, float] = IDENTITY_QUAT_WXYZ
    status: str = "mapped"
    rationale: str = ""

    @property
    def source_to_target_rotation(self) -> np.ndarray:
        source_to_canonical = quat_wxyz_to_matrix(np.asarray(self.source_to_canonical_quat_wxyz, dtype=float))
        canonical_to_target = quat_wxyz_to_matrix(np.asarray(self.canonical_to_target_quat_wxyz, dtype=float))
        return source_to_canonical @ canonical_to_target


CANONICAL_TOOL_FRAME = {
    "name": "parallel_jaw_pinch_tcp",
    "origin": "pinch center / gripper TCP",
    "x_axis": "+Y cross +Z; completes a right-handed frame",
    "y_axis": "finger opening/closing direction between left and right jaws",
    "z_axis": "tool approach direction from palm/wrist toward the pinch point",
}

_CANONICAL_MAPPINGS: dict[str, CanonicalToolFrameMapping] = {
    "panda_true": CanonicalToolFrameMapping(
        robot_name="panda_true",
        target_frame="gripper0_grip_site",
        source_frame="libero_panda/gripper0_grip_site",
        status="source_canonical",
        rationale=(
            "LIBERO ee_ori is recorded at the Panda gripper0_grip_site; that site is the source "
            "canonical parallel-jaw TCP frame."
        ),
    ),
    "xarm6_true": CanonicalToolFrameMapping(
        robot_name="xarm6_true",
        target_frame="link_tcp",
        source_frame="libero_panda/gripper0_grip_site",
        status="mapped_tcp_offset_only",
        rationale=(
            "Official xArm gripper joint_tcp has rpy=0, +Z offset to the TCP, and symmetric "
            "finger structure along +/-Y, matching the canonical parallel-jaw axes. The benchmark "
            "model still includes only the TCP offset, not full gripper geometry."
        ),
    ),
    "ur5e_true_robotiq2f85_tcp_proxy": CanonicalToolFrameMapping(
        robot_name="ur5e_true_robotiq2f85_tcp_proxy",
        target_frame="ur5e_robotiq2f85_tcp",
        source_frame="libero_panda/gripper0_grip_site",
        status="mapped_tcp_offset_only",
        rationale=(
            "The Robotiq 2F-85 pinch site uses +Z from gripper base to pinch center and +/-Y "
            "jaw symmetry, matching the canonical parallel-jaw axes after composition onto the "
            "UR5e attachment_site. The benchmark model remains tcp_offset_only."
        ),
    ),
}


def canonical_tool_frame_mapping(robot: Any) -> CanonicalToolFrameMapping:
    name = str(getattr(robot, "name", "")).lower()
    mapping = _CANONICAL_MAPPINGS.get(name)
    if mapping is None:
        raise ValueError(
            f"Robot {getattr(robot, 'name', '<unknown>')!r} has no canonical_tool frame mapping. "
            "Use --tool-frame-mapping identity for diagnostic pose runs, or add an explicit "
            "source-to-canonical and canonical-to-target mapping before claiming pose fairness."
        )
    target_frame = str(getattr(robot, "target_frame", mapping.target_frame))
    if target_frame != mapping.target_frame:
        raise ValueError(
            f"Robot {getattr(robot, 'name', '<unknown>')!r} target frame {target_frame!r} does not "
            f"match canonical mapping target frame {mapping.target_frame!r}."
        )
    return mapping


def apply_tool_frame_mapping(rotations: np.ndarray, robot: Any, mode: str) -> np.ndarray:
    if mode not in TOOL_FRAME_MAPPING_MODES:
        raise ValueError(f"tool_frame_mapping must be one of {TOOL_FRAME_MAPPING_MODES}")
    rotations = np.asarray(rotations, dtype=float)
    if mode == "identity":
        return rotations
    mapping = canonical_tool_frame_mapping(robot)
    return rotations @ mapping.source_to_target_rotation


def tool_frame_mapping_metadata(robot: Any, mode: str) -> dict[str, Any]:
    if mode not in TOOL_FRAME_MAPPING_MODES:
        raise ValueError(f"tool_frame_mapping must be one of {TOOL_FRAME_MAPPING_MODES}")
    metadata: dict[str, Any] = {
        "mode": mode,
        "canonical_frame": CANONICAL_TOOL_FRAME,
        "target_robot": str(getattr(robot, "name", "unknown")),
        "target_frame": str(getattr(robot, "target_frame", "")),
        "pose_fairness_claim": mode == "canonical_tool",
    }
    if mode == "identity":
        metadata.update(
            {
                "status": "legacy_identity_mapping",
                "eligible": False,
                "rationale": (
                    "Source orientation is passed directly to the target TCP. This preserves legacy "
                    "diagnostic behavior but is not a canonical pose-fairness claim."
                ),
            }
        )
        return metadata
    try:
        mapping = canonical_tool_frame_mapping(robot)
    except ValueError as error:
        metadata.update(
            {
                "status": "unmapped",
                "eligible": False,
                "error": str(error),
            }
        )
        return metadata
    metadata.update(
        {
            "status": mapping.status,
            "eligible": True,
            "source_frame": mapping.source_frame,
            "source_to_canonical_quat_wxyz": list(mapping.source_to_canonical_quat_wxyz),
            "canonical_to_target_quat_wxyz": list(mapping.canonical_to_target_quat_wxyz),
            "source_to_target_quat_wxyz": list(IDENTITY_QUAT_WXYZ)
            if np.allclose(mapping.source_to_target_rotation, np.eye(3))
            else None,
            "rationale": mapping.rationale,
        }
    )
    return metadata
