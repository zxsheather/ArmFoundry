from __future__ import annotations

import numpy as np

from ddird.data.dataset import TrajectoryRecord


ORIENTATION_FORMATS = ("auto", "quat_wxyz", "quat_xyzw", "rotvec", "euler_xyz")


def skew(vector: np.ndarray) -> np.ndarray:
    x, y, z = np.asarray(vector, dtype=float).reshape(3)
    return np.array(
        [
            [0.0, -z, y],
            [z, 0.0, -x],
            [-y, x, 0.0],
        ],
        dtype=float,
    )


def quat_wxyz_to_matrix(quat: np.ndarray) -> np.ndarray:
    quat = np.asarray(quat, dtype=float).reshape(4)
    norm = float(np.linalg.norm(quat))
    if norm == 0.0:
        raise ValueError("Cannot convert zero-norm quaternion to a rotation matrix")
    w, x, y, z = quat / norm
    return np.array(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
        ],
        dtype=float,
    )


def quat_xyzw_to_matrix(quat: np.ndarray) -> np.ndarray:
    x, y, z, w = np.asarray(quat, dtype=float).reshape(4)
    return quat_wxyz_to_matrix(np.array([w, x, y, z], dtype=float))


def rotvec_to_matrix(rotvec: np.ndarray) -> np.ndarray:
    rotvec = np.asarray(rotvec, dtype=float).reshape(3)
    angle = float(np.linalg.norm(rotvec))
    if angle < 1e-12:
        return np.eye(3) + skew(rotvec)
    axis = rotvec / angle
    axis_skew = skew(axis)
    return np.eye(3) + np.sin(angle) * axis_skew + (1.0 - np.cos(angle)) * (axis_skew @ axis_skew)


def euler_xyz_to_matrix(euler_xyz: np.ndarray) -> np.ndarray:
    roll, pitch, yaw = np.asarray(euler_xyz, dtype=float).reshape(3)
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    rx = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]], dtype=float)
    ry = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]], dtype=float)
    rz = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]], dtype=float)
    return rz @ ry @ rx


def matrix_to_rotvec(rotation: np.ndarray) -> np.ndarray:
    rotation = np.asarray(rotation, dtype=float).reshape(3, 3)
    cos_angle = float(np.clip((np.trace(rotation) - 1.0) * 0.5, -1.0, 1.0))
    angle = float(np.arccos(cos_angle))
    if angle < 1e-12:
        return 0.5 * np.array(
            [
                rotation[2, 1] - rotation[1, 2],
                rotation[0, 2] - rotation[2, 0],
                rotation[1, 0] - rotation[0, 1],
            ],
            dtype=float,
        )
    if np.pi - angle < 1e-6:
        axis = np.sqrt(np.maximum((np.diag(rotation) + 1.0) * 0.5, 0.0))
        axis[0] = np.copysign(axis[0], rotation[2, 1] - rotation[1, 2])
        axis[1] = np.copysign(axis[1], rotation[0, 2] - rotation[2, 0])
        axis[2] = np.copysign(axis[2], rotation[1, 0] - rotation[0, 1])
        norm = float(np.linalg.norm(axis))
        if norm == 0.0:
            axis = np.array([1.0, 0.0, 0.0], dtype=float)
        else:
            axis = axis / norm
        return angle * axis
    scale = angle / (2.0 * np.sin(angle))
    return scale * np.array(
        [
            rotation[2, 1] - rotation[1, 2],
            rotation[0, 2] - rotation[2, 0],
            rotation[1, 0] - rotation[0, 1],
        ],
        dtype=float,
    )


def orientation_error_vector(target_rotation: np.ndarray, current_rotation: np.ndarray) -> np.ndarray:
    target_rotation = np.asarray(target_rotation, dtype=float).reshape(3, 3)
    current_rotation = np.asarray(current_rotation, dtype=float).reshape(3, 3)
    return matrix_to_rotvec(target_rotation @ current_rotation.T)


def _matrices_from_array(values: np.ndarray, fmt: str) -> np.ndarray:
    values = np.asarray(values, dtype=float)
    if values.ndim != 2:
        raise ValueError(f"Orientation array must have shape [T, 3] or [T, 4], got {values.shape}")
    if fmt == "quat_wxyz":
        if values.shape[1] != 4:
            raise ValueError(f"quat_wxyz orientation requires shape [T, 4], got {values.shape}")
        return np.asarray([quat_wxyz_to_matrix(row) for row in values], dtype=float)
    if fmt == "quat_xyzw":
        if values.shape[1] != 4:
            raise ValueError(f"quat_xyzw orientation requires shape [T, 4], got {values.shape}")
        return np.asarray([quat_xyzw_to_matrix(row) for row in values], dtype=float)
    if fmt == "rotvec":
        if values.shape[1] != 3:
            raise ValueError(f"rotvec orientation requires shape [T, 3], got {values.shape}")
        return np.asarray([rotvec_to_matrix(row) for row in values], dtype=float)
    if fmt == "euler_xyz":
        if values.shape[1] != 3:
            raise ValueError(f"euler_xyz orientation requires shape [T, 3], got {values.shape}")
        return np.asarray([euler_xyz_to_matrix(row) for row in values], dtype=float)
    raise ValueError(f"Unsupported orientation format {fmt!r}; expected one of {ORIENTATION_FORMATS}")


def record_orientation_matrices(record: TrajectoryRecord, orientation_format: str = "auto") -> np.ndarray | None:
    if orientation_format not in ORIENTATION_FORMATS:
        raise ValueError(
            f"Unsupported orientation format {orientation_format!r}; expected one of {ORIENTATION_FORMATS}"
        )

    if orientation_format == "auto":
        if record.ee_quat is not None:
            return _matrices_from_array(record.ee_quat, "quat_wxyz")
        if record.ee_ori is not None:
            values = np.asarray(record.ee_ori, dtype=float)
            if values.ndim != 2:
                raise ValueError(
                    f"{record.path} has invalid ee_ori shape {values.shape}; expected [T, 3] or [T, 4]"
                )
            if values.shape[1] == 3:
                return _matrices_from_array(values, "rotvec")
            if values.shape[1] == 4:
                return _matrices_from_array(values, "quat_wxyz")
            raise ValueError(
                f"{record.path} has invalid ee_ori shape {values.shape}; expected [T, 3] or [T, 4]"
            )
        return None

    source = record.ee_quat if orientation_format.startswith("quat") and record.ee_quat is not None else record.ee_ori
    if source is None:
        return None
    return _matrices_from_array(source, orientation_format)
