from __future__ import annotations

import json
from pathlib import Path
from typing import Any
from xml.etree import ElementTree

import numpy as np

from ddird.data.dataset import (
    TrajectoryRecord,
    create_synthetic_dataset,
    load_trajectory_npz,
    save_processed_trajectory,
    summarize_dataset,
    write_metadata,
)


POSITION_KEYS = (
    "ee_pos",
    "eef_pos",
    "end_effector_pos",
    "robot0_eef_pos",
    "obs/ee_pos",
    "obs/eef_pos",
    "obs/robot0_eef_pos",
    "observations/ee_pos",
    "observations/eef_pos",
    "observations/robot0_eef_pos",
)
QUAT_KEYS = (
    "ee_quat",
    "eef_quat",
    "end_effector_quat",
    "robot0_eef_quat",
    "obs/ee_quat",
    "obs/eef_quat",
    "obs/robot0_eef_quat",
    "observations/ee_quat",
    "observations/eef_quat",
    "observations/robot0_eef_quat",
)
ORI_KEYS = (
    "ee_ori",
    "eef_ori",
    "end_effector_ori",
    "robot0_eef_ori",
    "obs/ee_ori",
    "obs/eef_ori",
    "obs/robot0_eef_ori",
    "observations/ee_ori",
    "observations/eef_ori",
    "observations/robot0_eef_ori",
)
GRIPPER_KEYS = (
    "gripper",
    "gripper_states",
    "gripper_state",
    "robot0_gripper_qpos",
    "obs/gripper",
    "obs/gripper_states",
    "obs/gripper_state",
    "obs/robot0_gripper_qpos",
    "observations/gripper",
    "observations/gripper_states",
    "observations/robot0_gripper_qpos",
)


def _candidate_files(input_path: Path) -> list[Path]:
    if input_path.is_file():
        return [input_path]
    files: list[Path] = []
    for suffix in ("*.npz", "*.hdf5", "*.h5"):
        files.extend(input_path.rglob(suffix))
    return sorted(files)


def _array_info(array: np.ndarray) -> dict[str, Any]:
    return {"shape": list(array.shape), "dtype": str(array.dtype)}


def _find_npz_key(data: Any, candidates: tuple[str, ...]) -> str | None:
    available = set(data.files)
    for key in candidates:
        if key in available:
            return key
    by_basename = {key.split("/")[-1]: key for key in data.files}
    for key in candidates:
        base = key.split("/")[-1]
        if base in by_basename:
            return by_basename[base]
    return None


def _squeeze_gripper(gripper: np.ndarray, timesteps: int) -> np.ndarray:
    gripper = np.asarray(gripper, dtype=float)
    if gripper.ndim == 1:
        return gripper
    if gripper.shape[0] == timesteps:
        return gripper.reshape(timesteps, -1).mean(axis=1)
    return gripper.reshape(-1)


def _parse_float_vector(value: Any) -> list[float] | None:
    if value is None:
        return None
    if isinstance(value, bytes):
        value = value.decode("utf-8")
    try:
        return [float(item) for item in str(value).split()]
    except ValueError:
        return None


def _yaw_from_wxyz_quat(quat: list[float] | None) -> float:
    if quat is None or len(quat) != 4:
        return 0.0
    w, x, y, z = quat
    return float(np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)))


def _source_robot_base_metadata(group: Any) -> dict[str, Any]:
    model_file = group.attrs.get("model_file")
    if model_file is None:
        return {}
    if isinstance(model_file, bytes):
        model_file = model_file.decode("utf-8")
    try:
        root = ElementTree.fromstring(str(model_file))
    except ElementTree.ParseError:
        return {}
    base_body = root.find(".//body[@name='robot0_base']")
    if base_body is None:
        return {}

    base_xyz = _parse_float_vector(base_body.attrib.get("pos"))
    if base_xyz is None or len(base_xyz) != 3:
        return {}
    base_quat = _parse_float_vector(base_body.attrib.get("quat"))
    metadata: dict[str, Any] = {
        "source_robot": "Panda",
        "source_robot_base_body": "robot0_base",
        "source_robot_base_xyz": [round(value, 6) for value in base_xyz],
        "source_robot_base_yaw": round(_yaw_from_wxyz_quat(base_quat), 6),
    }
    if base_quat is not None and len(base_quat) == 4:
        metadata["source_robot_base_quat_wxyz"] = [round(value, 6) for value in base_quat]
    return metadata


def _audit_npz(path: Path) -> dict[str, Any]:
    with np.load(path, allow_pickle=True) as data:
        arrays = {key: _array_info(np.asarray(data[key])) for key in data.files}
        return {
            "path": str(path),
            "kind": "npz",
            "arrays": arrays,
            "position_key": _find_npz_key(data, POSITION_KEYS),
            "quat_key": _find_npz_key(data, QUAT_KEYS),
            "ori_key": _find_npz_key(data, ORI_KEYS),
            "gripper_key": _find_npz_key(data, GRIPPER_KEYS),
        }


def _import_h5py():
    try:
        import h5py  # type: ignore
    except ImportError:
        return None
    return h5py


def _walk_h5_datasets(group: Any, prefix: str = "") -> dict[str, Any]:
    datasets: dict[str, Any] = {}
    for key, value in group.items():
        path = f"{prefix}/{key}" if prefix else key
        if hasattr(value, "shape"):
            datasets[path] = {"shape": list(value.shape), "dtype": str(value.dtype)}
        elif hasattr(value, "items"):
            datasets.update(_walk_h5_datasets(value, path))
    return datasets


def _find_h5_dataset(group: Any, candidates: tuple[str, ...]) -> Any | None:
    for key in candidates:
        if key in group:
            return group[key]
    datasets = _walk_h5_datasets(group)
    candidate_bases = {key.split("/")[-1] for key in candidates}
    for path in datasets:
        if path.split("/")[-1] in candidate_bases:
            return group[path]
    return None


def _audit_hdf5(path: Path) -> dict[str, Any]:
    h5py = _import_h5py()
    if h5py is None:
        return {
            "path": str(path),
            "kind": "hdf5",
            "error": "h5py is not installed; install with `pip install -e .[libero]`.",
        }
    with h5py.File(path, "r") as handle:
        datasets = _walk_h5_datasets(handle)
        return {
            "path": str(path),
            "kind": "hdf5",
            "datasets": datasets,
            "position_candidates": [name for name in datasets if name.split("/")[-1] in {key.split("/")[-1] for key in POSITION_KEYS}],
            "quat_candidates": [name for name in datasets if name.split("/")[-1] in {key.split("/")[-1] for key in QUAT_KEYS}],
            "ori_candidates": [name for name in datasets if name.split("/")[-1] in {key.split("/")[-1] for key in ORI_KEYS}],
            "gripper_candidates": [name for name in datasets if name.split("/")[-1] in {key.split("/")[-1] for key in GRIPPER_KEYS}],
        }


def audit_source(input_path: str | Path | None) -> dict[str, Any]:
    if input_path is None:
        return {
            "input_path": None,
            "exists": False,
            "files": [],
            "note": "No LIBERO source path was provided.",
        }
    path = Path(input_path)
    if not path.exists():
        return {
            "input_path": str(path),
            "exists": False,
            "files": [],
            "note": "Input path does not exist.",
        }

    files = _candidate_files(path)
    file_audits = []
    for file_path in files:
        if file_path.suffix == ".npz":
            file_audits.append(_audit_npz(file_path))
        elif file_path.suffix in {".hdf5", ".h5"}:
            file_audits.append(_audit_hdf5(file_path))
    return {
        "input_path": str(path),
        "exists": True,
        "num_candidate_files": len(files),
        "files": file_audits,
    }


def write_audit_markdown(audit: dict[str, Any], output_path: str | Path) -> Path:
    output_path = Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    lines = [
        "# LIBERO Data Audit",
        "",
        f"- Input path: `{audit.get('input_path')}`",
        f"- Exists: `{audit.get('exists')}`",
        f"- Candidate files: `{audit.get('num_candidate_files', len(audit.get('files', [])))}`",
        "",
    ]
    if audit.get("note"):
        lines.extend(["## Note", "", str(audit["note"]), ""])

    for item in audit.get("files", [])[:25]:
        lines.extend([f"## {item.get('kind', 'file').upper()}: `{item.get('path')}`", ""])
        if item.get("error"):
            lines.extend([f"- Error: {item['error']}", ""])
            continue
        if item.get("kind") == "npz":
            lines.extend(
                [
                    f"- Position key: `{item.get('position_key')}`",
                    f"- Orientation key: `{item.get('quat_key')}`",
                    f"- Orientation-vector key: `{item.get('ori_key')}`",
                    f"- Gripper key: `{item.get('gripper_key')}`",
                    "",
                    "Arrays:",
                    "",
                ]
            )
            for key, info in item.get("arrays", {}).items():
                lines.append(f"- `{key}`: shape={info['shape']}, dtype={info['dtype']}")
            lines.append("")
        elif item.get("kind") == "hdf5":
            lines.extend(
                [
                    f"- Position candidates: `{item.get('position_candidates')}`",
                    f"- Orientation candidates: `{item.get('quat_candidates')}`",
                    f"- Orientation-vector candidates: `{item.get('ori_candidates')}`",
                    f"- Gripper candidates: `{item.get('gripper_candidates')}`",
                    "",
                    "Datasets:",
                    "",
                ]
            )
            for key, info in list(item.get("datasets", {}).items())[:80]:
                lines.append(f"- `{key}`: shape={info['shape']}, dtype={info['dtype']}")
            lines.append("")

    output_path.write_text("\n".join(lines).rstrip() + "\n", encoding="utf-8")
    return output_path


def _extract_npz_file(path: Path, output_root: Path, coordinate_frame: str) -> list[TrajectoryRecord]:
    with np.load(path, allow_pickle=True) as data:
        pos_key = _find_npz_key(data, POSITION_KEYS)
        if pos_key is None:
            return []
        ee_pos = np.asarray(data[pos_key], dtype=float)
        if ee_pos.ndim != 2 or ee_pos.shape[1] != 3:
            return []
        quat_key = _find_npz_key(data, QUAT_KEYS)
        ee_quat = np.asarray(data[quat_key], dtype=float) if quat_key is not None else None
        ori_key = _find_npz_key(data, ORI_KEYS)
        ee_ori = np.asarray(data[ori_key], dtype=float) if ori_key is not None else None
        gripper_key = _find_npz_key(data, GRIPPER_KEYS)
        gripper = _squeeze_gripper(np.asarray(data[gripper_key]), len(ee_pos)) if gripper_key is not None else None

    suite = path.parent.parent.name if path.parent.parent != path.parent else "libero_unknown_suite"
    task = path.parent.name
    episode_id = path.stem
    out_path = save_processed_trajectory(
        output_root,
        suite=suite,
        task=task,
        episode_id=episode_id,
        ee_pos=ee_pos,
        ee_quat=ee_quat,
        ee_ori=ee_ori,
        gripper=gripper,
        metadata={
            "source_file": str(path),
            "coordinate_frame": coordinate_frame,
            "source": "LIBERO",
        },
    )
    return [load_trajectory_npz(out_path)]


def _hdf5_episode_groups(handle: Any) -> list[tuple[str, Any]]:
    if "data" in handle and hasattr(handle["data"], "items"):
        return [(name, group) for name, group in handle["data"].items() if hasattr(group, "items")]
    return [(Path(str(handle.filename)).stem, handle)]


def _extract_hdf5_file(path: Path, output_root: Path, coordinate_frame: str) -> list[TrajectoryRecord]:
    h5py = _import_h5py()
    if h5py is None:
        raise ImportError("h5py is required to extract HDF5 demonstrations; install with `pip install -e .[libero]`.")

    records: list[TrajectoryRecord] = []
    with h5py.File(path, "r") as handle:
        for demo_name, group in _hdf5_episode_groups(handle):
            pos_dataset = _find_h5_dataset(group, POSITION_KEYS)
            if pos_dataset is None:
                continue
            ee_pos = np.asarray(pos_dataset, dtype=float)
            if ee_pos.ndim != 2 or ee_pos.shape[1] != 3:
                continue
            quat_dataset = _find_h5_dataset(group, QUAT_KEYS)
            ee_quat = np.asarray(quat_dataset, dtype=float) if quat_dataset is not None else None
            ori_dataset = _find_h5_dataset(group, ORI_KEYS)
            ee_ori = np.asarray(ori_dataset, dtype=float) if ori_dataset is not None else None
            gripper_dataset = _find_h5_dataset(group, GRIPPER_KEYS)
            gripper = _squeeze_gripper(np.asarray(gripper_dataset), len(ee_pos)) if gripper_dataset is not None else None
            task_name = str(group.attrs.get("task", path.stem))
            language = group.attrs.get("language_instruction")
            if isinstance(language, bytes):
                language = language.decode("utf-8")
            metadata = {
                "source_file": str(path),
                "coordinate_frame": coordinate_frame,
                "source": "LIBERO",
            }
            metadata.update(_source_robot_base_metadata(group))
            out_path = save_processed_trajectory(
                output_root,
                suite=path.parent.name or "libero_hdf5",
                task=task_name,
                episode_id=demo_name,
                ee_pos=ee_pos,
                ee_quat=ee_quat,
                ee_ori=ee_ori,
                gripper=gripper,
                language_instruction=str(language) if language is not None else None,
                metadata=metadata,
            )
            records.append(load_trajectory_npz(out_path))
    return records


def extract_source_to_processed(
    input_path: str | Path,
    output_root: str | Path,
    coordinate_frame: str,
    allow_unknown_frame: bool = False,
) -> list[TrajectoryRecord]:
    if not coordinate_frame or coordinate_frame == "unknown":
        if not allow_unknown_frame:
            raise ValueError("coordinate_frame must be explicit before extracting trajectories for IK evaluation")

    input_path = Path(input_path)
    output_root = Path(output_root)
    records: list[TrajectoryRecord] = []
    for file_path in _candidate_files(input_path):
        if file_path.suffix == ".npz":
            records.extend(_extract_npz_file(file_path, output_root, coordinate_frame))
        elif file_path.suffix in {".hdf5", ".h5"}:
            records.extend(_extract_hdf5_file(file_path, output_root, coordinate_frame))

    if not records:
        raise RuntimeError(f"No extractable EE trajectories found in {input_path}")

    write_metadata(
        output_root,
        records,
        {
            "source": "LIBERO",
            "robot_in_source": "Panda or source embodiment if known",
            "coordinate_frame": coordinate_frame,
            "note": "Extracted with best-effort field matching; verify frame semantics before using results.",
        },
    )
    return records


def write_stats(records: list[TrajectoryRecord], output_path: str | Path, extra: dict[str, Any] | None = None) -> Path:
    output_path = Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    stats = summarize_dataset(records)
    stats.update(extra or {})
    output_path.write_text(json.dumps(stats, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return output_path


__all__ = [
    "audit_source",
    "create_synthetic_dataset",
    "extract_source_to_processed",
    "write_audit_markdown",
    "write_stats",
]
