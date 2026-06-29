from __future__ import annotations

import json
import re
from dataclasses import dataclass, field
from dataclasses import replace
from pathlib import Path
from typing import Any, Iterable

import numpy as np


def slugify(value: str) -> str:
    value = value.strip().lower()
    value = re.sub(r"[^a-z0-9]+", "_", value)
    return value.strip("_") or "unknown"


def scalar_value(value: Any) -> Any:
    if isinstance(value, np.ndarray):
        if value.shape == ():
            return value.item()
        if value.size == 1:
            return value.reshape(-1)[0].item()
    return value


@dataclass(frozen=True)
class TrajectoryRecord:
    suite: str
    task: str
    episode_id: str
    path: Path
    ee_pos: np.ndarray
    ee_quat: np.ndarray | None = None
    ee_ori: np.ndarray | None = None
    gripper: np.ndarray | None = None
    task_id: str | int | None = None
    language_instruction: str | None = None
    metadata: dict[str, Any] = field(default_factory=dict)

    @property
    def num_waypoints(self) -> int:
        return int(self.ee_pos.shape[0])

    @property
    def contains_orientation(self) -> bool:
        quat_available = self.ee_quat is not None and len(self.ee_quat) == self.num_waypoints
        ori_available = self.ee_ori is not None and len(self.ee_ori) == self.num_waypoints
        return quat_available or ori_available

    @property
    def contains_gripper(self) -> bool:
        return self.gripper is not None and len(self.gripper) == self.num_waypoints

    def downsample(self, max_waypoints: int | None) -> "TrajectoryRecord":
        if max_waypoints is None or max_waypoints <= 0 or self.num_waypoints <= max_waypoints:
            return self
        indexes = np.linspace(0, self.num_waypoints - 1, max_waypoints, dtype=int)
        metadata = dict(self.metadata)
        metadata.update(
            {
                "downsampled": True,
                "original_num_waypoints": self.num_waypoints,
                "downsampled_num_waypoints": int(len(indexes)),
            }
        )
        return replace(
            self,
            ee_pos=self.ee_pos[indexes],
            ee_quat=self.ee_quat[indexes] if self.ee_quat is not None and len(self.ee_quat) == self.num_waypoints else self.ee_quat,
            ee_ori=self.ee_ori[indexes] if self.ee_ori is not None and len(self.ee_ori) == self.num_waypoints else self.ee_ori,
            gripper=self.gripper[indexes] if self.gripper is not None and len(self.gripper) == self.num_waypoints else self.gripper,
            metadata=metadata,
        )


def _metadata_from_npz(data: Any) -> dict[str, Any]:
    if "__metadata_json" not in data.files:
        return {}
    raw = scalar_value(data["__metadata_json"])
    if isinstance(raw, bytes):
        raw = raw.decode("utf-8")
    try:
        return json.loads(str(raw))
    except json.JSONDecodeError:
        return {}


def load_trajectory_npz(path: str | Path) -> TrajectoryRecord:
    path = Path(path)
    with np.load(path, allow_pickle=True) as data:
        if "ee_pos" not in data.files:
            raise ValueError(f"{path} does not contain required array 'ee_pos'")
        ee_pos = np.asarray(data["ee_pos"], dtype=float)
        if ee_pos.ndim != 2 or ee_pos.shape[1] != 3:
            raise ValueError(f"{path} has invalid ee_pos shape {ee_pos.shape}; expected [T, 3]")

        ee_quat = np.asarray(data["ee_quat"], dtype=float) if "ee_quat" in data.files else None
        ee_ori = np.asarray(data["ee_ori"], dtype=float) if "ee_ori" in data.files else None
        gripper = np.asarray(data["gripper"], dtype=float).reshape(-1) if "gripper" in data.files else None
        task_id = scalar_value(data["task_id"]) if "task_id" in data.files else None
        episode_id = str(scalar_value(data["episode_id"])) if "episode_id" in data.files else path.stem
        instruction = str(scalar_value(data["language_instruction"])) if "language_instruction" in data.files else None
        metadata = _metadata_from_npz(data)

    parts = path.parts
    suite = metadata.get("suite")
    task = metadata.get("task")
    if suite is None or task is None:
        try:
            trajectories_index = parts.index("trajectories")
            suite = suite or parts[trajectories_index + 1]
            task = task or parts[trajectories_index + 2]
        except (ValueError, IndexError):
            suite = suite or "unknown_suite"
            task = task or path.parent.name

    return TrajectoryRecord(
        suite=str(suite),
        task=str(task),
        episode_id=episode_id,
        path=path,
        ee_pos=ee_pos,
        ee_quat=ee_quat,
        ee_ori=ee_ori,
        gripper=gripper,
        task_id=task_id,
        language_instruction=instruction,
        metadata=metadata,
    )


def iter_trajectory_files(root: str | Path) -> Iterable[Path]:
    root = Path(root)
    if not root.exists():
        return []
    return sorted((root / "trajectories").glob("*/*/*.npz"))


def load_dataset(root: str | Path) -> list[TrajectoryRecord]:
    records = [load_trajectory_npz(path) for path in iter_trajectory_files(root)]
    if not records:
        raise FileNotFoundError(f"No processed trajectory files found under {Path(root) / 'trajectories'}")
    return records


def save_processed_trajectory(
    root: str | Path,
    suite: str,
    task: str,
    episode_id: str | int,
    ee_pos: np.ndarray,
    ee_quat: np.ndarray | None = None,
    ee_ori: np.ndarray | None = None,
    gripper: np.ndarray | None = None,
    task_id: str | int | None = None,
    language_instruction: str | None = None,
    metadata: dict[str, Any] | None = None,
) -> Path:
    root = Path(root)
    suite_slug = slugify(suite)
    task_slug = slugify(task)
    episode_slug = slugify(str(episode_id))
    out_dir = root / "trajectories" / suite_slug / task_slug
    out_dir.mkdir(parents=True, exist_ok=True)
    path = out_dir / f"episode_{episode_slug}.npz"

    ee_pos = np.asarray(ee_pos, dtype=float)
    if ee_pos.ndim != 2 or ee_pos.shape[1] != 3:
        raise ValueError(f"ee_pos must have shape [T, 3], got {ee_pos.shape}")

    payload: dict[str, Any] = {
        "ee_pos": ee_pos,
        "task_id": np.array(task_id if task_id is not None else task_slug),
        "episode_id": np.array(str(episode_id)),
    }
    if ee_quat is not None:
        payload["ee_quat"] = np.asarray(ee_quat, dtype=float)
    if ee_ori is not None:
        payload["ee_ori"] = np.asarray(ee_ori, dtype=float)
    if gripper is not None:
        payload["gripper"] = np.asarray(gripper, dtype=float).reshape(-1)
    if language_instruction is not None:
        payload["language_instruction"] = np.array(language_instruction)

    meta = dict(metadata or {})
    meta.update({"suite": suite_slug, "task": task_slug})
    payload["__metadata_json"] = np.array(json.dumps(meta, sort_keys=True))
    np.savez_compressed(path, **payload)
    return path


def summarize_dataset(records: list[TrajectoryRecord]) -> dict[str, Any]:
    if not records:
        return {
            "num_suites": 0,
            "num_tasks": 0,
            "num_episodes": 0,
            "num_waypoints": 0,
            "contains_orientation": False,
            "contains_gripper": False,
        }

    suites = {record.suite for record in records}
    tasks = {(record.suite, record.task) for record in records}
    points = np.concatenate([record.ee_pos for record in records], axis=0)
    return {
        "num_suites": len(suites),
        "num_tasks": len(tasks),
        "num_episodes": len(records),
        "num_waypoints": int(points.shape[0]),
        "contains_orientation": all(record.contains_orientation for record in records),
        "contains_gripper": all(record.contains_gripper for record in records),
        "position_bounds": {
            "min": points.min(axis=0).round(6).tolist(),
            "max": points.max(axis=0).round(6).tolist(),
        },
        "suites": sorted(suites),
        "tasks": sorted([f"{suite}/{task}" for suite, task in tasks]),
    }


def write_metadata(root: str | Path, records: list[TrajectoryRecord], extra: dict[str, Any] | None = None) -> Path:
    root = Path(root)
    root.mkdir(parents=True, exist_ok=True)
    metadata = {
        "source": "LIBERO",
        "robot_in_source": "Panda or source embodiment if known",
        "coordinate_frame": "unknown",
        "num_suites": 0,
        "num_tasks": 0,
        "num_episodes": 0,
        "contains_orientation": False,
        "contains_gripper": False,
    }
    metadata.update(summarize_dataset(records))
    metadata.update(extra or {})
    path = root / "metadata.json"
    path.write_text(json.dumps(metadata, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return path


def trajectory_point_cloud(records: list[TrajectoryRecord]) -> np.ndarray:
    if not records:
        return np.zeros((0, 3), dtype=float)
    return np.concatenate([record.ee_pos for record in records], axis=0)


def split_trajectories(
    records: list[TrajectoryRecord],
    train_fraction: float = 0.7,
    split_by: str = "episode",
    seed: int = 7,
) -> tuple[list[TrajectoryRecord], list[TrajectoryRecord]]:
    if not 0.0 < train_fraction < 1.0:
        raise ValueError("train_fraction must be between 0 and 1")
    rng = np.random.default_rng(seed)

    if split_by == "task":
        keys = sorted({(record.suite, record.task) for record in records})
        rng.shuffle(keys)
        cutoff = max(1, int(round(len(keys) * train_fraction)))
        train_keys = set(keys[:cutoff])
        train = [record for record in records if (record.suite, record.task) in train_keys]
        test = [record for record in records if (record.suite, record.task) not in train_keys]
    elif split_by == "episode":
        shuffled = list(records)
        rng.shuffle(shuffled)
        cutoff = max(1, int(round(len(shuffled) * train_fraction)))
        train = shuffled[:cutoff]
        test = shuffled[cutoff:]
    else:
        raise ValueError("split_by must be 'episode' or 'task'")

    if not test and len(train) > 1:
        test = [train.pop()]
    return train, test


def create_synthetic_dataset(
    output_root: str | Path,
    episodes_per_task: int = 4,
    timesteps: int = 48,
    seed: int = 7,
) -> list[TrajectoryRecord]:
    output_root = Path(output_root)
    rng = np.random.default_rng(seed)
    task_specs = [
        ("libero_synthetic_spatial", "pick_low_left_place_high_right", np.array([0.34, -0.22, 0.24]), 0.17),
        ("libero_synthetic_spatial", "pick_high_right_place_low_left", np.array([0.42, 0.20, 0.36]), 0.15),
        ("libero_synthetic_object", "wipe_front_arc", np.array([0.52, 0.00, 0.22]), 0.21),
        ("libero_synthetic_object", "drawer_pull_line", np.array([0.28, 0.18, 0.30]), 0.13),
    ]

    records: list[TrajectoryRecord] = []
    t = np.linspace(0.0, 1.0, timesteps)
    for task_index, (suite, task, center, radius) in enumerate(task_specs):
        for episode in range(episodes_per_task):
            phase = rng.uniform(-0.25, 0.25)
            jitter = rng.normal(0.0, 0.012, size=3)
            local_center = center + jitter
            if "line" in task:
                x = local_center[0] + np.linspace(-radius, radius, timesteps)
                y = local_center[1] + 0.035 * np.sin(2.0 * np.pi * (t + phase))
                z = local_center[2] + 0.05 * np.sin(np.pi * t)
            else:
                angle = np.pi * (0.15 + 0.85 * t + phase)
                x = local_center[0] + radius * np.cos(angle)
                y = local_center[1] + radius * np.sin(angle)
                z = local_center[2] + 0.06 * np.sin(np.pi * t) + 0.025 * t
            ee_pos = np.stack([x, y, z], axis=1)
            ee_pos += rng.normal(0.0, 0.004, size=ee_pos.shape)
            ee_pos[:, 2] = np.maximum(ee_pos[:, 2], 0.06)
            ee_quat = np.tile(np.array([1.0, 0.0, 0.0, 0.0]), (timesteps, 1))
            gripper = (np.sin(2.0 * np.pi * t + task_index) > 0).astype(float)
            episode_id = f"{task_index:02d}_{episode:03d}"
            metadata = {
                "source": "synthetic_libero_like_fixture",
                "coordinate_frame": "world",
                "robot_in_source": "synthetic_panda_proxy",
            }
            path = save_processed_trajectory(
                output_root,
                suite=suite,
                task=task,
                episode_id=episode_id,
                ee_pos=ee_pos,
                ee_quat=ee_quat,
                gripper=gripper,
                task_id=task_index,
                language_instruction=task.replace("_", " "),
                metadata=metadata,
            )
            records.append(load_trajectory_npz(path))

    write_metadata(
        output_root,
        records,
        {
            "source": "Synthetic LIBERO-like fixture",
            "robot_in_source": "synthetic_panda_proxy",
            "coordinate_frame": "world",
            "note": "Generated because real LIBERO demonstrations were not provided.",
        },
    )
    return records
