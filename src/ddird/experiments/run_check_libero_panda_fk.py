from __future__ import annotations

import argparse
from pathlib import Path
from typing import Any

import numpy as np

from ddird.experiments.common import write_csv, write_json
from ddird.robots.mjcf_chain import panda_from_mjcf_xml


def _import_h5py():
    try:
        import h5py  # type: ignore
    except ImportError as exc:
        raise ImportError("h5py is required for LIBERO HDF5 FK checks; install with `uv sync --extra libero`.") from exc
    return h5py


def _candidate_files(path: Path) -> list[Path]:
    if path.is_file():
        return [path]
    return sorted([*path.rglob("*.hdf5"), *path.rglob("*.h5")])


def _episode_groups(handle: Any) -> list[tuple[str, Any]]:
    if "data" in handle and hasattr(handle["data"], "items"):
        return [(name, group) for name, group in handle["data"].items() if hasattr(group, "items")]
    return [(Path(str(handle.filename)).stem, handle)]


def _find_dataset(group: Any, path: str) -> Any | None:
    if path in group:
        return group[path]
    return None


def _summary(values: np.ndarray) -> dict[str, float]:
    values = np.asarray(values, dtype=float)
    if values.size == 0:
        return {"mean": 0.0, "max": 0.0, "p95": 0.0, "p99": 0.0}
    return {
        "mean": round(float(np.mean(values)), 8),
        "max": round(float(np.max(values)), 8),
        "p95": round(float(np.quantile(values, 0.95)), 8),
        "p99": round(float(np.quantile(values, 0.99)), 8),
    }


def _check_demo(group: Any, file_path: Path, demo_name: str, target_site: str, target_body: str) -> dict[str, Any] | None:
    joint_dataset = _find_dataset(group, "obs/joint_states")
    ee_dataset = _find_dataset(group, "obs/ee_pos")
    model_file = group.attrs.get("model_file")
    if joint_dataset is None or ee_dataset is None or model_file is None:
        return None
    if isinstance(model_file, bytes):
        model_file = model_file.decode("utf-8")

    robot = panda_from_mjcf_xml(str(model_file), target_site=target_site, target_body=target_body)
    joint_states = np.asarray(joint_dataset, dtype=float)
    ee_pos = np.asarray(ee_dataset, dtype=float)
    if joint_states.ndim != 2 or ee_pos.ndim != 2 or ee_pos.shape[1] != 3:
        return None

    q = joint_states[:, : robot.dof]
    achieved = np.asarray([robot.end_effector_position(row) for row in q], dtype=float)
    errors = np.linalg.norm(achieved - ee_pos[: len(achieved)], axis=1)
    stats = _summary(errors)
    return {
        "file": str(file_path),
        "suite": file_path.parent.name,
        "task": file_path.stem,
        "demo": demo_name,
        "num_samples": int(len(errors)),
        "target_frame": robot.target_frame,
        "base_xyz": robot.base_xyz.round(6).tolist(),
        "mean_position_error": stats["mean"],
        "max_position_error": stats["max"],
        "p95_position_error": stats["p95"],
        "p99_position_error": stats["p99"],
    }


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Check LIBERO source Panda FK against recorded obs/ee_pos.")
    parser.add_argument("--input", required=True, help="Raw LIBERO HDF5 file or directory.")
    parser.add_argument("--outputs", default="outputs/libero_panda_fk_check")
    parser.add_argument("--target-site", default="gripper0_grip_site")
    parser.add_argument("--target-body", default="gripper0_eef")
    parser.add_argument("--max-files", type=int, default=None)
    parser.add_argument("--max-demos-per-file", type=int, default=None)
    return parser


def main() -> None:
    args = build_parser().parse_args()
    h5py = _import_h5py()
    files = _candidate_files(Path(args.input))
    if args.max_files is not None:
        files = files[: args.max_files]
    if not files:
        raise SystemExit(f"No HDF5 files found under {args.input}")

    rows = []
    for file_path in files:
        with h5py.File(file_path, "r") as handle:
            groups = _episode_groups(handle)
            if args.max_demos_per_file is not None:
                groups = groups[: args.max_demos_per_file]
            for demo_name, group in groups:
                row = _check_demo(group, file_path, demo_name, args.target_site, args.target_body)
                if row is not None:
                    rows.append(row)

    if not rows:
        raise SystemExit("No checkable demos found; expected obs/joint_states, obs/ee_pos, and model_file attrs.")

    errors = np.asarray([float(row["mean_position_error"]) for row in rows], dtype=float)
    summary = {
        "num_demos": len(rows),
        "target_site": args.target_site,
        "target_body": args.target_body,
        "demo_mean_error": _summary(errors),
    }
    write_csv(Path(args.outputs) / "panda_fk_check.csv", rows)
    write_json(Path(args.outputs) / "panda_fk_summary.json", summary)
    print(
        f"Checked {len(rows)} demos; mean demo error={summary['demo_mean_error']['mean']:.6f}, "
        f"max demo mean error={summary['demo_mean_error']['max']:.6f}"
    )


if __name__ == "__main__":
    main()
