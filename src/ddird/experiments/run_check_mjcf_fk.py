from __future__ import annotations

import argparse
from pathlib import Path
from typing import Any

import numpy as np

from ddird.experiments.common import write_csv, write_json
from ddird.robots.mjcf_chain import MJCFSerialRobot, serial_robot_from_mjcf_xml
from ddird.robots.tool_frames import tool_frame_metadata


def _import_mujoco():
    try:
        import mujoco  # type: ignore
    except ImportError as exc:
        raise ImportError("mujoco is required for MJCF FK checks; install with `uv sync --extra mujoco`.") from exc
    return mujoco


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


def _sample_joint_states(robot: MJCFSerialRobot, num_samples: int, seed: int) -> list[np.ndarray]:
    samples = [robot.neutral_q.copy()]
    if num_samples <= 1:
        return samples
    limits = robot.joint_limits
    rng = np.random.default_rng(seed)
    random_count = max(0, num_samples - len(samples))
    for row in rng.uniform(limits[:, 0], limits[:, 1], size=(random_count, robot.dof)):
        samples.append(np.asarray(row, dtype=float))
    return samples


def _append_keyframe_samples(mujoco: Any, model: Any, robot: MJCFSerialRobot, samples: list[np.ndarray]) -> None:
    if getattr(model, "nkey", 0) <= 0:
        return
    joint_addresses = _joint_qpos_addresses(mujoco, model, robot.joint_names)
    for key_index in range(model.nkey):
        key_qpos = np.asarray(model.key_qpos[key_index], dtype=float)
        samples.append(np.asarray([key_qpos[address] for address in joint_addresses], dtype=float))


def _joint_qpos_addresses(mujoco: Any, model: Any, joint_names: list[str]) -> list[int]:
    addresses = []
    for joint_name in joint_names:
        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
        if joint_id < 0:
            raise ValueError(f"MuJoCo model does not contain joint {joint_name!r}")
        addresses.append(int(model.jnt_qposadr[joint_id]))
    return addresses


def _mujoco_target_position(mujoco: Any, model: Any, data: Any, target_site: str, target_body: str) -> np.ndarray:
    site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, target_site)
    if site_id >= 0:
        return np.asarray(data.site_xpos[site_id], dtype=float).copy()
    body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, target_body)
    if body_id < 0:
        raise ValueError(f"MuJoCo model does not contain target site {target_site!r} or body {target_body!r}")
    return np.asarray(data.xpos[body_id], dtype=float).copy()


def _mujoco_fk_positions(
    mujoco: Any,
    model: Any,
    robot: MJCFSerialRobot,
    q_samples: list[np.ndarray],
    target_site: str,
    target_body: str,
) -> list[np.ndarray]:
    data = mujoco.MjData(model)
    joint_addresses = _joint_qpos_addresses(mujoco, model, robot.joint_names)
    positions = []
    for q in q_samples:
        data.qpos[:] = 0.0
        for value, address in zip(q, joint_addresses, strict=True):
            data.qpos[address] = float(value)
        mujoco.mj_forward(model, data)
        positions.append(_mujoco_target_position(mujoco, model, data, target_site, target_body))
    return positions


def build_rows_and_summary(
    robot: MJCFSerialRobot,
    q_samples: list[np.ndarray],
    reference_positions: list[np.ndarray],
    mjcf_path: str,
    base_body: str,
    target_site: str,
    target_body: str,
) -> tuple[list[dict[str, Any]], dict[str, Any]]:
    rows = []
    for index, (q, reference_pos) in enumerate(zip(q_samples, reference_positions, strict=True)):
        armfoundry_pos = robot.end_effector_position(q)
        error = float(np.linalg.norm(armfoundry_pos - reference_pos))
        rows.append(
            {
                "sample": index,
                "robot_name": robot.name,
                "q": np.asarray(q, dtype=float).round(8).tolist(),
                "armfoundry_position": armfoundry_pos.round(8).tolist(),
                "mujoco_position": np.asarray(reference_pos, dtype=float).round(8).tolist(),
                "position_error": round(error, 10),
            }
        )

    errors = np.asarray([row["position_error"] for row in rows], dtype=float)
    summary = {
        "mjcf": mjcf_path,
        "robot_name": robot.name,
        "base_body": base_body,
        "target_site": target_site,
        "target_body": target_body,
        "target_frame": robot.target_frame,
        "dof": robot.dof,
        "joint_names": robot.joint_names,
        "tool_frame": tool_frame_metadata(
            robot,
            base_body=base_body,
            target_site=target_site,
            target_body=target_body,
            model_path=mjcf_path,
        ),
        "num_samples": len(rows),
        "position_error": _summary(errors),
    }
    return rows, summary


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Check ArmFoundry MJCF FK against MuJoCo FK.")
    parser.add_argument("--mjcf", required=True, help="Path to a MuJoCo XML model.")
    parser.add_argument("--robot-name", required=True, help="Name to use in output files.")
    parser.add_argument("--base-body", required=True, help="Body name for the serial-chain base.")
    parser.add_argument("--target-site", default="tool0", help="Preferred TCP site name.")
    parser.add_argument("--target-body", default="tool0", help="Fallback TCP body name.")
    parser.add_argument("--outputs", default="outputs/mjcf_fk_check")
    parser.add_argument("--num-samples", type=int, default=64)
    parser.add_argument("--seed", type=int, default=7)
    return parser


def main() -> None:
    args = build_parser().parse_args()
    mujoco = _import_mujoco()
    mjcf_path = Path(args.mjcf)
    robot = serial_robot_from_mjcf_xml(
        mjcf_path.read_text(encoding="utf-8"),
        name=args.robot_name,
        base_body_name=args.base_body,
        target_site=args.target_site,
        target_body=args.target_body,
    )
    model = mujoco.MjModel.from_xml_path(str(mjcf_path))
    samples = _sample_joint_states(robot, args.num_samples, args.seed)
    _append_keyframe_samples(mujoco, model, robot, samples)
    reference_positions = _mujoco_fk_positions(mujoco, model, robot, samples, args.target_site, args.target_body)
    rows, summary = build_rows_and_summary(
        robot,
        samples,
        reference_positions,
        str(mjcf_path),
        args.base_body,
        args.target_site,
        args.target_body,
    )

    outputs = Path(args.outputs)
    write_csv(outputs / "mjcf_fk_check.csv", rows)
    write_json(outputs / "mjcf_fk_summary.json", summary)
    stats = summary["position_error"]
    print(
        f"Checked {robot.name} FK on {len(rows)} samples; "
        f"mean error={stats['mean']:.8f}, max error={stats['max']:.8f}"
    )


if __name__ == "__main__":
    main()
