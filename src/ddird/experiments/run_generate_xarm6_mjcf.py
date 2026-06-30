from __future__ import annotations

import argparse
import subprocess
from pathlib import Path
from typing import Any
from xml.etree import ElementTree

import numpy as np

from ddird.experiments.common import write_json
from ddird.robots.mjcf_chain import serial_robot_from_mjcf_xml
from ddird.robots.tool_frames import tool_frame_metadata

JOINT_LIMITS = {
    "joint1": (-2.0 * np.pi, 2.0 * np.pi),
    "joint2": (-2.059, 2.0944),
    "joint3": (-3.927, 0.19198),
    "joint4": (-2.0 * np.pi, 2.0 * np.pi),
    "joint5": (-1.69297, np.pi),
    "joint6": (-2.0 * np.pi, 2.0 * np.pi),
}

DEFAULT_XARM_GRIPPER_TCP_OFFSET = (0.0, 0.0, 0.172)


def _load_xarm6_kinematics(path: Path) -> dict[str, dict[str, float]]:
    kinematics: dict[str, dict[str, float]] = {}
    current_joint: str | None = None
    for raw_line in path.read_text(encoding="utf-8").splitlines():
        line = raw_line.split("#", 1)[0].rstrip()
        if not line.strip():
            continue
        stripped = line.strip()
        if line.startswith("  ") and not line.startswith("    ") and stripped.endswith(":"):
            current_joint = stripped[:-1]
            kinematics[current_joint] = {}
            continue
        if line.startswith("    ") and current_joint is not None and ":" in stripped:
            key, value = stripped.split(":", 1)
            kinematics[current_joint][key] = float(value.strip())

    expected = [f"joint{index}" for index in range(1, 7)]
    missing = [joint for joint in expected if set(kinematics.get(joint, {})) != {"x", "y", "z", "roll", "pitch", "yaw"}]
    if missing:
        raise ValueError(f"Missing xArm6 kinematics entries for: {', '.join(missing)}")
    return {joint: kinematics[joint] for joint in expected}


def _rpy_to_quat_wxyz(roll: float, pitch: float, yaw: float) -> np.ndarray:
    cy = float(np.cos(yaw * 0.5))
    sy = float(np.sin(yaw * 0.5))
    cp = float(np.cos(pitch * 0.5))
    sp = float(np.sin(pitch * 0.5))
    cr = float(np.cos(roll * 0.5))
    sr = float(np.sin(roll * 0.5))
    return np.array(
        [
            cr * cp * cy + sr * sp * sy,
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
        ],
        dtype=float,
    )


def _format_vector(values: list[float] | np.ndarray) -> str:
    return " ".join(f"{float(value):.8g}" for value in values)


def _git_commit(path: Path) -> str | None:
    try:
        return subprocess.check_output(["git", "-C", str(path), "rev-parse", "HEAD"], text=True).strip()
    except (FileNotFoundError, subprocess.CalledProcessError):
        return None


def generate_xarm6_mjcf(
    kinematics_yaml: Path,
    output_xml: Path,
    robot_name: str = "xarm6_true",
    tcp_site: str = "link_tcp",
    tcp_offset: tuple[float, float, float] | list[float] | np.ndarray = DEFAULT_XARM_GRIPPER_TCP_OFFSET,
) -> dict[str, Any]:
    kinematics = _load_xarm6_kinematics(kinematics_yaml)
    tcp_offset_array = np.asarray(tcp_offset, dtype=float).reshape(3)
    root = ElementTree.Element("mujoco", {"model": robot_name})
    ElementTree.SubElement(root, "compiler", {"angle": "radian", "autolimits": "true"})
    worldbody = ElementTree.SubElement(root, "worldbody")
    parent = ElementTree.SubElement(worldbody, "body", {"name": "link_base"})

    for index in range(1, 7):
        joint_name = f"joint{index}"
        params = kinematics[joint_name]
        quat = _rpy_to_quat_wxyz(params["roll"], params["pitch"], params["yaw"])
        body = ElementTree.SubElement(
            parent,
            "body",
            {
                "name": f"link{index}",
                "pos": _format_vector([params["x"], params["y"], params["z"]]),
                "quat": _format_vector(quat),
            },
        )
        ElementTree.SubElement(
            body,
            "inertial",
            {
                "pos": "0 0 0",
                "mass": "1",
                "diaginertia": "0.001 0.001 0.001",
            },
        )
        lower, upper = JOINT_LIMITS[joint_name]
        ElementTree.SubElement(
            body,
            "joint",
            {
                "name": joint_name,
                "axis": "0 0 1",
                "range": _format_vector([lower, upper]),
            },
        )
        parent = body

    ElementTree.SubElement(parent, "site", {"name": tcp_site, "pos": _format_vector(tcp_offset_array)})
    ElementTree.indent(root, space="  ")
    output_xml.parent.mkdir(parents=True, exist_ok=True)
    ElementTree.ElementTree(root).write(output_xml, encoding="utf-8", xml_declaration=True)

    xarm_ros_root = next((parent for parent in kinematics_yaml.parents if parent.name == "xarm_ros"), None)
    robot = serial_robot_from_mjcf_xml(
        ElementTree.tostring(root, encoding="unicode"),
        name=robot_name,
        base_body_name="link_base",
        target_site=tcp_site,
        target_body="link6",
    )
    metadata = {
        "robot_name": robot_name,
        "source": "xArm-Developer/xarm_ros",
        "source_root": str(xarm_ros_root) if xarm_ros_root is not None else None,
        "source_commit": _git_commit(xarm_ros_root) if xarm_ros_root is not None else None,
        "kinematics_yaml": str(kinematics_yaml),
        "xacro_source": str(kinematics_yaml.parents[3] / "urdf/xarm6/xarm6.urdf.xacro"),
        "output_xml": str(output_xml),
        "base_body": "link_base",
        "target_site": tcp_site,
        "target_body": "link6",
        "tcp_offset_xyz": [round(float(value), 8) for value in tcp_offset_array],
        "tcp_offset_source": "xarm_description/urdf/gripper/xarm_gripper.urdf.xacro joint_tcp origin",
        "tool_frame": tool_frame_metadata(
            robot,
            base_body="link_base",
            target_site=tcp_site,
            target_body="link6",
            model_path=output_xml,
        ),
        "joint_names": list(JOINT_LIMITS),
        "joint_limits": {joint: [round(float(low), 8), round(float(high), 8)] for joint, (low, high) in JOINT_LIMITS.items()},
    }
    write_json(output_xml.parent / "model_metadata.json", metadata)
    return metadata


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Generate an ArmFoundry-evaluable xArm6 MJCF from official xArm ROS kinematics.")
    parser.add_argument("--xarm-ros-root", default="data/armforge/models/xarm_ros")
    parser.add_argument("--kinematics-yaml", default=None)
    parser.add_argument("--outputs", default="outputs/generated_models/xarm6_true")
    parser.add_argument("--robot-name", default="xarm6_true")
    parser.add_argument("--tcp-site", default="link_tcp")
    parser.add_argument(
        "--tcp-offset",
        nargs=3,
        type=float,
        default=DEFAULT_XARM_GRIPPER_TCP_OFFSET,
        metavar=("X", "Y", "Z"),
        help=(
            "TCP site offset in link6 local coordinates. Defaults to the official xArm gripper "
            "link_tcp offset, 0 0 0.172. Use 0 0 0 for wrist-origin diagnostics."
        ),
    )
    return parser


def main() -> None:
    args = build_parser().parse_args()
    xarm_ros_root = Path(args.xarm_ros_root)
    kinematics_yaml = (
        Path(args.kinematics_yaml)
        if args.kinematics_yaml is not None
        else xarm_ros_root / "xarm_description/config/kinematics/default/xarm6_default_kinematics.yaml"
    )
    output_xml = Path(args.outputs) / f"{args.robot_name}.xml"
    metadata = generate_xarm6_mjcf(
        kinematics_yaml=kinematics_yaml,
        output_xml=output_xml,
        robot_name=args.robot_name,
        tcp_site=args.tcp_site,
        tcp_offset=args.tcp_offset,
    )
    print(f"Wrote {metadata['output_xml']} from {metadata['kinematics_yaml']}")


if __name__ == "__main__":
    main()
