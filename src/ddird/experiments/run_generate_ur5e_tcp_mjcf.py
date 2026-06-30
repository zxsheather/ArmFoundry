from __future__ import annotations

import argparse
import os
from pathlib import Path
from typing import Any
from xml.etree import ElementTree

import numpy as np

from ddird.experiments.common import write_json
from ddird.robots.mjcf_chain import serial_robot_from_mjcf_xml
from ddird.robots.tool_frames import tool_frame_metadata

DEFAULT_UR5E_MJCF = Path("data/armforge/models/mujoco_menagerie/universal_robots_ur5e/ur5e.xml")
DEFAULT_ROBOTIQ_MJCF = Path("data/armforge/models/mujoco_menagerie/robotiq_2f85/2f85.xml")
DEFAULT_OUTPUTS = Path("outputs/generated_models/ur5e_robotiq2f85_tcp_proxy")
DEFAULT_TCP_SITE = "ur5e_robotiq2f85_tcp"


def _parse_vector(value: str | None, default: tuple[float, ...]) -> np.ndarray:
    if value is None:
        return np.asarray(default, dtype=float)
    return np.asarray([float(item) for item in value.split()], dtype=float)


def _quat_wxyz_to_matrix(quat: np.ndarray) -> np.ndarray:
    quat = np.asarray(quat, dtype=float).reshape(4)
    norm = float(np.linalg.norm(quat))
    if norm == 0.0:
        return np.eye(3)
    w, x, y, z = quat / norm
    return np.array(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
        ],
        dtype=float,
    )


def _matrix_to_quat_wxyz(rotation: np.ndarray) -> np.ndarray:
    rotation = np.asarray(rotation, dtype=float).reshape(3, 3)
    trace = float(np.trace(rotation))
    if trace > 0.0:
        scale = np.sqrt(trace + 1.0) * 2.0
        quat = np.array(
            [
                0.25 * scale,
                (rotation[2, 1] - rotation[1, 2]) / scale,
                (rotation[0, 2] - rotation[2, 0]) / scale,
                (rotation[1, 0] - rotation[0, 1]) / scale,
            ],
            dtype=float,
        )
    else:
        index = int(np.argmax(np.diag(rotation)))
        if index == 0:
            scale = np.sqrt(1.0 + rotation[0, 0] - rotation[1, 1] - rotation[2, 2]) * 2.0
            quat = np.array(
                [
                    (rotation[2, 1] - rotation[1, 2]) / scale,
                    0.25 * scale,
                    (rotation[0, 1] + rotation[1, 0]) / scale,
                    (rotation[0, 2] + rotation[2, 0]) / scale,
                ],
                dtype=float,
            )
        elif index == 1:
            scale = np.sqrt(1.0 + rotation[1, 1] - rotation[0, 0] - rotation[2, 2]) * 2.0
            quat = np.array(
                [
                    (rotation[0, 2] - rotation[2, 0]) / scale,
                    (rotation[0, 1] + rotation[1, 0]) / scale,
                    0.25 * scale,
                    (rotation[1, 2] + rotation[2, 1]) / scale,
                ],
                dtype=float,
            )
        else:
            scale = np.sqrt(1.0 + rotation[2, 2] - rotation[0, 0] - rotation[1, 1]) * 2.0
            quat = np.array(
                [
                    (rotation[1, 0] - rotation[0, 1]) / scale,
                    (rotation[0, 2] + rotation[2, 0]) / scale,
                    (rotation[1, 2] + rotation[2, 1]) / scale,
                    0.25 * scale,
                ],
                dtype=float,
            )
    norm = float(np.linalg.norm(quat))
    if norm == 0.0:
        return np.array([1.0, 0.0, 0.0, 0.0], dtype=float)
    return quat / norm


def _format_vector(values: list[float] | np.ndarray) -> str:
    return " ".join(f"{float(value):.8g}" for value in values)


def _compose(
    parent_pos: np.ndarray,
    parent_quat_wxyz: np.ndarray,
    child_pos: np.ndarray,
    child_quat_wxyz: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    parent_rotation = _quat_wxyz_to_matrix(parent_quat_wxyz)
    child_rotation = _quat_wxyz_to_matrix(child_quat_wxyz)
    position = np.asarray(parent_pos, dtype=float).reshape(3) + parent_rotation @ np.asarray(child_pos, dtype=float).reshape(3)
    rotation = parent_rotation @ child_rotation
    return position, _matrix_to_quat_wxyz(rotation)


def _site_transform_from_worldbody(root: ElementTree.Element, site_name: str) -> tuple[np.ndarray, np.ndarray]:
    worldbody = root.find("worldbody")
    if worldbody is None:
        raise ValueError("MJCF does not contain a worldbody")

    def visit(element: ElementTree.Element, parent_pos: np.ndarray, parent_quat: np.ndarray) -> tuple[np.ndarray, np.ndarray] | None:
        if element.tag == "body":
            element_pos = _parse_vector(element.attrib.get("pos"), (0.0, 0.0, 0.0))
            element_quat = _parse_vector(element.attrib.get("quat"), (1.0, 0.0, 0.0, 0.0))
            current_pos, current_quat = _compose(parent_pos, parent_quat, element_pos, element_quat)
        else:
            current_pos, current_quat = parent_pos, parent_quat

        for child in element:
            if child.tag == "site" and child.attrib.get("name") == site_name:
                site_pos = _parse_vector(child.attrib.get("pos"), (0.0, 0.0, 0.0))
                site_quat = _parse_vector(child.attrib.get("quat"), (1.0, 0.0, 0.0, 0.0))
                return _compose(current_pos, current_quat, site_pos, site_quat)
            if child.tag == "body":
                found = visit(child, current_pos, current_quat)
                if found is not None:
                    return found
        return None

    identity_pos = np.zeros(3, dtype=float)
    identity_quat = np.array([1.0, 0.0, 0.0, 0.0], dtype=float)
    for body in [child for child in worldbody if child.tag == "body"]:
        found = visit(body, identity_pos, identity_quat)
        if found is not None:
            return found
    raise ValueError(f"MJCF does not contain site {site_name!r}")


def _find_body(root: ElementTree.Element, body_name: str) -> ElementTree.Element:
    body = root.find(f".//body[@name='{body_name}']")
    if body is None:
        raise ValueError(f"MJCF does not contain body {body_name!r}")
    return body


def _find_site(body: ElementTree.Element, site_name: str) -> ElementTree.Element:
    site = body.find(f"site[@name='{site_name}']")
    if site is None:
        raise ValueError(f"Body {body.attrib.get('name', '<unnamed>')!r} does not contain direct site {site_name!r}")
    return site


def generate_ur5e_robotiq_tcp_proxy(
    ur5e_mjcf: Path,
    robotiq_mjcf: Path,
    output_xml: Path,
    robot_name: str = "ur5e_true_robotiq2f85_tcp_proxy",
    base_body: str = "base",
    attachment_body: str = "wrist_3_link",
    attachment_site: str = "attachment_site",
    tool_tcp_site: str = "pinch",
    tcp_site: str = DEFAULT_TCP_SITE,
) -> dict[str, Any]:
    ur5e_root = ElementTree.fromstring(ur5e_mjcf.read_text(encoding="utf-8"))
    robotiq_root = ElementTree.fromstring(robotiq_mjcf.read_text(encoding="utf-8"))
    attachment_body_element = _find_body(ur5e_root, attachment_body)
    attachment_site_element = _find_site(attachment_body_element, attachment_site)

    attachment_pos = _parse_vector(attachment_site_element.attrib.get("pos"), (0.0, 0.0, 0.0))
    attachment_quat = _parse_vector(attachment_site_element.attrib.get("quat"), (1.0, 0.0, 0.0, 0.0))
    tool_tcp_pos, tool_tcp_quat = _site_transform_from_worldbody(robotiq_root, tool_tcp_site)
    tcp_pos, tcp_quat = _compose(attachment_pos, attachment_quat, tool_tcp_pos, tool_tcp_quat)

    for existing in list(attachment_body_element.findall(f"site[@name='{tcp_site}']")):
        attachment_body_element.remove(existing)
    ElementTree.SubElement(
        attachment_body_element,
        "site",
        {
            "name": tcp_site,
            "pos": _format_vector(tcp_pos),
            "quat": _format_vector(tcp_quat),
        },
    )
    compiler = ur5e_root.find("compiler")
    if compiler is not None and compiler.attrib.get("meshdir") is not None:
        source_meshdir = ur5e_mjcf.parent / compiler.attrib["meshdir"]
        compiler.attrib["meshdir"] = os.path.relpath(source_meshdir, output_xml.parent)

    ElementTree.indent(ur5e_root, space="  ")
    output_xml.parent.mkdir(parents=True, exist_ok=True)
    ElementTree.ElementTree(ur5e_root).write(output_xml, encoding="utf-8", xml_declaration=True)

    robot = serial_robot_from_mjcf_xml(
        ElementTree.tostring(ur5e_root, encoding="unicode"),
        name=robot_name,
        base_body_name=base_body,
        target_site=tcp_site,
        target_body=attachment_body,
    )
    metadata = {
        "robot_name": robot_name,
        "source": "MuJoCo Menagerie UR5e with Robotiq 2F-85 pinch TCP proxy",
        "ur5e_mjcf": str(ur5e_mjcf),
        "robotiq_mjcf": str(robotiq_mjcf),
        "output_xml": str(output_xml),
        "base_body": base_body,
        "attachment_body": attachment_body,
        "attachment_site": attachment_site,
        "target_site": tcp_site,
        "target_body": attachment_body,
        "tool_tcp_site": tool_tcp_site,
        "attachment_to_tool_tcp_pos_xyz": [round(float(value), 8) for value in tool_tcp_pos],
        "attachment_to_tool_tcp_quat_wxyz": [round(float(value), 8) for value in tool_tcp_quat],
        "target_offset_xyz": [round(float(value), 8) for value in robot.target_offset],
        "target_quat_wxyz": [round(float(value), 8) for value in robot.target_quat_wxyz],
        "tcp_offset_source": f"{robotiq_mjcf} site {tool_tcp_site!r}, composed onto UR5e {attachment_site!r}",
        "tool_modeling": "tcp_offset_only",
        "concrete_gripper_modeled": False,
        "tool_frame": tool_frame_metadata(
            robot,
            base_body=base_body,
            target_site=tcp_site,
            target_body=attachment_body,
            model_path=output_xml,
        ),
    }
    write_json(output_xml.parent / "model_metadata.json", metadata)
    return metadata


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Generate a UR5e MJCF with an explicit Robotiq 2F-85 pinch TCP proxy site.")
    parser.add_argument("--ur5e-mjcf", default=DEFAULT_UR5E_MJCF)
    parser.add_argument("--robotiq-mjcf", default=DEFAULT_ROBOTIQ_MJCF)
    parser.add_argument("--outputs", default=DEFAULT_OUTPUTS)
    parser.add_argument("--robot-name", default="ur5e_true_robotiq2f85_tcp_proxy")
    parser.add_argument("--base-body", default="base")
    parser.add_argument("--attachment-body", default="wrist_3_link")
    parser.add_argument("--attachment-site", default="attachment_site")
    parser.add_argument("--tool-tcp-site", default="pinch")
    parser.add_argument("--tcp-site", default=DEFAULT_TCP_SITE)
    return parser


def main() -> None:
    args = build_parser().parse_args()
    output_xml = Path(args.outputs) / f"{args.robot_name}.xml"
    metadata = generate_ur5e_robotiq_tcp_proxy(
        ur5e_mjcf=Path(args.ur5e_mjcf),
        robotiq_mjcf=Path(args.robotiq_mjcf),
        output_xml=output_xml,
        robot_name=args.robot_name,
        base_body=args.base_body,
        attachment_body=args.attachment_body,
        attachment_site=args.attachment_site,
        tool_tcp_site=args.tool_tcp_site,
        tcp_site=args.tcp_site,
    )
    print(f"Wrote {metadata['output_xml']} with target site {metadata['target_site']}")


if __name__ == "__main__":
    main()
