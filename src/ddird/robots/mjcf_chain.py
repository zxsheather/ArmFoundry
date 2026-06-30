from __future__ import annotations

from dataclasses import dataclass, replace
from typing import Any
from xml.etree import ElementTree

import numpy as np

_KINEMATIC_DEFAULT_TAGS = {"body", "joint", "site"}


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


def _axis_angle(axis: np.ndarray, angle: float) -> np.ndarray:
    axis = np.asarray(axis, dtype=float).reshape(3)
    norm = float(np.linalg.norm(axis))
    if norm == 0.0:
        return np.eye(3)
    x, y, z = axis / norm
    c = float(np.cos(angle))
    s = float(np.sin(angle))
    t = 1.0 - c
    return np.array(
        [
            [t * x * x + c, t * x * y - s * z, t * x * z + s * y],
            [t * x * y + s * z, t * y * y + c, t * y * z - s * x],
            [t * x * z - s * y, t * y * z + s * x, t * z * z + c],
        ],
        dtype=float,
    )


def _wrap_to_pi(values: np.ndarray) -> np.ndarray:
    return (values + np.pi) % (2.0 * np.pi) - np.pi


@dataclass(frozen=True)
class MJCFBodySegment:
    name: str
    pos: np.ndarray
    quat_wxyz: np.ndarray
    joint_name: str | None = None
    joint_axis: np.ndarray | None = None
    joint_pos: np.ndarray | None = None
    joint_range: tuple[float, float] | None = None


@dataclass(frozen=True)
class MJCFSerialRobot:
    name: str
    base_xyz: np.ndarray
    base_yaw: float
    base_quat_wxyz: np.ndarray
    segments: tuple[MJCFBodySegment, ...]
    target_offset: np.ndarray
    target_frame: str
    table_height: float = 0.0
    workspace_bounds: np.ndarray | None = None
    source: str = "real_kinematics"

    def __post_init__(self) -> None:
        object.__setattr__(self, "base_xyz", np.asarray(self.base_xyz, dtype=float).reshape(3))
        object.__setattr__(self, "base_quat_wxyz", np.asarray(self.base_quat_wxyz, dtype=float).reshape(4))
        object.__setattr__(self, "target_offset", np.asarray(self.target_offset, dtype=float).reshape(3))
        if self.workspace_bounds is not None:
            object.__setattr__(self, "workspace_bounds", np.asarray(self.workspace_bounds, dtype=float))

    @property
    def dof(self) -> int:
        return sum(1 for segment in self.segments if segment.joint_name is not None)

    @property
    def joint_names(self) -> list[str]:
        return [segment.joint_name for segment in self.segments if segment.joint_name is not None]

    @property
    def link_lengths(self) -> np.ndarray:
        return np.asarray([float(np.linalg.norm(segment.pos)) for segment in self.segments], dtype=float)

    @property
    def reach_proxy(self) -> float:
        return float(np.sum(self.link_lengths) + np.linalg.norm(self.target_offset))

    @property
    def joint_limits(self) -> np.ndarray:
        limits = []
        for segment in self.segments:
            if segment.joint_name is None:
                continue
            if segment.joint_range is None:
                limits.append((-np.pi, np.pi))
            else:
                limits.append(segment.joint_range)
        return np.asarray(limits, dtype=float)

    @property
    def neutral_q(self) -> np.ndarray:
        return self.joint_limits.mean(axis=1)

    def with_base(self, base_xyz: np.ndarray | None = None, base_yaw: float | None = None) -> "MJCFSerialRobot":
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
    ) -> "MJCFSerialRobot":
        if link_lengths is not None:
            raise ValueError("MJCFSerialRobot does not support link-length redesign")
        return replace(
            self,
            name=name or self.name,
            base_xyz=np.asarray(base_xyz, dtype=float).reshape(3) if base_xyz is not None else self.base_xyz,
            base_yaw=float(base_yaw) if base_yaw is not None else self.base_yaw,
        )

    def forward_kinematics(self, q: np.ndarray) -> dict[str, Any]:
        q = np.asarray(q, dtype=float).reshape(self.dof)
        position = self.base_xyz.astype(float).copy()
        rotation = _axis_angle(np.array([0.0, 0.0, 1.0]), self.base_yaw) @ _quat_wxyz_to_matrix(self.base_quat_wxyz)
        origins: list[np.ndarray] = []
        axes_world: list[np.ndarray] = []
        joint_positions: list[np.ndarray] = [position.copy()]

        joint_index = 0
        for segment in self.segments:
            position = position + rotation @ segment.pos
            rotation = rotation @ _quat_wxyz_to_matrix(segment.quat_wxyz)
            joint_positions.append(position.copy())
            if segment.joint_name is not None:
                assert segment.joint_axis is not None
                joint_pos = segment.joint_pos if segment.joint_pos is not None else np.zeros(3, dtype=float)
                origin = position + rotation @ joint_pos
                axis_world = rotation @ segment.joint_axis
                origins.append(origin.copy())
                axes_world.append(axis_world.copy())
                rotation = rotation @ _axis_angle(segment.joint_axis, float(q[joint_index]))
                joint_index += 1

        position = position + rotation @ self.target_offset
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
        limits = self.joint_limits
        return np.minimum(q - limits[:, 0], limits[:, 1] - q)

    def clip_to_limits(self, q: np.ndarray) -> np.ndarray:
        q = _wrap_to_pi(np.asarray(q, dtype=float).reshape(self.dof))
        limits = self.joint_limits
        return np.clip(q, limits[:, 0], limits[:, 1])

    def within_limits(self, q: np.ndarray, tolerance: float = 1e-8) -> bool:
        q = np.asarray(q, dtype=float).reshape(self.dof)
        limits = self.joint_limits
        return bool(np.all(q >= limits[:, 0] - tolerance) and np.all(q <= limits[:, 1] + tolerance))

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
            "base_quat_wxyz": self.base_quat_wxyz.round(6).tolist(),
            "joint_names": self.joint_names,
            "joint_limits": self.joint_limits.round(6).tolist(),
            "reach_proxy": round(self.reach_proxy, 6),
            "source": self.source,
            "target_frame": self.target_frame,
        }


def _body_children(body: ElementTree.Element) -> list[ElementTree.Element]:
    return [child for child in body if child.tag == "body"]


def _path_to_target_body(body: ElementTree.Element, target_body: str) -> list[ElementTree.Element] | None:
    if body.attrib.get("name") == target_body:
        return [body]
    for child in _body_children(body):
        child_path = _path_to_target_body(child, target_body)
        if child_path is not None:
            return [body, *child_path]
    return None


def _path_to_target_site(body: ElementTree.Element, target_site: str) -> tuple[list[ElementTree.Element], ElementTree.Element] | None:
    for child in body:
        if child.tag == "site" and child.attrib.get("name") == target_site:
            return [body], child
    for child in _body_children(body):
        child_path = _path_to_target_site(child, target_site)
        if child_path is not None:
            bodies, site = child_path
            return [body, *bodies], site
    return None


def _copy_default_attrs(attrs: dict[str, dict[str, str]]) -> dict[str, dict[str, str]]:
    return {tag: dict(values) for tag, values in attrs.items()}


def _collect_default_classes(root: ElementTree.Element) -> dict[str, dict[str, dict[str, str]]]:
    classes: dict[str, dict[str, dict[str, str]]] = {"main": {}}

    def visit(default: ElementTree.Element, inherited: dict[str, dict[str, str]], inherited_name: str) -> None:
        class_name = default.attrib.get("class", inherited_name)
        attrs = _copy_default_attrs(inherited)
        for child in default:
            if child.tag in _KINEMATIC_DEFAULT_TAGS:
                attrs.setdefault(child.tag, {}).update(child.attrib)
        classes[class_name] = _copy_default_attrs(attrs)
        for child in default:
            if child.tag == "default":
                visit(child, attrs, class_name)

    for default in root.findall("default"):
        visit(default, classes["main"], default.attrib.get("class", "main"))
    return classes


def _effective_attrs(
    element: ElementTree.Element,
    defaults: dict[str, dict[str, dict[str, str]]],
    class_name: str,
    tag: str,
) -> dict[str, str]:
    attrs = dict(defaults.get(class_name, {}).get(tag, {}))
    element_class = element.attrib.get("class")
    if element_class is not None:
        attrs.update(defaults.get(element_class, {}).get(tag, {}))
    attrs.update(element.attrib)
    return attrs


def _body_class_contexts(base_body: ElementTree.Element) -> dict[int, str]:
    contexts: dict[int, str] = {}

    def visit(body: ElementTree.Element, inherited_child_class: str) -> None:
        body_class = body.attrib.get("class", inherited_child_class)
        contexts[id(body)] = body_class
        child_class = body.attrib.get("childclass", body_class)
        for child in _body_children(body):
            visit(child, child_class)

    visit(base_body, "main")
    return contexts


def _orientation_quat_wxyz(attrs: dict[str, str], label: str) -> np.ndarray:
    if "quat" in attrs:
        return _parse_vector(attrs.get("quat"), (1.0, 0.0, 0.0, 0.0))
    unsupported = sorted({"axisangle", "euler", "xyaxes", "zaxis"} & set(attrs))
    if unsupported:
        raise ValueError(f"{label} uses unsupported MJCF orientation attributes: {', '.join(unsupported)}")
    return np.array([1.0, 0.0, 0.0, 0.0], dtype=float)


def _segment_from_body(
    body: ElementTree.Element,
    defaults: dict[str, dict[str, dict[str, str]]],
    body_class: str,
) -> MJCFBodySegment:
    body_attrs = _effective_attrs(body, defaults, body_class, "body")
    joints = [child for child in body if child.tag == "joint"]
    if len(joints) > 1:
        raise ValueError(f"Body {body.attrib.get('name', '<unnamed>')!r} has multiple joints; only serial one-joint bodies are supported")
    joint = joints[0] if joints else None
    joint_range = None
    joint_name = None
    joint_axis = None
    joint_pos = None
    if joint is not None:
        joint_class = joint.attrib.get("class", body_class)
        joint_attrs = _effective_attrs(joint, defaults, joint_class, "joint")
        joint_type = joint_attrs.get("type", "hinge")
        if joint_type != "hinge":
            raise ValueError(f"Joint {joint_attrs.get('name', body_attrs.get('name', '<unnamed>'))!r} has unsupported type {joint_type!r}")
        joint_name = joint_attrs.get("name", body_attrs.get("name"))
        joint_axis = _parse_vector(joint_attrs.get("axis"), (0.0, 0.0, 1.0))
        joint_pos = _parse_vector(joint_attrs.get("pos"), (0.0, 0.0, 0.0))
        range_values = _parse_vector(joint_attrs.get("range"), (-np.pi, np.pi))
        if range_values.shape == (2,):
            joint_range = (float(range_values[0]), float(range_values[1]))
        else:
            raise ValueError(f"Joint {joint_name!r} has invalid range {joint_attrs.get('range')!r}")
    return MJCFBodySegment(
        name=body_attrs.get("name", "unnamed_body"),
        pos=_parse_vector(body_attrs.get("pos"), (0.0, 0.0, 0.0)),
        quat_wxyz=_orientation_quat_wxyz(body_attrs, f"Body {body_attrs.get('name', '<unnamed>')!r}"),
        joint_name=joint_name,
        joint_axis=joint_axis,
        joint_pos=joint_pos,
        joint_range=joint_range,
    )


def serial_robot_from_mjcf_xml(
    xml: str,
    name: str,
    base_body_name: str,
    target_site: str = "gripper0_grip_site",
    target_body: str = "gripper0_eef",
) -> MJCFSerialRobot:
    root = ElementTree.fromstring(xml)
    defaults = _collect_default_classes(root)
    base_body = root.find(f".//body[@name='{base_body_name}']")
    if base_body is None:
        raise ValueError(f"MJCF does not contain body name={base_body_name!r}")
    body_contexts = _body_class_contexts(base_body)

    site_path = _path_to_target_site(base_body, target_site)
    if site_path is not None:
        bodies, site = site_path
        site_body_class = body_contexts[id(bodies[-1])]
        site_class = site.attrib.get("class", site_body_class)
        site_attrs = _effective_attrs(site, defaults, site_class, "site")
        _orientation_quat_wxyz(site_attrs, f"Site {site_attrs.get('name', target_site)!r}")
        target_offset = _parse_vector(site_attrs.get("pos"), (0.0, 0.0, 0.0))
        target_frame = target_site
    else:
        body_path = _path_to_target_body(base_body, target_body)
        if body_path is None:
            raise ValueError(f"MJCF does not contain target site {target_site!r} or body {target_body!r}")
        bodies = body_path
        target_offset = np.zeros(3, dtype=float)
        target_frame = target_body

    base_attrs = _effective_attrs(base_body, defaults, body_contexts[id(base_body)], "body")
    segments = tuple(_segment_from_body(body, defaults, body_contexts[id(body)]) for body in bodies[1:])
    return MJCFSerialRobot(
        name=name,
        base_xyz=_parse_vector(base_attrs.get("pos"), (0.0, 0.0, 0.0)),
        base_yaw=0.0,
        base_quat_wxyz=_orientation_quat_wxyz(base_attrs, f"Base body {base_body_name!r}"),
        segments=segments,
        target_offset=target_offset,
        target_frame=target_frame,
        workspace_bounds=np.array([[-1.2, 1.2], [-1.2, 1.2], [-0.02, 1.4]], dtype=float),
        source="real_kinematics",
    )


def panda_from_mjcf_xml(
    xml: str,
    name: str = "panda_true",
    target_site: str = "gripper0_grip_site",
    target_body: str = "gripper0_eef",
) -> MJCFSerialRobot:
    return serial_robot_from_mjcf_xml(
        xml,
        name=name,
        base_body_name="robot0_base",
        target_site=target_site,
        target_body=target_body,
    )


def make_libero_panda_true(name: str = "panda_true") -> MJCFSerialRobot:
    segments = (
        MJCFBodySegment("robot0_link0", np.array([0.0, 0.0, 0.0]), np.array([1.0, 0.0, 0.0, 0.0])),
        MJCFBodySegment(
            "robot0_link1",
            np.array([0.0, 0.0, 0.333]),
            np.array([1.0, 0.0, 0.0, 0.0]),
            "robot0_joint1",
            np.array([0.0, 0.0, 1.0]),
            np.zeros(3),
            (-2.8973, 2.8973),
        ),
        MJCFBodySegment(
            "robot0_link2",
            np.array([0.0, 0.0, 0.0]),
            np.array([0.707107, -0.707107, 0.0, 0.0]),
            "robot0_joint2",
            np.array([0.0, 0.0, 1.0]),
            np.zeros(3),
            (-1.7628, 1.7628),
        ),
        MJCFBodySegment(
            "robot0_link3",
            np.array([0.0, -0.316, 0.0]),
            np.array([0.707107, 0.707107, 0.0, 0.0]),
            "robot0_joint3",
            np.array([0.0, 0.0, 1.0]),
            np.zeros(3),
            (-2.8973, 2.8973),
        ),
        MJCFBodySegment(
            "robot0_link4",
            np.array([0.0825, 0.0, 0.0]),
            np.array([0.707107, 0.707107, 0.0, 0.0]),
            "robot0_joint4",
            np.array([0.0, 0.0, 1.0]),
            np.zeros(3),
            (-3.0718, -0.0698),
        ),
        MJCFBodySegment(
            "robot0_link5",
            np.array([-0.0825, 0.384, 0.0]),
            np.array([0.707107, -0.707107, 0.0, 0.0]),
            "robot0_joint5",
            np.array([0.0, 0.0, 1.0]),
            np.zeros(3),
            (-2.8973, 2.8973),
        ),
        MJCFBodySegment(
            "robot0_link6",
            np.array([0.0, 0.0, 0.0]),
            np.array([0.707107, 0.707107, 0.0, 0.0]),
            "robot0_joint6",
            np.array([0.0, 0.0, 1.0]),
            np.zeros(3),
            (-0.0175, 3.7525),
        ),
        MJCFBodySegment(
            "robot0_link7",
            np.array([0.088, 0.0, 0.0]),
            np.array([0.707107, 0.707107, 0.0, 0.0]),
            "robot0_joint7",
            np.array([0.0, 0.0, 1.0]),
            np.zeros(3),
            (-2.8973, 2.8973),
        ),
        MJCFBodySegment(
            "robot0_right_hand",
            np.array([0.0, 0.0, 0.1065]),
            np.array([0.923785, 0.0, 0.0, -0.382911]),
        ),
        MJCFBodySegment(
            "gripper0_right_gripper",
            np.array([0.0, 0.0, 0.0]),
            np.array([0.707107, 0.0, 0.0, -0.707107]),
        ),
        MJCFBodySegment("gripper0_eef", np.array([0.0, 0.0, 0.097]), np.array([1.0, 0.0, 0.0, 0.0])),
    )
    return MJCFSerialRobot(
        name=name,
        base_xyz=np.zeros(3, dtype=float),
        base_yaw=0.0,
        base_quat_wxyz=np.array([1.0, 0.0, 0.0, 0.0]),
        segments=segments,
        target_offset=np.zeros(3, dtype=float),
        target_frame="gripper0_grip_site",
        workspace_bounds=np.array([[-1.2, 1.2], [-1.2, 1.2], [-0.02, 1.4]], dtype=float),
        source="real_kinematics",
    )
