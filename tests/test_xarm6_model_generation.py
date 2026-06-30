from __future__ import annotations

import numpy as np

from ddird.experiments.run_generate_xarm6_mjcf import generate_xarm6_mjcf
from ddird.robots.mjcf_chain import serial_robot_from_mjcf_xml


def test_generate_xarm6_mjcf_from_official_kinematics_shape(tmp_path):
    kinematics = tmp_path / "xarm6_default_kinematics.yaml"
    kinematics.write_text(
        """
kinematics:
  joint1:
    x: 0
    y: 0
    z: 0.267
    roll: 0
    pitch: 0
    yaw: 0
  joint2:
    x: 0
    y: 0
    z: 0
    roll: -1.5708
    pitch: 0
    yaw: 0
  joint3:
    x: 0.0535
    y: -0.2845
    z: 0
    roll: 0
    pitch: 0
    yaw: 0
  joint4:
    x: 0.0775
    y: 0.3425
    z: 0
    roll: -1.5708
    pitch: 0
    yaw: 0
  joint5:
    x: 0
    y: 0
    z: 0
    roll: 1.5708
    pitch: 0
    yaw: 0
  joint6:
    x: 0.076
    y: 0.097
    z: 0
    roll: -1.5708
    pitch: 0
    yaw: 0
""",
        encoding="utf-8",
    )
    output_xml = tmp_path / "xarm6_true.xml"

    metadata = generate_xarm6_mjcf(kinematics, output_xml)
    robot = serial_robot_from_mjcf_xml(
        output_xml.read_text(encoding="utf-8"),
        name="xarm6_true",
        base_body_name="link_base",
        target_site="link_tcp",
        target_body="link6",
    )

    assert metadata["base_body"] == "link_base"
    assert metadata["target_site"] == "link_tcp"
    assert metadata["tcp_offset_xyz"] == [0.0, 0.0, 0.172]
    assert metadata["tool_frame"]["tool_modeling"] == "tcp_offset_only"
    assert metadata["tool_frame"]["tcp_offset_included"] is True
    assert metadata["tool_frame"]["concrete_gripper_modeled"] is False
    assert robot.dof == 6
    assert robot.joint_names == ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
    assert robot.joint_limits.shape == (6, 2)
    assert np.allclose(robot.target_offset, [0.0, 0.0, 0.172])
    assert np.isfinite(robot.end_effector_position(robot.neutral_q)).all()
