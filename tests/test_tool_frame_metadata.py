from __future__ import annotations

import numpy as np

from ddird.robots.mjcf_chain import make_libero_panda_true, serial_robot_from_mjcf_xml
from ddird.robots.tool_frames import tool_frame_metadata


def test_panda_true_tool_frame_metadata_marks_source_gripper_chain():
    robot = make_libero_panda_true()

    metadata = tool_frame_metadata(
        robot,
        base_body="robot0_base",
        target_site="gripper0_grip_site",
        target_body="gripper0_eef",
    )

    assert metadata["robot_name"] == "panda_true"
    assert metadata["target_frame"] == "gripper0_grip_site"
    assert metadata["tool_modeling"] == "source_gripper_chain"
    assert metadata["tcp_offset_included"] is True
    assert metadata["concrete_gripper_modeled"] is True


def test_mjcf_tool_frame_metadata_marks_attachment_site_without_gripper():
    xml = """
    <mujoco>
      <worldbody>
        <body name="base">
          <body name="wrist_3_link" pos="0 0 0.1">
            <joint name="joint1" axis="0 0 1" range="-1 1"/>
            <site name="attachment_site" pos="0 0.1 0"/>
          </body>
        </body>
      </worldbody>
    </mujoco>
    """
    robot = serial_robot_from_mjcf_xml(
        xml,
        name="ur5e_true",
        base_body_name="base",
        target_site="attachment_site",
        target_body="wrist_3_link",
    )

    metadata = tool_frame_metadata(
        robot,
        base_body="base",
        target_site="attachment_site",
        target_body="wrist_3_link",
        model_path="fixture.xml",
    )

    assert metadata["robot_name"] == "ur5e_true"
    assert metadata["target_frame"] == "attachment_site"
    assert np.allclose(metadata["target_offset_xyz"], [0.0, 0.1, 0.0])
    assert metadata["tool_modeling"] == "attachment_site_only"
    assert metadata["tcp_offset_included"] is True
    assert metadata["concrete_gripper_modeled"] is False


def test_ur5e_explicit_tcp_offset_is_marked_without_concrete_gripper():
    xml = """
    <mujoco>
      <worldbody>
        <body name="base">
          <body name="wrist_3_link" pos="0 0 0.1">
            <joint name="joint1" axis="0 0 1" range="-1 1"/>
            <site name="ur5e_robotiq2f85_tcp" pos="0 0.1 0.2"/>
          </body>
        </body>
      </worldbody>
    </mujoco>
    """
    robot = serial_robot_from_mjcf_xml(
        xml,
        name="ur5e_true_robotiq2f85_tcp_proxy",
        base_body_name="base",
        target_site="ur5e_robotiq2f85_tcp",
        target_body="wrist_3_link",
    )

    metadata = tool_frame_metadata(
        robot,
        base_body="base",
        target_site="ur5e_robotiq2f85_tcp",
        target_body="wrist_3_link",
    )

    assert metadata["tool_modeling"] == "tcp_offset_only"
    assert metadata["tcp_offset_included"] is True
    assert metadata["concrete_gripper_modeled"] is False
    assert metadata["target_quat_wxyz"] == [1.0, 0.0, 0.0, 0.0]


def test_xarm6_zero_tcp_offset_is_marked_as_wrist_origin_diagnostic():
    xml = """
    <mujoco>
      <worldbody>
        <body name="link_base">
          <body name="link6">
            <joint name="joint6" axis="0 0 1" range="-1 1"/>
            <site name="link_tcp" pos="0 0 0"/>
          </body>
        </body>
      </worldbody>
    </mujoco>
    """
    robot = serial_robot_from_mjcf_xml(
        xml,
        name="xarm6_true",
        base_body_name="link_base",
        target_site="link_tcp",
        target_body="link6",
    )

    metadata = tool_frame_metadata(
        robot,
        base_body="link_base",
        target_site="link_tcp",
        target_body="link6",
    )

    assert metadata["tool_modeling"] == "wrist_origin_diagnostic"
    assert metadata["tcp_offset_included"] is False
