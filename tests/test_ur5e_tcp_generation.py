from __future__ import annotations

import numpy as np

from ddird.experiments.run_generate_ur5e_tcp_mjcf import generate_ur5e_robotiq_tcp_proxy
from ddird.robots.mjcf_chain import serial_robot_from_mjcf_xml


def test_generate_ur5e_robotiq_tcp_proxy_composes_pinch_site(tmp_path):
    ur5e_xml = tmp_path / "ur5e.xml"
    ur5e_xml.write_text(
        """
<mujoco>
  <compiler meshdir="assets"/>
  <worldbody>
    <body name="base">
      <body name="wrist_3_link">
        <joint name="wrist_3_joint" axis="0 0 1" range="-1 1"/>
        <site name="attachment_site" pos="0 0.1 0"/>
      </body>
    </body>
  </worldbody>
</mujoco>
""",
        encoding="utf-8",
    )
    robotiq_xml = tmp_path / "2f85.xml"
    robotiq_xml.write_text(
        """
<mujoco>
  <worldbody>
    <body name="base_mount" pos="0 0 0.007">
      <body name="base" pos="0 0 0.0038">
        <site name="pinch" pos="0 0 0.145"/>
      </body>
    </body>
  </worldbody>
</mujoco>
""",
        encoding="utf-8",
    )
    output_xml = tmp_path / "generated" / "ur5e_true_robotiq2f85_tcp_proxy.xml"

    metadata = generate_ur5e_robotiq_tcp_proxy(
        ur5e_mjcf=ur5e_xml,
        robotiq_mjcf=robotiq_xml,
        output_xml=output_xml,
    )
    robot = serial_robot_from_mjcf_xml(
        output_xml.read_text(encoding="utf-8"),
        name="ur5e_true_robotiq2f85_tcp_proxy",
        base_body_name="base",
        target_site="ur5e_robotiq2f85_tcp",
        target_body="wrist_3_link",
    )

    assert output_xml.exists()
    assert 'meshdir="../assets"' in output_xml.read_text(encoding="utf-8")
    assert metadata["target_site"] == "ur5e_robotiq2f85_tcp"
    assert metadata["tool_modeling"] == "tcp_offset_only"
    assert metadata["concrete_gripper_modeled"] is False
    assert np.allclose(metadata["attachment_to_tool_tcp_pos_xyz"], [0.0, 0.0, 0.1558])
    assert np.allclose(robot.target_offset, [0.0, 0.1, 0.1558])
    assert metadata["tool_frame"]["tool_modeling"] == "tcp_offset_only"
