from __future__ import annotations

import numpy as np

from ddird.experiments.run_check_mjcf_fk import build_rows_and_summary
from ddird.robots.mjcf_chain import serial_robot_from_mjcf_xml


def test_mjcf_fk_check_builds_summary_shape():
    xml = """
    <mujoco>
      <worldbody>
        <body name="base">
          <body name="link" pos="0.1 0.2 0.3">
            <joint name="joint1" axis="0 0 1" range="-1 1"/>
            <site name="tcp" pos="0.2 0 0"/>
          </body>
        </body>
      </worldbody>
    </mujoco>
    """
    robot = serial_robot_from_mjcf_xml(xml, name="fixture", base_body_name="base", target_site="tcp", target_body="link")
    q = [np.array([0.0]), np.array([0.1])]
    reference_positions = [robot.end_effector_position(row) for row in q]

    rows, summary = build_rows_and_summary(
        robot,
        q,
        reference_positions,
        "fixture.xml",
        "base",
        "tcp",
        "link",
    )

    assert len(rows) == 2
    assert rows[0]["position_error"] == 0.0
    assert summary["robot_name"] == "fixture"
    assert summary["base_body"] == "base"
    assert summary["target_site"] == "tcp"
    assert summary["joint_names"] == ["joint1"]
    assert summary["position_error"]["max"] == 0.0
