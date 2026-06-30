from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest

from ddird.data.dataset import TrajectoryRecord
from ddird.eval.evaluator import EvaluationConfig, evaluate_robot
from ddird.experiments.run_check_libero_panda_fk import _check_demo
from ddird.robots.mjcf_chain import make_libero_panda_true, panda_from_mjcf_xml, serial_robot_from_mjcf_xml


PANDA_XML = """
<mujoco model="base">
  <worldbody>
    <body name="robot0_base" pos="{base_pos}">
      <body name="robot0_link0">
        <body name="robot0_link1" pos="0 0 0.333">
          <joint name="robot0_joint1" axis="0 0 1" range="-2.8973 2.8973"/>
          <body name="robot0_link2" quat="0.707107 -0.707107 0 0">
            <joint name="robot0_joint2" axis="0 0 1" range="-1.7628 1.7628"/>
            <body name="robot0_link3" pos="0 -0.316 0" quat="0.707107 0.707107 0 0">
              <joint name="robot0_joint3" axis="0 0 1" range="-2.8973 2.8973"/>
              <body name="robot0_link4" pos="0.0825 0 0" quat="0.707107 0.707107 0 0">
                <joint name="robot0_joint4" axis="0 0 1" range="-3.0718 -0.0698"/>
                <body name="robot0_link5" pos="-0.0825 0.384 0" quat="0.707107 -0.707107 0 0">
                  <joint name="robot0_joint5" axis="0 0 1" range="-2.8973 2.8973"/>
                  <body name="robot0_link6" quat="0.707107 0.707107 0 0">
                    <joint name="robot0_joint6" axis="0 0 1" range="-0.0175 3.7525"/>
                    <body name="robot0_link7" pos="0.088 0 0" quat="0.707107 0.707107 0 0">
                      <joint name="robot0_joint7" axis="0 0 1" range="-2.8973 2.8973"/>
                      <body name="robot0_right_hand" pos="0 0 0.1065" quat="0.923785 0 0 -0.382911">
                        <body name="gripper0_right_gripper" quat="0.707107 0 0 -0.707107">
                          <body name="gripper0_eef" pos="0 0 0.097">
                            <site name="gripper0_grip_site" pos="0 0 0"/>
                          </body>
                        </body>
                      </body>
                    </body>
                  </body>
                </body>
              </body>
            </body>
          </body>
        </body>
      </body>
    </body>
  </worldbody>
</mujoco>
"""

UR5E_STYLE_XML = """
<mujoco model="ur5e_style">
  <default>
    <default class="ur5e">
      <joint axis="0 1 0" range="-6.28319 6.28319"/>
      <default class="size3"/>
      <default class="size3_limited">
        <joint range="-3.1415 3.1415"/>
      </default>
      <default class="size1"/>
    </default>
  </default>
  <worldbody>
    <body name="base" quat="0 0 0 -1" childclass="ur5e">
      <body name="shoulder_link" pos="0 0 0.163">
        <joint name="shoulder_pan_joint" class="size3" axis="0 0 1"/>
        <body name="upper_arm_link" pos="0 0.138 0" quat="1 0 1 0">
          <joint name="shoulder_lift_joint" class="size3"/>
          <body name="forearm_link" pos="0 -0.131 0.425">
            <joint name="elbow_joint" class="size3_limited"/>
            <body name="wrist_1_link" pos="0 0 0.392" quat="1 0 1 0">
              <joint name="wrist_1_joint" class="size1"/>
              <body name="wrist_2_link" pos="0 0.127 0">
                <joint name="wrist_2_joint" axis="0 0 1" class="size1"/>
                <body name="wrist_3_link" pos="0 0 0.1">
                  <joint name="wrist_3_joint" class="size1"/>
                  <site name="attachment_site" pos="0 0.1 0"/>
                </body>
              </body>
            </body>
          </body>
        </body>
      </body>
    </body>
  </worldbody>
</mujoco>
"""


class FakeHDF5Group:
    def __init__(self, joint_states: np.ndarray, ee_pos: np.ndarray, model_file: str):
        self.attrs = {"model_file": model_file}
        self._datasets = {
            "obs/joint_states": joint_states,
            "obs/ee_pos": ee_pos,
        }

    def __contains__(self, key: str) -> bool:
        return key in self._datasets

    def __getitem__(self, key: str) -> np.ndarray:
        return self._datasets[key]


def test_libero_panda_true_has_seven_dof():
    robot = make_libero_panda_true()

    assert robot.name == "panda_true"
    assert robot.dof == 7
    assert robot.joint_limits.shape == (7, 2)
    assert robot.target_frame == "gripper0_grip_site"


def test_panda_mjcf_parser_matches_builtin_chain_at_neutral():
    xml = PANDA_XML.format(base_pos="0 0 0")
    parsed = panda_from_mjcf_xml(xml)
    builtin = make_libero_panda_true()

    assert np.allclose(parsed.end_effector_position(parsed.neutral_q), builtin.end_effector_position(builtin.neutral_q))


def test_panda_mjcf_parser_reads_source_base_position():
    parsed = panda_from_mjcf_xml(PANDA_XML.format(base_pos="0.4 -0.2 0.9"))

    assert np.allclose(parsed.base_xyz, [0.4, -0.2, 0.9])


def test_fk_check_uses_arm_dof_from_wider_joint_state():
    xml = PANDA_XML.format(base_pos="0 0 0")
    robot = panda_from_mjcf_xml(xml)
    q = np.vstack([robot.neutral_q, robot.neutral_q + np.array([0.01, 0.0, -0.01, 0.0, 0.0, 0.01, 0.0])])
    joint_states = np.hstack([q, np.ones((len(q), 2))])
    ee_pos = np.asarray([robot.end_effector_position(row) for row in q])
    group = FakeHDF5Group(joint_states=joint_states, ee_pos=ee_pos, model_file=xml)

    row = _check_demo(group, Path("libero_goal/task/demo.hdf5"), "demo_0", "gripper0_grip_site", "gripper0_eef")

    assert row is not None
    assert row["num_samples"] == 2
    assert row["target_frame"] == "gripper0_grip_site"
    assert row["mean_position_error"] == 0.0


def test_panda_true_reaches_its_neutral_fk_target_with_ik(tmp_path):
    robot = make_libero_panda_true()
    target = robot.end_effector_position(robot.neutral_q)
    record = TrajectoryRecord(
        suite="libero_goal",
        task="neutral",
        episode_id="0",
        path=tmp_path / "episode.npz",
        ee_pos=target.reshape(1, 3),
    )

    result = evaluate_robot(robot, [record], config=EvaluationConfig(max_iters=1, base_pose_mode="fixed"))

    assert result.aggregate["robot_name"] == "panda_true"
    assert result.aggregate["ik_success_rate"] == 1.0


def test_mjcf_parser_inherits_default_joint_axis_and_range():
    robot = serial_robot_from_mjcf_xml(
        UR5E_STYLE_XML,
        name="ur5e_true",
        base_body_name="base",
        target_site="attachment_site",
        target_body="wrist_3_link",
    )

    assert robot.dof == 6
    assert robot.joint_names == [
        "shoulder_pan_joint",
        "shoulder_lift_joint",
        "elbow_joint",
        "wrist_1_joint",
        "wrist_2_joint",
        "wrist_3_joint",
    ]
    assert np.allclose(robot.base_quat_wxyz, [0.0, 0.0, 0.0, -1.0])
    assert np.allclose(robot.segments[1].joint_axis, [0.0, 1.0, 0.0])
    assert np.allclose(robot.joint_limits[1], [-6.28319, 6.28319])
    assert np.allclose(robot.joint_limits[2], [-3.1415, 3.1415])


def test_mjcf_parser_applies_base_quaternion_to_fk():
    xml = """
    <mujoco>
      <worldbody>
        <body name="base" quat="0 0 0 -1">
          <body name="link" pos="1 0 0">
            <site name="tcp" pos="0 0 0"/>
          </body>
        </body>
      </worldbody>
    </mujoco>
    """
    robot = serial_robot_from_mjcf_xml(xml, name="quat_fixture", base_body_name="base", target_site="tcp", target_body="link")

    assert np.allclose(robot.end_effector_position(np.zeros(0)), [-1.0, 0.0, 0.0])


def test_downloaded_menagerie_ur5e_parses_when_available():
    path = Path("data/armforge/models/mujoco_menagerie/universal_robots_ur5e/ur5e.xml")
    if not path.exists():
        pytest.skip("Menagerie UR5e model is not present")

    robot = serial_robot_from_mjcf_xml(
        path.read_text(encoding="utf-8"),
        name="ur5e_true",
        base_body_name="base",
        target_site="attachment_site",
        target_body="wrist_3_link",
    )

    assert robot.dof == 6
    assert robot.joint_names == [
        "shoulder_pan_joint",
        "shoulder_lift_joint",
        "elbow_joint",
        "wrist_1_joint",
        "wrist_2_joint",
        "wrist_3_joint",
    ]
    assert np.allclose(robot.joint_limits[2], [-3.1415, 3.1415])
