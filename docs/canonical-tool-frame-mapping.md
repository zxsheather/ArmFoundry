# Canonical Tool-Frame Mapping

This document defines how ArmFoundry maps source LIBERO Panda end-effector
orientations onto target robot TCP frames for pose-aware reachability
benchmarks.

## Problem

The LIBERO trajectories store end-effector orientation as `ee_ori`. In the
current dataset this is a 3-vector orientation from the source Panda gripper
frame, not a robot-independent task orientation.

Passing that orientation directly to every target robot is a diagnostic test,
but it is not enough for a fair pose comparison unless the target TCP frame has
the same semantic axes as the source Panda TCP frame.

The canonical tool-frame mapping makes that assumption explicit and executable.

## Canonical Frame

ArmFoundry uses a parallel-jaw pinch TCP frame:

- Origin: pinch center / gripper TCP.
- +Z: tool approach direction, from palm or wrist toward the pinch point.
- +Y: finger opening and closing direction between the left and right jaws.
- +X: `+Y cross +Z`, completing a right-handed frame.

The LIBERO Panda `gripper0_grip_site` is the source canonical frame. In the
source MuJoCo model, `gripper0_ee_x`, `gripper0_ee_y`, and `gripper0_ee_z`
visualize this frame, and the finger slide joints move along the gripper Y
axis.

## Rotation Mapping

Let:

- `R_world_source` be the orientation decoded from LIBERO `ee_ori`.
- `R_source_canonical` be the fixed rotation from the source Panda TCP frame to
  the canonical frame.
- `R_canonical_target` be the fixed rotation from the canonical frame to the
  target robot TCP frame.

The pose target sent to a target robot is:

```text
R_world_target = R_world_source * R_source_canonical * R_canonical_target
```

The default legacy mode is `identity`, which means:

```text
R_world_target = R_world_source
```

The pose-fairness mode is `canonical_tool`, which requires an explicit mapping
entry for the target robot.

## Current Mapping Table

| Robot | Target frame | Mapping status | Source to canonical | Canonical to target | Notes |
|---|---|---|---|---|---|
| `panda_true` | `gripper0_grip_site` | `source_canonical` | identity | identity | Source Panda reference. |
| `xarm6_true` | `link_tcp` | `mapped_tcp_offset_only` | identity | identity | Official xArm gripper `joint_tcp` has `rpy=0`, +Z TCP offset, and symmetric finger structure along +/-Y. |
| `ur5e_true_robotiq2f85_tcp_proxy` | `ur5e_robotiq2f85_tcp` | `mapped_tcp_offset_only` | identity | identity | Robotiq 2F-85 `pinch` site uses +Z to pinch center and +/-Y jaw symmetry, then is composed onto UR5e `attachment_site`. |

Rows intentionally not mapped:

| Robot group | Reason |
|---|---|
| `ur5e_true` | Evaluates UR5e `attachment_site`, not a gripper TCP. |
| `*_proxy` | Simplified endpoint chains do not define concrete tool axes. |
| wrist-origin diagnostics | No tool/TCP orientation semantics. |

Current mapped target rotations are identity corrections because the mapped
Panda, xArm, and Robotiq TCP frames already use the same canonical parallel-jaw
axis convention. The important change is not a numerical rotation for these
rows; it is the explicit eligibility gate and the documented semantic contract.

## CLI Usage

Legacy diagnostic pose behavior:

```bash
uv run --extra mujoco python -m ddird.experiments.run_eval_mjcf_robot \
  --evaluation-mode pose \
  --orientation-format rotvec \
  --tool-frame-mapping identity \
  ...
```

Canonical pose-fairness behavior:

```bash
uv run --extra mujoco python -m ddird.experiments.run_eval_mjcf_robot \
  --evaluation-mode pose \
  --orientation-format rotvec \
  --tool-frame-mapping canonical_tool \
  ...
```

If `canonical_tool` is requested for an unmapped robot, the evaluator raises an
error instead of silently producing a pose result that could be misread as fair.

## What This Solves

The mapping solves the frame-level fairness problem for pose-aware reachability:

```text
All canonical pose rows ask whether the same semantic gripper TCP frame can
reach the source Panda TCP poses.
```

This is stronger than the old diagnostic behavior, where a bare wrist frame, a
proxy endpoint, or a TCP frame could all receive the same source orientation
without an explicit frame contract.

## What This Does Not Solve

`canonical_tool` is still a kinematic pose benchmark. It does not add:

- gripper meshes;
- collision geometry;
- gripper joints;
- actuation;
- contact;
- grasp success.

For `xarm6_true` and `ur5e_true_robotiq2f85_tcp_proxy`, the mapped pose result
is still a `tcp_offset_only` result. It is fair at the TCP frame level, not a
full manipulation simulation.

## Reporting Rule

Reports should label pose-aware results with both fields:

- `tool_frame_mapping`: `identity` or `canonical_tool`
- `tool_modeling`: for example `source_gripper_chain`, `tcp_offset_only`,
  `attachment_site_only`, or `proxy_endpoint`

Only rows with `tool_frame_mapping=canonical_tool` and an eligible mapping entry
should be described as canonical pose-fairness results.
