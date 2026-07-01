# Recent Work Summary: Canonical Pose Benchmarking and xArm6 Diagnosis

Date: 2026-07-01

## Executive Summary

Recent work focused on making ArmFoundry's pose-aware cross-robot reachability
benchmark semantically clearer and then diagnosing why `xarm6_true` performed
poorly under that benchmark.

The main outcome is that the low `xarm6_true` canonical pose score is now
understood much better. Under the strict source-base benchmark, xArm6 has low
trajectory pose success (`0.242`) even though its position-only source-base
success is `1.000`. A diagnostic base-placement search then showed that this is
largely a base-placement effect: with suite-specific base placements, the same
xArm6 TCP model, orientation tolerance, canonical tool-frame mapping, and IK
settings reached `1.000` trajectory pose success on all three suites.

In short:

- The original Panda-source pose data are valid, but source-base placement is
  not necessarily favorable to xArm6.
- Canonical tool-frame mapping now makes pose comparisons explicit and gated.
- `xarm6_true` is not intrinsically unable to solve the canonical pose
  distribution in this evaluator.
- Source-base and optimized-base results answer different benchmark questions
  and should be reported separately.

## Background

The project is evaluating whether different robot arms can reproduce LIBERO
end-effector trajectories. The source data come from a Panda-arm setup and
include end-effector position and orientation. For position-only evaluation,
the benchmark asks whether a robot TCP can reach the recorded positions. For
pose-aware evaluation, the benchmark also requires the robot TCP orientation to
match the recorded source orientation.

This created a fairness question: the recorded orientation is tied to the Panda
gripper TCP frame, not to an abstract task frame. Therefore, pose-aware
comparison is only meaningful when target robot TCP frames are mapped into the
same semantic tool frame.

## Work Completed

### 1. Added UR5e Robotiq TCP proxy support

We added and benchmarked a `ur5e_true_robotiq2f85_tcp_proxy` model. This model
adds the Robotiq 2F-85 pinch TCP offset onto the UR5e attachment site. It is a
`tcp_offset_only` model: it includes the TCP offset and orientation semantics,
but not Robotiq meshes, collision geometry, finger joints, actuation, or
contact.

Key artifacts:

- `outputs/generated_models/ur5e_robotiq2f85_tcp_proxy/`
- `outputs/ur5e_robotiq2f85_tcp_proxy_sourcebase_it80/`

Relevant commits:

- `2948b2b Add UR5e Robotiq TCP proxy model`
- `a4db81f Add UR5e Robotiq TCP proxy benchmark`

### 2. Added source-base tool-frame audit

We produced a source-base tool-frame audit to distinguish different endpoint
semantics:

- source Panda gripper-chain TCP;
- true robot TCP offsets;
- UR5e attachment-site diagnostics;
- simplified proxy endpoints.

This clarified which rows can be used for pose-fairness claims and which rows
should remain diagnostic only.

Key artifact:

- `outputs/tool_frame_audit_sourcebase_it80/analysis_report.md`

Relevant commit:

- `325b776 Add source-base tool-frame audit`

### 3. Implemented Canonical Tool-Frame Mapping

We defined and implemented a canonical parallel-jaw TCP frame:

- origin: pinch center / gripper TCP;
- +Z: tool approach direction;
- +Y: finger opening/closing direction;
- +X: right-handed completion.

The evaluator now supports:

```text
--tool-frame-mapping identity
--tool-frame-mapping canonical_tool
```

Only explicitly mapped robots can run in `canonical_tool` mode. Current eligible
rows are:

| Robot | Target frame | Mapping status |
| --- | --- | --- |
| `panda_true` | `gripper0_grip_site` | `source_canonical` |
| `xarm6_true` | `link_tcp` | `mapped_tcp_offset_only` |
| `ur5e_true_robotiq2f85_tcp_proxy` | `ur5e_robotiq2f85_tcp` | `mapped_tcp_offset_only` |

For these rows, the current mapping rotations are identity rotations because
their TCP axes already match the canonical parallel-jaw convention. The
important change is the explicit semantic contract and eligibility gate, not a
numerical rotation change.

Key artifacts:

- `docs/canonical-tool-frame-mapping.md`
- `src/ddird/eval/tool_frame_mapping.py`
- `tests/test_tool_frame_mapping.py`

Relevant commit:

- `54b5a43 Add canonical tool-frame mapping`

### 4. Reran the canonical pose benchmark

We reran the source-base, pose-aware benchmark using:

- `--base-pose-mode source`
- `--max-iters 80`
- `--max-waypoints-per-trajectory 80`
- `--evaluation-mode pose`
- `--orientation-format rotvec`
- `--orientation-tolerance 0.10`
- `--tool-frame-mapping canonical_tool`

Results:

| Robot | Pose waypoint success | Pose trajectory success | Mean position error | Mean orientation error |
| --- | ---: | ---: | ---: | ---: |
| `panda_true` | 0.995299 | 0.946667 | 0.000495 m | 0.011822 rad |
| `xarm6_true` | 0.719183 | 0.242000 | 0.011200 m | 0.023499 rad |
| `ur5e_true_robotiq2f85_tcp_proxy` | 0.996833 | 0.967333 | 0.000395 m | 0.001411 rad |

The canonical run numerically matched the earlier identity-mapping pose run,
which is expected because the current eligible mappings are identity rotations.
The value of the rerun is that the output is now explicitly labeled and gated
as a canonical pose-fairness result.

Key artifact:

- `outputs/sourcebase_canonical_pose_it80/analysis_report.md`

Relevant commit:

- `cf60f73 Add canonical pose benchmark results`

### 5. Diagnosed why `xarm6_true` was low

We investigated the low xArm6 canonical pose result. The first diagnosis showed
that this was not a basic position reachability issue:

| Run | Waypoint success | Trajectory success | Mean position error |
| --- | ---: | ---: | ---: |
| xArm6 position-only source-base IT80 | 1.000000 | 1.000000 | 0.000297 m |
| xArm6 canonical pose source-base IT80 | 0.719183 | 0.242000 | 0.011200 m |

We also ran diagnostic sweeps:

- relaxing orientation tolerance from `0.10` to `0.20` or `0.30` did not improve
  the result;
- increasing `max-iters` from `80` to `160` did not improve the result;
- reducing orientation weight improved position error somewhat but sacrificed
  orientation accuracy;
- removing the xArm6 TCP offset as a diagnostic improved the small-sample
  trajectory score, confirming that TCP offset contributes to pose difficulty.

The failure classification showed that xArm6 trajectory failures were not
orientation-only failures. They appeared as position failures under the
combined pose IK solve:

| Class | Trajectories |
| --- | ---: |
| Success | 363 |
| Max position and orientation exceed thresholds | 574 |
| Max position exceeds threshold, orientation does not | 563 |
| Max orientation exceeds threshold, position does not | 0 |

Key artifact:

- `outputs/xarm6_pose_diagnosis_it80/analysis_report.md`

Relevant commit:

- `fd88998 Add xArm6 canonical pose diagnosis`

### 6. Ran a pose-aware xArm6 base-placement search

The next diagnosis tested whether xArm6's low score came from the source-base
placement. We searched suite-specific base placements around the existing
source bases while keeping:

- the same `xarm6_true` model;
- the same TCP offset `[0, 0, 0.172]`;
- the same `canonical_tool` mapping;
- the same `0.10 rad` orientation tolerance;
- the same `max-iters=80`.

Full-suite validation results:

| Suite | Source-base trajectory pose | Best-base trajectory pose | Delta |
| --- | ---: | ---: | ---: |
| `libero_goal` | 0.394 | 1.000 | +0.606 |
| `libero_object` | 0.250 | 1.000 | +0.750 |
| `libero_spatial` | 0.082 | 1.000 | +0.918 |

Aggregated validation:

| Placement | Pose success | Trajectory pose success | Mean position error | Mean orientation error |
| --- | ---: | ---: | ---: | ---: |
| Source base | 0.719183 | 0.242 | 0.011200 m | 0.023499 rad |
| Suite-specific best base | 1.000000 | 1.000 | 0.000495 m | 0.001722 rad |

Best bases:

| Suite | Best base xyz | Best yaw |
| --- | --- | ---: |
| `libero_goal` | `(-0.420951, -0.106435, 0.931218)` | `0.006215` |
| `libero_object` | `(-0.271160, +0.044749, 0.057728)` | `0.107620` |
| `libero_spatial` | `(-0.465993, +0.027575, 0.896745)` | `0.635845` |

This changed the interpretation of the low xArm6 source-base result. The
strongest current explanation is that the Panda source-base placement is a poor
placement for xArm6 under pose constraints. The source-base result remains a
valid benchmark row, but it answers a strict question: how well xArm6 reproduces
Panda TCP poses when placed at the Panda source base. It does not mean xArm6 is
kinematically unable to solve the same pose distribution under a better
workcell placement.

Key artifact:

- `outputs/xarm6_pose_base_search_it80/analysis_report.md`

Relevant commit:

- `336ab99 Add xArm6 pose base placement search`

## Current Interpretation

The project now has a clearer benchmark structure:

1. Position-only source-base evaluation measures basic task-space position
   reachability.
2. Canonical pose source-base evaluation measures whether a robot can reproduce
   Panda TCP poses from the Panda source base using semantically aligned TCP
   frames.
3. Base-placement diagnostics measure whether failures are intrinsic to the
   robot model or caused by workcell placement.

For xArm6, the evidence now points to base placement as the dominant issue in
the low source-base canonical pose score. TCP offset and orientation constraints
still matter, but the suite-specific base search shows that those constraints
can be satisfied when xArm6 is mounted more favorably.

## Caveats

- The suite-specific best bases were found and validated on the same suite
  distributions. They are diagnostic results, not held-out generalization
  claims.
- The base-placement search was local and random. It proves good bases exist,
  not that the reported bases are globally optimal.
- The best-base result is suite-specific. A single global xArm6 base across all
  suites has not been tested yet.
- All pose results remain kinematic IK results. They do not model gripper
  geometry, collisions, dynamics, contacts, grasp success, or full task
  execution.
- `xarm6_true` and `ur5e_true_robotiq2f85_tcp_proxy` are still
  `tcp_offset_only` models, not full gripper simulations.

## Recommended Next Steps

1. Add a first-class MJCF base-placement search CLI so the xArm6 diagnostic is
   reproducible without ad hoc scripting.
2. Add train/test splitting for base-placement search to separate optimization
   from held-out evaluation.
3. Test a single global xArm6 base across all suites.
4. Add evaluator-level pose failure classification fields, such as
   `position_bad_count`, `orientation_bad_count`, and
   `position_and_orientation_bad_count`.
5. Keep source-base, canonical pose, optimized-base, and diagnostic zero-TCP
   rows clearly separated in reports.

## Recent Commit Timeline

| Commit | Summary |
| --- | --- |
| `2948b2b` | Add UR5e Robotiq TCP proxy model |
| `a4db81f` | Add UR5e Robotiq TCP proxy benchmark |
| `325b776` | Add source-base tool-frame audit |
| `54b5a43` | Add canonical tool-frame mapping |
| `cf60f73` | Add canonical pose benchmark results |
| `fd88998` | Add xArm6 canonical pose diagnosis |
| `336ab99` | Add xArm6 pose base placement search |
