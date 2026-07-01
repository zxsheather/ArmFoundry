# Tool-Frame Audit: Source-Base IT80 Benchmarks

## Summary

This audit consolidates the tool-frame semantics for the source-base, `max-iters=80` benchmark family. It uses the existing position-only baseline, pose-aware baseline, and the newly added UR5e Robotiq TCP proxy benchmark.

Main conclusion: the benchmark now has a clear `tcp_offset_only` cross-robot family containing `xarm6_true` and `ur5e_true_robotiq2f85_tcp_proxy`. These two rows are the most comparable current real-arm TCP reachability baselines, but both still omit full gripper geometry, collisions, actuation, and contact. `ur5e_true` remains an attachment-site diagnostic, not a gripper TCP baseline.

## Benchmark Inputs

- Position-only baseline: `outputs/sourcebase_true_proxy_all_it80/baseline_results.csv`
- Pose-aware baseline: `outputs/sourcebase_true_pose_it80/baseline_results.csv`
- UR5e Robotiq TCP position baseline: `outputs/ur5e_robotiq2f85_tcp_proxy_sourcebase_it80/position/baseline_results.csv`
- UR5e Robotiq TCP pose baseline: `outputs/ur5e_robotiq2f85_tcp_proxy_sourcebase_it80/pose/baseline_results.csv`

Run settings: source-base placement, max 80 waypoints per trajectory, `max-iters=80`, position tolerance `0.002 m`, pose orientation tolerance `0.10 rad`, and `ee_ori` interpreted as `rotvec` in pose-aware runs. Quaternions in this audit are unit-normalized for readability.

## Audit Table

| Robot | Group | Tool modeling | Target frame | Offset | Unit quat | Concrete gripper | Pos traj | Pose traj |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| `simple_default` | `proxy_endpoint_diagnostic` | `proxy_endpoint` | `end_effector` | `[0, 0, 0]` | `[1, 0, 0, 0]` | false | 0.603333 | n/a |
| `panda_proxy` | `proxy_endpoint_diagnostic` | `proxy_endpoint` | `end_effector` | `[0, 0, 0]` | `[1, 0, 0, 0]` | false | 0.283333 | n/a |
| `ur5_proxy` | `proxy_endpoint_diagnostic` | `proxy_endpoint` | `end_effector` | `[0, 0, 0]` | `[1, 0, 0, 0]` | false | 1 | n/a |
| `xarm6_proxy` | `proxy_endpoint_diagnostic` | `proxy_endpoint` | `end_effector` | `[0, 0, 0]` | `[1, 0, 0, 0]` | false | 0.198 | n/a |
| `panda_true` | `source_robot_gripper_chain` | `source_gripper_chain` | `gripper0_grip_site` | `[0, 0, 0]` | `[1, 0, 0, 0]` | true | 1 | 0.946667 |
| `ur5e_true` | `attachment_site_diagnostic` | `attachment_site_only` | `attachment_site` | `[0, 0.1, 0]` | `[-0.70710678, 0.70710678, 0, 0]` | false | 1 | 0.788 |
| `xarm6_true` | `tcp_offset_only_kinematic_tcp` | `tcp_offset_only` | `link_tcp` | `[0, 0, 0.172]` | `[1, 0, 0, 0]` | false | 1 | 0.242 |
| `ur5e_true_robotiq2f85_tcp_proxy` | `tcp_offset_only_kinematic_tcp` | `tcp_offset_only` | `ur5e_robotiq2f85_tcp` | `[0, 0.2558, 0]` | `[-0.5, 0.5, 0.5, 0.5]` | false | 1 | 0.967333 |

## Comparison Groups

| Group | Robots | Status |
| --- | --- | --- |
| `attachment_site_diagnostic` | ur5e_true | diagnostic_only_missing_gripper_tcp |
| `proxy_endpoint_diagnostic` | simple_default, panda_proxy, ur5_proxy, xarm6_proxy | diagnostic_only |
| `source_robot_gripper_chain` | panda_true | source_reference_only |
| `tcp_offset_only_kinematic_tcp` | xarm6_true, ur5e_true_robotiq2f85_tcp_proxy | currently_best_cross_robot_tcp_family |

## Current Fair Comparison Set

The current kinematic TCP comparison set is:

| Robot | Target frame | Offset | Unit quat | Pose IK | Pose traj | Mean ori err rad | Gap |
| --- | --- | --- | --- | --- | --- | --- | --- |
| `xarm6_true` | `link_tcp` | `[0, 0, 0.172]` | `[1, 0, 0, 0]` | 0.719183 | 0.242 | 0.023499 | Missing meshes, collision geometry, gripper joints, actuation, and contact behavior. |
| `ur5e_true_robotiq2f85_tcp_proxy` | `ur5e_robotiq2f85_tcp` | `[0, 0.2558, 0]` | `[-0.5, 0.5, 0.5, 0.5]` | 0.996833 | 0.967333 | 0.001411 | Missing meshes, collision geometry, gripper joints, actuation, and contact behavior. |

This comparison is fair only at the frame-level kinematic TCP layer. It asks whether the documented TCP frame can reach the source Panda end-effector poses from the source-base placement. It does not test gripper-body collision, finger closure, contact, or task success.

## Interpretation Rules

- `panda_true` is the source Panda gripper-chain reference. It is the correct source robot baseline, but it should not be treated as a cross-embodiment hardware comparison row.
- `ur5e_true` evaluates the MuJoCo Menagerie UR5e `attachment_site` on `wrist_3_link`. It is useful for wrist/tool-mount diagnostics, but should not be compared as a Robotiq gripper TCP result.
- `ur5e_true_robotiq2f85_tcp_proxy` evaluates a Robotiq 2F-85 `pinch` TCP proxy composed onto the UR5e `attachment_site`. This fixes the UR5e target-frame semantics for frame-level TCP reachability, but it remains `tcp_offset_only`.
- `xarm6_true` evaluates the official xArm gripper TCP offset from `link6` to `link_tcp`. It is also `tcp_offset_only` and therefore belongs in the same current comparison family as the UR5e TCP proxy.
- `simple_default`, `panda_proxy`, `ur5_proxy`, and `xarm6_proxy` are simplified endpoint chains. They are useful for proxy sanity checks and optimization experiments, not for real hardware claims.

## Remaining Fairness Gaps

1. The `tcp_offset_only` family still lacks physical gripper geometry, collision, joints, actuation, and contact.
2. Pose-aware comparison is still diagnostic unless the source Panda tool frame and each target robot TCP frame are explicitly mapped through a canonical tool-frame convention.
3. `panda_true` uses the source gripper chain, while xArm6 and UR5e currently use TCP offsets without full gripper bodies. This difference should remain visible in tables and reports.
4. The old `ur5e_true` attachment-site result should be kept as a diagnostic baseline but excluded from gripper-TCP fairness claims.

## Recommended Next Step

Use this audit as the comparison contract for future runs:

1. Report `source_gripper_chain`, `tcp_offset_only`, `attachment_site_only`, and `proxy_endpoint` rows separately.
2. For near-term cross-robot kinematic comparison, compare `xarm6_true` against `ur5e_true_robotiq2f85_tcp_proxy` as `tcp_offset_only` rows.
3. If the project needs stronger physical realism, add full gripper models for UR5e and xArm6 before making manipulation or collision claims.
4. If the project needs pose fairness, define and document a canonical source-to-target tool-frame mapping before treating pose success as a fair cross-robot task metric.

## Outputs

- Audit CSV: `outputs/tool_frame_audit_sourcebase_it80/tool_frame_audit.csv`
- Group summary CSV: `outputs/tool_frame_audit_sourcebase_it80/comparison_groups.csv`
- Audit metadata: `outputs/tool_frame_audit_sourcebase_it80/audit_metadata.json`
