# Canonical Pose-Fairness Benchmark: Source-Base IT80

## Summary

This rerun evaluates only the rows eligible for `--tool-frame-mapping canonical_tool`: `panda_true`, `xarm6_true`, and `ur5e_true_robotiq2f85_tcp_proxy`. The run uses source-base placement, `max-iters=80`, at most 80 waypoints per trajectory, `ee_ori` decoded as `rotvec`, and orientation tolerance `0.10 rad`.

The numerical results match the earlier identity-mapping pose runs, as expected. The current canonical mappings are identity rotations because these TCP frames are already aligned with the canonical parallel-jaw pinch TCP axes. The difference is semantic: these outputs now carry `tool_frame_mapping=canonical_tool`, `pose_fairness_claim=true`, and explicit mapping eligibility metadata.

## Canonical Results

| Robot | Tool modeling | Target frame | Mapping status | Pose IK | Pose traj | Mean pos err m | Mean ori err rad | Max ori err rad |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| `panda_true` | `source_gripper_chain` | `gripper0_grip_site` | `source_canonical` | 0.995299 | 0.946667 | 0.000495 | 0.011822 | 1.686391 |
| `xarm6_true` | `tcp_offset_only` | `link_tcp` | `mapped_tcp_offset_only` | 0.719183 | 0.242 | 0.0112 | 0.023499 | 1.244216 |
| `ur5e_true_robotiq2f85_tcp_proxy` | `tcp_offset_only` | `ur5e_robotiq2f85_tcp` | `mapped_tcp_offset_only` | 0.996833 | 0.967333 | 0.000395 | 0.001411 | 0.09955 |

## Identity vs Canonical Check

| Robot | Identity pose traj | Canonical pose traj | Delta | Identity mean ori | Canonical mean ori | Delta |
| --- | --- | --- | --- | --- | --- | --- |
| `panda_true` | 0.946667 | 0.946667 | 0 | 0.011822 | 0.011822 | 0 |
| `xarm6_true` | 0.242 | 0.242 | 0 | 0.023499 | 0.023499 | 0 |
| `ur5e_true_robotiq2f85_tcp_proxy` | 0.967333 | 0.967333 | 0 | 0.001411 | 0.001411 | 0 |

The zero deltas confirm that this rerun did not change orientation targets numerically. It changed the result set from a diagnostic identity run into an explicitly gated canonical pose-fairness run.

## Included Rows

- `panda_true`: source Panda gripper-chain reference; source canonical TCP.
- `xarm6_true`: official xArm gripper `link_tcp`; `tcp_offset_only`; canonical mapping eligible.
- `ur5e_true_robotiq2f85_tcp_proxy`: Robotiq 2F-85 pinch TCP proxy; `tcp_offset_only`; canonical mapping eligible.

## Excluded Rows

- `ur5e_true`: remains an attachment-site diagnostic, not a gripper TCP canonical frame.
- `*_proxy`: simplified endpoint chains without concrete tool-frame axes.

## Interpretation

This benchmark is fair at the frame-level kinematic TCP layer. It asks whether the same canonical parallel-jaw TCP orientation can be reached by each eligible robot's documented TCP. It still does not model gripper meshes, collision geometry, finger joints, actuation, contact, or grasp success.

The main technical follow-up is unchanged: `xarm6_true` remains much lower in pose trajectory success (`0.242`) than `panda_true` (`0.946667`) and `ur5e_true_robotiq2f85_tcp_proxy` (`0.967333`). Since canonical mapping did not change its targets, the likely causes are xArm6 kinematics, joint limits, base placement, TCP offset, orientation constraint difficulty, or IK convergence behavior.

## Outputs

- Root results: `outputs/sourcebase_canonical_pose_it80/baseline_results.csv`
- Suite results: `outputs/sourcebase_canonical_pose_it80/baseline_results_by_suite.csv`
- Identity comparison: `outputs/sourcebase_canonical_pose_it80/identity_vs_canonical_pose_comparison.csv`
- Benchmark metadata: `outputs/sourcebase_canonical_pose_it80/benchmark_metadata.json`
- Per-robot outputs: `outputs/sourcebase_canonical_pose_it80/<robot_name>/`
