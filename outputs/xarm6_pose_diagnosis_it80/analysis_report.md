# xArm6 Canonical Pose Diagnosis

## Background

The latest canonical pose benchmark evaluates `panda_true`, `xarm6_true`, and
`ur5e_true_robotiq2f85_tcp_proxy` on the same source-base LIBERO trajectories
with:

- `--base-pose-mode source`
- `--max-iters 80`
- `--max-waypoints-per-trajectory 80`
- `--evaluation-mode pose`
- `--orientation-format rotvec`
- `--orientation-tolerance 0.10`
- `--tool-frame-mapping canonical_tool`

The main question is why `xarm6_true` reports much lower pose trajectory success
than Panda and UR5e TCP:

| Robot | Pose waypoint success | Pose trajectory success |
| --- | ---: | ---: |
| `panda_true` | 0.995299 | 0.946667 |
| `xarm6_true` | 0.719183 | 0.242000 |
| `ur5e_true_robotiq2f85_tcp_proxy` | 0.996833 | 0.967333 |

This report diagnoses whether the low `xarm6_true` score is caused by basic
position reachability, orientation tolerance, iteration budget, TCP offset, or
the combined pose IK problem.

## Full-Run Findings

The strongest contrast is between position-only and pose-aware xArm6 runs.

| Run | Waypoint success | Trajectory success | Mean position error |
| --- | ---: | ---: | ---: |
| xArm6 position-only source-base IT80 | 1.000000 | 1.000000 | 0.000297 m |
| xArm6 canonical pose source-base IT80 | 0.719183 | 0.242000 | 0.011200 m |

So the low pose score is not because xArm6 cannot reach the LIBERO positions.
It appears when the benchmark requires the xArm6 TCP to match both the Panda
TCP position and the canonical tool orientation.

By suite, `libero_spatial` is the hardest:

| Suite | Pose waypoint success | Pose trajectory success | Mean position error | Mean orientation error |
| --- | ---: | ---: | ---: | ---: |
| `libero_goal` | 0.784373 | 0.394000 | 0.009383 m | 0.027339 rad |
| `libero_object` | 0.761950 | 0.250000 | 0.007608 m | 0.016330 rad |
| `libero_spatial` | 0.611242 | 0.082000 | 0.016609 m | 0.026832 rad |

Across the 1,500 canonical pose trajectories, the most common cross-robot
pattern is: Panda succeeds, UR5e TCP succeeds, but xArm6 fails. That happens in
1,026 trajectories.

Using each xArm6 trajectory's max errors against the benchmark thresholds
(`position <= 0.002 m`, `orientation <= 0.10 rad`), the failure split is:

| Class | Trajectories |
| --- | ---: |
| Success | 363 |
| Max position and orientation exceed thresholds | 574 |
| Max position exceeds threshold, orientation does not | 563 |
| Max orientation exceeds threshold, position does not | 0 |

This is important: the trajectory failures are not primarily
orientation-only failures. They show up as position failure under the combined
pose IK solve. Orientation can also be bad, but every failed xArm6 trajectory
has a position violation.

## Parameter Sweep

A small deterministic sweep used the same 150 sampled trajectories and 12,000
waypoints for each config. These are diagnostic runs, not replacement benchmark
rows.

| Config | TCP offset | Tol | Weight | Max iters | Waypoint success | Trajectory success | Mean pos err | Mean ori err |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| position-only | `[0, 0, 0.172]` | - | - | 80 | 1.000000 | 1.000000 | 0.000302 | - |
| pose baseline | `[0, 0, 0.172]` | 0.10 | 1.00 | 80 | 0.727250 | 0.240000 | 0.010535 | 0.022164 |
| pose tol 0.20 | `[0, 0, 0.172]` | 0.20 | 1.00 | 80 | 0.727250 | 0.240000 | 0.010537 | 0.022382 |
| pose tol 0.30 | `[0, 0, 0.172]` | 0.30 | 1.00 | 80 | 0.727250 | 0.240000 | 0.010538 | 0.022443 |
| pose weight 0.25 | `[0, 0, 0.172]` | 0.10 | 0.25 | 80 | 0.735500 | 0.253333 | 0.006457 | 0.032160 |
| pose weight 0.10 | `[0, 0, 0.172]` | 0.10 | 0.10 | 80 | 0.780583 | 0.313333 | 0.002044 | 0.051004 |
| pose max-iters 160 | `[0, 0, 0.172]` | 0.10 | 1.00 | 160 | 0.727250 | 0.240000 | 0.010535 | 0.022171 |
| zero-TCP diagnostic | `[0, 0, 0]` | 0.10 | 1.00 | 80 | 0.829583 | 0.513333 | 0.007013 | 0.006668 |

Interpretation:

- Relaxing orientation tolerance from 0.10 to 0.20 or 0.30 does not improve the
  result. This is expected because most failures violate position tolerance.
- Increasing `max-iters` from 80 to 160 does not improve the result. The current
  failures are not simply due to stopping too early.
- Lowering `orientation_weight` improves position error and success somewhat,
  but increases orientation error. It trades away orientation fidelity rather
  than solving the full pose requirement.
- Removing the 0.172 m TCP offset improves the small-sample trajectory score
  from 0.24 to 0.513333. This confirms that TCP offset materially contributes
  to xArm6 pose difficulty. The zero-TCP row is diagnostic only; it is not the
  true xArm6 gripper TCP benchmark.

## Representative Episode

The episode
`libero_spatial/pick_up_the_black_bowl_in_the_top_drawer_of_the_wooden_cabinet_and_place_it_on_the_plate_demo/demo_31`
was selected because Panda and UR5e TCP complete it, while xArm6 only solves
10% of the waypoints in the canonical pose run.

| Config | Success count | Position-bad count | Orientation-bad count | Position+orientation bad | Mean pos err | Mean ori err |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| position-only, TCP offset 0.172 | 80 / 80 | 0 | 0 | 0 | 0.000388 | - |
| pose baseline, TCP offset 0.172 | 8 / 80 | 72 | 40 | 40 | 0.057982 | 0.120905 |
| pose weight 0.10, TCP offset 0.172 | 11 / 80 | 69 | 66 | 66 | 0.009299 | 0.245581 |
| pose zero-TCP diagnostic | 15 / 80 | 65 | 12 | 12 | 0.044755 | 0.048280 |

Again, all pose failures in this representative episode include position
violation. There are no orientation-only failures. Reducing orientation weight
reduces position error but leaves orientation far outside tolerance. Removing
the TCP offset reduces orientation error but does not make this hard spatial
episode pass.

## Conclusion

The current `xarm6_true` canonical pose score is plausible for the benchmark as
defined. The data are Panda-arm TCP poses. Under position-only evaluation,
xArm6 can reach the sampled positions. Under canonical pose evaluation, xArm6
must place its gripper TCP at the same points while also matching the same
parallel-jaw TCP orientation. That combined requirement is much harder for the
xArm6 kinematic chain and 0.172 m gripper TCP offset.

The low trajectory score is also amplified by the metric definition: a
trajectory succeeds only if every selected waypoint succeeds. A waypoint-level
pose success rate of 0.719183 can therefore become a much lower trajectory
success rate of 0.242.

This diagnosis does not prove that every xArm6 pose failure is physically
infeasible. It shows that the current evaluator's pose IK cannot satisfy those
position and orientation constraints for many xArm6 waypoints, and that the
main observed failure mode is position violation under the combined pose solve.

## Recommended Next Work

1. Add built-in pose failure classification to the evaluator output:
   `position_bad_count`, `orientation_bad_count`, and `position_and_orientation_bad_count`
   per trajectory.
2. Run a pose-aware xArm6 base-placement search. Source-base placement is fair
   as a baseline, but it may not be the best xArm6 workcell placement for
   canonical pose reachability.
3. Add an IK robustness experiment with more diverse seeds and solver settings
   to separate true kinematic infeasibility from local-minimum solver failures.
4. Keep zero-TCP and low-orientation-weight rows clearly labeled as diagnostics,
   not canonical fair benchmark rows.

## Artifacts

- Full-run summary: `outputs/xarm6_pose_diagnosis_it80/full_run_diagnostic_summary.csv`
- Cross-robot success patterns: `outputs/xarm6_pose_diagnosis_it80/cross_robot_success_patterns.csv`
- xArm6 failure classes: `outputs/xarm6_pose_diagnosis_it80/xarm6_failure_classification.csv`
- xArm6 hardest tasks: `outputs/xarm6_pose_diagnosis_it80/xarm6_hardest_tasks.csv`
- Sweep summary: `outputs/xarm6_pose_diagnosis_it80/sweep_summary.csv`
- Representative episode summary: `outputs/xarm6_pose_diagnosis_it80/representative_episode_summary.csv`
- Representative episode waypoint details: `outputs/xarm6_pose_diagnosis_it80/representative_episode_waypoints.csv`
