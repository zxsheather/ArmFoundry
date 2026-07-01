# xArm6 Pose-Aware Base Placement Search

## Purpose

The previous diagnosis showed that `xarm6_true` is low in the canonical pose
benchmark:

| Suite | Source-base pose success | Source-base trajectory pose success |
| --- | ---: | ---: |
| `libero_goal` | 0.784373 | 0.394 |
| `libero_object` | 0.761950 | 0.250 |
| `libero_spatial` | 0.611242 | 0.082 |

The open question was whether this low score is intrinsic to xArm6 pose
reachability, or whether the Panda source-base placement is simply a poor
placement for xArm6.

This experiment performs a suite-specific pose-aware base placement search for
`xarm6_true`. It keeps the real xArm6 TCP offset (`[0, 0, 0.172]`), canonical
tool-frame mapping, orientation tolerance `0.10 rad`, and `max-iters=80`.

## Method

For each suite:

1. Start from the existing source base:
   - `libero_goal`: `(-0.66, 0.0, 0.912)`, yaw `0`
   - `libero_object`: `(-0.60, 0.0, 0.0)`, yaw `0`
   - `libero_spatial`: `(-0.66, 0.0, 0.912)`, yaw `0`
2. Evaluate the source base and 15 deterministic random base candidates around
   it.
3. Search sample: 60 trajectories per suite, max 40 waypoints per trajectory.
4. Select the best candidate by:
   trajectory pose success, then waypoint pose success, then lower position
   and orientation error.
5. Validate source base and best candidate on the full suite:
   500 trajectories, max 80 waypoints per trajectory.

This is a diagnostic base-placement search, not a replacement for the canonical
source-base benchmark. The best bases are suite-specific, matching the fact
that the benchmark source base is already suite-specific.

## Full-Suite Validation

| Suite | Source traj pose | Best traj pose | Delta | Source waypoint pose | Best waypoint pose |
| --- | ---: | ---: | ---: | ---: | ---: |
| `libero_goal` | 0.394 | 1.000 | +0.606 | 0.784373 | 1.000 |
| `libero_object` | 0.250 | 1.000 | +0.750 | 0.761950 | 1.000 |
| `libero_spatial` | 0.082 | 1.000 | +0.918 | 0.611242 | 1.000 |

Aggregated over the three full-suite validation rows:

| Placement | Trajectories | Waypoints | Pose success | Trajectory pose success | Mean pos err | Mean ori err |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| Source base | 1500 | 119975 | 0.719183 | 0.242 | 0.011200 | 0.023499 |
| Suite-specific best base | 1500 | 119975 | 1.000000 | 1.000 | 0.000495 | 0.001722 |

The full validation is decisive: for this dataset and evaluator, xArm6 can
solve all sampled canonical pose waypoints when placed better for each suite.
The low source-base score is therefore not mainly an intrinsic xArm6 pose
workspace limitation.

## Best Bases

| Suite | Best base xyz | Best yaw | Offset from source |
| --- | --- | ---: | --- |
| `libero_goal` | `(-0.420951, -0.106435, 0.931218)` | `0.006215` | `(+0.239049, -0.106435, +0.019218)`, yaw `+0.006215` |
| `libero_object` | `(-0.271160, +0.044749, 0.057728)` | `0.107620` | `(+0.328840, +0.044749, +0.057728)`, yaw `+0.107620` |
| `libero_spatial` | `(-0.465993, +0.027575, 0.896745)` | `0.635845` | `(+0.194007, +0.027575, -0.015255)`, yaw `+0.635845` |

All three best bases move xArm6 in the positive world-x direction relative to
the Panda source base. `libero_spatial` additionally needs a large yaw rotation
of about `0.636 rad` (`36.4 deg`). This suggests that xArm6's difficult region
under source-base placement is strongly tied to where the arm is mounted and
how its shoulder/elbow/wrist chain is oriented relative to the Panda-generated
task poses.

## Interpretation

The earlier low `xarm6_true` result is now best explained as a base-placement
effect under a strict source-base benchmark:

- The source-base benchmark is useful because it asks: "What happens if each
  robot is put at the LIBERO source robot base?"
- But that source base was not optimized for xArm6. It was inherited from the
  Panda source setup.
- When xArm6 is moved to a better suite-specific base, the same robot, same TCP
  offset, same canonical tool-frame mapping, same orientation tolerance, and
  same IK settings can solve the full selected benchmark.

This does not make the source-base result a bug. It means source-base and
optimized-base answer different questions:

- Source-base result: how well xArm6 reproduces Panda TCP poses from the Panda
  base placement.
- Optimized-base diagnostic: whether xArm6 has enough kinematic capability if
  its workcell placement is allowed to change.

## Caveats

- The best bases were found using the same suite distribution later validated.
  The full-suite result is strong for this dataset, but it is not a held-out
  generalization claim.
- The search was random and local around source base; it proves that good bases
  exist, not that these are globally optimal.
- The best bases are suite-specific. A single fixed base across all LIBERO
  suites was not tested here.
- This remains a kinematic pose IK benchmark. It does not model gripper
  geometry, collisions, dynamics, grasp success, or task execution.

## Updated Answer To The Cause Question

The most likely reason `xarm6_true` was low is not that xArm6 cannot reach the
canonical Panda TCP pose distribution at all. The strongest evidence now says
the Panda source-base placement is a poor placement for xArm6. The real TCP
offset and orientation constraint still matter, but a better base placement
removes the observed failures in this benchmark.

## Recommended Next Work

1. Add a first-class base-placement experiment CLI for MJCF robots so this
   diagnostic is reproducible without ad hoc scripts.
2. Add train/test splitting for base placement search to separate optimization
   from held-out evaluation.
3. Test a single global xArm6 base across all suites, because this run only
   searched suite-specific bases.
4. Add evaluator-level pose failure classification to make future reports show
   whether failures are position-only, orientation-only, or both.

## Artifacts

- Search candidates: `outputs/xarm6_pose_base_search_it80/search_candidates.csv`
- Best search rows: `outputs/xarm6_pose_base_search_it80/best_search_rows.csv`
- Full validation rows: `outputs/xarm6_pose_base_search_it80/validation_results.csv`
- Full validation comparison: `outputs/xarm6_pose_base_search_it80/validation_comparison.csv`
- Aggregate comparison: `outputs/xarm6_pose_base_search_it80/validation_aggregate_comparison.csv`
- Metadata: `outputs/xarm6_pose_base_search_it80/metadata.json`
