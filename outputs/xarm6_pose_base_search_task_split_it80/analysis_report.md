# xArm6 Pose Base Search with Task Train/Test Split

## Purpose

The previous suite-specific base search showed that good xArm6 base placements
exist for the canonical pose benchmark, but it searched and validated on the
same suite distributions. This run adds a stricter train/test split:

- split mode: `task`
- train fraction: `0.7`
- search scope: `suite`
- robot: `xarm6_true`
- TCP: official xArm gripper TCP offset `[0, 0, 0.172]`
- evaluation: canonical pose IK, `rotvec`, orientation tolerance `0.10 rad`
- IK budget: `max-iters=80`

For each suite, candidate bases are selected only using train tasks. The chosen
train base is then evaluated on held-out test tasks.

## Method

For each suite:

1. Split tasks into train and test sets.
2. Evaluate the reference source base and 15 random candidates around that
   reference base.
3. Use at most 60 train trajectories for candidate ranking.
4. Select the best train candidate by trajectory pose success, then waypoint
   pose success, then lower position and orientation error.
5. Validate both the reference base and the best train-selected base on all
   train and test trajectories for that suite.

Each suite has 350 train trajectories and 150 test trajectories. All validation
rows use up to 80 waypoints per trajectory.

## Held-Out Test Results

| Suite | Reference test traj pose | Best-base test traj pose | Delta | Reference test waypoint pose | Best-base test waypoint pose |
| --- | ---: | ---: | ---: | ---: | ---: |
| `libero_goal` | 0.360000 | 1.000000 | +0.640000 | 0.776167 | 1.000000 |
| `libero_object` | 0.040000 | 0.986667 | +0.946667 | 0.573833 | 0.999000 |
| `libero_spatial` | 0.073333 | 0.466667 | +0.393334 | 0.632417 | 0.767333 |

The train-selected base generalizes very strongly for `libero_goal` and
`libero_object`. For `libero_spatial`, the result still improves substantially,
but it does not fully solve the held-out tasks.

## Aggregate Results

| Split | Base | Pose success | Traj pose success | Mean pos err | Mean ori err |
| --- | --- | ---: | ---: | ---: | ---: |
| Train | Reference base | 0.744210 | 0.278095 | 0.010780 | 0.022984 |
| Train | Best train base | 0.951878 | 0.860952 | 0.002368 | 0.010911 |
| Test | Reference base | 0.660806 | 0.157778 | 0.012178 | 0.024702 |
| Test | Best train base | 0.922111 | 0.817778 | 0.003413 | 0.021943 |

The aggregate held-out test trajectory pose success increases from `0.157778`
to `0.817778`. This is strong evidence that base placement improvements are not
only an in-sample artifact.

## Best Train-Selected Bases

| Suite | Candidate | Base xyz | Base yaw | Train traj pose | Test traj pose |
| --- | ---: | --- | ---: | ---: | ---: |
| `libero_goal` | 14 | `(-0.315288, -0.133681, 1.025004)` | `-0.467094` | 0.985714 | 1.000000 |
| `libero_object` | 14 | `(-0.447235, 0.229120, 0.074962)` | `-0.348135` | 0.997143 | 0.986667 |
| `libero_spatial` | 9 | `(-0.538064, 0.186874, 0.718466)` | `-0.563609` | 0.600000 | 0.466667 |

The best bases selected under task split differ from the previous full-suite
diagnostic bases. That is expected because the objective is now train-task
performance, not full-suite performance.

## Interpretation

This run strengthens the earlier base-placement conclusion:

- `libero_goal`: base placement generalizes cleanly to held-out tasks.
- `libero_object`: base placement generalizes almost completely to held-out
  tasks.
- `libero_spatial`: base placement still helps a lot, but the train-selected
  local search does not fully cover held-out spatial tasks.

So the source-base xArm6 score is still best explained as a placement-sensitive
result, not as a pure xArm6 kinematic impossibility. However, the spatial suite
shows that base search needs either more candidates, a better search strategy,
or a more robust objective if we want held-out task success near 1.0.

## Caveats

- The search is random and local around each suite's reference base.
- Only 15 random candidates were evaluated per suite.
- Candidate ranking used at most 60 train trajectories per suite, while
  validation used all train/test trajectories.
- This run tests suite-specific bases, not one global base across all suites.
- This remains a kinematic pose IK benchmark. It does not model collision
  geometry, gripper dynamics, contact, grasp success, or full task execution.

## Recommended Next Work

1. Increase the candidate budget for `libero_spatial`, or use a two-stage
   coarse-to-fine search around promising candidates.
2. Add a global-base task-split run to test whether one xArm6 placement can
   serve all suites.
3. Add pose failure classification to the base-search outputs so search can
   optimize against position vs orientation failures explicitly.
4. Consider ranking candidates on a validation-like train holdout rather than a
   single train-search subset.

## Artifacts

- Search candidates: `outputs/xarm6_pose_base_search_task_split_it80/search_candidates.csv`
- Best train bases: `outputs/xarm6_pose_base_search_task_split_it80/best_train_base.csv`
- Train/test rows: `outputs/xarm6_pose_base_search_task_split_it80/train_test_results.csv`
- Train/test comparison: `outputs/xarm6_pose_base_search_task_split_it80/train_test_comparison.csv`
- Aggregate comparison: `outputs/xarm6_pose_base_search_task_split_it80/train_test_aggregate_comparison.csv`
- Metadata and split details: `outputs/xarm6_pose_base_search_task_split_it80/metadata.json`
