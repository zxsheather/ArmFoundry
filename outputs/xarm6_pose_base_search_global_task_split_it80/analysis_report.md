# xArm6 Global Base Search with Task Train/Test Split

## Purpose

The previous task-split search used one xArm6 base per LIBERO suite. This run
tests a stricter and more deployment-like question:

```text
Can one global xArm6 base placement serve all goal/object/spatial tasks?
```

The run keeps the same canonical pose evaluation setup:

- robot: `xarm6_true`
- TCP: official xArm gripper TCP offset `[0, 0, 0.172]`
- tool-frame mapping: `canonical_tool`
- orientation format: `rotvec`
- orientation tolerance: `0.10 rad`
- IK budget: `max-iters=80`
- split mode: task split, train fraction `0.7`
- search scope: `global`

## Method

All 1,500 trajectories are grouped into one global search group. Tasks are split
into train and test sets:

- train: 1,050 trajectories from 21 tasks
- test: 450 trajectories from 9 held-out tasks

The search evaluates a global reference base plus 31 random candidate bases.
Candidate ranking uses at most 180 train trajectories, with up to 80 waypoints
per trajectory. The best train candidate is then validated on all train and test
trajectories.

Important: for `search-scope=global`, the reference base is the mean of all
selected source bases. It is not the original suite-specific source-base setup.
Here that mean reference base is:

```text
(-0.640000, 0.000000, 0.608000), yaw 0
```

This reference is useful for global-search comparison, but it should not be
reported as the canonical suite-specific source-base benchmark.

## Result

| Base | Split | Pose success | Traj pose success | Mean pos err | Mean ori err |
| --- | --- | ---: | ---: | ---: | ---: |
| Reference global base | Train | 0.200669 | 0.000000 | 0.049394 | 0.170142 |
| Best global train base | Train | 0.900064 | 0.547619 | 0.003489 | 0.007943 |
| Reference global base | Test | 0.290348 | 0.000000 | 0.051215 | 0.193582 |
| Best global train base | Test | 0.839638 | 0.448889 | 0.008424 | 0.014155 |

The best global base is:

```text
base xyz = (-0.304931, -0.138489, 0.644111)
base yaw = 0.677363
```

Relative to the global reference base, this is:

```text
dx = +0.335069 m
dy = -0.138489 m
dz = +0.036111 m
yaw delta = +0.677363 rad
```

## Comparison To Suite-Specific Task Split

| Search scope | Test pose success | Test traj pose success |
| --- | ---: | ---: |
| Global best train base | 0.839638 | 0.448889 |
| Suite-specific best train bases | 0.922111 | 0.817778 |

The global base improves substantially over the global reference base, but it
does not match suite-specific placement. A single xArm6 base has to compromise
between goal, object, and spatial pose distributions, while suite-specific
placement can align the robot differently for each distribution.

## Interpretation

This run answers the global-base question:

- A single global xArm6 base can generalize to held-out tasks better than the
  global reference base.
- The improvement is meaningful: held-out trajectory pose success rises from
  `0.000000` to `0.448889`.
- The result is still much weaker than suite-specific base placement, which
  reached `0.817778` held-out trajectory pose success.

So the current evidence suggests:

1. Base placement is still a major driver of xArm6 canonical pose success.
2. A single global base is possible but not enough to recover suite-specific
   performance.
3. The remaining failures likely come from conflicting placement requirements
   across suites, especially the spatial tasks.

## Caveats

- The search is random and local around the global reference base.
- Only 31 random candidates were evaluated.
- Candidate ranking used a subset of 180 train trajectories, while validation
  used all train/test trajectories.
- The global reference base is an averaged source base, not the canonical
  suite-specific source-base baseline.
- This remains a kinematic pose IK benchmark. It does not model collisions,
  full gripper geometry, contact, grasp success, or task execution.

## Recommended Next Work

1. Run a larger global search or a two-stage coarse-to-fine global search.
2. Add per-suite breakdown for the global best base to identify which suite
   dominates the remaining held-out failures.
3. Run a targeted `libero_spatial` coarse-to-fine search, since spatial remains
   the hardest suite under task split.
4. Consider multi-objective ranking that penalizes suite imbalance, not just
   aggregate train trajectory success.

## Artifacts

- Search candidates: `outputs/xarm6_pose_base_search_global_task_split_it80/search_candidates.csv`
- Best train base: `outputs/xarm6_pose_base_search_global_task_split_it80/best_train_base.csv`
- Train/test rows: `outputs/xarm6_pose_base_search_global_task_split_it80/train_test_results.csv`
- Train/test comparison: `outputs/xarm6_pose_base_search_global_task_split_it80/train_test_comparison.csv`
- Aggregate comparison: `outputs/xarm6_pose_base_search_global_task_split_it80/train_test_aggregate_comparison.csv`
- Metadata and task split: `outputs/xarm6_pose_base_search_global_task_split_it80/metadata.json`
