# Source-Base True/Proxy Benchmark, max-iters=80

## Summary

This is the unified source-base benchmark for all currently available proxy and true kinematic models. It reruns the formal comparison with `--max-iters 80` for every robot so the result reflects kinematic reachability rather than inconsistent IK solver budget.

The run evaluates 1,500 LIBERO trajectories and 119,975 selected waypoints per robot.

## Why rerun

The previous `ur5e_true` run used `--max-iters 35` and produced:

| Robot | max-iters | IK success | Trajectory success |
|---|---:|---:|---:|
| `ur5e_true` | 35 | 0.987 | 0.000 |

Per-trajectory inspection showed exactly one failed waypoint in every trajectory, and sampled failures were at waypoint index 0. A 50-trajectory probe with `max-iters=80` resolved those failures, so the formal benchmark was rerun with a uniform 80-iteration IK budget.

## Commands

Internal baselines plus `panda_true`:

```bash
uv run python -m ddird.experiments.run_eval_baselines \
  --data data/libero_ee_trajectories_armforge \
  --outputs outputs/sourcebase_true_proxy_all_it80/internal_baselines \
  --max-waypoints-per-trajectory 80 \
  --num-workers 16 \
  --max-iters 80 \
  --base-pose-mode source \
  --include-true-models
```

External UR5e true model:

```bash
uv run python -m ddird.experiments.run_eval_mjcf_robot \
  --mjcf data/armforge/models/mujoco_menagerie/universal_robots_ur5e/ur5e.xml \
  --robot-name ur5e_true \
  --base-body base \
  --target-site attachment_site \
  --target-body wrist_3_link \
  --data data/libero_ee_trajectories_armforge \
  --outputs outputs/sourcebase_true_proxy_all_it80/ur5e_true \
  --max-waypoints-per-trajectory 80 \
  --num-workers 16 \
  --max-iters 80 \
  --base-pose-mode source
```

External xArm6 true-kinematics model:

```bash
uv run python -m ddird.experiments.run_eval_mjcf_robot \
  --mjcf outputs/generated_models/xarm6_true/xarm6_true.xml \
  --robot-name xarm6_true \
  --base-body link_base \
  --target-site link_tcp \
  --target-body link6 \
  --data data/libero_ee_trajectories_armforge \
  --outputs outputs/sourcebase_true_proxy_all_it80/xarm6_true \
  --max-waypoints-per-trajectory 80 \
  --num-workers 16 \
  --max-iters 80 \
  --base-pose-mode source
```

The run was executed in `tmux` as three sequential commands.

## Overall Results

| Robot | Type | IK success | Trajectory success | Mean position error | Mean manipulability |
|---|---|---:|---:|---:|---:|
| `panda_true` | source true model | 1.000 | 1.000 | 0.00029 | 0.19146 |
| `ur5_proxy` | simplified proxy | 1.000 | 1.000 | 0.00031 | 0.12882 |
| `ur5e_true` | external true model | 1.000 | 1.000 | 0.00031 | 0.12557 |
| `xarm6_true` | generated true-kinematics model | 0.891 | 0.655 | 0.00364 | 0.06523 |
| `simple_default` | simplified proxy | 0.856 | 0.603 | 0.00571 | 0.06204 |
| `panda_proxy` | simplified proxy | 0.694 | 0.283 | 0.02236 | 0.03651 |
| `xarm6_proxy` | simplified proxy | 0.637 | 0.198 | 0.02620 | 0.03008 |

## Suite Breakdown

| Robot | Goal IK | Object IK | Spatial IK | Goal traj | Object traj | Spatial traj |
|---|---:|---:|---:|---:|---:|---:|
| `panda_true` | 1.000 | 1.000 | 1.000 | 1.000 | 1.000 | 1.000 |
| `ur5_proxy` | 1.000 | 1.000 | 1.000 | 1.000 | 1.000 | 1.000 |
| `ur5e_true` | 1.000 | 1.000 | 1.000 | 1.000 | 1.000 | 1.000 |
| `xarm6_true` | 0.952 | 1.000 | 0.720 | 0.872 | 1.000 | 0.094 |
| `simple_default` | 0.923 | 1.000 | 0.645 | 0.792 | 0.998 | 0.020 |
| `panda_proxy` | 0.766 | 0.875 | 0.439 | 0.434 | 0.416 | 0.000 |
| `xarm6_proxy` | 0.725 | 0.783 | 0.405 | 0.350 | 0.244 | 0.000 |

## Interpretation

The `max-iters=80` rerun confirms that the earlier `ur5e_true` trajectory-success collapse was an IK solver-budget artifact. With a consistent 80-iteration budget, the Menagerie UR5e true model reaches all selected LIBERO positions under source-base placement.

`panda_true`, `ur5e_true`, and `ur5_proxy` all score 1.000 / 1.000 on this position-only benchmark. They still have different meanings:

- `panda_true` is the source robot kinematic baseline for LIBERO Panda demonstrations.
- `ur5e_true` is an external Menagerie UR5e true kinematic model placed at the LIBERO source base for cross-embodiment reachability.
- `ur5_proxy` is still a simplified proxy, not a real UR5e model.

`xarm6_true` is substantially stronger than `xarm6_proxy`, especially on trajectory success, but still struggles on `libero_spatial`. This suggests the simplified xArm6 proxy underestimates the official xArm6 kinematic chain, while the spatial suite remains the hardest workspace slice for xArm6.

This benchmark remains position-only. It does not include orientation constraints, gripper semantics, scene collision, dynamics, controller behavior, or manipulation policy execution.

## Output Files

Merged results:

```text
outputs/sourcebase_true_proxy_all_it80/baseline_results.csv
outputs/sourcebase_true_proxy_all_it80/baseline_results_by_suite.csv
outputs/sourcebase_true_proxy_all_it80/baseline_trajectory_results.csv
```

Raw sub-runs:

```text
outputs/sourcebase_true_proxy_all_it80/internal_baselines
outputs/sourcebase_true_proxy_all_it80/ur5e_true
outputs/sourcebase_true_proxy_all_it80/xarm6_true
```
