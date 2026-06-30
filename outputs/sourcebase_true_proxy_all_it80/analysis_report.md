# Source-Base True/Proxy Benchmark, max-iters=80

## Summary

This is the unified source-base benchmark for all currently available proxy and true kinematic models. It reruns the formal comparison with `--max-iters 80` for every robot so the result reflects kinematic reachability rather than inconsistent IK solver budget.

The xArm6 true model in this report uses the official xArm gripper TCP offset (`0 0 0.172`) from `link6` to `link_tcp`. An earlier wrist-origin xArm6 generated model placed `link_tcp` at `link6` and understated xArm6 reachability, especially on `libero_spatial`.

The run evaluates 1,500 LIBERO trajectories and 119,975 selected waypoints per robot.

## Why rerun

The previous `ur5e_true` run used `--max-iters 35` and produced:

| Robot | max-iters | IK success | Trajectory success |
|---|---:|---:|---:|
| `ur5e_true` | 35 | 0.987 | 0.000 |

Per-trajectory inspection showed exactly one failed waypoint in every trajectory, and sampled failures were at waypoint index 0. A 50-trajectory probe with `max-iters=80` resolved those failures, so the formal benchmark was rerun with a uniform 80-iteration IK budget.

The previous generated `xarm6_true` model also used `link6` as the TCP by placing `link_tcp` at `0 0 0`. The official xArm gripper URDF defines `link_tcp` at `0 0 0.172` from the gripper base. A diagnostic probe showed:

| xArm6 model | max-iters | Spatial IK success | Spatial trajectory success |
|---|---:|---:|---:|
| wrist-origin TCP | 80 | 0.720 | 0.094 |
| wrist-origin TCP | 160 | 0.720 | 0.094 |
| gripper TCP offset | 80 | 1.000 | 1.000 |

The xArm6 sub-run was therefore regenerated and rerun with the gripper TCP offset.

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
uv run python -m ddird.experiments.run_generate_xarm6_mjcf \
  --xarm-ros-root data/armforge/models/xarm_ros \
  --outputs outputs/generated_models/xarm6_true \
  --robot-name xarm6_true \
  --tcp-site link_tcp

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

The initial full run was executed in `tmux` as three sequential commands. After the xArm6 TCP correction, the xArm6 model was regenerated, sanity-checked with FK, rerun, and merged back into the unified result files.

## Overall Results

| Robot | Type | IK success | Trajectory success | Mean position error | Mean manipulability |
|---|---|---:|---:|---:|---:|
| `panda_true` | source true model | 1.000 | 1.000 | 0.00029 | 0.19146 |
| `xarm6_true` | generated true-kinematics model with gripper TCP | 1.000 | 1.000 | 0.00030 | 0.13909 |
| `ur5_proxy` | simplified proxy | 1.000 | 1.000 | 0.00031 | 0.12882 |
| `ur5e_true` | external true model | 1.000 | 1.000 | 0.00031 | 0.12557 |
| `simple_default` | simplified proxy | 0.856 | 0.603 | 0.00571 | 0.06204 |
| `panda_proxy` | simplified proxy | 0.694 | 0.283 | 0.02236 | 0.03651 |
| `xarm6_proxy` | simplified proxy | 0.637 | 0.198 | 0.02620 | 0.03008 |

## Suite Breakdown

| Robot | Goal IK | Object IK | Spatial IK | Goal traj | Object traj | Spatial traj |
|---|---:|---:|---:|---:|---:|---:|
| `panda_true` | 1.000 | 1.000 | 1.000 | 1.000 | 1.000 | 1.000 |
| `xarm6_true` | 1.000 | 1.000 | 1.000 | 1.000 | 1.000 | 1.000 |
| `ur5_proxy` | 1.000 | 1.000 | 1.000 | 1.000 | 1.000 | 1.000 |
| `ur5e_true` | 1.000 | 1.000 | 1.000 | 1.000 | 1.000 | 1.000 |
| `simple_default` | 0.923 | 1.000 | 0.645 | 0.792 | 0.998 | 0.020 |
| `panda_proxy` | 0.766 | 0.875 | 0.439 | 0.434 | 0.416 | 0.000 |
| `xarm6_proxy` | 0.725 | 0.783 | 0.405 | 0.350 | 0.244 | 0.000 |

## Interpretation

The `max-iters=80` rerun confirms that the earlier `ur5e_true` trajectory-success collapse was an IK solver-budget artifact. With a consistent 80-iteration budget, the Menagerie UR5e true model reaches all selected LIBERO positions under source-base placement.

`panda_true`, `xarm6_true`, `ur5e_true`, and `ur5_proxy` all score 1.000 / 1.000 on this position-only benchmark. They still have different meanings:

- `panda_true` is the source robot kinematic baseline for LIBERO Panda demonstrations.
- `xarm6_true` is an external xArm6 true-kinematics model generated from official xArm ROS kinematics and evaluated at its gripper TCP.
- `ur5e_true` is an external Menagerie UR5e true kinematic model placed at the LIBERO source base for cross-embodiment reachability.
- `ur5_proxy` is still a simplified proxy, not a real UR5e model.

The earlier low `xarm6_true` result was not a real reachability limitation. It was caused by evaluating a wrist-origin generated model against LIBERO end-effector positions. Because trajectory success requires every selected waypoint to solve, a small number of unreachable wrist-origin waypoints collapsed trajectory-level success. Adding the official gripper TCP offset resolves that artifact.

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
