# UR5e True-Model Source-Base Benchmark

## Summary

This run evaluates `ur5e_true` on the processed LIBERO trajectories as a real-model, cross-embodiment reachability baseline.

`ur5e_true` is loaded from MuJoCo Menagerie:

```text
data/armforge/models/mujoco_menagerie/universal_robots_ur5e/ur5e.xml
```

The model uses `base` as the base body and `attachment_site` as the TCP. The comparison is source-base, position-only reachability. It is not a manipulation-success result.

## FK Sanity Check

Before benchmarking, ArmFoundry FK was checked against MuJoCo FK for the same Menagerie MJCF.

| Metric | Value |
|---|---:|
| Samples | 65 |
| DoF | 6 |
| Mean FK error | 0.00000000 m |
| p95 FK error | 0.00000000 m |
| p99 FK error | 0.00000000 m |
| Max FK error | 0.00000000 m |

The FK check output is in:

```text
outputs/ur5e_mjcf_fk_check
```

## Benchmark Setup

Sample run:

```bash
uv run python -m ddird.experiments.run_eval_mjcf_robot \
  --mjcf data/armforge/models/mujoco_menagerie/universal_robots_ur5e/ur5e.xml \
  --robot-name ur5e_true \
  --base-body base \
  --target-site attachment_site \
  --target-body wrist_3_link \
  --data data/libero_ee_trajectories_armforge \
  --outputs outputs/ur5e_true_sourcebase_sample \
  --max-trajectories 50 \
  --max-waypoints-per-trajectory 20 \
  --num-workers 4 \
  --max-iters 35 \
  --base-pose-mode source
```

Full run:

```bash
uv run python -m ddird.experiments.run_eval_mjcf_robot \
  --mjcf data/armforge/models/mujoco_menagerie/universal_robots_ur5e/ur5e.xml \
  --robot-name ur5e_true \
  --base-body base \
  --target-site attachment_site \
  --target-body wrist_3_link \
  --data data/libero_ee_trajectories_armforge \
  --outputs outputs/ur5e_true_sourcebase \
  --max-waypoints-per-trajectory 80 \
  --num-workers 16 \
  --max-iters 35 \
  --base-pose-mode source
```

The full run was executed in `tmux` and evaluated 1,500 trajectories with 119,975 selected waypoints.

## Overall Results

| Robot | Type | IK success | Trajectory success | Mean position error | Mean manipulability |
|---|---|---:|---:|---:|---:|
| `panda_true` | source true model | 1.000 | 1.000 | 0.00029 | 0.19146 |
| `ur5_proxy` | simplified proxy | 1.000 | 1.000 | 0.00031 | 0.12882 |
| `ur5e_true` | external true model | 0.987 | 0.000 | 0.00204 | 0.12508 |
| `xarm6_true` | generated true-kinematics model | 0.891 | 0.655 | 0.00364 | 0.06523 |
| `xarm6_proxy` | simplified proxy | 0.637 | 0.198 | 0.02620 | 0.03008 |

`ur5e_true` reaches most selected waypoints, but its trajectory success is 0 because the metric requires every selected waypoint in a trajectory to pass. With 80 selected waypoints per trajectory, a small number of local failures is enough to fail the whole trajectory.

This is the important difference from `ur5_proxy=1.000`: `ur5_proxy` is a simplified serial-chain proxy, while `ur5e_true` uses the Menagerie UR5e joint layout, base transform, TCP site, and inherited MJCF joint defaults.

## Suite Breakdown

| Suite | IK success | Trajectory success | Mean position error | Mean manipulability |
|---|---:|---:|---:|---:|
| `libero_goal` | 0.987 | 0.000 | 0.00211 | 0.11638 |
| `libero_object` | 0.988 | 0.000 | 0.00204 | 0.12512 |
| `libero_spatial` | 0.987 | 0.000 | 0.00198 | 0.13375 |

## Interpretation

`ur5e_true` is a strong position-level reachability baseline, but it should not be reported as task execution performance. The benchmark does not include orientation constraints, gripper semantics, scene collision, policy execution, or UR5e-specific workcell design.

The current result is best read as: under LIBERO source-base placement and position-only IK, a Menagerie UR5e true kinematic model covers about 98.7% of selected LIBERO waypoints, but the all-waypoint trajectory metric is still fragile to sparse local failures.
