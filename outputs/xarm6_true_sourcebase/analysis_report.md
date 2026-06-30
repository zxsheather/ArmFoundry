# xArm6 True-Model Source-Base Benchmark

## Summary

This run evaluates `xarm6_true` on the processed LIBERO trajectories as a real-model, cross-embodiment reachability baseline.

The model is generated from the official xArm ROS default xArm6 kinematics:

```text
data/armforge/models/xarm_ros/xarm_description/config/kinematics/default/xarm6_default_kinematics.yaml
```

The generated evaluator-compatible MJCF is:

```text
outputs/generated_models/xarm6_true/xarm6_true.xml
```

The model uses `link_base` as the base body and `link_tcp` as the TCP site. The comparison is source-base, position-only reachability. It is not a manipulation-success result.

## Model Generation

Command:

```bash
uv run python -m ddird.experiments.run_generate_xarm6_mjcf \
  --xarm-ros-root data/armforge/models/xarm_ros \
  --outputs outputs/generated_models/xarm6_true \
  --robot-name xarm6_true \
  --tcp-site link_tcp
```

The generator records source metadata in:

```text
outputs/generated_models/xarm6_true/model_metadata.json
```

The generated MJCF uses official xArm6 joint origins and joint limits from xArm ROS. It intentionally uses minimal inertial values so MuJoCo can compile the kinematic model for FK checks. It does not include xArm meshes, detailed collision geometry, dynamics, or a gripper.

## FK Sanity Check

ArmFoundry FK was checked against MuJoCo FK for the generated xArm6 MJCF.

| Metric | Value |
|---|---:|
| Samples | 64 |
| DoF | 6 |
| Mean FK error | 0.00000000 m |
| p95 FK error | 0.00000000 m |
| p99 FK error | 0.00000000 m |
| Max FK error | 0.00000000 m |

The FK check output is in:

```text
outputs/xarm6_mjcf_fk_check
```

## Benchmark Setup

Sample run:

```bash
uv run python -m ddird.experiments.run_eval_mjcf_robot \
  --mjcf outputs/generated_models/xarm6_true/xarm6_true.xml \
  --robot-name xarm6_true \
  --base-body link_base \
  --target-site link_tcp \
  --target-body link6 \
  --data data/libero_ee_trajectories_armforge \
  --outputs outputs/xarm6_true_sourcebase_sample \
  --max-trajectories 50 \
  --max-waypoints-per-trajectory 20 \
  --num-workers 4 \
  --max-iters 35 \
  --base-pose-mode source
```

Full run:

```bash
uv run python -m ddird.experiments.run_eval_mjcf_robot \
  --mjcf outputs/generated_models/xarm6_true/xarm6_true.xml \
  --robot-name xarm6_true \
  --base-body link_base \
  --target-site link_tcp \
  --target-body link6 \
  --data data/libero_ee_trajectories_armforge \
  --outputs outputs/xarm6_true_sourcebase \
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
| `ur5e_true` | external true model | 0.987 | 0.000 | 0.00204 | 0.12508 |
| `xarm6_true` | generated true-kinematics model | 0.891 | 0.655 | 0.00364 | 0.06523 |
| `xarm6_proxy` | simplified proxy | 0.637 | 0.198 | 0.02620 | 0.03008 |
| `panda_proxy` | simplified proxy | 0.694 | 0.283 | 0.02236 | 0.03651 |

`xarm6_true` performs substantially better than `xarm6_proxy` on this benchmark. This is expected: the proxy is a simplified chain, while `xarm6_true` uses the official xArm6 joint origins and joint limits.

## Suite Breakdown

| Suite | IK success | Trajectory success | Mean position error | Mean manipulability |
|---|---:|---:|---:|---:|
| `libero_goal` | 0.952 | 0.872 | 0.00172 | 0.06768 |
| `libero_object` | 1.000 | 1.000 | 0.00037 | 0.08005 |
| `libero_spatial` | 0.720 | 0.094 | 0.00882 | 0.04796 |

`libero_spatial` remains the hardest suite for xArm6. `libero_object` is fully covered under this position-only benchmark.

## Interpretation

`xarm6_true` is a useful true-kinematics baseline for position-level reachability, but it is still not a full robot-performance claim. The generated model excludes detailed collision geometry, gripper behavior, dynamics, and orientation-aware IK. The result should be reported as source-base, position-only, cross-embodiment reachability.
