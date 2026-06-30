# ArmFoundry / LIBERO True-Model Baseline Report

## Summary

This run adds `panda_true` to the source-base LIBERO evaluation. Unlike `panda_proxy`, `panda_true` uses the 7-DoF Panda kinematic chain reconstructed from the MuJoCo XML stored in the raw LIBERO HDF5 files.

The purpose is to separate two questions:

- Does the source Panda kinematic model reproduce the LIBERO demonstrations?
- How do simplified proxy chains compare under the same position-only evaluator?

## FK Sanity Check

Before using `panda_true`, the source Panda FK was checked against raw HDF5 `obs/joint_states` and `obs/ee_pos` for all 1,500 demos.

| Metric | Value |
|---|---:|
| Demos checked | 1,500 |
| Mean demo FK error | 0.000303 m |
| p95 demo FK error | 0.000371 m |
| p99 demo FK error | 0.000390 m |
| Max demo mean FK error | 0.000417 m |

This confirms that the reconstructed Panda kinematic chain and target frame (`gripper0_grip_site`) match the recorded LIBERO end-effector positions closely enough for this position-level evaluation.

## Evaluation Setup

Command:

```bash
uv run python -m ddird.experiments.run_eval_baselines \
  --data data/libero_ee_trajectories_armforge \
  --outputs outputs/server_all1500_wp80_it35_w16_true_models \
  --max-waypoints-per-trajectory 80 \
  --num-workers 16 \
  --max-iters 35 \
  --base-pose-mode source \
  --include-true-models
```

The run evaluates 1,500 trajectories and 119,975 selected waypoints.

## Overall Results

| Robot | Type | IK success | Trajectory success | Mean position error | Mean manipulability |
|---|---|---:|---:|---:|---:|
| `panda_true` | real source kinematic model | 1.000 | 1.000 | 0.00029 | 0.19146 |
| `ur5_proxy` | simplified proxy | 1.000 | 1.000 | 0.00031 | 0.12882 |
| `simple_default` | simplified proxy | 0.856 | 0.603 | 0.00571 | 0.06204 |
| `panda_proxy` | simplified proxy | 0.694 | 0.283 | 0.02236 | 0.03651 |
| `xarm6_proxy` | simplified proxy | 0.637 | 0.198 | 0.02620 | 0.03008 |

`panda_true` reaches all selected LIBERO positions, as expected for the source robot. This resolves the earlier concern that Panda appeared unable to reach its own demonstrations.

`panda_proxy` remains much lower because it is not the real Panda. It is a simplified 6-DoF serial-chain proxy with approximate link lengths and no true Panda joint layout, wrist geometry, gripper target frame, or 7-DoF redundancy.

## Suite Breakdown

| Robot | Goal IK | Object IK | Spatial IK | Goal traj | Object traj | Spatial traj |
|---|---:|---:|---:|---:|---:|---:|
| `panda_true` | 1.000 | 1.000 | 1.000 | 1.000 | 1.000 | 1.000 |
| `ur5_proxy` | 1.000 | 1.000 | 1.000 | 1.000 | 1.000 | 1.000 |
| `simple_default` | 0.923 | 1.000 | 0.645 | 0.792 | 0.998 | 0.020 |
| `panda_proxy` | 0.766 | 0.875 | 0.439 | 0.434 | 0.416 | 0.000 |
| `xarm6_proxy` | 0.725 | 0.783 | 0.405 | 0.350 | 0.244 | 0.000 |

`ur5_proxy=100%` is still a proxy result. It means the simplified UR5-style chain covers this position-only benchmark under source-base placement. It does not mean a real UR5 can execute LIBERO manipulation tasks without a real UR5 model, tool frame, collision model, and orientation-aware evaluation.

## Conclusion

The source-base + true-Panda results establish the correct interpretation:

- `panda_true` validates that the source Panda can reach the recorded LIBERO end-effector positions.
- `panda_proxy` measures only a simplified approximate chain and should not be treated as real Panda performance.
- `ur5_proxy` and `xarm6_proxy` remain proxy-level reachability comparisons until external real MJCF / URDF models are evaluated.

Future real UR5 and xArm results should be reported as `ur5_true` and `xarm6_true`, using the semantics in `docs/real-robot-baseline-semantics.md`.
