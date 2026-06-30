# True-Model Pose-Aware Benchmark Report

## Summary

This run evaluates whether the true-model 1.000 / 1.000 position-only reachability result remains strong after adding orientation constraints. The source data is the processed ArmFoundry LIBERO trajectory set, downsampled to at most 80 waypoints per trajectory, with each candidate robot placed at the source Panda base pose.

The pose-aware run uses the #13 evaluator with position tolerance `0.002 m`, orientation tolerance `0.10 rad`, `max-iters=80`, and `ee_ori` interpreted as `rotvec`. Per #12, these pose-aware results are diagnostic for non-source robots unless a source-Panda-TCP to target-robot-TCP frame mapping is explicitly defined.

## Output Locations

- Position-only reference: `outputs/sourcebase_true_proxy_all_it80`
- Pose-aware root: `outputs/sourcebase_true_pose_it80`
- Combined pose CSV: `outputs/sourcebase_true_pose_it80/baseline_results.csv`
- By-suite pose CSV: `outputs/sourcebase_true_pose_it80/baseline_results_by_suite.csv`
- By-trajectory pose CSV: `outputs/sourcebase_true_pose_it80/baseline_trajectory_results.csv`

## Overall Results

| Robot | Position IK | Position traj | Pose IK | Pose traj | Mean pos err m | Mean ori err rad | Max ori err rad |
|---|---:|---:|---:|---:|---:|---:|---:|
| `panda_true` | 1.000 | 1.000 | 0.995 | 0.947 | 0.0005 | 0.0118 | 1.6864 |
| `ur5e_true` | 1.000 | 1.000 | 0.967 | 0.788 | 0.0012 | 0.0022 | 0.4286 |
| `xarm6_true` | 1.000 | 1.000 | 0.719 | 0.242 | 0.0112 | 0.0235 | 1.2442 |

Position-only reachability remains perfect for all three true models in the reference run. Pose-aware evaluation changes the conclusion: `panda_true` remains near-perfect, `ur5e_true` remains high overall but drops on `libero_spatial`, and `xarm6_true` drops substantially once the source orientation is also required.

## Suite Breakdown

| Robot | Suite | Position IK | Position traj | Pose IK | Pose traj | Mean pos err m | Mean ori err rad | Max ori err rad |
|---|---|---:|---:|---:|---:|---:|---:|---:|
| `panda_true` | `libero_goal` | 1.000 | 1.000 | 0.992 | 0.944 | 0.0006 | 0.0065 | 1.6864 |
| `panda_true` | `libero_object` | 1.000 | 1.000 | 1.000 | 0.990 | 0.0004 | 0.0119 | 0.2444 |
| `panda_true` | `libero_spatial` | 1.000 | 1.000 | 0.994 | 0.906 | 0.0005 | 0.0171 | 1.3887 |
| `ur5e_true` | `libero_goal` | 1.000 | 1.000 | 0.999 | 0.972 | 0.0004 | 0.0027 | 0.1544 |
| `ur5e_true` | `libero_object` | 1.000 | 1.000 | 0.999 | 0.960 | 0.0005 | 0.0009 | 0.0909 |
| `ur5e_true` | `libero_spatial` | 1.000 | 1.000 | 0.901 | 0.432 | 0.0025 | 0.0031 | 0.4286 |
| `xarm6_true` | `libero_goal` | 1.000 | 1.000 | 0.784 | 0.394 | 0.0094 | 0.0273 | 1.2442 |
| `xarm6_true` | `libero_object` | 1.000 | 1.000 | 0.762 | 0.250 | 0.0076 | 0.0163 | 0.6077 |
| `xarm6_true` | `libero_spatial` | 1.000 | 1.000 | 0.611 | 0.082 | 0.0166 | 0.0268 | 0.4646 |

## Interpretation

- `panda_true` is the source robot model, so its orientation target is the most meaningful direct comparison. The pose-aware run is still strong (`0.995` waypoint pose success, `0.947` trajectory pose success), with residual failures concentrated in strict all-waypoint trajectory success rather than broad reachability loss.
- `ur5e_true` uses the Menagerie `attachment_site` on `wrist_3_link`, not a concrete gripper TCP. Its pose score is therefore diagnostic. Goal/object suites remain near-perfect at waypoint level, while `libero_spatial` falls to `0.901` waypoint and `0.432` trajectory pose success, consistent with stricter orientation constraints and the missing source-to-UR5e tool-frame mapping.
- `xarm6_true` includes the official xArm gripper TCP offset, but no source-Panda-to-xArm tool-frame mapping. Its pose-aware score (`0.719` waypoint, `0.242` trajectory) is much lower than its position-only score. The drop is largest on `libero_spatial` (`0.611` waypoint, `0.082` trajectory), which indicates that the xArm6 true model can reach the positions but often cannot satisfy the Panda source orientation under the current diagnostic convention and solver limits.
- Mean orientation errors are small for successful portions of the run, but max orientation errors show isolated hard failures. Because trajectory success requires every selected waypoint to satisfy both position and orientation, even sparse failures strongly reduce trajectory-level pose success.

## Commands

```bash
uv run python -m ddird.experiments.run_eval_baselines --data data/libero_ee_trajectories_armforge --outputs outputs/sourcebase_true_pose_it80/panda_true --robots panda_true --max-waypoints-per-trajectory 80 --max-iters 80 --num-workers 16 --base-pose-mode source --evaluation-mode pose --orientation-format rotvec --orientation-tolerance 0.10
```

```bash
uv run --extra mujoco python -m ddird.experiments.run_eval_mjcf_robot --mjcf data/armforge/models/mujoco_menagerie/universal_robots_ur5e/ur5e.xml --robot-name ur5e_true --base-body base --target-site attachment_site --target-body wrist_3_link --data data/libero_ee_trajectories_armforge --outputs outputs/sourcebase_true_pose_it80/ur5e_true --max-waypoints-per-trajectory 80 --max-iters 80 --num-workers 16 --base-pose-mode source --evaluation-mode pose --orientation-format rotvec --orientation-tolerance 0.10
```

```bash
uv run --extra mujoco python -m ddird.experiments.run_eval_mjcf_robot --mjcf outputs/generated_models/xarm6_true/xarm6_true.xml --robot-name xarm6_true --base-body link_base --target-site link_tcp --target-body link6 --data data/libero_ee_trajectories_armforge --outputs outputs/sourcebase_true_pose_it80/xarm6_true --max-waypoints-per-trajectory 80 --max-iters 80 --num-workers 16 --base-pose-mode source --evaluation-mode pose --orientation-format rotvec --orientation-tolerance 0.10
```

## Run Times

- `panda_true`: 30.065 s wall time.
- `ur5e_true`: 63.962 s wall time.
- `xarm6_true`: 380.349 s wall time.
