# UR5e Robotiq TCP Proxy Benchmark Report

## Summary

This benchmark compares the old `ur5e_true` attachment-site baseline with the new `ur5e_true_robotiq2f85_tcp_proxy` model. The new model adds an explicit TCP frame derived from the MuJoCo Menagerie Robotiq 2F-85 `pinch` site and composed onto the UR5e `attachment_site`.

The new model is still `tcp_offset_only`: it includes the Robotiq pinch TCP offset and fixed TCP orientation, but it does not include Robotiq meshes, collision geometry, gripper joints, actuation, or contact behavior.

Run settings: source-base placement, max 80 waypoints per trajectory, `max-iters=80`, position tolerance `0.002 m`, pose orientation tolerance `0.10 rad`, and `ee_ori` interpreted as `rotvec` for pose-aware evaluation.

## Overall Results

| Model | Tool frame | Position IK | Position traj | Pose IK | Pose traj | Pose mean pos err m | Pose mean ori err rad | Pose max ori err rad |
|---|---|---:|---:|---:|---:|---:|---:|---:|
| `ur5e_true` | `attachment_site` | 1.000 | 1.000 | 0.967 | 0.788 | 0.0012 | 0.0022 | 0.4286 |
| `ur5e_true_robotiq2f85_tcp_proxy` | `ur5e_robotiq2f85_tcp` | 1.000 | 1.000 | 0.997 | 0.967 | 0.0004 | 0.0014 | 0.0995 |

The explicit TCP proxy preserves perfect position-only reachability (`1.000 / 1.000`) and improves pose-aware trajectory success from `0.788` to `0.967`. This suggests the old attachment-site pose result was partly measuring a mismatched tool frame rather than only UR5e arm feasibility.

## Suite Breakdown

| Model | Suite | Position IK | Position traj | Pose IK | Pose traj | Pose mean pos err m | Pose mean ori err rad | Pose max ori err rad |
|---|---|---:|---:|---:|---:|---:|---:|---:|
| `ur5e_true` | `libero_goal` | 1.000 | 1.000 | 0.999 | 0.972 | 0.0004 | 0.0027 | 0.1544 |
| `ur5e_true` | `libero_object` | 1.000 | 1.000 | 0.999 | 0.960 | 0.0005 | 0.0009 | 0.0909 |
| `ur5e_true` | `libero_spatial` | 1.000 | 1.000 | 0.901 | 0.432 | 0.0025 | 0.0031 | 0.4286 |
| `ur5e_true_robotiq2f85_tcp_proxy` | `libero_goal` | 1.000 | 1.000 | 0.990 | 0.902 | 0.0005 | 0.0024 | 0.0995 |
| `ur5e_true_robotiq2f85_tcp_proxy` | `libero_object` | 1.000 | 1.000 | 1.000 | 1.000 | 0.0004 | 0.0007 | 0.0909 |
| `ur5e_true_robotiq2f85_tcp_proxy` | `libero_spatial` | 1.000 | 1.000 | 1.000 | 1.000 | 0.0003 | 0.0011 | 0.0953 |

## Interpretation

- The old `ur5e_true` result is a bare-arm attachment-site baseline. It is useful as a wrist-mount reachability diagnostic, but it is not a gripper TCP baseline.
- The new `ur5e_true_robotiq2f85_tcp_proxy` target is a more meaningful TCP for gripper-like manipulation because it uses the Robotiq 2F-85 `pinch` site as the source of the offset and fixed target-site orientation.
- Position-only success stays perfect after adding the TCP offset, so the new TCP does not create broad reachability loss on this benchmark.
- Pose-aware success improves strongly, especially on `libero_spatial`: trajectory pose success changes from `0.432` for the attachment site to `1.000` for the TCP proxy. This is consistent with the attachment-site frame being a poor orientation target for gripper-centric demonstrations.
- The result is still not a full UR5e + Robotiq simulation. It does not model gripper geometry, collisions, gripper joints, or actuation; it only makes the evaluated TCP frame explicit and source-backed.

## Outputs

- Position-only TCP proxy output: `outputs/ur5e_robotiq2f85_tcp_proxy_sourcebase_it80/position`
- Pose-aware TCP proxy output: `outputs/ur5e_robotiq2f85_tcp_proxy_sourcebase_it80/pose`
- Comparison CSV: `outputs/ur5e_robotiq2f85_tcp_proxy_sourcebase_it80/comparison_results.csv`
- Suite comparison CSV: `outputs/ur5e_robotiq2f85_tcp_proxy_sourcebase_it80/comparison_results_by_suite.csv`
- Benchmark metadata: `outputs/ur5e_robotiq2f85_tcp_proxy_sourcebase_it80/benchmark_metadata.json`

## Commands

```bash
uv run --extra mujoco python -m ddird.experiments.run_eval_mjcf_robot --mjcf outputs/generated_models/ur5e_robotiq2f85_tcp_proxy/ur5e_true_robotiq2f85_tcp_proxy.xml --robot-name ur5e_true_robotiq2f85_tcp_proxy --base-body base --target-site ur5e_robotiq2f85_tcp --target-body wrist_3_link --data data/libero_ee_trajectories_armforge --outputs outputs/ur5e_robotiq2f85_tcp_proxy_sourcebase_it80/position --max-waypoints-per-trajectory 80 --max-iters 80 --num-workers 16 --base-pose-mode source
```

```bash
uv run --extra mujoco python -m ddird.experiments.run_eval_mjcf_robot --mjcf outputs/generated_models/ur5e_robotiq2f85_tcp_proxy/ur5e_true_robotiq2f85_tcp_proxy.xml --robot-name ur5e_true_robotiq2f85_tcp_proxy --base-body base --target-site ur5e_robotiq2f85_tcp --target-body wrist_3_link --data data/libero_ee_trajectories_armforge --outputs outputs/ur5e_robotiq2f85_tcp_proxy_sourcebase_it80/pose --max-waypoints-per-trajectory 80 --max-iters 80 --num-workers 16 --base-pose-mode source --evaluation-mode pose --orientation-format rotvec --orientation-tolerance 0.10
```
