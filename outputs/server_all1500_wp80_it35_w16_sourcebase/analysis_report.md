# ArmFoundry / LIBERO Source-Base 校准评估汇报

## 1. 为什么要重跑

前一版大规模评估把所有 robot proxy 都放在 world origin：`base_xyz=(0, 0, 0)`，`base_yaw=0`。这对 LIBERO 数据是不准确的。

LIBERO 的原始 demonstration 来自 Panda arm，但 HDF5 中的 `ee_pos` 是 world-frame 末端位置。要用这些点做 IK，必须知道 robot base 在 world frame 里的真实位置。

本次从 HDF5 保存的 MuJoCo `model_file` 中解析出 `robot0_base`。评估时使用 `--base-pose-mode source`，让每条轨迹按原始 LIBERO 场景中的 source Panda base pose 来评估。

实现上，提取器现在支持把 source base 写入每条 processed trajectory 的 metadata。为避免仅因 metadata 造成 1,500 个 `.npz` 二进制文件变更，当前评估器也内置了 LIBERO suite-level source base fallback，可在旧 processed 数据上复现实验。

## 2. 校准得到的 base pose

从 30 个原始 HDF5 文件中解析到两类 source robot base：

| Suite | Source robot base |
|---|---|
| `libero_goal` | `[-0.66, 0.0, 0.912]` |
| `libero_spatial` | `[-0.66, 0.0, 0.912]` |
| `libero_object` | `[-0.6, 0.0, 0.0]` |

这些值来自每个 demo 的 MuJoCo XML 中的 `body name="robot0_base"`。当前没有发现额外 base quaternion，因此 base yaw 记为 `0.0`。

## 3. 本次完成的修改

本轮完成了三件事：

1. 提取阶段：从 HDF5 `model_file` 解析 `robot0_base`，支持写入 `source_robot_base_xyz` 和 `source_robot_base_yaw`。
2. 评估阶段：新增 `base_pose_mode`，支持 `fixed` 和 `source` 两种模式；`source` 模式优先读 trajectory metadata，缺失时使用 LIBERO suite-level fallback。
3. 实验阶段：用 `--base-pose-mode source` 重跑 1,500 条真实 LIBERO trajectory 的大规模 baseline。

校准版命令：

```bash
uv run python -m ddird.experiments.run_eval_baselines \
  --data data/libero_ee_trajectories_armforge \
  --outputs outputs/server_all1500_wp80_it35_w16_sourcebase \
  --max-waypoints-per-trajectory 80 \
  --num-workers 16 \
  --max-iters 35 \
  --base-pose-mode source
```

## 4. 校准版总体结果

| Robot | IK 成功率 | 轨迹成功率 | 成功轨迹数 | 平均位置误差 | 平均 manipulability |
|---|---:|---:|---:|---:|---:|
| `ur5_proxy` | 1.000 | 1.000 | 1500 / 1500 | 0.0003 | 0.12882 |
| `simple_default` | 0.856 | 0.603 | 905 / 1500 | 0.0057 | 0.06204 |
| `panda_proxy` | 0.694 | 0.283 | 425 / 1500 | 0.0224 | 0.03651 |
| `xarm6_proxy` | 0.637 | 0.198 | 297 / 1500 | 0.0262 | 0.03008 |

校准 base 后，所有 robot proxy 的结果都明显提高。这说明前一版 fixed-base 评估确实混入了错误 base pose 的影响。

## 5. Fixed-base 与 source-base 对比

| Robot | Fixed-base IK | Source-base IK | IK 提升 | Fixed-base 轨迹 | Source-base 轨迹 | 轨迹提升 |
|---|---:|---:|---:|---:|---:|---:|
| `simple_default` | 0.614 | 0.856 | +0.242 | 0.101 | 0.603 | +0.502 |
| `panda_proxy` | 0.525 | 0.694 | +0.169 | 0.000 | 0.283 | +0.283 |
| `ur5_proxy` | 0.706 | 1.000 | +0.294 | 0.097 | 1.000 | +0.903 |
| `xarm6_proxy` | 0.339 | 0.637 | +0.299 | 0.000 | 0.198 | +0.198 |

这个对比回答了前面的疑问：原始数据确实来自 Panda，但前一版 `panda_proxy` 使用了错误的 base pose，所以它会错误地 reach 不到很多位置。

校准后 `panda_proxy` 明显改善，但仍不是 100%。原因是 `panda_proxy` 仍然不是 LIBERO 中真实的 Panda：

- 当前 `panda_proxy` 是统一的简化 6-DoF serial-chain。
- 真实 Panda 是 7-DoF。
- 当前 proxy 没有真实 Panda URDF、joint layout、tool offset、controller 和完整关节限制。
- 当前评估仍是 position-only，没有使用真实 MuJoCo FK 来验证 `joint_states -> ee_pos`。

因此，`panda_proxy=69.4%` 不应解读为“真实 Panda 只能 reach 69.4% 的 LIBERO position”。它只能说明“当前简化 Panda proxy 在 source-base 校准后仍不能完全复现真实 Panda 的运动学”。

## 6. 分 suite 结果

| Robot | Goal IK | Object IK | Spatial IK | Goal 轨迹 | Object 轨迹 | Spatial 轨迹 |
|---|---:|---:|---:|---:|---:|---:|
| `simple_default` | 0.923 | 1.000 | 0.645 | 0.792 | 0.998 | 0.020 |
| `panda_proxy` | 0.766 | 0.875 | 0.439 | 0.434 | 0.416 | 0.000 |
| `ur5_proxy` | 1.000 | 1.000 | 1.000 | 1.000 | 1.000 | 1.000 |
| `xarm6_proxy` | 0.725 | 0.783 | 0.405 | 0.350 | 0.244 | 0.000 |

`libero_spatial` 仍然是最难的 suite。即使在 source-base 模式下，`panda_proxy` 和 `xarm6_proxy` 的 spatial 轨迹成功率仍为 0。

`ur5_proxy` 在当前简化模型下达到 100%。这说明 UR5 风格 proxy 的 reach / link geometry 对当前 position-only benchmark 非常有利，但不能直接推论真实 UR5 一定能完整执行 LIBERO manipulation。

## 7. 阶段性结论

第一，前一版 fixed-base 结果不应作为主结论。它可以作为“未校准 base 会低估 reachability”的对照实验。

第二，source-base 版本应作为当前更可信的 baseline。它至少消除了明显的 world-frame / robot-base-frame 错配。

第三，Panda 相关疑问已经明确：原始数据来自真实 Panda，但当前 `panda_proxy` 不是真实 Panda。校准 base 只能修正安装位姿，不能修正 6-DoF proxy 与真实 7-DoF Panda 的结构差异。

第四，下一步如果要验证“真实 Panda 应该 reach 自己的数据”，需要引入真实 Panda URDF / MuJoCo model，并用 HDF5 中的 `joint_states` 做 FK sanity check。

## 8. 下一步建议

1. 增加 `panda_true` baseline：加载真实 Panda kinematics，而不是简化 proxy。
2. 用 HDF5 中的 `joint_states` 验证真实 Panda FK 是否能复现 `obs/ee_pos`。
3. 把当前 `ur5_proxy=100%` 视为 proxy-level 上界，而不是工业 UR5 的真实性能结论。
4. 后续所有 LIBERO world-frame 评估默认使用 `--base-pose-mode source`。
5. 在引入真实 robot model 后，再重新比较 Panda / UR5 / xArm 的硬件可达性。

## 9. 输出文件

本次 source-base 评估结果位于：

```text
outputs/server_all1500_wp80_it35_w16_sourcebase
```

主要文件：

- `baseline_results.csv`
- `baseline_results_by_suite.csv`
- `baseline_trajectory_results.csv`
- `analysis_report.md`
