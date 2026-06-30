# ArmFoundry / LIBERO 大规模评估阶段汇报

> 更新说明：本报告对应旧的 fixed-base 评估，所有 robot proxy 都被放在 world origin。后续已完成 source-base 校准评估，当前主结论应以 `outputs/server_all1500_wp80_it35_w16_sourcebase/analysis_report.md` 为准。本报告保留为未校准 base 的对照实验。

## 1. 工作背景

ArmFoundry 当前在做一个数据驱动的机器人硬件评估原型。它的基本思路是：从真实机器人任务数据中提取末端执行器轨迹，把这些轨迹看作任务空间需求，然后评估不同机械臂结构是否能够覆盖这些需求。

本阶段不训练策略，也不做 LIBERO policy learning。LIBERO 在这里的作用不是提供学习目标，而是提供一批真实 demonstration 中的 end-effector trajectory。我们要回答的问题是：如果这些轨迹代表任务要求，现有或候选机械臂在运动学上是否能到达这些位置，并且是否具备足够好的姿态质量。

因此，这一阶段更接近“硬件筛选”和“任务空间覆盖评估”，而不是完整机器人学习实验。软件控制侧保持固定，比较对象是不同机械臂几何 proxy。

## 2. 当前完成的工作

本轮已经完成从真实 LIBERO 数据到大规模 baseline 评估的闭环。

首先，已将 `libero_goal`、`libero_object`、`libero_spatial` 三个 suite 的原始 HDF5 数据下载到服务器 SSD。随后通过 symlink 接入 ArmFoundry 仓库，并重新提取为项目内部使用的 processed trajectory 数据。

处理后的数据集包含 1,500 条 trajectory、30 个 task、3 个 suite，总计 200,485 个 waypoint。数据中保留了 end-effector position、orientation 和 gripper trace。

在此基础上，已完成一次大规模 baseline 评估。本次评估不是 synthetic smoke test，也不是 20 条轨迹的小样本验证，而是在完整真实数据集上进行的全量对比实验。

## 3. 数据与实验设置

本次使用的数据目录是：

```text
data/libero_ee_trajectories_armforge
```

本次实验输出目录是：

```text
outputs/server_all1500_wp80_it35_w16
```

数据规模如下：

| 项目 | 数值 |
|---|---:|
| Suite 数量 | 3 |
| Task 数量 | 30 |
| Trajectory 数量 | 1,500 |
| 处理后 waypoint 总数 | 200,485 |
| 本次评估 waypoint 数量 | 119,975 |
| 坐标系设置 | `world` |
| 是否保留 orientation | 是 |
| 是否保留 gripper | 是 |

本次评估对每条轨迹最多均匀采样 80 个 waypoint。这个设置不会改写原始 `.npz` 数据，只是在评估时控制计算量。最终实际参与 IK 评估的 waypoint 数量为 119,975。

运行配置为：

```bash
uv run python -m ddird.experiments.run_eval_baselines \
  --data data/libero_ee_trajectories_armforge \
  --outputs outputs/server_all1500_wp80_it35_w16 \
  --max-waypoints-per-trajectory 80 \
  --num-workers 16 \
  --max-iters 35
```

本次运行约耗时 7 分 34 秒，生成了总体结果、按 suite 汇总结果和 per-trajectory 结果。

## 4. 被评估的 baseline

本次比较了 4 个简化机械臂 baseline。这里的 `proxy` 指“代理模型”：它借用某类真实机械臂的大致连杆尺度，但没有使用真实 URDF、CAD、动力学、执行器模型或控制器。

所有 baseline 都被实现为同一种 6-DoF serial-chain 运动学模型，使用相同的关节轴设置、默认关节限位、workspace bounds 和 base pose。本次实验中它们的 base 都放在 `(0, 0, 0)`，base yaw 为 `0`。

它们之间最主要的区别是 4 个主连杆长度不同。因此，本次比较衡量的是不同 reach / link geometry proxy 对 LIBERO 轨迹覆盖率的影响，而不是严格比较真实商业机械臂的完整性能。

| Robot | 具体含义 | 本次实现中的连杆长度 |
|---|---|---:|
| `simple_default` | 项目内置的参数化简化机械臂，作为 simple-arm 设计搜索的默认起点 | `(0.30, 0.34, 0.28, 0.16)` |
| `panda_proxy` | 以 Franka Panda 尺度为参考的商业机械臂代理模型 | `(0.333, 0.316, 0.284, 0.107)` |
| `ur5_proxy` | 以 UR5 / UR5e 尺度为参考的工业协作臂代理模型 | `(0.1625, 0.425, 0.3922, 0.1333)` |
| `xarm6_proxy` | 以 xArm6 尺度为参考的较紧凑商业机械臂代理模型 | `(0.267, 0.293, 0.284, 0.102)` |

例如，`ur5_proxy` 并不是在仿真中加载真实 UR5，也不是使用 UR5 的完整关节结构、碰撞几何或控制限制。它只是用 UR5 风格的主连杆长度填入当前项目的统一简化 6-DoF 模型。

这样做的好处是比较公平、实现简单、计算快，适合第一阶段筛选。但它的代价是不能把结果直接解释为真实 Panda、UR5 或 xArm6 在真实场景中的最终表现。

## 5. 评估方法

评估器把每个 waypoint 的末端执行器位置作为目标点，对每个 baseline 机械臂运行 position IK。若 IK 能找到有效关节解，则该 waypoint 被视为成功。

本阶段主要关注以下指标：

- `IK success rate`：成功求解 IK 的 waypoint 比例，反映点级任务空间覆盖率。
- `Trajectory success rate`：一条轨迹中所有选中 waypoint 都成功的比例，反映整条 demonstration 是否可连续覆盖。
- `Mean position error`：IK 解对应末端位置与目标位置的平均误差。
- `Mean joint margin`：关节解距离关节上下限的平均余量。
- `Mean manipulability`：机械臂在求解姿态附近的操作灵活性。
- `Manipulability below threshold`：低 manipulability waypoint 的比例。

当前评估是 position-only。虽然数据中已经保留 orientation 和 gripper 信息，但它们还没有进入 IK 求解与评分。因此，本阶段结果应理解为“位置可达性和运动学质量评估”，还不是完整 manipulation feasibility。

## 6. 总体结果

总体上，`ur5_proxy` 是当前最强 baseline。它在点级 IK 成功率、平均位置误差和 manipulability 上都表现最好。

| Robot | IK 成功率 | 轨迹成功率 | 成功轨迹数 | 平均位置误差 | 平均 manipulability |
|---|---:|---:|---:|---:|---:|
| `ur5_proxy` | 0.706 | 0.097 | 146 / 1500 | 0.0254 | 0.01680 |
| `simple_default` | 0.614 | 0.101 | 152 / 1500 | 0.0384 | 0.01030 |
| `panda_proxy` | 0.525 | 0.000 | 0 / 1500 | 0.0535 | 0.00713 |
| `xarm6_proxy` | 0.339 | 0.000 | 0 / 1500 | 0.1041 | 0.00424 |

相对 `simple_default`，`ur5_proxy` 的 waypoint 级 IK 成功率提升约 9.20 个百分点，平均位置误差更低，manipulability 更好。因此，后续 optimized design 不应只和 `simple_default` 比较，也应该直接和 `ur5_proxy` 比较。

不过，`simple_default` 的整条轨迹成功率略高于 `ur5_proxy`，分别为 10.13% 和 9.73%。这说明 trajectory success 是一个更严格、也更容易受局部失败影响的指标。一个 robot 可以有更高的平均 waypoint 成功率，但仍然因为少数局部失败点损失整条轨迹。

`panda_proxy` 和 `xarm6_proxy` 的整条轨迹成功率均为 0。这不代表它们所有点都失败，而是说明几乎每条轨迹中都存在至少一个失败 waypoint。

## 7. 分 suite 结果

三个 suite 的难度差异明显。`libero_object` 最容易，`libero_goal` 和 `libero_spatial` 明显更难。

| Suite | 最优 baseline | 最优 IK 成功率 | 简要判断 |
|---|---|---:|---|
| `libero_goal` | `ur5_proxy` | 0.549 | 抽屉、柜体、炉灶、酒架任务较难 |
| `libero_object` | `ur5_proxy` | 0.973 | basket pick-and-place 任务较容易 |
| `libero_spatial` | `ur5_proxy` | 0.597 | 空间关系和边界位置是主要瓶颈 |

详细结果如下：

| Robot | Goal IK | Object IK | Spatial IK | Goal 轨迹 | Object 轨迹 | Spatial 轨迹 |
|---|---:|---:|---:|---:|---:|---:|
| `simple_default` | 0.399 | 0.970 | 0.475 | 0.000 | 0.304 | 0.000 |
| `panda_proxy` | 0.282 | 0.946 | 0.347 | 0.000 | 0.000 | 0.000 |
| `ur5_proxy` | 0.549 | 0.973 | 0.597 | 0.000 | 0.292 | 0.000 |
| `xarm6_proxy` | 0.069 | 0.938 | 0.009 | 0.000 | 0.000 | 0.000 |

`libero_object` 中大多数目标点位于较中心、较容易覆盖的工作空间，因此各个 robot 的 waypoint 成功率都很高。相比之下，`libero_goal` 和 `libero_spatial` 包含更多柜体、抽屉、高处、炉灶和边界区域目标，对 reach、关节余量和姿态质量要求更高。

这也说明，如果后续优化只看总体平均值，容易被 `libero_object` 的高成功率掩盖。更合理的做法是按 suite 或 task 分层评估，确保优化没有只提升容易任务。

## 8. 困难任务与失败模式

以总体最强的 `ur5_proxy` 为例，最困难的任务集中在高处、柜体、抽屉、酒架和炉灶相关区域。

| Suite | Task | IK 成功率 |
|---|---|---:|
| `libero_spatial` | `pick_up_the_black_bowl_on_the_wooden_cabinet_and_place_it_on_the_plate_demo` | 0.223 |
| `libero_spatial` | `pick_up_the_black_bowl_in_the_top_drawer_of_the_wooden_cabinet_and_place_it_on_the_plate_demo` | 0.248 |
| `libero_goal` | `put_the_wine_bottle_on_top_of_the_cabinet_demo` | 0.275 |
| `libero_goal` | `put_the_wine_bottle_on_the_rack_demo` | 0.281 |
| `libero_goal` | `open_the_top_drawer_and_put_the_bowl_inside_demo` | 0.301 |

这些任务的共同点是目标位置更靠近工作空间边界，或者要求机器人进入柜体、抽屉、酒架等局部区域。后续硬件搜索应重点关注这些区域，而不是只优化整体平均 reach。

轨迹成功率低也说明失败点具有“局部破坏性”。也就是说，许多 trajectory 中大部分 waypoint 可以求解，但少数失败 waypoint 足以导致整条轨迹失败。这对后续目标函数设计很重要。

## 9. 阶段性判断

第一，ArmFoundry 的基础流水线已经跑通。真实 LIBERO 原始数据可以被下载、接入、提取、转换，并用于大规模 baseline 评估。

第二，当前最强 baseline 是 `ur5_proxy`。后续 optimized design 如果要证明有效，至少应在相同 benchmark 上接近或超过 `ur5_proxy`，而不是只超过 `simple_default`。

第三，下一阶段的优化目标不应只追求 waypoint 级 IK 成功率。当前更关键的问题是轨迹级鲁棒性：少数局部失败点会破坏整条 demonstration。因此，目标函数应加入 trajectory-aware penalty。

第四，困难任务的空间分布已经比较清楚。高处、柜体、抽屉、酒架和炉灶区域是当前 baseline 的主要短板。这些区域应成为 base pose、link length 或 mounting configuration 优化的重点。

## 10. 局限性

当前结果仍然是第一阶段运动学评估，不能直接解释为真实机器人最终性能。

主要局限包括：

- 当前只做 position IK，没有把 orientation 约束纳入求解。
- Baseline robot 是简化 proxy，不是真实 URDF 模型。
- Collision 指标是简化 proxy，不是场景级碰撞检测。
- LIBERO `obs/ee_pos` 当前按 `world` 坐标系处理，但坐标系语义仍需要正式验证。
- 本次评估对每条 trajectory 做了 waypoint 下采样，最终结论应补充无下采样实验。

这些局限不影响当前阶段作为“硬件运动学筛选”的价值，但在进入更接近真实 manipulation 的评估前必须补齐。

## 11. 下一步计划

下一阶段建议围绕以下方向推进：

1. 在同一 benchmark 上评估 optimized design candidate，并直接与 `ur5_proxy` 对比。
2. 增加 trajectory-aware objective，降低局部 IK 失败对整条轨迹的破坏。
3. 对困难任务做 workspace-region 诊断，明确失败点在空间中的分布。
4. 使用更高 `max-iters` 重跑部分实验，区分 solver budget 不足和几何不可达。
5. 引入 orientation-aware IK，让评估从位置可达性推进到姿态可行性。
6. 后续接入真实 URDF 或更精确 robot model，验证 proxy baseline 的结论是否保持。

总体来看，本次实验说明 ArmFoundry 已经具备继续进行硬件设计搜索的基础。下一步重点不再是证明流程能跑，而是把优化目标从平均点级可达性推进到轨迹级稳定性和更真实的机器人约束。

## 12. 输出文件

本次评估结果保存在：

```text
outputs/server_all1500_wp80_it35_w16
```

其中：

- `baseline_results.csv`：按 robot 聚合的总体指标。
- `baseline_results_by_suite.csv`：按 robot 和 suite 聚合的指标。
- `baseline_trajectory_results.csv`：per-trajectory 指标，共 6,000 行。
- `analysis_report.md`：本汇报。
