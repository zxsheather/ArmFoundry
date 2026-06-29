# LIBERO DDIRD Prototype Report

## Summary

This prototype treats demonstration end-effector trajectories as a task-space requirement distribution and evaluates candidate robot-arm hardware with a fixed position-IK pipeline.

LIBERO can validate the structure of a DDIRD prototype, but it is not real industrial operation data.

We optimized candidate hardware for reachability, IK feasibility, and kinematic quality under a task-space trajectory distribution.

## Simplification

The current evaluator is position-only. Orientation and gripper traces are preserved when present, but orientation feasibility is not yet included in IK or manipulability scoring.

## Baseline Comparison

| Robot | IK success | Trajectory success | Mean joint margin | Mean manipulability | Hardware cost |
|---|---:|---:|---:|---:|---:|
| simple_default | 1.000 | 1.000 | 1.007 | 0.05379 | 1.080 |
| panda_proxy | 0.954 | 0.750 | 1.172 | 0.04103 | 1.040 |
| ur5_proxy | 1.000 | 1.000 | 0.800 | 0.08153 | 1.113 |
| xarm6_proxy | 0.931 | 0.750 | 1.251 | 0.03569 | 0.946 |
| simple_optimized_links | 1.000 | 1.000 | 1.167 | 0.08828 | 1.212 |

## Viability Assessment

The prototype is viable if optimized hardware improves held-out IK success, joint-limit margin, or manipulability relative to the initial simple model while staying comparable to commercial proxy baselines.

Failure cases concentrate where target points fall outside the arm's reachable shell, near table/workspace boundaries, or around low-manipulability postures.

For real DDIRD, the next data needed is factory operation logs with task trajectories, failures, cycle-time pressure, torque/stress traces, maintenance events, and real mounting constraints.

## Figures

- [figures/ee_point_cloud.png](figures/ee_point_cloud.png)
- [figures/reachable_unreachable_points.png](figures/reachable_unreachable_points.png)
- [figures/ik_success_by_robot.png](figures/ik_success_by_robot.png)
- [figures/joint_margin_histogram.png](figures/joint_margin_histogram.png)
- [figures/manipulability_boxplot.png](figures/manipulability_boxplot.png)
- [figures/robot_geometry_before_after.png](figures/robot_geometry_before_after.png)
