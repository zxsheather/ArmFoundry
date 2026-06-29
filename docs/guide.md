# Agent Guide: Building a LIBERO EE-Trajectory Prototype for DDIRD

**Purpose:** This guide tells an implementation agent how to use LIBERO demonstration data to build a minimal DDIRD prototype. The prototype should extract end-effector trajectories, treat them as a task-space requirement distribution, and use them to evaluate and optimize candidate robot-arm hardware designs.

**Current research framing:** This is not a meta-learning project, not a per-task generative robot design project, and not a full policy-learning benchmark. In the first stage, keep the software side fixed and use it only as an evaluator. The design variable is the hardware.

**Core question:** Given a distribution of task-space trajectories, which robot-arm geometry or mounting configuration best covers the tasks?

## 1. What the Agent Should Build

The agent should not train a robot policy. It should build the following pipeline:

```text
LIBERO demonstrations
    -> extract end-effector trajectories
    -> normalize and organize task-space requirements
    -> define a candidate arm design space
    -> evaluate IK / reachability / manipulability / joint-limit margin
    -> optimize simple hardware parameters
    -> compare against real robot baselines
    -> produce figures, metrics, and a short report
```

The final deliverables should include:

- a reusable data-extraction script;
- a cleaned end-effector trajectory dataset;
- a simple parameterized robot-arm model;
- a hardware evaluator;
- one optimization experiment;
- comparisons against at least two real robot baselines;
- an experiment report explaining whether the DDIRD prototype is viable.

## 2. What Not to Do

Do not expand this into a full robot-learning project.

Avoid the following:

- do not train a LIBERO policy;
- do not solve lifelong learning;
- do not frame the work as meta-learning;
- do not generate a new robot for every LIBERO task;
- do not start by optimizing a full URDF, CAD model, inertia model, or actuator model;
- do not treat Panda joint trajectories as targets that must be imitated;
- do not use policy success rate as the main evaluation metric.

Instead:

- treat LIBERO as a source of task-space trajectory distributions;
- focus on end-effector pose requirements;
- evaluate hardware designs with a fixed IK / planning / evaluation pipeline;
- start by optimizing base pose and link lengths;
- use real robot models as baseline comparisons.

## 3. Core Mathematical Formulation

The first version can be written as a single-level optimization problem:

$$
\theta^*
=
\arg\min_{\theta \in \Theta}
\mathbb{E}_{\xi^{ee}\sim \mathcal{D}_{LIBERO}}
\left[
\ell_{eval}(\theta,\xi^{ee})
\right]
+ \lambda c_{hw}(\theta)
$$

where:

- $\theta$ denotes candidate robot-arm hardware parameters;
- $\Theta$ is the allowed design space;
- $\xi^{ee}$ is one end-effector trajectory;
- $\mathcal{D}_{LIBERO}$ is the end-effector trajectory distribution extracted from LIBERO demonstrations;
- $\ell_{eval}$ is the evaluation loss for executing the trajectory with the candidate hardware;
- $c_{hw}$ is a hardware regularization term, such as total reach, size, complexity, or cost.

A recommended evaluation loss is:

$$
\ell_{eval}
=
w_1\ell_{IK}
+ w_2\ell_{joint}
+ w_3\ell_{sing}
+ w_4\ell_{manip}
+ w_5\ell_{collision}
+ w_6\ell_{smooth}
$$

with:

- $\ell_{IK}$: IK failure rate;
- $\ell_{joint}$: penalty for being close to joint limits;
- $\ell_{sing}$: penalty for being close to singular configurations;
- $\ell_{manip}$: penalty for low manipulability;
- $\ell_{collision}$: penalty for self-collision or environment collision;
- $\ell_{smooth}$: penalty for discontinuous or jumping joint trajectories.

If each end-effector trajectory first requires solving for a corresponding joint trajectory, the problem can be written as a weak bi-level problem:

$$
q^*(\theta,\xi^{ee})
=
\arg\min_q
J_{IK/traj}(q;\theta,\xi^{ee})
$$

$$
\theta^*
=
\arg\min_{\theta \in \Theta}
\mathbb{E}_{\xi^{ee}\sim \mathcal{D}_{LIBERO}}
\left[
\ell_{eval}(\theta,q^*(\theta,\xi^{ee}),\xi^{ee})
\right]
+ \lambda c_{hw}(\theta)
$$

The inner problem is not learning. It is IK or trajectory fitting. The outer problem selects the hardware design.

## 4. Data Processing Requirements

### 4.1 What to Extract from LIBERO

Extract the following fields when available:

- end-effector position;
- end-effector orientation;
- gripper open/close state;
- episode id;
- task id;
- task suite;
- language instruction;
- object initial states;
- success/failure flag, if available;
- robot proprioception, if directly available.

The most important object is the end-effector pose sequence:

$$
\xi^{ee}
=
\{T^{ee}_t\}_{t=1}^{T}
$$

where:

$$
T^{ee}_t
=
(p_t, R_t)
$$

and:

- $p_t\in\mathbb{R}^3$ is the end-effector position;
- $R_t\in SO(3)$ is the end-effector orientation.

If orientation handling is difficult in the first implementation, start with position-only trajectories:

$$
\xi^{pos}
=
\{p_t\}_{t=1}^{T}
$$

The report must state clearly that this is a simplification.

### 4.2 Recommended Data Format

Save the processed data under:

```text
data/libero_ee_trajectories/
    metadata.json
    trajectories/
        suite_name/
            task_name/
                episode_000001.npz
                episode_000002.npz
```

Each `.npz` file should contain at least:

```text
ee_pos:      shape [T, 3]
ee_quat:     shape [T, 4], or ee_rotmat: shape [T, 3, 3]
gripper:     shape [T]
task_id:     string or integer
episode_id:  string or integer
```

The metadata file should contain at least:

```json
{
  "source": "LIBERO",
  "robot_in_source": "Panda or source embodiment if known",
  "coordinate_frame": "world or robot base, specify clearly",
  "num_suites": 0,
  "num_tasks": 0,
  "num_episodes": 0,
  "contains_orientation": true,
  "contains_gripper": true
}
```

### 4.3 Coordinate Frames Must Be Explicit

The agent must verify which coordinate frame the extracted trajectory uses:

```text
world frame
robot base frame
workspace/table frame
```

For DDIRD evaluation, it is usually best to represent trajectories in a world or task frame. The candidate robot base pose $\theta_{base}$ then defines the transform from robot base frame to world frame.

Do not run IK if the frame convention is unclear. A wrong coordinate frame will invalidate all results.

## 5. Candidate Robot-Arm Models

### 5.1 Stage 1: Simple Parameterized Model

Use a simple model in the first stage. Do not start from complex real URDFs.

Recommended starting point:

```text
fixed 6-DoF or 7-DoF serial manipulator topology
optimize only base pose + link lengths
ignore detailed actuator / inertia / CAD mesh in the first pass
```

For example:

$$
\theta
=
(x_b,y_b,z_b,\psi_b,l_1,l_2,l_3,l_4)
$$

where:

- $(x_b,y_b,z_b)$ is the base position;
- $\psi_b$ is the base yaw angle;
- $l_i$ are the main link lengths.

For the smallest useful experiment, optimize only the base pose:

$$
\theta=(x_b,y_b,z_b,\psi_b)
$$

and keep the robot geometry fixed.

### 5.2 Stage 2: Real Robot Baselines

After the simple pipeline works, add real robot baselines such as:

- Franka Panda;
- UR5 / UR10;
- xArm6 / xArm7;
- KUKA iiwa.

These baselines answer:

```text
How do existing commercial arms perform on the same EE trajectories?
Can a simple optimized model outperform them on the selected metrics?
Are failures mainly caused by reachability, joint limits, singularities, or collision?
```

Real models do not need to be used for the first continuous optimization run, but they are important for making the report convincing.

## 6. Evaluation Metrics

### 6.1 IK Success Rate

Run IK for every waypoint of every trajectory.

```text
success = IK solver finds a valid q
failure = no solution / joint-limit violation / solver does not converge
```

Compute:

$$
\mathrm{IKSuccess}(\theta)
=
\frac{\#\text{successful waypoints}}
{\#\text{total waypoints}}
$$

Also consider trajectory-level metrics:

```text
full trajectory success
partial trajectory success
task-level success
```

### 6.2 Joint-Limit Margin

IK success alone is not enough. The solution should not sit too close to joint limits.

For joint $i$:

$$
m_i(q)
=
\min(q_i-q_i^{min}, q_i^{max}-q_i)
$$

Small margin means the joint is close to a limit.

A simple penalty is:

$$
\ell_{joint}
=
\sum_i \max(0,\epsilon_m-m_i(q))
$$

### 6.3 Manipulability

Use the Jacobian $J(q)$ to compute Yoshikawa manipulability:

$$
\mu(q)
=
\sqrt{\det(J(q)J(q)^T)}
$$

When $\mu(q)$ is small, the robot is close to a singular configuration and has poor motion capability in some end-effector directions.

Record:

```text
mean manipulability
minimum manipulability
percentage below threshold
```

### 6.4 Trajectory Continuity

Solving IK independently at each waypoint may produce valid but discontinuous joint solutions.

Compute:

$$
\ell_{smooth}
=
\frac{1}{T-1}
\sum_{t=1}^{T-1}
\|q_{t+1}-q_t\|_2^2
$$

The agent should prefer warm-started IK:

```text
IK at time t uses q_{t-1} as the initial guess
```

This is closer to realistic trajectory execution.

### 6.5 Workspace Coverage

Build a task-space point cloud from all end-effector positions and measure what fraction can be covered by each candidate robot.

Useful analyses:

```text
3D voxel grid coverage
height-slice heatmaps
reachable / unreachable point visualizations
```

This is important for the report because it makes hardware limitations visually interpretable.

### 6.6 Collision

In the first stage, use simplified collision checks:

- self-collision;
- table collision;
- workspace boundary collision.

Do not start with complex object-level collision unless a ready-made interface already exists.

## 7. Optimization Methods

### 7.1 Start with Black-Box Optimization

The first version should use simple black-box methods:

- grid search;
- random search;
- CMA-ES;
- Bayesian optimization;
- differential evolution.

Reasons:

- IK success, collision, and solver failure are often non-differentiable;
- the first design space should be low-dimensional;
- black-box optimization is fast to implement and suitable for validating the prototype.

### 7.2 Do Not Start with Complex Gradient Methods

Unless the agent already has stable differentiable IK or differentiable kinematics, do not start with gradient-based design optimization.

The first DDIRD prototype should demonstrate:

```text
a data-driven task distribution can change hardware design choices
```

It does not need to prove that the optimizer is advanced.

### 7.3 Recommended Optimization Sequence

First run:

```text
optimize base pose only
```

Second run:

```text
optimize base pose + link lengths
```

Third run:

```text
add joint limits / manipulability / collision
```

Fourth run:

```text
compare with real robot baselines
```

## 8. Experiment Design

### 8.1 Minimum Viable Experiment

The minimum experiment must answer:

```text
Starting from LIBERO EE trajectories,
does optimizing base pose or link lengths improve
IK success / joint-limit margin / manipulability?
```

Experiment groups:

- baseline simple arm;
- optimized simple arm;
- Panda baseline;
- UR5 or xArm baseline.

Metrics:

- IK success rate;
- trajectory-level success;
- mean joint-limit margin;
- minimum manipulability;
- average smoothness cost;
- hardware cost proxy, such as total link length.

### 8.2 Ablation

Run at least two ablations:

```text
reachability only
reachability + joint limit
reachability + joint limit + manipulability
```

The purpose is to show that different metrics lead to different hardware designs.

### 8.3 Train/Test Split

Do not optimize and report on exactly the same trajectories.

Recommended splits:

```text
split by task
or split by episode
```

For example:

```text
70% trajectories for design optimization
30% trajectories for evaluation
```

This answers:

```text
Does the optimized hardware generalize, or does it overfit specific trajectories?
```

## 9. Visualization Requirements

The report should contain at least:

1. end-effector trajectory point cloud;
2. reachable vs unreachable points;
3. robot geometry before and after optimization;
4. IK success by task suite;
5. joint-limit margin histogram;
6. manipulability heatmap or boxplot;
7. table comparing optimized design with commercial baselines.

Recommended plots:

```text
3D scatter:
    gray = all target EE points
    green = reachable
    red = unreachable

bar plot:
    IK success rate for each robot model

box plot:
    joint-limit margin distribution
```

## 10. Recommended Code Structure

If the repository does not yet contain code, use this structure:

```text
src/ddird/
    data/
        extract_libero_ee.py
        dataset.py
    robots/
        simple_chain.py
        urdf_robot.py
        robot_registry.py
    eval/
        ik.py
        metrics.py
        evaluator.py
    optim/
        search.py
        objectives.py
    viz/
        plots.py
    experiments/
        run_extract_libero.py
        run_eval_baselines.py
        run_optimize_simple_arm.py
        run_report_figures.py
```

Recommended output directory:

```text
outputs/libero_ddird/
    extracted_data_stats.json
    baseline_results.csv
    optimization_results.csv
    figures/
    report.md
```

## 11. Execution Order for the Agent

### Step 1: Audit the Data

First, inspect the actual LIBERO data fields:

- whether EE pose is directly available;
- if not, whether it can be computed from robot state or simulator state;
- how orientation is represented;
- how robot base frame and world frame are defined;
- how many tasks and episodes each suite contains.

Output:

```text
outputs/libero_ddird/data_audit.md
```

### Step 2: Extract EE Trajectories

Write a script that extracts and saves:

```text
ee_pos
ee_quat or ee_rotmat
gripper
task metadata
```

Outputs:

```text
data/libero_ee_trajectories/
outputs/libero_ddird/extracted_data_stats.json
```

### Step 3: Implement the Simple Chain

Implement a simple 6-DoF or 7-DoF serial chain with:

- forward kinematics;
- Jacobian;
- joint limits;
- IK solver.

Use an existing robotics library if available. If no library is available, implement a simplified version first.

### Step 4: Implement the Evaluator

Inputs:

```text
robot model theta
EE trajectory xi_ee
```

Outputs:

```text
IK success
joint-limit margin
manipulability
smoothness
collision proxy
```

### Step 5: Run Baselines

Evaluate:

```text
simple default arm
Panda baseline
UR5 / xArm baseline
```

Output:

```text
outputs/libero_ddird/baseline_results.csv
```

### Step 6: Optimize the Simple Arm

First optimize:

```text
base pose only
```

Then optimize:

```text
base pose + link lengths
```

Outputs:

```text
outputs/libero_ddird/optimization_results.csv
outputs/libero_ddird/best_design.json
```

### Step 7: Write the Report

The report must answer:

- Can LIBERO EE trajectories serve as a task-space requirement distribution?
- Does hardware optimization improve the metrics relative to the initial simple model?
- How does the optimized design compare with real robot baselines?
- Where do failure cases concentrate?
- What real industrial data would be needed next?

## 12. Success Criteria

Minimum success:

```text
The agent extracts EE trajectories from LIBERO
and uses them to compare at least 3 candidate robot-arm models.
```

Good success:

```text
The agent optimizes the simple arm's base pose or link lengths
and improves IK success or manipulability on held-out trajectories.
```

Strong success:

```text
The agent shows that different task suites prefer different hardware parameters
and explains those preferences through workspace distribution,
orientation demand, or joint-limit pressure.
```

## 13. Statements to Avoid in the Report

Do not write:

```text
LIBERO proves that DDIRD works in real factories.
```

Write instead:

```text
LIBERO can validate the structure of a DDIRD prototype,
but it is not real industrial operation data.
```

Do not write:

```text
We optimized robot intelligence.
```

Write instead:

```text
We optimized candidate hardware for reachability,
IK feasibility, and kinematic quality under a task-space trajectory distribution.
```

Do not write:

```text
We learned a new robot from LIBERO.
```

Write instead:

```text
We used LIBERO demonstration trajectories to estimate task-space requirements
and then optimized or compared candidate robot-arm designs.
```

## 14. Relationship to Real DDIRD

The LIBERO prototype corresponds to:

```text
simulation demonstration trajectories
    -> task-space requirement distribution
    -> kinematic hardware optimization
```

The target form of real DDIRD is:

```text
factory operation logs
    -> task / failure / torque / stress distribution
    -> kinematic + static + dynamic hardware optimization
```

LIBERO covers only part of DDIRD:

| DDIRD requirement | Covered by LIBERO? | Notes |
|---|---:|---|
| Task-space trajectories | Partially | EE trajectories can be extracted from demonstrations |
| Reachability requirements | Yes | IK and reachability analysis are feasible |
| Failure logs | Mostly no | LIBERO mainly contains successful demonstrations |
| Joint-level torque / stress | Usually no | Requires dynamics simulation or real robot logs |
| Industrial cycle time / maintenance | No | Requires real factory data |
| Hardware inverse-design prototype | Yes | Useful for validating the method structure |

## 15. One-Sentence Summary

Use LIBERO as:

```text
a simulation dataset for prototyping DDIRD
```

not as:

```text
the final industrial validation dataset for DDIRD
```

The most important chain to prove is:

```text
demonstration EE trajectories
    -> task-space requirements
    -> candidate hardware evaluation
    -> hardware parameter optimization
```
