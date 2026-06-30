# ArmFoundry

ArmFoundry is a minimal DDIRD prototype for evaluating robot-arm hardware
against a distribution of end-effector trajectories. The implementation follows
`docs/guide.md`: extract or generate EE trajectories, evaluate candidate arms
with IK/reachability/manipulability metrics, optimize simple hardware
parameters, compare against commercial-arm proxies or explicitly loaded real
kinematic models, and write a report.

## Setup

```bash
uv sync --extra libero --extra dev
```

`h5py` is only needed when extracting from HDF5/robomimic-style LIBERO exports.

## Run the Prototype

The repo can run end-to-end without LIBERO data by creating a deterministic
synthetic trajectory fixture:

```bash
uv run python -m ddird.experiments.run_extract_libero --synthetic
uv run python -m ddird.experiments.run_eval_baselines
uv run python -m ddird.experiments.run_optimize_simple_arm
uv run python -m ddird.experiments.run_report_figures
```

With real LIBERO exports, point the extractor at a directory or file and set the
coordinate frame explicitly:

```bash
uv run python -m ddird.experiments.run_extract_libero \
  --input /path/to/libero/demos \
  --coordinate-frame world
```

The extractor refuses to produce processed trajectories from real data unless
the coordinate frame is explicit, because running IK with an unclear frame would
invalidate the results.

## Using the ArmForge Download

This checkout is configured to use the LIBERO data downloaded by ArmForge via a
symlink:

```text
data/armforge -> /Users/zxsheather/Project/ArmForge/data
```

Extract the linked raw HDF5 files into this repo's processed format:

```bash
uv run python -m ddird.experiments.run_extract_libero \
  --input data/armforge/raw/libero \
  --coordinate-frame world \
  --output data/libero_ee_trajectories_armforge \
  --outputs outputs/libero_ddird_armforge
```

The linked dataset is large enough that full evaluation is slower than the
synthetic smoke test. Use a bounded subset first:

```bash
uv run python -m ddird.experiments.run_eval_baselines \
  --data data/libero_ee_trajectories_armforge \
  --outputs outputs/libero_ddird_armforge_sample \
  --suite libero_spatial \
  --max-trajectories 12 \
  --max-waypoints-per-trajectory 40 \
  --num-workers 4 \
  --base-pose-mode source
```

`--max-trajectories` limits how many trajectories are selected. `--max-waypoints-per-trajectory`
keeps each selected trajectory in temporal order but evenly samples at most that
many waypoints for the current run. It does not rewrite the processed `.npz`
files.

For LIBERO world-frame data, source-base evaluation is the default. It places
the robot model at the source workcell base pose parsed from LIBERO metadata or
from the built-in suite-level fallback. Use `--base-pose-mode fixed` only as a
diagnostic.

Built-in `*_proxy` robots are simplified serial-chain approximations, not real
commercial robot models. Use `--include-true-models` to include available true
kinematic baselines such as `panda_true`:

```bash
uv run python -m ddird.experiments.run_eval_baselines \
  --data data/libero_ee_trajectories_armforge \
  --outputs outputs/libero_true_models_sample \
  --suite libero_spatial \
  --max-trajectories 12 \
  --max-waypoints-per-trajectory 40 \
  --num-workers 4 \
  --include-true-models
```

Check the source Panda FK against raw LIBERO `joint_states` with:

```bash
uv run python -m ddird.experiments.run_check_libero_panda_fk \
  --input data/armforge/raw/libero \
  --outputs outputs/libero_panda_fk_check
```

Evaluate an external real MJCF robot model, such as a UR5 or xArm model, with:

```bash
uv run python -m ddird.experiments.run_eval_mjcf_robot \
  --mjcf /path/to/robot.xml \
  --robot-name ur5_true \
  --base-body robot0_base \
  --target-site tool0 \
  --target-body tool0 \
  --data data/libero_ee_trajectories_armforge \
  --outputs outputs/ur5_true_sourcebase \
  --max-waypoints-per-trajectory 80 \
  --base-pose-mode source
```

For the downloaded MuJoCo Menagerie UR5e model:

```bash
uv run --extra mujoco python -m ddird.experiments.run_check_mjcf_fk \
  --mjcf data/armforge/models/mujoco_menagerie/universal_robots_ur5e/ur5e.xml \
  --robot-name ur5e_true \
  --base-body base \
  --target-site attachment_site \
  --target-body wrist_3_link \
  --outputs outputs/ur5e_mjcf_fk_check

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
  --max-iters 80 \
  --base-pose-mode source
```

For xArm6, generate an evaluator-compatible MJCF from the downloaded official
xArm ROS kinematics first:

```bash
uv run python -m ddird.experiments.run_generate_xarm6_mjcf \
  --xarm-ros-root data/armforge/models/xarm_ros \
  --outputs outputs/generated_models/xarm6_true \
  --robot-name xarm6_true \
  --tcp-site link_tcp

uv run --extra mujoco python -m ddird.experiments.run_check_mjcf_fk \
  --mjcf outputs/generated_models/xarm6_true/xarm6_true.xml \
  --robot-name xarm6_true \
  --base-body link_base \
  --target-site link_tcp \
  --target-body link6 \
  --outputs outputs/xarm6_mjcf_fk_check
```

The generated xArm6 model places `link_tcp` at the official xArm gripper TCP
offset, `0 0 0.172`, relative to `link6`. Pass `--tcp-offset 0 0 0` only for
wrist-origin diagnostics.

## Outputs

Processed trajectory data is written to:

```text
data/libero_ee_trajectories/
```

Experiment outputs are written to:

```text
outputs/libero_ddird/
```

Important outputs include:

- `data_audit.md`
- `extracted_data_stats.json`
- `baseline_results.csv`
- `optimization_results.csv`
- `best_design.json`
- `figures/`
- `report.md`

## Scope

This is a kinematic first-stage prototype. It does not train a LIBERO policy,
does not optimize robot intelligence, and does not claim industrial validation.
It uses demonstration end-effector trajectories as task-space requirements for
hardware evaluation and simple hardware search. `*_proxy` results are proxy
results only; real robot claims require explicitly loaded real kinematic models
such as `panda_true` or an external MJCF robot evaluated with
`run_eval_mjcf_robot`.
