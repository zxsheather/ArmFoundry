# ArmFoundry

ArmFoundry is a minimal DDIRD prototype for evaluating robot-arm hardware
against a distribution of end-effector trajectories. The implementation follows
`docs/guide.md`: extract or generate EE trajectories, evaluate candidate arms
with IK/reachability/manipulability metrics, optimize simple hardware
parameters, compare against commercial-arm proxies, and write a report.

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
  --num-workers 4
```

`--max-trajectories` limits how many trajectories are selected. `--max-waypoints-per-trajectory`
keeps each selected trajectory in temporal order but evenly samples at most that
many waypoints for the current run. It does not rewrite the processed `.npz`
files.

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
hardware evaluation and simple hardware search.
