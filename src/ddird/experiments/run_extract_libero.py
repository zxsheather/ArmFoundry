from __future__ import annotations

import argparse
from pathlib import Path

from ddird.data.extract_libero_ee import (
    audit_source,
    create_synthetic_dataset,
    extract_source_to_processed,
    write_audit_markdown,
    write_stats,
)
from ddird.experiments.common import DEFAULT_DATA_ROOT, DEFAULT_OUTPUT_ROOT


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Audit and extract LIBERO end-effector trajectories.")
    parser.add_argument("--input", type=Path, default=None, help="LIBERO demonstration file or directory.")
    parser.add_argument("--output", type=Path, default=DEFAULT_DATA_ROOT, help="Processed trajectory output directory.")
    parser.add_argument("--outputs", type=Path, default=DEFAULT_OUTPUT_ROOT, help="Experiment output directory.")
    parser.add_argument("--coordinate-frame", default="unknown", help="world, robot_base, workspace, etc.")
    parser.add_argument("--synthetic", action="store_true", help="Generate a deterministic synthetic LIBERO-like fixture.")
    parser.add_argument("--episodes-per-task", type=int, default=4)
    parser.add_argument("--timesteps", type=int, default=48)
    parser.add_argument("--seed", type=int, default=7)
    return parser


def main() -> None:
    args = build_parser().parse_args()
    args.outputs.mkdir(parents=True, exist_ok=True)

    audit = audit_source(args.input)
    if args.synthetic:
        audit["note"] = "Synthetic fixture requested; no real LIBERO source was required."
    write_audit_markdown(audit, args.outputs / "data_audit.md")

    if args.synthetic:
        records = create_synthetic_dataset(
            args.output,
            episodes_per_task=args.episodes_per_task,
            timesteps=args.timesteps,
            seed=args.seed,
        )
        stats_extra = {
            "source": "Synthetic LIBERO-like fixture",
            "coordinate_frame": "world",
            "position_only_evaluation": True,
        }
    else:
        if args.input is None:
            raise SystemExit("--input is required unless --synthetic is used")
        records = extract_source_to_processed(args.input, args.output, coordinate_frame=args.coordinate_frame)
        stats_extra = {
            "source": "LIBERO",
            "coordinate_frame": args.coordinate_frame,
            "position_only_evaluation": True,
        }

    write_stats(records, args.outputs / "extracted_data_stats.json", extra=stats_extra)
    print(f"Wrote {len(records)} processed trajectories to {args.output}")
    print(f"Wrote audit and stats to {args.outputs}")


if __name__ == "__main__":
    main()
