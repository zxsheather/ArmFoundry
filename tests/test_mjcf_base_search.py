from __future__ import annotations

import csv
import json
from pathlib import Path

import numpy as np

from ddird.data.dataset import save_processed_trajectory
from ddird.experiments.run_mjcf_base_search import build_parser, main


FIXTURE_XML = """
<mujoco model="fixture">
  <worldbody>
    <body name="base">
      <body name="link">
        <joint name="joint1" axis="0 0 1" range="-1 1"/>
        <site name="tcp" pos="0 0 0"/>
      </body>
    </body>
  </worldbody>
</mujoco>
"""


def _read_csv(path: Path) -> list[dict[str, str]]:
    with path.open(newline="") as handle:
        return list(csv.DictReader(handle))


def _write_fixture_dataset(root: Path) -> None:
    for task in ["task_a", "task_b"]:
        for episode in range(2):
            save_processed_trajectory(
                root,
                suite="fixture_suite",
                task=task,
                episode_id=f"{episode}",
                ee_pos=np.zeros((3, 3), dtype=float),
                metadata={
                    "source": "fixture",
                    "coordinate_frame": "world",
                    "source_robot_base_xyz": [0.0, 0.0, 0.0],
                    "source_robot_base_yaw": 0.0,
                },
            )


def test_mjcf_base_search_cli_exposes_train_test_options():
    args = build_parser().parse_args(
        [
            "--mjcf",
            "robot.xml",
            "--robot-name",
            "xarm6_true",
            "--base-body",
            "link_base",
            "--search-scope",
            "suite",
            "--split-by",
            "task",
            "--train-fraction",
            "0.6",
            "--num-candidates",
            "12",
            "--evaluation-mode",
            "pose",
            "--tool-frame-mapping",
            "canonical_tool",
        ]
    )

    assert args.search_scope == "suite"
    assert args.split_by == "task"
    assert args.train_fraction == 0.6
    assert args.num_candidates == 12
    assert args.evaluation_mode == "pose"
    assert args.tool_frame_mapping == "canonical_tool"


def test_mjcf_base_search_writes_train_test_outputs(tmp_path):
    data_root = tmp_path / "data"
    outputs = tmp_path / "outputs"
    xml_path = tmp_path / "fixture.xml"
    xml_path.write_text(FIXTURE_XML, encoding="utf-8")
    _write_fixture_dataset(data_root)

    main(
        [
            "--mjcf",
            str(xml_path),
            "--robot-name",
            "fixture_robot",
            "--base-body",
            "base",
            "--target-site",
            "tcp",
            "--target-body",
            "link",
            "--data",
            str(data_root),
            "--outputs",
            str(outputs),
            "--search-scope",
            "suite",
            "--split-by",
            "task",
            "--train-fraction",
            "0.5",
            "--num-candidates",
            "1",
            "--seed",
            "3",
            "--max-waypoints-per-trajectory",
            "2",
            "--max-iters",
            "1",
            "--evaluation-mode",
            "position",
            "--num-workers",
            "1",
        ]
    )

    search_rows = _read_csv(outputs / "search_candidates.csv")
    best_rows = _read_csv(outputs / "best_train_base.csv")
    validation_rows = _read_csv(outputs / "train_test_results.csv")
    comparison_rows = _read_csv(outputs / "train_test_comparison.csv")
    aggregate_rows = _read_csv(outputs / "train_test_aggregate_comparison.csv")
    metadata = json.loads((outputs / "metadata.json").read_text(encoding="utf-8"))

    assert len(search_rows) == 2
    assert len(best_rows) == 1
    assert len(validation_rows) == 4
    assert len(comparison_rows) == 1
    assert len(aggregate_rows) == 4
    assert {row["split"] for row in validation_rows} == {"train", "test"}
    assert {row["base_label"] for row in validation_rows} == {"reference_base", "best_train_base"}
    assert comparison_rows[0]["group"] == "fixture_suite"
    assert metadata["split_by"] == "task"
    assert metadata["split"]["fixture_suite"]["num_train_trajectories"] == 2
    assert metadata["split"]["fixture_suite"]["num_test_trajectories"] == 2
    assert (outputs / "analysis_report.md").exists()
