from __future__ import annotations

from ddird.data.dataset import create_synthetic_dataset, load_dataset, summarize_dataset


def test_create_and_load_synthetic_dataset(tmp_path):
    root = tmp_path / "libero_ee_trajectories"
    records = create_synthetic_dataset(root, episodes_per_task=1, timesteps=8, seed=1)

    loaded = load_dataset(root)
    summary = summarize_dataset(loaded)

    assert len(records) == 4
    assert len(loaded) == 4
    assert summary["num_tasks"] == 4
    assert summary["num_waypoints"] == 32
    assert summary["contains_orientation"] is True
    assert summary["contains_gripper"] is True


def test_downsample_preserves_endpoints_and_metadata(tmp_path):
    root = tmp_path / "libero_ee_trajectories"
    record = create_synthetic_dataset(root, episodes_per_task=1, timesteps=11, seed=1)[0]

    downsampled = record.downsample(5)

    assert record.num_waypoints == 11
    assert downsampled.num_waypoints == 5
    assert (downsampled.ee_pos[0] == record.ee_pos[0]).all()
    assert (downsampled.ee_pos[-1] == record.ee_pos[-1]).all()
    assert downsampled.metadata["downsampled"] is True
    assert downsampled.metadata["original_num_waypoints"] == 11
