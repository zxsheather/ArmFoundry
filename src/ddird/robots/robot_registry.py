from __future__ import annotations

from ddird.robots.mjcf_chain import make_libero_panda_true
from ddird.robots.simple_chain import SimpleChainRobot, make_simple_arm


def create_robot(name: str):
    key = name.lower()
    if key in {"simple_default", "simple"}:
        return make_simple_arm(
            name="simple_default",
            base_xyz=(0.0, 0.0, 0.0),
            base_yaw=0.0,
            link_lengths=(0.30, 0.34, 0.28, 0.16),
            source="parameterized",
        )
    if key in {"panda", "franka_panda"}:
        return make_simple_arm(
            name="panda_proxy",
            base_xyz=(0.0, 0.0, 0.0),
            base_yaw=0.0,
            link_lengths=(0.333, 0.316, 0.284, 0.107),
            source="commercial_proxy",
        )
    if key in {"panda_true", "franka_panda_true"}:
        return make_libero_panda_true()
    if key in {"ur5", "ur5e"}:
        return make_simple_arm(
            name="ur5_proxy",
            base_xyz=(0.0, 0.0, 0.0),
            base_yaw=0.0,
            link_lengths=(0.1625, 0.425, 0.3922, 0.1333),
            source="commercial_proxy",
        )
    if key in {"xarm6", "xarm"}:
        return make_simple_arm(
            name="xarm6_proxy",
            base_xyz=(0.0, 0.0, 0.0),
            base_yaw=0.0,
            link_lengths=(0.267, 0.293, 0.284, 0.102),
            source="commercial_proxy",
        )
    raise KeyError(f"Unknown robot {name!r}")


def baseline_robots(include_true_models: bool = False) -> list:
    robots = [
        create_robot("simple_default"),
        create_robot("panda"),
        create_robot("ur5"),
        create_robot("xarm6"),
    ]
    if include_true_models:
        robots.insert(2, create_robot("panda_true"))
    return robots
