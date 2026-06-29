from __future__ import annotations

from ddird.robots.simple_chain import SimpleChainRobot, make_simple_arm


def create_robot(name: str) -> SimpleChainRobot:
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


def baseline_robots() -> list[SimpleChainRobot]:
    return [
        create_robot("simple_default"),
        create_robot("panda"),
        create_robot("ur5"),
        create_robot("xarm6"),
    ]
