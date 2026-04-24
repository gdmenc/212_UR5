"""Shared data structures for hard-coded autonomous trials.

This layer is intentionally simple: a trial is
    - one arm
    - a set of named joint targets
    - a set of named task-frame waypoints
    - workspace limits / keep-out boxes in the task frame
    - a sequence of steps that references those names

That gives you one place to edit the values you care about tomorrow without
rewriting the runner.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence

import numpy as np

from ..util.poses import Pose
from ..util.rotations import Rotation


@dataclass
class BoxRegion:
    """Axis-aligned box in the shared task frame."""

    name: str
    xyz_min: Sequence[float]
    xyz_max: Sequence[float]

    def __post_init__(self) -> None:
        self.xyz_min = np.asarray(self.xyz_min, dtype=float).reshape(3)
        self.xyz_max = np.asarray(self.xyz_max, dtype=float).reshape(3)
        if np.any(self.xyz_max < self.xyz_min):
            raise ValueError(
                f"box {self.name!r} has xyz_max < xyz_min: "
                f"{self.xyz_min} -> {self.xyz_max}"
            )

    def contains(self, xyz: Sequence[float], padding: float = 0.0) -> bool:
        point = np.asarray(xyz, dtype=float).reshape(3)
        return bool(
            np.all(point >= (self.xyz_min - padding))
            and np.all(point <= (self.xyz_max + padding))
        )


@dataclass
class TaskWaypoint:
    """Named Cartesian target expressed in the task frame.

    `xyz` is the part you will edit most often.
    `rotvec` is optional so a path can be defined as mostly 3D points while
    keeping the current tool orientation.
    """

    name: str
    xyz: Sequence[float]
    rotvec: Optional[Sequence[float]] = None

    def __post_init__(self) -> None:
        self.xyz = np.asarray(self.xyz, dtype=float).reshape(3)
        if self.rotvec is not None:
            self.rotvec = np.asarray(self.rotvec, dtype=float).reshape(3)

    def to_pose(self, fallback_rotvec: Optional[Sequence[float]] = None) -> Pose:
        rotvec = self.rotvec if self.rotvec is not None else fallback_rotvec
        if rotvec is None:
            raise ValueError(
                f"waypoint {self.name!r} has no orientation. "
                "Set rotvec explicitly or provide a fallback rotation."
            )
        return Pose(
            translation=self.xyz.copy(),
            rotation=Rotation.from_rotvec(np.asarray(rotvec, dtype=float).reshape(3)),
        )

    @classmethod
    def from_pose(cls, name: str, pose: Pose) -> "TaskWaypoint":
        return cls(
            name=name,
            xyz=pose.translation.copy(),
            rotvec=pose.rotation.as_rotvec(),
        )


@dataclass
class JointTarget:
    """Named joint-space target stored in degrees for easier editing."""

    name: str
    joints_deg: Sequence[float]
    speed: float = 1.05
    accel: float = 1.4

    def __post_init__(self) -> None:
        self.joints_deg = np.asarray(self.joints_deg, dtype=float).reshape(6)

    def joints_rad(self) -> np.ndarray:
        return np.radians(self.joints_deg)


@dataclass
class TrialStep:
    kind: str
    target: Optional[str] = None
    label: Optional[str] = None
    duration: Optional[float] = None
    speed: Optional[float] = None
    accel: Optional[float] = None


@dataclass
class TrialDefinition:
    name: str
    description: str
    arm: str
    x_base_task: Pose
    tcp_offset: Sequence[float]
    joint_targets: Dict[str, JointTarget]
    waypoints: Dict[str, TaskWaypoint]
    sequence: List[TrialStep]

    ip: Optional[str] = None
    gripper: Optional[str] = None
    default_tool_rotvec_task: Optional[Sequence[float]] = None
    workspace_limits: Optional[BoxRegion] = None
    keepout_boxes: List[BoxRegion] = field(default_factory=list)
    default_linear_speed: float = 0.10
    default_linear_accel: float = 0.25
    samples_per_segment: int = 25

    def __post_init__(self) -> None:
        if self.arm not in {"ur_left", "ur_right"}:
            raise ValueError(f"unsupported arm {self.arm!r}")
        self.tcp_offset = np.asarray(self.tcp_offset, dtype=float).reshape(6)
        if self.default_tool_rotvec_task is not None:
            self.default_tool_rotvec_task = np.asarray(
                self.default_tool_rotvec_task,
                dtype=float,
            ).reshape(3)

    def resolved_ip(self) -> str:
        if self.ip is not None:
            return self.ip
        return "192.168.1.101" if self.arm == "ur_left" else "192.168.1.102"


_SUPPORTED_STEP_KINDS = {
    "report_state",
    "move_home",
    "move_j",
    "move_l",
    "wait",
    "open_gripper",
    "close_gripper",
}


def _step(kind: str, **kwargs) -> TrialStep:
    if kind not in _SUPPORTED_STEP_KINDS:
        raise ValueError(f"unsupported trial step kind {kind!r}")
    return TrialStep(kind=kind, **kwargs)


def report_state(label: str) -> TrialStep:
    return _step("report_state", label=label)


def move_home(target: str = "home") -> TrialStep:
    return _step("move_home", target=target)


def move_j(
    target: str,
    speed: Optional[float] = None,
    accel: Optional[float] = None,
    label: Optional[str] = None,
) -> TrialStep:
    return _step("move_j", target=target, speed=speed, accel=accel, label=label)


def move_l(
    target: str,
    speed: Optional[float] = None,
    accel: Optional[float] = None,
    label: Optional[str] = None,
) -> TrialStep:
    return _step("move_l", target=target, speed=speed, accel=accel, label=label)


def wait_seconds(seconds: float, label: Optional[str] = None) -> TrialStep:
    return _step("wait", duration=float(seconds), label=label)


def open_gripper() -> TrialStep:
    return _step("open_gripper")


def close_gripper() -> TrialStep:
    return _step("close_gripper")
