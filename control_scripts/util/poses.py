"""Lightweight Pose type and compositional helpers.

Uses numpy plus the local ``util.rotations.Rotation`` helper so this module
has no pydrake dependency and no scipy dependency.

A ``Pose`` is (translation: (3,) ndarray, rotation: Rotation).
Composition is ``A @ B`` (left-to-right, same convention as pydrake's
RigidTransform). ``Pose.inverse()`` gives the frame swap.

Primitives think in Pose. Conversion to the RTDE pose list
``[x, y, z, rx, ry, rz]`` happens at the boundary in ``rtde_convert.py``.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Sequence

import numpy as np
from .rotations import Rotation


@dataclass
class Pose:
    """Rigid transform. All poses in this package live in the arm's base
    frame unless explicitly annotated otherwise."""

    translation: np.ndarray = field(
        default_factory=lambda: np.zeros(3)
    )
    rotation: Rotation = field(default_factory=Rotation.identity)

    def __post_init__(self) -> None:
        self.translation = np.asarray(self.translation, dtype=float).reshape(3)

    def __matmul__(self, other: "Pose") -> "Pose":
        """Compose two poses: ``(A @ B).apply(p) == A.apply(B.apply(p))``."""
        return Pose(
            translation=self.translation + self.rotation.apply(other.translation),
            rotation=self.rotation * other.rotation,
        )

    def inverse(self) -> "Pose":
        r_inv = self.rotation.inv()
        return Pose(translation=-r_inv.apply(self.translation), rotation=r_inv)

    @property
    def xyz(self) -> np.ndarray:
        return self.translation.copy()


def translation(xyz: Sequence[float]) -> Pose:
    """Pose with identity rotation at xyz."""
    return Pose(translation=np.asarray(xyz, dtype=float), rotation=Rotation.identity())


def offset_along_tool_z(grasp: Pose, distance: float) -> Pose:
    """Return a pose ``distance`` meters behind ``grasp`` along the grasp's
    local -Z axis (standard 'pregrasp' / 'preplace' offset for a gripper
    whose approach direction is +tool_Z)."""
    return grasp @ translation([0.0, 0.0, -distance])


def pose_at_altitude(pose: Pose, z: float) -> Pose:
    """Same XY and rotation as ``pose`` but Z replaced with ``z`` in the
    base frame. Used to build the 'lift' and 'transit' waypoints."""
    xy = pose.translation[:2]
    return Pose(
        translation=np.array([xy[0], xy[1], z]),
        rotation=pose.rotation,
    )


def with_rotation_of(xy_from: Pose, rotation_from: Pose) -> Pose:
    """XY from one pose, rotation from another, same Z as ``xy_from``. Used
    when transiting: hold current XY/Z for lift, then adopt target orientation
    for the horizontal transit step."""
    return Pose(translation=xy_from.translation.copy(), rotation=rotation_from.rotation)
