"""Pose <-> UR axis-angle list conversion.

UR's RTDE API expresses poses as a 6-element list
    [x, y, z, rx, ry, rz]
where (rx, ry, rz) is an **axis-angle rotation vector** — its magnitude is
the rotation angle in radians, its direction is the axis. This is *not*
RPY, quaternion, or Euler. Our local ``Rotation`` helper uses the same
representation.

Keep this boundary code in one place so the move/pick/place primitives
never touch the UR-specific representation directly.
"""

from __future__ import annotations

from typing import List, Sequence

import numpy as np

from .poses import Pose
from .rotations import Rotation


def pose_to_rtde(pose: Pose) -> List[float]:
    """Pose -> [x, y, z, rx, ry, rz] ready for rtde_c.moveL / moveJ_IK."""
    rotvec = pose.rotation.as_rotvec()
    return [
        float(pose.translation[0]),
        float(pose.translation[1]),
        float(pose.translation[2]),
        float(rotvec[0]),
        float(rotvec[1]),
        float(rotvec[2]),
    ]


def rtde_to_pose(rtde_pose: Sequence[float]) -> Pose:
    """[x, y, z, rx, ry, rz] -> Pose. Typically used with
    ``rtde_r.getActualTCPPose()`` to read the current pose."""
    assert len(rtde_pose) == 6, f"expected length-6 pose, got {len(rtde_pose)}"
    t = np.asarray(rtde_pose[:3], dtype=float)
    rot = Rotation.from_rotvec(np.asarray(rtde_pose[3:], dtype=float))
    return Pose(translation=t, rotation=rot)
