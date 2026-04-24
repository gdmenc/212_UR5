"""Moves 3/4 of pick: straight-line approach along the gripper's tool -Z.

Assumes the TCP is already at the target's task-frame XY and orientation
(``transit_xy`` already ran). ``moveL`` from the current pose to
``target_pose_task`` traces a straight line in Cartesian space, with
orientation interpolated along the way.

IMPORTANT: the approach direction is the **gripper's local tool -Z**, not
task-frame -Z. For a top-down grasp the two coincide (it looks like a
descent); for a side grasp the "approach" is horizontal in task frame.
This is why pregrasp is computed via ``offset_along_tool_z(grasp, d)`` —
so the moveL between pregrasp and grasp is always along the gripper
approach axis regardless of gripper orientation.

Used for:
  - transit altitude -> pregrasp    (distance: ~transit_z − object_z)
  - pregrasp         -> grasp       (distance: pregrasp_offset)
  - transit altitude -> preplace    (place primitive)
"""

from ..arm import ArmHandle
from ..util.poses import Pose
from ..util.rtde_convert import pose_to_rtde


def approach_to(
    arm: ArmHandle,
    target_pose_task: Pose,
    speed: float,
    accel: float,
) -> None:
    arm.control.moveL(pose_to_rtde(arm.to_base(target_pose_task)), speed, accel)
