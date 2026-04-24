"""Move 2 of pick/place: horizontal transit at the safe altitude (task frame).

Moves to the target pose's XY and simultaneously rotates to the target
orientation, all at task-frame Z = ``transit_z``. After this move the TCP
is directly above the pick/place target with the correct gripper
orientation — a pure descend along tool -Z handles the rest.

``target_pose_task`` is in task frame. Conversion to base happens at the
RTDE boundary.
"""

from ..arm import ArmHandle
from ..util.poses import Pose, pose_at_altitude
from ..util.rtde_convert import pose_to_rtde


def transit_xy(
    arm: ArmHandle,
    target_pose_task: Pose,
    transit_z: float,
    speed: float,
    accel: float,
) -> None:
    waypoint_task = pose_at_altitude(target_pose_task, transit_z)
    arm.control.moveL(pose_to_rtde(arm.to_base(waypoint_task)), speed, accel)
