"""Move 1 of pick/place: straight-up lift to transit altitude (task frame).

Holds current task-frame XY and orientation, raises task-frame Z to
``transit_z``. Pure vertical in *task* frame — this is the universal-safe
first move: going straight up in the shared workspace frame cannot collide
with anything else in the workspace.

Because the arm is mounted at 45° pointing down at the table, "straight up
in task frame" is NOT "straight up in base frame" — it's a diagonal move
in base-frame coordinates. The base conversion happens via arm.to_base().
"""

from ..arm import ArmHandle
from ..util.poses import pose_at_altitude
from ..util.rtde_convert import pose_to_rtde, rtde_to_pose


def lift_to_transit(
    arm: ArmHandle,
    transit_z: float,
    speed: float,
    accel: float,
) -> None:
    current_base = rtde_to_pose(arm.receive.getActualTCPPose())
    current_task = arm.to_task(current_base)
    target_task = pose_at_altitude(current_task, transit_z)
    arm.control.moveL(pose_to_rtde(arm.to_base(target_task)), speed, accel)
