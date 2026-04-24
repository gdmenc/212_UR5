"""Moves 6/7 of pick, 6/7 of place: retract along the gripper's tool +Z.

Symmetric to ``approach_to`` — same moveL call, inverse direction. Kept
separate so pick/place reads like prose (``approach_to`` on the way in,
``retract_to`` on the way out) and so retract-while-holding can diverge
later in speed (e.g., slower retract with a plate to avoid sloshing).

``target_pose_task`` is in task frame.
"""

from ..arm import ArmHandle
from ..util.poses import Pose
from ..util.rtde_convert import pose_to_rtde


def retract_to(
    arm: ArmHandle,
    target_pose_task: Pose,
    speed: float,
    accel: float,
) -> None:
    arm.control.moveL(pose_to_rtde(arm.to_base(target_pose_task)), speed, accel)
