"""Bimanual tray carry: both arms grasp handles, carry to target, keep level.

Used in the recipe's final step: the tray is loaded with bowl/plate/cup
and must reach its final location without spilling.

Why not single-arm: a single UR5e grasping the tray at one handle would
swing under the tray's mass + payload torque. Two arms with rigid grasps
on opposite handles let us treat the tray+arms as a closed kinematic loop
that we drive in world space, and put a direct constraint on the tray's
tilt to bound spillage.

Key constraint: the tray's surface normal must stay within ``tilt_bound``
radians of world-up throughout the motion. We enforce this by computing
the two arms' target wrist poses from a commanded tray pose (tray keeps
a level orientation at every waypoint) rather than planning each arm's
trajectory independently.

High-level flow:

    1. Both arms already grasping the tray handles (pick-for-tray already
       happened as a precondition, or via two parallel `pick`s on the
       left/right handle indices). Caller is expected to hand in the
       current X_LeftHandle_World and X_RightHandle_World in ``scene``.
    2. Generate a Cartesian path for the tray center pose from its
       current pose to ``target_pose``, with orientation held level
       (only position interpolates).
    3. For each tray waypoint, compute per-arm wrist targets using
       the fixed handle offsets in the tray frame.
    4. Send the two per-arm trajectories through the backend
       simultaneously (ServoStream each, same dt, same N).
    5. Release handles at destination (optional; usually another step).
"""

from typing import Optional

from pydrake.math import RigidTransform

from ..base import PrimitiveResult


def carry_tray(
    backend,
    planner,
    scene,
    target_pose: RigidTransform,  # world pose of the TRAY center at destination
    left_arm: str = "ur_left",
    right_arm: str = "ur_right",
    tilt_bound: float = 0.1,       # radians; max deviation from level
    duration: float = 3.0,         # seconds to interpolate the carry
    release_at_end: bool = False,
) -> PrimitiveResult:
    """Move the tray from its current pose to ``target_pose``, keeping level.

    Preconditions (caller is responsible):
      - Both arms are already grasping the tray handles.
      - ``scene.named_poses["tray_current"]`` (or similar) gives the tray
        pose at start; handle offsets in the tray frame are known.
    """
    # TODO
    raise NotImplementedError
