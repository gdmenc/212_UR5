"""Place primitive: approach, descend until contact, release.

Inverse of pick. Assumes the arm is already holding something (``pick`` was
called earlier and returned success). ``target_pose`` is the intended
resting pose of the OBJECT, not the gripper — we compute the gripper pose
from ``target_pose * inverse(X_OG_used_for_pick)``. ``MoveUntilContact`` is
the last inch so we do not have to hand-measure table/shelf heights.

High-level flow:

    X_WG_above = target_object_pose * X_OG_inv * hover_offset
    q_goal = solve_ik(plant, X_WG_above, q_seed=scene.q(arm))
    backend.execute(arm, ServoStream(sample(plan_trajopt(...))))
    backend.execute(arm, MoveUntilContact(v_base=downward))
    backend.execute(arm, Wait(0.2))               # settle
    backend.execute(arm, GripperCommand("open"))
    backend.execute(arm, MoveL(X_WG_above))        # retract
"""

from typing import Optional

from pydrake.math import RigidTransform

from .base import PrimitiveResult


def place(
    backend,
    planner,
    scene,
    target_pose: RigidTransform,
    arm: str = "ur_left",
    hover_offset: float = 0.1,   # meters above target
    contact_threshold: float = 10,  # Newtons for MoveUntilContact
) -> PrimitiveResult:
    """Release the held object at ``target_pose``."""
    # TODO
    raise NotImplementedError
