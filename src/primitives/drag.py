"""Drag primitive: pull an object along a horizontal direction with the hook.

Used in the recipe after the plate has been placed in the microwave and
the two-finger arm can't quite reach to the back of the cavity — the hook
arm latches the plate rim and drags it forward a few centimeters so the
two-finger arm can grasp it.

Mechanically a hook-specific cousin of ``push``. Position control on the
drag itself is fine because we are pulling AWAY from any obstacle the
object is leaning against (a push-in-contact would need force mode).
Before and after the drag we still need compliant behavior so the hook
does not overload the handle during latch/unlatch.

High-level flow:

    1. MoveJ / ServoStream to approach pose just outside the object rim.
    2. MoveL onto the latching pose.
    3. GripperCommand("close")       # hook latches
    4. MoveL by `travel` along `direction` (horizontal, away from the
       resistance — e.g. out of the microwave).
    5. GripperCommand("open")        # unlatch
    6. MoveL retract back from the object.
"""

from typing import Optional

import numpy as np
from pydrake.math import RigidTransform

from .base import PrimitiveResult


def drag(
    backend,
    planner,
    scene,
    arm: str,                       # normally "ur_left" (hook arm)
    start_pose: RigidTransform,     # world pose of the hook latching point
    direction: np.ndarray,           # shape (3,), unit vector in world frame
    travel: float,                   # meters; how far to drag
    approach_standoff: float = 0.05,
) -> PrimitiveResult:
    """Latch with the hook at ``start_pose``, drag ``travel`` meters along
    ``direction``, release."""
    # TODO
    raise NotImplementedError
