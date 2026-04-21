"""Push primitive: compliant Cartesian push along a task-frame axis.

Used for:
  - Tray rotation: both arms push opposite corners to produce torque about
    the tray center. Non-prehensile — we do not grasp the tray.
  - Microwave door close: single-arm push against the door.

Force mode, not position control. Position control would apply unbounded
force through the environment's compliance, which is dangerous and
unreliable. Force mode commands a target wrench along the push axis and lets
the arm be compliant — the arm reacts to resistance instead of fighting it.

High-level flow:

    1. MoveL to approach_pose (just off the contact surface).
    2. MoveUntilContact along push direction until contact detected.
    3. ForceMode with the selected wrench; monitor travel and torque each
       tick (the backend exposes ``state(arm)["wrench"]``).
    4. Terminate when travel >= travel_target OR wrench exceeds safety
       limits OR the goal (e.g. tray angle) is reached.
    5. Zero the commanded force; MoveL retract.
"""

from typing import Optional

import numpy as np
from pydrake.math import RigidTransform

from .base import PrimitiveResult


def push(
    backend,
    planner,
    scene,
    arm: str,
    push_frame: RigidTransform,  # task frame for ForceMode
    direction: np.ndarray,        # shape (3,), unit vector in push_frame
    max_force: float,             # N
    travel: float,                # m, termination distance
    approach_standoff: float = 0.03,
) -> PrimitiveResult:
    """Compliant push along ``direction`` in ``push_frame``."""
    # TODO
    raise NotImplementedError
