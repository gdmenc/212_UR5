"""Open-microwave primitive: hook the handle and sweep along the door arc.

Specialized for the microwave task:
  - The door rotates about a fixed hinge axis. The handle therefore moves
    on a known circular arc — radius = distance from hinge to handle,
    center on the hinge axis.
  - We parameterize the TCP target as a function of door angle theta. At
    each theta we have a desired handle position and approach orientation,
    which together define X_WTcp(theta).
  - The custom ``HookGripper`` latches onto the handle; we then stream the
    swept Cartesian path via DiffIK + ServoStream.
  - The hook stays latched throughout; we only unlatch after the door is
    fully open and the arm has retracted.

High-level flow:

    1. MoveJ / ServoStream to approach pose just outside the handle.
    2. MoveL onto the handle.
    3. GripperCommand("close")  # latch the hook
    4. diff_ik sweep from theta=0 to theta=theta_open; ServoStream.
    5. GripperCommand("open")   # unlatch
    6. MoveL retract.

The hinge axis and handle radius live on the scene's microwave object;
scene.py is expected to expose them as ``scene.microwave.hinge_axis``,
``scene.microwave.handle_radius``, etc.
"""

from typing import Optional

import numpy as np

from .base import PrimitiveResult


def open_microwave(
    backend,
    planner,
    scene,
    arm: str = "ur_left",
    theta_open: float = np.pi / 2,  # radians; 90deg door open
    num_samples: int = 100,
) -> PrimitiveResult:
    """Latch the hook on the door handle and sweep to theta_open."""
    # TODO
    raise NotImplementedError
