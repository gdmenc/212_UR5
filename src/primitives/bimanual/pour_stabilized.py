"""Bimanual pour: hook stabilizes the cup, two-finger pours the bottle.

Used in the recipe when the cup is free on the counter and the pour
needs a steady reference. The hook latches the cup's handle/rim to
prevent it from tipping or sliding under reaction forces from the pour;
the two-finger arm executes a standard single-arm ``pour`` while the
hook arm holds position.

Bimanual because the two motions are concurrent (the cup must be held
throughout the pour, not before-and-after). The sequencer cannot just
call ``pick(cup, hook_arm)`` -> ``pour(bottle, pour_arm)`` because the
hook arm would need to be static-holding while the pour is running, and
that's a coordination pattern the sequencer is not set up for.

High-level flow:

    1. (Precondition) hook_arm has already latched the cup and is
       holding it at its rest position.
    2. (Precondition) pour_arm already holds the bottle above the cup's
       pour origin.
    3. Loop: run pour trajectory on pour_arm (rotating the bottle) while
       holding cup position on hook_arm. In sim, both execute via the
       backend in the same AdvanceTo window. On real, hook_arm stays in
       forceMode with zero commanded wrench (compliant hold) while
       pour_arm runs servoJ.
    4. After pour completes, optionally return bottle to upright before
       releasing.
"""

from typing import Optional

import numpy as np

from ..base import PrimitiveResult


def pour_stabilized(
    backend,
    planner,
    scene,
    pour_arm: str = "ur_right",   # two-finger arm, holding the bottle
    holder_arm: str = "ur_left",   # hook arm, latched onto the cup
    pour_origin: Optional[np.ndarray] = None,  # world (3,)
    pour_axis: Optional[np.ndarray] = None,    # world (3,)
    pour_angle: float = 1.5,        # radians
    hold_time: float = 2.0,
) -> PrimitiveResult:
    """Execute a pour on ``pour_arm`` while ``holder_arm`` holds the cup."""
    # TODO
    raise NotImplementedError
