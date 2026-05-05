"""Shared helpers for hook-on-rim grasps.

Any object with rotational symmetry around its vertical axis (bowl,
bottle, cup, ...) can be grasped by the hook the same canonical way:
vertical descent with the throat opening facing down, rim wall clamped
between the finger (radially inside the rim) and the fixed jaw
(radially outside). The TCP rotation pattern and the pregrasp standoff
are identical across those objects — only the rim radius and rim height
differ. So both live here; the per-object grasp factories
(``bowl_hook_grasp``, ``bottle_hook_grasp``, ...) plug in their own
radii / heights.
"""

from __future__ import annotations

import numpy as np

from ..util.rotations import Rotation


HOOK_RIM_PREGRASP_OFFSET = 0.05
"""Vertical pregrasp standoff (m) for any hook rim grasp. The hook's
throat opening sits 1.6 cm below the TCP in task z (per
``HOOK_FINGER_TIP_LATERAL_M``); 5 cm leaves ~3.4 cm of clear vertical
space between the finger tip and the rim plane during pregrasp."""


def hook_rim_rotation(
    angle_rad: float,
    approach_tilt_rad: float = 0.0,
) -> Rotation:
    """TCP rotation (object frame) for a hook rim grasp at ``angle_rad``
    around the object's +z axis. Combined with
    ``TCP_OFFSET_HOOK = R_y(+π/2)`` this produces a TCP frame at the
    grasp pose with (for ``approach_tilt_rad = 0``):
        - Tool +Z = task -Z (vertical descent direction)
        - Tool +X = radial OUTWARD at angle_rad (away from object center)
        - Tool +Y = horizontal tangent

    The zero-tilt baseline is verified against a recorded grasp pose at
    angle π — see ``calibration.TCP_OFFSET_HOOK``.

    ``approach_tilt_rad`` rotates the grasp around tool +Y (the
    horizontal-tangent axis), preserving the rim engagement geometry but
    tipping the gripper out of pure-vertical descent. Sign convention:

        > 0 :  gripper "dives" into the object — front (throat) tips
               toward task -Z, back (flange/wrist) lifts in +Z. Tool +Z's
               horizontal component points AWAY from the object center.
               Useful when the wrist would otherwise dip too close to
               the table at low rim heights (small bowls, etc.).
        < 0 :  the inverse — throat angles toward the object interior,
               wrist drops lower. Rare; only useful when wrist clearance
               isn't the constraint.

    The pregrasp computation in pick.py (``offset_along_tool_z``)
    automatically follows the tilt: pregrasp sits ``d`` meters back along
    the (now non-vertical) tool -Z, so the descent stays a single moveL.
    """
    yaw = Rotation.from_rotvec([0.0, 0.0, angle_rad])
    flip = Rotation.from_rotvec([np.pi, 0.0, 0.0])
    tilt = Rotation.from_rotvec([0.0, approach_tilt_rad, 0.0])
    return yaw * flip * tilt