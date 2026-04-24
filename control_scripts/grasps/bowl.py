"""Bowl grasps (bowl.sdf / evo_bowl from src/assets/).

Same object-frame convention as the plate: origin at the bowl's resting
point on the table, +z up out of the opening, rotational symmetry about
+z.

Grasp strategy is identical in kind to the plate — top-down rim pinch —
but the geometry differs in two ways that matter:

    1. The rim is higher off the table (≈ 5 cm for evo_bowl vs. 2 cm for
       plate), so Z of the rim point is bigger.
    2. The bowl's curved sidewall means a narrow pregrasp standoff risks
       clipping the outer wall during the final approach. A larger
       pregrasp_offset (≈ 5 cm) pulls the pre-grasp point clear of the
       curvature so the straight-line descent stays off the wall.

TODO: measure the actual bowl at the lab (height, rim radius, rim
thickness) and replace these estimates. The 5 cm bowl-height figure is a
rough guess from the SDF inertia values; could easily be off by 2 cm.
"""

from __future__ import annotations

import numpy as np

from ..util.poses import Pose
from ..util.rotations import Rotation
from .base import Grasp


BOWL_RIM_RADIUS = 0.075
"""Radial distance from bowl center to rim (m). Approximate — measure."""

BOWL_RIM_Z_OFFSET = 0.055
"""Height of the bowl rim above the table (m). Approximate — measure."""

BOWL_PREGRASP_OFFSET = 0.05
"""Larger than plate's because the curved sidewall demands more clearance
at pregrasp to keep the final moveL clear of the outer wall."""

BOWL_GRASP_FORCE = 15.0
"""Similar to plate — thin rim, no need to squeeze hard."""


def _top_down_rim_rotation(angle_rad: float) -> Rotation:
    """Rotation: tool -Z into the bowl, tool X radial outward at
    ``angle_rad``, tool Y tangent. Fingers close along tool X (pinches
    radially across rim thickness).

    If on the real robot the 2F-85 turns out to close along tool Y
    instead of tool X (wrong mount orientation), the pinch will land on
    the rim tangent instead of across the sheet. Fix by replacing
    ``angle_rad + np.pi`` below with ``angle_rad + 3 * np.pi / 2``.
    """
    flip = Rotation.from_rotvec([0.0, np.pi, 0.0])
    yaw = Rotation.from_rotvec([0.0, 0.0, angle_rad + np.pi])
    return yaw * flip


def bowl_rim_grasp(
    X_task_bowl: Pose,
    angle_rad: float = 0.0,
) -> Grasp:
    rim_xy = np.array([
        BOWL_RIM_RADIUS * np.cos(angle_rad),
        BOWL_RIM_RADIUS * np.sin(angle_rad),
        BOWL_RIM_Z_OFFSET,
    ])
    X_bowl_grasp = Pose(
        translation=rim_xy,
        rotation=_top_down_rim_rotation(angle_rad),
    )
    return Grasp(
        grasp_pose=X_task_bowl @ X_bowl_grasp,
        pregrasp_offset=BOWL_PREGRASP_OFFSET,
        grasp_force=BOWL_GRASP_FORCE,
        description=f"bowl rim grasp @ {np.degrees(angle_rad):+.0f}°",
    )


def bowl_rim_candidates(
    X_task_bowl: Pose,
    n: int = 8,
) -> list[Grasp]:
    return [
        bowl_rim_grasp(X_task_bowl, angle_rad=2.0 * np.pi * i / n)
        for i in range(n)
    ]
