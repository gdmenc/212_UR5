"""Cup grasps (the receiving cup — what gets poured into).

Object-frame convention (matches plate / bowl): origin at the cup's resting
point on the table, +z up out of the opening, rotational symmetry about +z.

Cup shape (measured at the 212 lab):
    - Top (rim) radius:    4.4 cm   (rim diameter 8.8 cm)
    - Base radius:         3.35 cm  (base diameter 6.7 cm)
    - Total height:        15.4 cm
    - Rim wall thickness:  ≈ 0.2 cm
    - Profile: gentle taper — flares OUTWARD from base to rim, opposite
      direction from the bowl. The outer wall is approximately a STRAIGHT
      diagonal from (r=3.35, z=0) to (r=4.4, z=15.4), so outward inset of
      the outer wall is ≈ (4.4 − 3.35) / 15.4 ≈ 0.068 cm per cm of height.

Grasp strategy: top-down rim pinch with the 2F-85 — fingers straddle the
rim edge (one inside the cup, one outside), closing radially across the
~2 mm rim wall. A side body grasp is NOT viable here: the rim diameter
of 8.8 cm exceeds the 2F-85's ~85 mm working aperture, so a horizontal
pinch across the cup body cannot close. Top-down across the thin rim
wall is the only vertical-grasp option.
"""

from __future__ import annotations

import numpy as np

from ..util.poses import Pose
from ..util.rotations import Rotation
from .base import Grasp


CUP_RIM_OUTER_RADIUS_M = 0.044
"""Radial distance from cup center to the outer edge of the rim. Measured
at the lab — top diameter 8.8 cm."""

CUP_BASE_RADIUS_M = 0.0335
"""Radial distance from cup center to the outer edge of the base. Measured
6.7 cm diameter. Cup flares outward from base to rim."""

CUP_HEIGHT_M = 0.154
"""Total cup height (resting surface to top of rim). When the cup sits on
the table, this is also the rim's task-z."""

CUP_RIM_WALL_THICKNESS_M = 0.002
"""Rim wall thickness, ≈ 2 mm. Sets the closed-aperture target during the
pinch and informs the release aperture (open enough to clear the wall +
finger margin during withdrawal)."""

CUP_PREGRASP_OFFSET = 0.05
"""Distance (m) along tool -Z from grasp to pregrasp. 5 cm is comfortable:
at pregrasp the inner finger sits 5 cm above the rim in clear air above
the cup's interior, the outer finger 5 cm above outside the cup. Final
moveL descends both into their straddle positions."""

CUP_GRASP_FORCE = 15.0
"""Newtons. Thin rim wall — keep the squeeze light. Same as plate."""


def _top_down_rim_rotation(angle_rad: float) -> Rotation:
    """Top-down approach: tool -Z = -task_z (straight down), tool Y is
    the radial direction (the close axis — fingers close radially across
    the rim wall thickness, one inside / one outside the rim), tool X is
    the rim tangent.

    Mirrors plate's _top_down_rim_rotation: ``flip`` (R_y(π)) sends tool
    Z from +task_z to -task_z. ``yaw`` rotates about world Z to point the
    gripper toward the desired rim angle. The trailing ``tool_z_90``
    (R_z(π/2) applied LOCALLY) swaps tool X and tool Y so the close
    direction (tool Y on the lab 2F-85) is radial, not tangent — same
    correction we landed on for the bottle and the plate.
    """
    flip = Rotation.from_rotvec([0.0, np.pi, 0.0])
    yaw = Rotation.from_rotvec([0.0, 0.0, angle_rad + np.pi])
    tool_z_90 = Rotation.from_rotvec([0.0, 0.0, np.pi / 2])
    return (yaw * flip) * tool_z_90


def cup_rim_grasp(
    X_task_cup: Pose,
    angle_rad: float = 0.0,
) -> Grasp:
    """Top-down rim pinch at ``angle_rad`` (cup frame). TCP lands at the
    rim's outer edge with tool -Z straight down — fingers straddle the
    rim and close radially across the ~2 mm wall."""
    rim_xy_z = np.array([
        CUP_RIM_OUTER_RADIUS_M * np.cos(angle_rad),
        CUP_RIM_OUTER_RADIUS_M * np.sin(angle_rad),
        CUP_HEIGHT_M,
    ])
    X_cup_grasp = Pose(
        translation=rim_xy_z,
        rotation=_top_down_rim_rotation(angle_rad),
    )
    return Grasp(
        grasp_pose=X_task_cup @ X_cup_grasp,
        pregrasp_offset=CUP_PREGRASP_OFFSET,
        grasp_force=CUP_GRASP_FORCE,
        description=f"cup rim grasp @ {np.degrees(angle_rad):+.0f}°",
    )


def cup_rim_candidates(
    X_task_cup: Pose,
    n: int = 8,
) -> list[Grasp]:
    return [
        cup_rim_grasp(X_task_cup, angle_rad=2.0 * np.pi * i / n)
        for i in range(n)
    ]