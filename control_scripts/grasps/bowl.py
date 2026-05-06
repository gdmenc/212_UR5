"""Bowl grasps.

Same object-frame convention as the plate: origin at the bowl's resting
point on the table, +z up out of the opening, rotational symmetry about
+z.

Bowl shape (measured at the 212 lab):
    - Outer rim diameter:  7.4 cm  (radius 3.7 cm)
    - Base diameter:       4.25 cm (radius 2.125 cm)
    - Total height:        7.2 cm
    - Rim wall thickness:  ≈ 0.27 cm (rough — easily ±0.05 cm)
    - Profile: significantly concave / tapered. The sidewall sweeps inward
      from the rim down to a much smaller base, so the rim is the widest
      point. Approximate inward inset of the outer wall ≈ 0.22 cm per cm
      of depth (linear approximation; the actual curve is smooth).

Two grasp families on this object:
    1. Top-down rim pinch with the 2F-85 — implemented below.
    2. Vertical rim hook with the welded hook gripper — implemented below.
       This mirrors ``bottle_hook_grasp``: the hook descends vertically at
       a chosen rim angle, with the hook throat straddling the rim wall.
       Throat width (3.6 cm, see ``grippers/hook_gripper.py``) easily
       accommodates the 0.27 cm rim wall, so the rim slides into the throat
       with generous lateral margin.

Pregrasp standoff: the concave sidewall means a narrow vertical pregrasp
risks clipping the outer wall during descent. ``BOWL_PREGRASP_OFFSET``
of 5 cm keeps the final moveL clear of the curvature.
"""

from __future__ import annotations

import numpy as np

from ..util.poses import Pose
from ..util.rotations import Rotation
from ._hook_rim import HOOK_RIM_PREGRASP_OFFSET, hook_rim_rotation
from .base import Grasp


BOWL_RIM_OUTER_RADIUS_M = 0.037
"""Radial distance from bowl center to the OUTER edge of the rim. Measured
at the lab — outer rim diameter 7.4 cm."""

BOWL_BASE_RADIUS_M = 0.02125
"""Radial distance from bowl center to the bowl's base (the part that rests
on the table). Measured 4.25 cm diameter. The bowl tapers from
BOWL_RIM_OUTER_RADIUS_M at the top down to this at the bottom."""

BOWL_RIM_Z_OFFSET_M = 0.072
"""Height of the bowl rim above the bowl's resting surface (= total bowl
height). When the bowl sits on the table, this is also the rim's task-z."""

BOWL_RIM_WALL_THICKNESS_M = 0.0027
"""Rim wall thickness, ROUGH estimate (≈ 2.7 mm, easily ±0.5 mm). Sets the
required throat width for any hook grasp; the hook's 3.6 cm throat
(see ``grippers/hook_gripper.HOOK_THROAT_WIDTH_M``) gives ~13× margin so
the precise value isn't load-bearing."""

BOWL_PREGRASP_OFFSET = 0.05
"""Larger than plate's because the curved sidewall demands more clearance
at pregrasp to keep the final moveL clear of the outer wall. With a
~0.22 cm/cm inward sidewall inset, a 5 cm vertical pregrasp standoff sees
the wall ≈ 1.1 cm inset from the rim — comfortable clearance for 2F-85
fingers."""

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
        BOWL_RIM_OUTER_RADIUS_M * np.cos(angle_rad),
        BOWL_RIM_OUTER_RADIUS_M * np.sin(angle_rad),
        BOWL_RIM_Z_OFFSET_M,
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


def bowl_hook_grasp(
    X_task_bowl: Pose,
    angle_rad: float = 0.0,
    approach_tilt_rad: float = 0.0,
    *,
    rim_outer_radius_m: float | None = None,
    rim_z_offset_m: float | None = None,
) -> Grasp:
    """Hook-gripper rim grasp at the bowl's outer rim.

    The hook descends at the selected rim angle. The moving finger sits
    just inside the bowl rim, the fixed jaw stays outside, and the rim
    wall is captured in the hook throat. The bowl sidewall slant is NOT
    compensated — uses the outer rim radius and horizontal rim height
    directly. A future tilted-rim variant should compensate.

    ``approach_tilt_rad`` (default 0) tips the descent off pure-vertical
    around tool +Y. Positive values "dive" the gripper into the bowl
    from above-and-outward, lifting the wrist away from the table — see
    ``hook_rim_rotation`` for the full sign convention.

    ``rim_outer_radius_m`` / ``rim_z_offset_m`` override the lab defaults
    (``BOWL_RIM_OUTER_RADIUS_M``, ``BOWL_RIM_Z_OFFSET_M``) when the physical
    bowl or bench calibration differs.
    """
    r = float(rim_outer_radius_m) if rim_outer_radius_m is not None else BOWL_RIM_OUTER_RADIUS_M
    z_rim = float(rim_z_offset_m) if rim_z_offset_m is not None else BOWL_RIM_Z_OFFSET_M
    rim_xyz = np.array([
        r * np.cos(angle_rad),
        r * np.sin(angle_rad),
        z_rim,
    ])
    X_bowl_grasp = Pose(
        translation=rim_xyz,
        rotation=hook_rim_rotation(angle_rad, approach_tilt_rad),
    )
    tilt_label = (
        ""
        if approach_tilt_rad == 0.0
        else f", tilt {np.degrees(approach_tilt_rad):+.0f}°"
    )
    return Grasp(
        grasp_pose=X_task_bowl @ X_bowl_grasp,
        pregrasp_offset=HOOK_RIM_PREGRASP_OFFSET,
        grasp_force=BOWL_GRASP_FORCE,
        description=(
            f"bowl hook rim grasp @ {np.degrees(angle_rad):+.0f}°{tilt_label}"
        ),
    )


def bowl_hook_candidates(
    X_task_bowl: Pose,
    n: int = 8,
) -> list[Grasp]:
    return [
        bowl_hook_grasp(X_task_bowl, angle_rad=2.0 * np.pi * i / n)
        for i in range(n)
    ]


def bowl_rim_candidates(
    X_task_bowl: Pose,
    n: int = 8,
) -> list[Grasp]:
    return [
        bowl_rim_grasp(X_task_bowl, angle_rad=2.0 * np.pi * i / n)
        for i in range(n)
    ]