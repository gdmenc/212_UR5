"""Plate grasps (plate_8in from src/assets/dishes/).

Physical measurements (supplied, not guessed):

    outer rim radius    = 0.125 m    (12.5 cm center-to-outer-edge)
    flat bottom radius  = 0.0835 m   ( 8.35 cm center-to-flat-edge)
    rim height          = 0.02 m     (2 cm bottom-to-rim-top)
    plate thickness     = 0.0025 m   (0.25 cm sheet thickness)

The plate is a bent 2.5 mm sheet: a flat disc of radius 8.35 cm forms the
food-contact center, and the outer 4.15 cm (from r=8.35 to r=12.5) rises
to form the raised rim at slope angle ``atan2(0.02, 0.0415) ≈ 25.7°``
from horizontal.

Object frame (from plate_8in.sdf): origin on the plate's symmetry axis
at the point that touches the table when the plate is sitting flat, +z
out of the food side.

Two grasp functions here:

    plate_rim_grasp       — simple top-down pinch at the OUTER EDGE
                             (r=0.125, z=0.02). Gripper straight down,
                             pinches across the 2.5 mm cut edge. Works
                             but relies on catching a thin edge.
    plate_rim_grasp_edge  — tilts the gripper around the rim tangent by
                             the slope angle so the pinch axis ends up
                             perpendicular to the sheet surface. Caller
                             picks the grasp radius (anywhere on the
                             slope from flat edge to outer edge).
                             Higher-contact, more robust grasp.

Rotational symmetry: any ``angle_rad`` around the plate's +z is valid;
the caller picks it to minimise wrist twist or avoid obstacles.
"""

from __future__ import annotations

import numpy as np

from ..util.poses import Pose
from ..util.rotations import Rotation
from .base import Grasp


# --- Physical geometry (measured) -----------------------------------------
PLATE_OUTER_RADIUS = 0.125
"""Center to outer edge of the raised rim (m)."""

PLATE_FLAT_RADIUS = 0.0835
"""Center to the edge of the flat bottom (m) — where the sloped rim begins."""

PLATE_RIM_HEIGHT = 0.02
"""Height of the rim's top edge above the plate bottom (m)."""

PLATE_THICKNESS = 0.0025
"""Sheet thickness (m) — the rim is just this sheet bent upward."""

PLATE_RIM_SLOPE_RAD = float(
    np.arctan2(PLATE_RIM_HEIGHT*5/6, PLATE_OUTER_RADIUS - PLATE_FLAT_RADIUS)
)
"""Slope angle of the rim sheet, measured from horizontal. Derived from
the three measurements above — ≈ 25.7° for the current plate."""


# --- Grasp parameters ------------------------------------------------------
PLATE_PREGRASP_OFFSET = 0.05
"""Distance (m) along gripper tool -Z from grasp to pregrasp. Made a bit
larger than the bare 3 cm of the simple grasp because the tilted
approach has the pregrasp point offset inward-upward into the bowl
interior — more room improves clearance during the final approach."""

PLATE_GRASP_FORCE = 15.0
"""Newtons. Thin sheet — keep it light."""


# =========================================================================
#  Simple top-down outer-edge grasp (no tilt)
# =========================================================================

def _top_down_rim_rotation(angle_rad: float) -> Rotation:
    """Straight-down approach: tool -Z = -task_z, tool X radial outward,
    tool Y tangent. If the installed 2F-85 closes along tool Y instead
    of tool X, replace ``angle_rad + np.pi`` with
    ``angle_rad + 3 * np.pi / 2`` below."""
    flip = Rotation.from_rotvec([0.0, np.pi, 0.0])
    yaw = Rotation.from_rotvec([0.0, 0.0, angle_rad + np.pi])
    tool_z_90 = Rotation.from_rotvec([0.0, 0.0, np.pi / 2])
    return (yaw * flip) * tool_z_90


def plate_rim_grasp(
    X_task_plate: Pose,
    angle_rad: float = 0.0,
) -> Grasp:
    """Top-down rim-edge pinch at ``angle_rad``. TCP lands on the rim's
    outer edge (r=0.125, z=0.02) with tool -Z straight down."""
    rim_xy_z = np.array([
        PLATE_OUTER_RADIUS * np.cos(angle_rad),
        PLATE_OUTER_RADIUS * np.sin(angle_rad),
        PLATE_RIM_HEIGHT,
    ])
    X_plate_grasp = Pose(
        translation=rim_xy_z,
        rotation=_top_down_rim_rotation(angle_rad),
    )
    return Grasp(
        grasp_pose=X_task_plate @ X_plate_grasp,
        pregrasp_offset=0.03,
        grasp_force=PLATE_GRASP_FORCE,
        description=f"plate rim edge grasp (top-down) @ {np.degrees(angle_rad):+.0f}°",
    )


def plate_rim_candidates(
    X_task_plate: Pose,
    n: int = 8,
) -> list[Grasp]:
    return [
        plate_rim_grasp(X_task_plate, angle_rad=2.0 * np.pi * i / n)
        for i in range(n)
    ]


# =========================================================================
#  Tilted rim grasp — pinch perpendicular to the sloped sheet
# =========================================================================

def _tilted_rim_rotation(angle_rad: float, slope_rad: float) -> Rotation:
    """Rim rotation at ``angle_rad`` with the tool tilted so the
    gripper's approach direction (tool +Z, per the package convention
    'approach = +tool_Z') is **along the slope going inward-downward**,
    and the gripper body sits outward-and-upward from the grasp point.

    After this rotation (at angle_rad=0, slope α > 0):
        tool X = sheet outward-downward normal (pointing from the sheet
                 surface toward the gap under the plate). This is the
                 PINCH axis — fingers close across the 2.5 mm sheet
                 thickness, one on the top surface (bowl interior), one
                 on the bottom surface (below-plate gap).
        tool Y = rim tangent, CW — unchanged by the tilt.
        tool Z = along-slope INWARD-DOWNWARD ( (-cos α, 0, -sin α) at
                 angle 0 ). So the gripper body is at +tool -Z direction
                 from grasp = outward-and-upward.
        tool -Z = outward-upward. Pregrasp = ``offset_along_tool_z(grasp,
                  d)`` lands at grasp + d * (-tool_Z) = outward-upward
                  of the grasp, which is the correct staging position.

    Tilt magnitude: rotation around local tool Y by ``slope_rad - π/2``
    (i.e., -64.3° for our 25.7° slope). Derivation: starting from the
    top-down rim rotation (tool Z = -task_z straight down), we need
    tool Z to end up at (-cos α, 0, -sin α); solving
    R_y(φ) @ (0, 0, -1) → this target gives φ = slope_rad - π/2 when
    combined with the (yaw * flip) outer rotation's z-flip.
    """
    flip = Rotation.from_rotvec([0.0, np.pi, 0.0])
    yaw = Rotation.from_rotvec([0.0, 0.0, angle_rad + np.pi])
    tilt_local_y = Rotation.from_rotvec([0.0, slope_rad - np.pi / 2, 0.0])
    tool_z_90 = Rotation.from_rotvec([0.0, 0.0, np.pi / 2])
    return ((yaw * flip) * tilt_local_y) * tool_z_90


def plate_rim_grasp_edge(
    X_task_plate: Pose,
    angle_rad: float = 0.0,
    grasp_radius: float = PLATE_OUTER_RADIUS,
) -> Grasp:
    """Tilted rim pinch at ``angle_rad``, at ``grasp_radius`` along the
    slope. The gripper approaches along the slope direction so fingers
    pinch perpendicular to the sheet (across its 2.5 mm thickness).

    ``grasp_radius`` — where on the rim to pinch:
        PLATE_OUTER_RADIUS (default): at the outer edge
        PLATE_FLAT_RADIUS: at the base of the slope (right where the
                           flat bottom begins to rise)
        anything in between: mid-slope, most secure contact

    Raises ``ValueError`` if ``grasp_radius`` is outside
    [PLATE_FLAT_RADIUS, PLATE_OUTER_RADIUS].
    """
    if not (PLATE_FLAT_RADIUS <= grasp_radius <= PLATE_OUTER_RADIUS):
        raise ValueError(
            f"grasp_radius={grasp_radius} must be in "
            f"[{PLATE_FLAT_RADIUS}, {PLATE_OUTER_RADIUS}] m "
            f"(on the sloped rim). Outside the slope there is no rim "
            f"to pinch — flat bottom inside, empty air outside."
        )

    # Interpolate z linearly along the slope (slope is a straight line
    # from (flat_radius, 0) to (outer_radius, rim_height) in r-z).
    t = (grasp_radius - PLATE_FLAT_RADIUS) / (
        PLATE_OUTER_RADIUS - PLATE_FLAT_RADIUS
    )
    grasp_z = t * PLATE_RIM_HEIGHT

    grasp_position = np.array([
        grasp_radius * np.cos(angle_rad),
        grasp_radius * np.sin(angle_rad),
        grasp_z,
    ])
    X_plate_grasp = Pose(
        translation=grasp_position,
        rotation=_tilted_rim_rotation(angle_rad, PLATE_RIM_SLOPE_RAD),
    )
    return Grasp(
        grasp_pose=X_task_plate @ X_plate_grasp,
        pregrasp_offset=PLATE_PREGRASP_OFFSET,
        grasp_force=PLATE_GRASP_FORCE,
        description=(
            f"plate tilted rim grasp @ {np.degrees(angle_rad):+.0f}°, "
            f"r={grasp_radius*100:.1f} cm, slope {np.degrees(PLATE_RIM_SLOPE_RAD):.1f}°"
        ),
    )
