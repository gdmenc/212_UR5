"""Tray layout: where on the tray each object goes.

The tray is modeled as a rectangle with slanted side walls. Slot
positions live in the tray's local frame (origin at the centre of the
bottom face, +x along the long side, +y along the short side, +z up —
same convention as planning/scene/objects.py) and are composed with the
tray's task-frame pose to yield an OBJECT-CENTRE place pose.

Slot convention (top-down view of the tray, +x right, +y up):

    cup    .    .
    .      .    .          plate at right (y-centred — plate diameter
    .      .    plate      ~25 cm vs tray floor width ~18.6 cm leaves
    .      .    .          no room for a +y/-y offset)
    bowl   .    .          cup   at top left
                           bowl  at bottom left

Run-time integration: ``TRAY_DEFAULT_POSE_TASK`` is the lab-measured
fallback. Replace at call time with a visually-estimated pose by
passing it as ``place_pose_on_tray(..., tray=perceived_pose)``; the
slot table stays the same.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict, Optional, Tuple

import numpy as np

from .poses import Pose
from .rotations import Rotation


# --- Tray geometry (lab-measured 2026-05-04). -------------------------------
TRAY_LENGTH_X = 0.420
"""Outer length, along the tray's local +x (long axis)."""

TRAY_WIDTH_Y = 0.278
"""Outer width, along the tray's local +y (short axis)."""

TRAY_TOTAL_HEIGHT = 0.030
"""Vertical distance from table top to top rim of tray walls."""

TRAY_WALL_SLANT = 0.055
"""Slope length of the slanted side walls (lab-measured)."""

# Horizontal projection of the slanted wall — how much narrower the
# usable floor is than the outer rim, on each side.
_TRAY_WALL_HORIZ = math.sqrt(
    max(TRAY_WALL_SLANT ** 2 - TRAY_TOTAL_HEIGHT ** 2, 0.0)
)  # ≈ 0.0461 m

TRAY_FLOOR_LENGTH_X = TRAY_LENGTH_X - 2 * _TRAY_WALL_HORIZ  # ≈ 0.328 m
TRAY_FLOOR_WIDTH_Y = TRAY_WIDTH_Y - 2 * _TRAY_WALL_HORIZ    # ≈ 0.186 m


# --- Tray pose (default; replace at run-time with the perceived pose). ------
@dataclass
class TrayPose:
    """Tray pose in the task frame.

    ``z`` is the tray's bottom-face task z (= table top if the tray is
    on the table). ``yaw`` is rotation about task +z; yaw=0 means tray
    local +x aligns with task +x.
    """

    x: float
    y: float
    z: float = 0.0
    yaw: float = 0.0


TRAY_DEFAULT_POSE_TASK = TrayPose(
    x=0.22, y=0.165, z=0.0, yaw=0.0,
)
"""Lab-measured tray pose (task xy = +22 cm / +16.5 cm from task origin,
yaw=0, on table top). **Single source of truth** — both
``planning/scene/objects.py::TRAY_DEFAULT_TASK_XYZ`` and the tray pose
used in microwave tasks are derived from this. Replace at run-time with
a perceived pose if vision is available."""


# --- Per-object slots (tray-local XY). --------------------------------------
# Slot offsets are chosen so the planner-side cylinders (with the +5 mm
# pad applied in planning/scene/objects.py) just barely don't overlap.
# Earlier 25%-of-floor offsets put plate/bowl/cup well into each other.
#
#   BOWL_RADIUS = 0.079 m  (= 0.037*2 + 0.005 pad — see objects.py:64)
#   CUP_RADIUS  = 0.049 m  (= 0.044   + 0.005 pad)
#   PLATE_RADIUS= 0.130 m  (= 0.125   + 0.005 pad — wider than the tray
#                            floor in y, so it can never be off-axis)
#
# Bowl and cup share the -x half. Putting them at y = ±_BOWL_CUP_DY
# with _BOWL_CUP_DY = (BOWL_RADIUS + CUP_RADIUS) / 2 = 0.064 m makes
# their padded cylinders just touch — minimal overlap given that both
# objects must fit between the tray's slanted walls.
#
# Plate sits y-centred (geometrically forced). Pushing it slightly past
# the previous 0.082 m to 0.090 m widens the plate-vs-{bowl,cup}
# separation enough that plate↔cup goes from -9 mm overlap to +5 mm
# clear. Plate↔bowl still overlaps (~25 mm) — physically unavoidable
# because the plate diameter exceeds the tray floor width and the bowl
# is 79 mm radius itself.
_PLATE_DX = 0.090
_BOWL_CUP_DX = 0.092
_BOWL_CUP_DY = 0.064

TRAY_SLOT_LOCAL_XY: Dict[str, Tuple[float, float]] = {
    "plate": (+_PLATE_DX, 0.0),                # right, y-centred
    "bowl":  (-_BOWL_CUP_DX, -_BOWL_CUP_DY),   # bottom left
    "cup":   (-_BOWL_CUP_DX, +_BOWL_CUP_DY),   # top left
}

# Object-centre z above the tray's base z. Default 0 = object origin
# sits at the tray's bottom-face z, matching the "object on table"
# convention used by the existing pick poses (table top z = 0). Override
# per object if the tray floor isn't paper-thin or if a specific object
# needs a small lift.
TRAY_OBJECT_REST_DZ: Dict[str, float] = {
    "plate": 0.05,   # 5 cm lift so the 2F-85 fingers clear the tray rim
    "bowl":  0.05,   # 5 cm lift so the hook clears the tray rim
    "cup":   0.0,
}


def _yaw_rotation(yaw: float) -> Rotation:
    return Rotation.from_rotvec([0.0, 0.0, yaw])


def place_pose_on_tray(
    kind: str,
    tray: Optional[TrayPose] = None,
    *,
    slot_local_xy: Optional[Tuple[float, float]] = None,
    rest_dz: Optional[float] = None,
) -> Pose:
    """OBJECT-CENTRE place pose in the task frame for ``kind`` at its
    assigned tray slot.

    The returned pose has rotation = tray yaw about +z, so when the
    caller passes it through a grasp factory (e.g. ``plate_rim_grasp_
    edge``) the grasp angle is interpreted in the tray's local frame —
    the plate's rim angle stays "the same edge of the plate" regardless
    of how the tray is rotated.

    ``slot_local_xy`` and ``rest_dz`` override the per-kind defaults from
    ``TRAY_SLOT_LOCAL_XY`` / ``TRAY_OBJECT_REST_DZ`` for one-off layouts
    (e.g. centring the plate on the tray, or lifting it for gripper
    clearance) without mutating the module-level dicts.
    """
    if tray is None:
        tray = TRAY_DEFAULT_POSE_TASK

    if slot_local_xy is None:
        if kind not in TRAY_SLOT_LOCAL_XY:
            raise ValueError(
                f"unknown tray slot for {kind!r}; have {sorted(TRAY_SLOT_LOCAL_XY)}"
            )
        dx_local, dy_local = TRAY_SLOT_LOCAL_XY[kind]
    else:
        dx_local, dy_local = slot_local_xy

    if rest_dz is None:
        if kind not in TRAY_OBJECT_REST_DZ:
            raise ValueError(
                f"unknown tray rest_dz for {kind!r}; have {sorted(TRAY_OBJECT_REST_DZ)}"
            )
        dz_local = TRAY_OBJECT_REST_DZ[kind]
    else:
        dz_local = rest_dz

    c, s = math.cos(tray.yaw), math.sin(tray.yaw)
    x_task = tray.x + c * dx_local - s * dy_local
    y_task = tray.y + s * dx_local + c * dy_local
    z_task = tray.z + dz_local

    return Pose(
        translation=np.array([x_task, y_task, z_task]),
        rotation=_yaw_rotation(tray.yaw),
    )
