"""Tray layout: where on the tray each object goes.

The tray is modeled as a rectangle with slanted side walls. Slot
positions live in the tray's local frame (origin at the centre of the
bottom face, +x along the long side, +y along the short side, +z up —
same convention as planning/scene/objects.py) and are composed with the
tray's task-frame pose to yield an OBJECT-CENTRE place pose.

Slot convention (top-down view of the tray, +x right, +y up):

    cup    .    .
    .      .    .          plate at top right
    .      .    plate      cup   at top left
    .      .    .          bowl  at bottom left
    bowl   .    .

Run-time integration: replace ``TRAY_DEFAULT_POSE_TASK`` with the
visually-estimated pose; the slot table and ``place_pose_on_tray`` stay
the same.
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
    x=0.295521, y=0.10, z=0.0, yaw=0.0,
)
"""Placeholder until vision provides one. Mirrors the task-frame xy used
by planning/scene/objects.py::TRAY_DEFAULT_TASK_XYZ."""


# --- Per-object slots (tray-local XY). --------------------------------------
# Initial guesses near quarter-points of the usable floor; tune at the
# lab once the tray and grasp orientations are checked end-to-end.
_DX = TRAY_FLOOR_LENGTH_X * 0.25  # ≈ 0.082 m
_DY = TRAY_FLOOR_WIDTH_Y * 0.25   # ≈ 0.046 m

TRAY_SLOT_LOCAL_XY: Dict[str, Tuple[float, float]] = {
    "plate": (+_DX, +_DY),  # top right
    "bowl":  (-_DX, -_DY),  # bottom left
    "cup":   (-_DX, +_DY),  # top left
}

# Object-centre z above the tray's base z. Default 0 = object origin
# sits at the tray's bottom-face z, matching the "object on table"
# convention used by the existing pick poses (table top z = 0). Override
# per object if the tray floor isn't paper-thin or if a specific object
# needs a small lift.
TRAY_OBJECT_REST_DZ: Dict[str, float] = {
    "plate": 0.01,
    "bowl":  0.0,
    "cup":   0.0,
}


def _yaw_rotation(yaw: float) -> Rotation:
    return Rotation.from_rotvec([0.0, 0.0, yaw])


def place_pose_on_tray(
    kind: str,
    tray: Optional[TrayPose] = None,
) -> Pose:
    """OBJECT-CENTRE place pose in the task frame for ``kind`` at its
    assigned tray slot.

    The returned pose has rotation = tray yaw about +z, so when the
    caller passes it through a grasp factory (e.g. ``plate_rim_grasp_
    edge``) the grasp angle is interpreted in the tray's local frame —
    the plate's rim angle stays "the same edge of the plate" regardless
    of how the tray is rotated.
    """
    if tray is None:
        tray = TRAY_DEFAULT_POSE_TASK
    if kind not in TRAY_SLOT_LOCAL_XY:
        raise ValueError(
            f"unknown tray slot for {kind!r}; have {sorted(TRAY_SLOT_LOCAL_XY)}"
        )

    dx_local, dy_local = TRAY_SLOT_LOCAL_XY[kind]
    dz_local = TRAY_OBJECT_REST_DZ[kind]

    c, s = math.cos(tray.yaw), math.sin(tray.yaw)
    x_task = tray.x + c * dx_local - s * dy_local
    y_task = tray.y + s * dx_local + c * dy_local
    z_task = tray.z + dz_local

    return Pose(
        translation=np.array([x_task, y_task, z_task]),
        rotation=_yaw_rotation(tray.yaw),
    )
