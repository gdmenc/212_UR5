"""Microwave geometry — task-frame, single source of truth.

Coordinates are in the shared task frame the rest of the package uses
(origin at top-centre of the heavy-duty table clear area; +y toward the
microwave; +z up).

Layout assumptions
------------------
- Door opens toward task -Y. Entering the cavity means translating in
  the +Y direction; exiting means -Y.
- Cavity is a rectangular prism with horizontal floor and ceiling.
- The glass tray is the floor of the *interior* — objects rest at
  ``MICROWAVE_FLOOR_Z``.
- The microwave sits on a "white table" 3 cm below the main table top
  (white table top at z = -0.030). The microwave outer body is 7 cm
  above the white table → outer bottom at z = +0.040. Sim welds the
  body in place; the white table itself is not modelled.
- The 13 cm-thick right wall houses the magnetron + control panel; the
  left wall is only 3 cm. So the cavity centre is **offset** ~5 cm to
  the left of the outer body centre. ``MICROWAVE_CENTER_XY_TASK`` is
  the **cavity** centre (where objects go), not the outer-body centre.
- The front face is split: the door spans 36 cm on the left (hinged at
  the outer left edge); the right 8 cm is a fixed control-panel face
  carrying the red on/off button.

Imports
-------
``planning/scene/microwave.py`` builds the sim collision geometry from
these constants — keep this module the single source of truth and the
sim will track edits automatically.
"""

from __future__ import annotations

import numpy as np


# --- Cavity (interior) geometry — drives object-placement code -----------

MICROWAVE_CENTER_XY_TASK = np.array([-0.210, 0.5025])
"""Task-frame XY of the interior cavity centre (the canonical place
location). Derived from the rig measurement: cavity opening spans
x ∈ [-0.350, -0.070] (28 cm wide, with 3 cm left wall and 13 cm right
wall inside the 44 cm outer body); door plane at y = +0.375; cavity
depth 25.5 cm → centre y = +0.5025."""

MICROWAVE_INTERIOR_DEPTH = 0.255
"""Interior depth along the door axis (task Y). The door plane sits at
``MICROWAVE_CENTER_XY_TASK[1] - MICROWAVE_INTERIOR_DEPTH/2``."""

MICROWAVE_INTERIOR_WIDTH_X = 0.28
"""Interior width along task X (cavity wall to cavity wall). Outer body
is 44 cm; left wall is 3 cm; right wall is 13 cm; difference = 28 cm."""

MICROWAVE_FLOOR_Z = 0.07
"""Top surface of the rotating glass tray, in task z. White table top at
z = -0.030, objects rest 10 cm above it → +0.070."""

MICROWAVE_CEILING_Z = 0.23
"""Interior ceiling, in task z. White table top at z = -0.030, interior
height 26 cm above it → +0.230. Useful cavity height
(``CEILING_Z - FLOOR_Z``) is 16 cm."""


# --- Outer body geometry — drives sim collision walls ---------------------

MICROWAVE_OUTER_W_X = 0.44
"""Outer body width along task X (44 cm)."""

MICROWAVE_OUTER_D_Y = 0.285
"""Outer body depth along task Y (28.5 cm)."""

MICROWAVE_OUTER_H_Z = 0.24
"""Outer body height along task Z (24 cm)."""

MICROWAVE_OUTER_BOTTOM_Z = 0.04
"""Z of the outer body's bottom face, derived from white table top
z = -0.030 plus the 7 cm stand-off. Outer top face is at
``MICROWAVE_OUTER_BOTTOM_Z + MICROWAVE_OUTER_H_Z`` = +0.280."""

WHITE_TABLE_TOP_Z = -0.030
"""Z of the supporting white-table top surface that the microwave sits
on. 3 cm below the main table top (which is at task z = 0). Used by
button-press geometry (``tasks/press_button.py``), which measures from
this surface, not from the main table."""

MICROWAVE_LEFT_WALL_X = 0.03
"""Thickness of the left side wall (cavity to outer left)."""

MICROWAVE_RIGHT_WALL_X = 0.13
"""Thickness of the right side wall (cavity to outer right). Houses the
magnetron + control electronics; the front face of this wall is the
control-panel face carrying ``BUTTON_TASK_XYZ``."""

MICROWAVE_BOTTOM_WALL_Z = 0.03
"""Thickness of the bottom (outer bottom to glass tray top)."""

MICROWAVE_TOP_WALL_Z = 0.05
"""Thickness of the top (cavity ceiling to outer top)."""

MICROWAVE_BACK_WALL_Y = 0.030
"""Thickness of the back wall along task Y (cavity back to outer back)."""

MICROWAVE_DOOR_WIDTH_X = 0.36
"""Width of the swinging door along task X. Door spans
x ∈ [-0.380, -0.020]; the right 8 cm of the front face is a fixed
control-panel face that does not move with the door."""

MICROWAVE_DOOR_THICKNESS_Y = 0.030
"""Door thickness along task Y when closed (measured: 3 cm)."""

MICROWAVE_HINGE_X = -0.360
"""Task-frame X of the vertical hinge axis (outer left face of the
microwave). The door swings in -y from this axis."""

MICROWAVE_ENTRY_CLEARANCE = 0.05
"""Default distance in -Y outside the door plane where the gripper
descends to ``entry_z`` before translating into the cavity. Pass a
different value to ``entry_xy_for`` to override per-object (e.g. a
larger clearance for the hook so the wrist doesn't crowd the door
frame)."""


def door_plane_y() -> float:
    """Task-frame Y of the door plane (front face of the cavity)."""
    return float(MICROWAVE_CENTER_XY_TASK[1] - MICROWAVE_INTERIOR_DEPTH / 2.0)


def entry_xy_for(
    target_xy_task,
    clearance: float = MICROWAVE_ENTRY_CLEARANCE,
) -> np.ndarray:
    """Task-frame XY where the gripper descends from transit_z to
    entry_z BEFORE translating laterally into the cavity. Same X as
    the target; Y is just outside the door plane (in -Y by
    ``clearance``)."""
    target_xy = np.asarray(target_xy_task, dtype=float).reshape(2)
    y = door_plane_y() - clearance
    return np.array([target_xy[0], y])


def entry_xy_for_motion_direction(
    target_xy_task,
    motion_angle_rad: float,
    clearance: float = MICROWAVE_ENTRY_CLEARANCE,
) -> np.ndarray:
    """Entry XY for a specified physical motion direction (independent
    of tool orientation).

    The lateral entry through the door is a straight-line move from
    ``entry_xy`` to ``target_xy``; ``motion_angle_rad`` is the heading
    of that move in the task xy plane (CCW from +x). Entry sits at the
    door clearance line (``door_plane_y() − clearance``) on the line
    through ``target_xy`` heading at ``motion_angle_rad``.

    Use this when you want to control where the arm comes FROM during
    cavity entry, while keeping the tool orientation set independently
    by your grasp / place pose. Sister of ``entry_xy_for_pose`` (which
    derives the heading from the tool axis).

    Falls back to ``entry_xy_for`` (perpendicular to the door) when the
    requested heading has no +y component (would never cross the door
    plane heading toward +y) or numerically degenerate.
    """
    target_xy = np.asarray(target_xy_task, dtype=float).reshape(2)
    entry_y = door_plane_y() - clearance
    motion_dir = np.array([np.cos(motion_angle_rad), np.sin(motion_angle_rad)])
    if abs(motion_dir[1]) < 1e-6:
        # Pure-x motion never crosses the door plane in y.
        return entry_xy_for(target_xy, clearance)
    s = (target_xy[1] - entry_y) / motion_dir[1]
    if s <= 0.0:
        # Motion direction points AWAY from the door (toward +y from
        # outside, or back into +y from inside) — no valid entry on
        # this line. Fall back to perpendicular.
        return entry_xy_for(target_xy, clearance)
    return target_xy - s * motion_dir


def entry_xy_for_pose(
    target_pose_task,
    clearance: float = MICROWAVE_ENTRY_CLEARANCE,
) -> np.ndarray:
    """Entry XY aligned with the target pose's tool-axis approach.

    The entry point still lies just outside the door plane, but its X is
    chosen by tracing the target pose's local -Z direction backward to that
    outside-door Y. This makes the in-cavity segment approach along the same
    XY direction as the final tool-axis pregrasp/grasp motion instead of
    always moving straight along task Y.

    Falls back to ``entry_xy_for`` when the tool-axis projection is nearly
    parallel to the door plane or points away from the door.
    """
    target_xy = target_pose_task.translation[:2]
    entry_y = door_plane_y() - clearance

    tool_minus_z_task = target_pose_task.rotation.apply([0.0, 0.0, -1.0])
    approach_xy = np.asarray(tool_minus_z_task[:2], dtype=float)
    norm = float(np.linalg.norm(approach_xy))
    if norm < 1e-6:
        return entry_xy_for(target_xy, clearance)

    approach_xy = approach_xy / norm
    dy = entry_y - target_xy[1]
    if abs(approach_xy[1]) < 1e-6:
        return entry_xy_for(target_xy, clearance)

    scale = dy / approach_xy[1]
    if scale <= 0.0:
        return entry_xy_for(target_xy, clearance)

    return target_xy + scale * approach_xy
