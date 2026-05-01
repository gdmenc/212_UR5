"""Microwave geometry — task-frame placeholders.

All numbers below are first-pass guesses except the center XY, which
the user already measured. Re-measure depth and clearance on the rig
before trusting any motion. Coordinates are in the shared task frame
the rest of the package uses (same as ``tasks/pick_place_*.py``).

Layout assumptions
------------------
- Door opens toward task -Y. Entering the cavity means translating in
  the +Y direction; exiting means -Y.
- Cavity is a rectangular prism with horizontal floor and ceiling.
- The glass tray is the floor of the *interior* — objects rest at
  ``MICROWAVE_FLOOR_Z``.

Constraints derived from these numbers
--------------------------------------
- Useful interior height: ``MICROWAVE_CEILING_Z - MICROWAVE_FLOOR_Z``
  = 15 cm (with the placeholders below).
- For a hook-on-rim grasp of a 7.2 cm bowl on the tray (rim at task z
  0.152 m), the wrist sits at the rim z (no vertical wrist offset, see
  ``calibration.TCP_OFFSET_HOOK``). At entry_z 0.18 m the wrist is 5 cm
  below the ceiling — comfortable.
- For a 2F-85 vertical rim grasp of a plate on the tray (TCP near task z
  0.10 m), the wrist sits ~18.4 cm above the TCP = 0.28 m. That is
  ABOVE the 0.23 m ceiling — vertical-descent plate-into-microwave does
  not fit with the current 2F-85 mount. See
  ``tasks/pick_place_plate_microwave.py`` for the warning.
"""

from __future__ import annotations

import numpy as np


MICROWAVE_CENTER_XY_TASK = np.array([-0.225458, 0.508696])
"""Task-frame XY of the interior center (the canonical place location).
Measured on the rig."""

MICROWAVE_INTERIOR_DEPTH = 0.26
"""Interior depth along the door axis (task Y). The door plane sits at
``MICROWAVE_CENTER_XY_TASK[1] - MICROWAVE_INTERIOR_DEPTH/2``.
PLACEHOLDER — typical microwaves are 25-30 cm."""

MICROWAVE_FLOOR_Z = 0.08
"""Top surface of the rotating glass tray, in task z. From user spec:
microwave 3 cm off the ground + 5 cm to glass tray top = 8 cm."""

MICROWAVE_CEILING_Z = 0.23
"""Interior ceiling, in task z. From user spec: microwave 3 cm off the
ground + 20 cm interior height = 23 cm."""

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
