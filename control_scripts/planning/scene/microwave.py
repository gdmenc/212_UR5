"""Microwave as primitive boxes — driven by ``control_scripts/microwave.py``.

The existing ``src/assets/microwave/microwave.sdf`` loads a single OBJ
mesh which Drake convex-hulls for collision. That convex hull fills
the cavity (where the door swings into) and the front (where the hook
needs to engage the handle), making collision queries useless.

This module models the microwave as primitive welded boxes:

  - Bottom wall      (full outer footprint, 3 cm thick)
  - Top wall         (full outer footprint, 5 cm thick)
  - Back wall        (full outer footprint, 3 cm thick)
  - Left side wall   (3 cm thick)            ← thin
  - Right side wall  (13 cm thick)           ← thick (magnetron + electronics)
  - Door             (36 cm wide, full body height, ``DOOR_THICKNESS`` along y)

The right wall's front face IS the control-panel face — the red on/off
button at ``BUTTON_TASK_XYZ`` sits on it. Door covers the cavity
opening on the left portion of the front face only; the right 8 cm of
the front face is the fixed control panel.

Geometry constants live in ``control_scripts/microwave.py`` (cavity
centre, interior dims, asymmetric wall thicknesses). Edit there and
this module follows automatically — no duplicate sources of truth.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from pydrake.geometry import Box
from pydrake.math import RigidTransform, RotationMatrix
from pydrake.multibody.plant import CoulombFriction, MultibodyPlant
from pydrake.multibody.tree import ModelInstanceIndex

from ...microwave import (
    MICROWAVE_BACK_WALL_Y,
    MICROWAVE_BOTTOM_WALL_Z,
    MICROWAVE_CENTER_XY_TASK,
    MICROWAVE_DOOR_THICKNESS_Y,
    MICROWAVE_DOOR_WIDTH_X,
    MICROWAVE_HINGE_X,
    MICROWAVE_INTERIOR_DEPTH,
    MICROWAVE_LEFT_WALL_X,
    MICROWAVE_OUTER_BOTTOM_Z,
    MICROWAVE_OUTER_D_Y,
    MICROWAVE_OUTER_H_Z,
    MICROWAVE_OUTER_W_X,
    MICROWAVE_RIGHT_WALL_X,
    MICROWAVE_TOP_WALL_Z,
)


# Outer body centre in task frame, derived from the cavity centre and
# the asymmetric wall thicknesses. Cavity centre is shifted from the
# outer centre by half the wall-thickness asymmetry: the right wall is
# 10 cm thicker than the left, so the outer body sits 5 cm to the +x
# of the cavity centre. Same logic for y (back wall pushes outer +y).
_OUTER_CENTRE_X = (
    MICROWAVE_CENTER_XY_TASK[0]
    + 0.5 * (MICROWAVE_RIGHT_WALL_X - MICROWAVE_LEFT_WALL_X)
)
_OUTER_CENTRE_Y = MICROWAVE_CENTER_XY_TASK[1] + 0.5 * MICROWAVE_BACK_WALL_Y
_OUTER_CENTRE_Z = MICROWAVE_OUTER_BOTTOM_Z + 0.5 * MICROWAVE_OUTER_H_Z

# Sanity: outer extents must reconcile with cavity + wall thicknesses.
# Catches an editor changing one constant without updating the other.
from ...microwave import MICROWAVE_INTERIOR_WIDTH_X as _INT_W
assert abs(
    MICROWAVE_OUTER_W_X
    - (_INT_W + MICROWAVE_LEFT_WALL_X + MICROWAVE_RIGHT_WALL_X)
) < 1e-9, (
    f"outer width {MICROWAVE_OUTER_W_X} ≠ "
    f"interior {_INT_W} + left {MICROWAVE_LEFT_WALL_X} "
    f"+ right {MICROWAVE_RIGHT_WALL_X}"
)
assert abs(
    MICROWAVE_OUTER_D_Y
    - (MICROWAVE_INTERIOR_DEPTH + MICROWAVE_BACK_WALL_Y)
) < 1e-9, (
    f"outer depth {MICROWAVE_OUTER_D_Y} ≠ "
    f"interior {MICROWAVE_INTERIOR_DEPTH} + back {MICROWAVE_BACK_WALL_Y}"
)

# Door plane y (front face of the cavity).
_DOOR_PLANE_Y = MICROWAVE_CENTER_XY_TASK[1] - 0.5 * MICROWAVE_INTERIOR_DEPTH

# Door spans from the outer-left edge eastward by ``DOOR_WIDTH_X``.
# The remaining 8 cm of the front face is the control-panel face,
# represented implicitly by the right side wall's front face.
_DOOR_LEFT_X = MICROWAVE_HINGE_X
_DOOR_HALF_W = 0.5 * MICROWAVE_DOOR_WIDTH_X


def _door_world_transform(open_angle_rad: float) -> RigidTransform:
    """World-frame transform for the door collision box.

    Closed (``open_angle_rad = 0``): box centered on the door plane,
    extending +x from the hinge by ``DOOR_WIDTH_X`` and ±DOOR_T/2 about
    the door plane in y.

    Opening rotates the box about the vertical hinge axis at
    ``(MICROWAVE_HINGE_X, door_plane_y)`` by *negative* ``open_angle_rad``
    (CW viewed from +z), so positive angles swing the door toward the
    operator (-y). The door's center sweeps a circle of radius
    ``DOOR_WIDTH_X / 2`` around the hinge.

    Typical real-microwave open angles: ``π/2`` (90°, door perpendicular
    to the front face) up to ``≈ 7π/12`` (105°, the mechanical hinge
    stop on most countertop units).
    """
    c = np.cos(open_angle_rad)
    s = np.sin(open_angle_rad)
    cx = MICROWAVE_HINGE_X + _DOOR_HALF_W * c
    cy = _DOOR_PLANE_Y - _DOOR_HALF_W * s
    R = RotationMatrix.MakeZRotation(-open_angle_rad)
    return RigidTransform(R, [cx, cy, _OUTER_CENTRE_Z])

_HOUSING_COLOR = np.array([0.85, 0.85, 0.83, 1.0])
_PANEL_COLOR = np.array([0.65, 0.65, 0.65, 1.0])
_DOOR_COLOR = np.array([0.20, 0.20, 0.20, 1.0])
_NO_FRICTION = CoulombFriction(0.4, 0.4)


@dataclass
class MicrowaveHandles:
    model_instance: ModelInstanceIndex


def _add_box(plant, model_instance, name, size, X_world, color):
    body = plant.AddRigidBody(name, model_instance)
    plant.WeldFrames(plant.world_frame(), body.body_frame(), X_world)
    box = Box(*size)
    plant.RegisterVisualGeometry(body, RigidTransform(), box, f"{name}_visual", color)
    plant.RegisterCollisionGeometry(
        body, RigidTransform(), box, f"{name}_collision", _NO_FRICTION,
    )
    return body


def add_microwave(
    plant: MultibodyPlant,
    *,
    name: str = "microwave",
    door_open_angle_rad: float = 0.0,
) -> MicrowaveHandles:
    """Add the microwave as asymmetric walls + a welded door.

    ``door_open_angle_rad`` controls the door state at scene-build time:
        0           → door closed (covers the cavity opening)
        π/2 (90°)   → door fully perpendicular to the front face
        ≈ 7π/12     → ~105°, typical mechanical hinge stop
    The door is **welded** at the chosen angle — this is a static
    collision body, not a hinged joint. Build separate scenes (or
    rebuild the plant) when you need to flip between door states.
    Once the door angle needs to be a state the planner varies online,
    replace the welded door box with a revolute joint at
    ``MICROWAVE_HINGE_X``.
    """
    model_instance = plant.AddModelInstance(name)
    cx, cy, cz = _OUTER_CENTRE_X, _OUTER_CENTRE_Y, _OUTER_CENTRE_Z

    half_w = 0.5 * MICROWAVE_OUTER_W_X
    half_d = 0.5 * MICROWAVE_OUTER_D_Y
    half_h = 0.5 * MICROWAVE_OUTER_H_Z

    # Back wall: full outer footprint, sits at the +y face.
    _add_box(
        plant, model_instance, "back",
        size=(MICROWAVE_OUTER_W_X, MICROWAVE_BACK_WALL_Y, MICROWAVE_OUTER_H_Z),
        X_world=RigidTransform([
            cx,
            cy + half_d - 0.5 * MICROWAVE_BACK_WALL_Y,
            cz,
        ]),
        color=_HOUSING_COLOR,
    )

    # Left side wall: thin (3 cm) — sits flush with outer left face.
    _add_box(
        plant, model_instance, "left",
        size=(MICROWAVE_LEFT_WALL_X, MICROWAVE_OUTER_D_Y, MICROWAVE_OUTER_H_Z),
        X_world=RigidTransform([
            cx - half_w + 0.5 * MICROWAVE_LEFT_WALL_X,
            cy,
            cz,
        ]),
        color=_HOUSING_COLOR,
    )

    # Right side wall: thick (13 cm) — magnetron + electronics housing.
    # Its front face (at y = door plane) is the control-panel surface;
    # ``BUTTON_TASK_XYZ`` lands on this face.
    _add_box(
        plant, model_instance, "right",
        size=(MICROWAVE_RIGHT_WALL_X, MICROWAVE_OUTER_D_Y, MICROWAVE_OUTER_H_Z),
        X_world=RigidTransform([
            cx + half_w - 0.5 * MICROWAVE_RIGHT_WALL_X,
            cy,
            cz,
        ]),
        color=_PANEL_COLOR,
    )

    # Top wall: 5 cm thick.
    _add_box(
        plant, model_instance, "top",
        size=(MICROWAVE_OUTER_W_X, MICROWAVE_OUTER_D_Y, MICROWAVE_TOP_WALL_Z),
        X_world=RigidTransform([
            cx, cy,
            cz + half_h - 0.5 * MICROWAVE_TOP_WALL_Z,
        ]),
        color=_HOUSING_COLOR,
    )

    # Bottom wall: 3 cm thick (glass-tray motor housing).
    _add_box(
        plant, model_instance, "bottom",
        size=(MICROWAVE_OUTER_W_X, MICROWAVE_OUTER_D_Y, MICROWAVE_BOTTOM_WALL_Z),
        X_world=RigidTransform([
            cx, cy,
            cz - half_h + 0.5 * MICROWAVE_BOTTOM_WALL_Z,
        ]),
        color=_HOUSING_COLOR,
    )

    # Door: 36 cm wide, full body height, sits on the LEFT portion of
    # the front face. The remaining 8 cm of the front face is the
    # right-side-wall front face (implicit; modelled by the "right"
    # box above which extends to the door plane). When
    # ``door_open_angle_rad > 0``, the door is welded rotated about its
    # vertical hinge axis at the outer-left edge.
    _add_box(
        plant, model_instance, "door",
        size=(MICROWAVE_DOOR_WIDTH_X, MICROWAVE_DOOR_THICKNESS_Y, MICROWAVE_OUTER_H_Z),
        X_world=_door_world_transform(door_open_angle_rad),
        color=_DOOR_COLOR,
    )

    return MicrowaveHandles(model_instance=model_instance)
