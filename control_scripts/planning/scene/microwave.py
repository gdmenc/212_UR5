"""Microwave as primitive boxes — same philosophy as the Vention stand.

The existing ``src/assets/microwave/microwave.sdf`` loads a single OBJ
mesh which Drake convex-hulls for collision. Like the Vention, that
fills the cavity (where the door swings into) and the front (where the
hook needs to engage the handle), making collision queries useless.

This module models the microwave as five welded box walls + a closed
door box. Dimensions are placeholders — measure your real microwave
and update the constants. They follow the same handle reference point
(``HANDLE_ENGAGE_POSE_TASK`` in ``examples/open_microwave.py``) so the
microwave lands where the recorded waypoints expect.

A hinged-door variant will follow once the door angle becomes a state
the planner needs to vary. For "is the door obstructing the arm during
transit" checks the closed-door geometry below is enough.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from pydrake.geometry import Box
from pydrake.math import RigidTransform
from pydrake.multibody.plant import CoulombFriction, MultibodyPlant
from pydrake.multibody.tree import ModelInstanceIndex


# ---------------------------------------------------------------------------
#  Default dimensions — calibrate against your physical microwave.
# ---------------------------------------------------------------------------

# Body footprint (width × depth × height) of a typical countertop microwave.
_BODY_W = 0.500    # x extent  (left-right in task frame)
_BODY_D = 0.380    # y extent  (depth, away from operator)
_BODY_H = 0.300    # z extent

_WALL = 0.020      # housing wall thickness
_DOOR_T = 0.025    # door thickness along y

# Body centre in world frame, derived so the front-right corner of
# the door matches HANDLE_ENGAGE_POSE_TASK in examples/open_microwave.py
# (translation [-0.071, 0.344, 0.155]).
_HANDLE_TASK_XYZ = np.array([-0.071, 0.344, 0.155])
_BODY_CENTRE_X = _HANDLE_TASK_XYZ[0] + _BODY_W * 0.5 - 0.015
_BODY_CENTRE_Y = _HANDLE_TASK_XYZ[1] + _BODY_D * 0.5 + _DOOR_T * 0.5
_BODY_CENTRE_Z = _HANDLE_TASK_XYZ[2]

_HOUSING_COLOR = np.array([0.85, 0.85, 0.83, 1.0])
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
) -> MicrowaveHandles:
    """Add the microwave as five housing walls + a welded closed door.

    Hinged-door variant deliberately deferred — closed-door collision
    geometry is sufficient for transit-collision and pre-engage
    reachability checks during initial scene bring-up.
    """
    model_instance = plant.AddModelInstance(name)
    cx, cy, cz = _BODY_CENTRE_X, _BODY_CENTRE_Y, _BODY_CENTRE_Z

    _add_box(
        plant, model_instance, "back",
        size=(_BODY_W, _WALL, _BODY_H),
        X_world=RigidTransform([cx, cy + _BODY_D * 0.5 - _WALL * 0.5, cz]),
        color=_HOUSING_COLOR,
    )
    _add_box(
        plant, model_instance, "left",
        size=(_WALL, _BODY_D, _BODY_H),
        X_world=RigidTransform([cx - _BODY_W * 0.5 + _WALL * 0.5, cy, cz]),
        color=_HOUSING_COLOR,
    )
    _add_box(
        plant, model_instance, "right",
        size=(_WALL, _BODY_D, _BODY_H),
        X_world=RigidTransform([cx + _BODY_W * 0.5 - _WALL * 0.5, cy, cz]),
        color=_HOUSING_COLOR,
    )
    _add_box(
        plant, model_instance, "top",
        size=(_BODY_W, _BODY_D, _WALL),
        X_world=RigidTransform([cx, cy, cz + _BODY_H * 0.5 - _WALL * 0.5]),
        color=_HOUSING_COLOR,
    )
    _add_box(
        plant, model_instance, "bottom",
        size=(_BODY_W, _BODY_D, _WALL),
        X_world=RigidTransform([cx, cy, cz - _BODY_H * 0.5 + _WALL * 0.5]),
        color=_HOUSING_COLOR,
    )
    _add_box(
        plant, model_instance, "door",
        size=(_BODY_W, _DOOR_T, _BODY_H),
        X_world=RigidTransform([cx, cy - _BODY_D * 0.5 + _DOOR_T * 0.5, cz]),
        color=_DOOR_COLOR,
    )

    return MicrowaveHandles(model_instance=model_instance)
