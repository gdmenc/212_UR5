"""Tabletop objects as primitive collision geometry.

Same philosophy as ``microwave.py`` / ``tables.py``: weld a primitive
shape to the world at a calibrated task-frame pose so the planner has
something to collide-check against during transit. Free-body dynamics
and in-hand grasping are deliberately out of scope here — the assumption
is that during a transit plan the object stays put on the table.

Frame conventions
-----------------
For each object we document its **object frame** (where the welded
shape's origin sits in the object) so the task-frame pose passed in
unambiguously describes "where the thing is on the table". Symmetric
shapes (cylinders) just need an axis convention; the tray needs a full
xyz convention.

* plate   — cylinder, origin at centre of bottom face, +z up
* cup     — cylinder, origin at centre of bottom face (resting surface), +z up
* bowl    — cylinder, origin at centre of bottom face (resting surface), +z up
* bottle  — cylinder, origin at centre of bottom face, +z up
* tray    — box, origin at centre of bottom face, +x along the LONG side
            (42 cm), +y along the SHORT side (27.8 cm), +z up

Default poses
-------------
Pick poses are **copied** (not imported) from the matching task scripts
in ``control_scripts/tasks/`` because those values are still being
calibrated and may drift; we don't want a planner-side scene to silently
re-anchor every time a task tweaks a number. When the lab-measured pose
stabilises, refactor both call sites at once.

Objects flagged TODO need lab measurements before relying on the
collision result for real motion.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np
from pydrake.geometry import Box, Cylinder
from pydrake.math import RigidTransform
from pydrake.multibody.plant import CoulombFriction, MultibodyPlant
from pydrake.multibody.tree import ModelInstanceIndex


# ---------------------------------------------------------------------------
#  Object dimensions — copied from control_scripts/grasps/*.py.
#  Radii are bumped a few mm for collision padding, which gives the
#  planner a safety margin without forcing the grasp scripts to lie
#  about the real geometry.
# ---------------------------------------------------------------------------
_PAD = 0.005  # uniform radial pad applied to cylindrical objects

PLATE_RADIUS = 0.125 + _PAD            # grasps/plate.py: PLATE_OUTER_RADIUS=0.125
PLATE_HEIGHT = 0.020                   # grasps/plate.py: PLATE_RIM_HEIGHT=0.02

CUP_RADIUS = 0.044 + _PAD              # grasps/cup.py: CUP_RIM_OUTER_RADIUS_M=0.044
CUP_HEIGHT = 0.154                     # grasps/cup.py: CUP_HEIGHT_M=0.154

BOWL_RADIUS = 0.037*2 + _PAD             # grasps/bowl.py: BOWL_RIM_OUTER_RADIUS_M=0.037
BOWL_HEIGHT = 0.072                    # grasps/bowl.py: BOWL_RIM_Z_OFFSET_M=0.072

BOTTLE_RADIUS = 0.0365 + _PAD          # grasps/bottle.py: BOTTLE_BODY_RADIUS_M=0.0365
BOTTLE_HEIGHT = 0.175                  # grasps/bottle.py: BOTTLE_TOTAL_HEIGHT_M=0.175

# Stick: thin rod sitting in the cup; placeholder dims, subject to change.
STICK_LENGTH = 1.5 * CUP_HEIGHT        # = 0.231 m  (per user spec, subject to change)
STICK_RADIUS = 0.005                   # ~chopstick / stir-stick diameter
STICK_SUBMERGED = 0.5 * CUP_HEIGHT     # half the stick is in the cup; rest pokes up

# Tray dimensions — lab-measured 2026-05-04.
TRAY_LENGTH_X = 0.420                  # along table's left-right (object +x)
TRAY_WIDTH_Y = 0.278                   # along table's depth     (object +y)
TRAY_HEIGHT = 0.025

# ---------------------------------------------------------------------------
#  Default task-frame poses (copied from control_scripts/tasks/*.py).
#  All as plain (x, y, z) tuples. Z is the OBJECT-FRAME ORIGIN's task z —
#  for cylinder-resting-on-table objects, that's z = 0 (table top), with
#  small per-object offsets matching what the task scripts use.
# ---------------------------------------------------------------------------
PLATE_DEFAULT_TASK_XYZ = (0.295521, -0.20, 0.01)
"""Copied from tasks/pick_place_plate.py::PLATE_PICK_POSE_TASK."""

CUP_DEFAULT_TASK_XYZ = (-0.08, -0.125, 0.0)
"""Copied from tasks/pick_place_cup.py::CUP_PICK_POSE_TASK."""

CUP_WITH_STICK_DEFAULT_TASK_XYZ = (-0.20, -0.125, 0.0)
"""Placeholder — left of the plain cup. TODO: measure at lab."""

BOWL_DEFAULT_TASK_XYZ = (0.05, -0.125, -0.01)
"""Copied from tasks/pick_place_bowl_hook_microwave.py::BOWL_PICK_POSE_TASK.
The bowl pose used by other variants ([0.1, 0, 0.01]) differs; pick whichever
matches the scene you're planning for via the X_task argument."""

BOTTLE_DEFAULT_TASK_XYZ = (-0.32, -0.125, -0.01)
"""Copied from tasks/pour_bottle_hook.py::BOTTLE_PICK_POSE_TASK."""

# Tray sits forward of the plate by 20 cm in +y (toward the microwave),
# long side along the table's left-right (object +x already aligned with
# task +x). Bottom on the table top.
TRAY_DEFAULT_TASK_XYZ = (PLATE_DEFAULT_TASK_XYZ[0],
                         PLATE_DEFAULT_TASK_XYZ[1] + 0.30,
                         0.0)


_PLATE_COLOR = np.array([0.95, 0.95, 0.92, 1.0])
_CUP_COLOR = np.array([0.85, 0.30, 0.30, 1.0])
_STICK_COLOR = np.array([0.40, 0.25, 0.10, 1.0])
_BOWL_COLOR = np.array([0.30, 0.65, 0.30, 1.0])
_BOTTLE_COLOR = np.array([0.30, 0.45, 0.85, 0.7])
_TRAY_COLOR = np.array([0.55, 0.55, 0.60, 1.0])
_NO_FRICTION = CoulombFriction(0.4, 0.4)


@dataclass
class ObjectHandles:
    model_instance: ModelInstanceIndex


def _add_cylinder(plant, model_instance, name, radius, length, X_world, color):
    """Add a Z-axis cylinder welded at X_world. Drake's Cylinder is
    centred on its own origin, so the caller is responsible for
    translating the bottom face to the desired height."""
    body = plant.AddRigidBody(name, model_instance)
    plant.WeldFrames(plant.world_frame(), body.body_frame(), X_world)
    shape = Cylinder(radius, length)
    plant.RegisterVisualGeometry(body, RigidTransform(), shape, f"{name}_visual", color)
    plant.RegisterCollisionGeometry(
        body, RigidTransform(), shape, f"{name}_collision", _NO_FRICTION,
    )
    return body


def _add_box(plant, model_instance, name, size, X_world, color):
    body = plant.AddRigidBody(name, model_instance)
    plant.WeldFrames(plant.world_frame(), body.body_frame(), X_world)
    shape = Box(*size)
    plant.RegisterVisualGeometry(body, RigidTransform(), shape, f"{name}_visual", color)
    plant.RegisterCollisionGeometry(
        body, RigidTransform(), shape, f"{name}_collision", _NO_FRICTION,
    )
    return body


def _xyz_to_rt(xyz: Tuple[float, float, float]) -> RigidTransform:
    return RigidTransform(np.asarray(xyz, dtype=float))


# ---------------------------------------------------------------------------
#  Public add_* entry points. Each takes a task-frame xyz (defaulting to
#  the constants above) and returns the model instance it created.
# ---------------------------------------------------------------------------

def add_plate(
    plant: MultibodyPlant,
    *,
    xyz_task: Tuple[float, float, float] = PLATE_DEFAULT_TASK_XYZ,
    name: str = "plate",
) -> ObjectHandles:
    """Plate as a flat cylinder. Origin at centre of bottom face."""
    inst = plant.AddModelInstance(name)
    cx, cy, cz = xyz_task
    _add_cylinder(
        plant, inst, name,
        radius=PLATE_RADIUS, length=PLATE_HEIGHT,
        X_world=_xyz_to_rt((cx, cy, cz + PLATE_HEIGHT * 0.5)),
        color=_PLATE_COLOR,
    )
    return ObjectHandles(model_instance=inst)


def add_cup(
    plant: MultibodyPlant,
    *,
    xyz_task: Optional[Tuple[float, float, float]] = None,
    with_stick: bool = False,
    name: Optional[str] = None,
) -> ObjectHandles:
    """Cup as a vertical cylinder. Origin at centre of base.

    ``with_stick=True`` adds a thin vertical rod welded into the cup.
    The rod is coaxial with the cup, half its length submerged. This
    keeps the combined object rotationally symmetric so the cup grasp
    geometry doesn't change.

    Default ``xyz_task`` depends on ``with_stick`` so calling
    ``add_cup(plant)`` and ``add_cup(plant, with_stick=True)`` in the
    same scene puts them in different spots, not stacked on top of
    each other.
    """
    if name is None:
        name = "cup_with_stick" if with_stick else "cup"
    if xyz_task is None:
        xyz_task = (CUP_WITH_STICK_DEFAULT_TASK_XYZ if with_stick
                    else CUP_DEFAULT_TASK_XYZ)
    inst = plant.AddModelInstance(name)
    cx, cy, cz = xyz_task

    _add_cylinder(
        plant, inst, "body",
        radius=CUP_RADIUS, length=CUP_HEIGHT,
        X_world=_xyz_to_rt((cx, cy, cz + CUP_HEIGHT * 0.5)),
        color=_CUP_COLOR,
    )

    if with_stick:
        # Stick base sits at cz + (CUP_HEIGHT - STICK_SUBMERGED); centre
        # is half of STICK_LENGTH above that.
        stick_base_z = cz + (CUP_HEIGHT - STICK_SUBMERGED)
        stick_centre_z = stick_base_z + STICK_LENGTH * 0.5
        _add_cylinder(
            plant, inst, "stick",
            radius=STICK_RADIUS, length=STICK_LENGTH,
            X_world=_xyz_to_rt((cx, cy, stick_centre_z)),
            color=_STICK_COLOR,
        )

    return ObjectHandles(model_instance=inst)


def add_bowl(
    plant: MultibodyPlant,
    *,
    xyz_task: Tuple[float, float, float] = BOWL_DEFAULT_TASK_XYZ,
    name: str = "bowl",
) -> ObjectHandles:
    """Bowl as a vertical cylinder (rim radius). Origin at centre of base."""
    inst = plant.AddModelInstance(name)
    cx, cy, cz = xyz_task
    _add_cylinder(
        plant, inst, name,
        radius=BOWL_RADIUS, length=BOWL_HEIGHT,
        X_world=_xyz_to_rt((cx, cy, cz + BOWL_HEIGHT * 0.5)),
        color=_BOWL_COLOR,
    )
    return ObjectHandles(model_instance=inst)


def add_bottle(
    plant: MultibodyPlant,
    *,
    xyz_task: Tuple[float, float, float] = BOTTLE_DEFAULT_TASK_XYZ,
    name: str = "bottle",
) -> ObjectHandles:
    """Bottle as a vertical cylinder (body radius). Origin at centre of base.
    Slight overestimate near the cap (the bottle tapers there) — fine for
    transit collision checks."""
    inst = plant.AddModelInstance(name)
    cx, cy, cz = xyz_task
    _add_cylinder(
        plant, inst, name,
        radius=BOTTLE_RADIUS, length=BOTTLE_HEIGHT,
        X_world=_xyz_to_rt((cx, cy, cz + BOTTLE_HEIGHT * 0.5)),
        color=_BOTTLE_COLOR,
    )
    return ObjectHandles(model_instance=inst)


def add_tray(
    plant: MultibodyPlant,
    *,
    xyz_task: Tuple[float, float, float] = TRAY_DEFAULT_TASK_XYZ,
    length_x: float = TRAY_LENGTH_X,
    width_y: float = TRAY_WIDTH_Y,
    height: float = TRAY_HEIGHT,
    name: str = "tray",
) -> ObjectHandles:
    """Rectangular tray. Origin at centre of bottom face, +x along the
    long side (42 cm by default), +y along the short side. The default
    pose puts it with long side along the table's left-right axis and
    bottom on the table top."""
    inst = plant.AddModelInstance(name)
    cx, cy, cz = xyz_task
    _add_box(
        plant, inst, name,
        size=(length_x, width_y, height),
        X_world=_xyz_to_rt((cx, cy, cz + height * 0.5)),
        color=_TRAY_COLOR,
    )
    return ObjectHandles(model_instance=inst)
