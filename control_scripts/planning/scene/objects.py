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
from pydrake.math import RigidTransform, RotationMatrix
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


def _weld_cylinder(plant, model_instance, name, radius, length,
                   parent_frame, X_PF, color):
    """Add a Z-axis cylinder welded to ``parent_frame`` at ``X_PF``.

    Drake's Cylinder is centred on its own origin, so the caller is
    responsible for translating the bottom face to the desired height.
    """
    body = plant.AddRigidBody(name, model_instance)
    plant.WeldFrames(parent_frame, body.body_frame(), X_PF)
    shape = Cylinder(radius, length)
    plant.RegisterVisualGeometry(body, RigidTransform(), shape, f"{name}_visual", color)
    plant.RegisterCollisionGeometry(
        body, RigidTransform(), shape, f"{name}_collision", _NO_FRICTION,
    )
    return body


def _weld_box(plant, model_instance, name, size,
              parent_frame, X_PF, color):
    body = plant.AddRigidBody(name, model_instance)
    plant.WeldFrames(parent_frame, body.body_frame(), X_PF)
    shape = Box(*size)
    plant.RegisterVisualGeometry(body, RigidTransform(), shape, f"{name}_visual", color)
    plant.RegisterCollisionGeometry(
        body, RigidTransform(), shape, f"{name}_collision", _NO_FRICTION,
    )
    return body


def _add_cylinder(plant, model_instance, name, radius, length, X_world, color):
    """Convenience: weld a cylinder to the world frame at ``X_world``."""
    return _weld_cylinder(
        plant, model_instance, name, radius, length,
        plant.world_frame(), X_world, color,
    )


def _add_box(plant, model_instance, name, size, X_world, color):
    """Convenience: weld a box to the world frame at ``X_world``."""
    return _weld_box(
        plant, model_instance, name, size,
        plant.world_frame(), X_world, color,
    )


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


# ---------------------------------------------------------------------------
#  In-hand attachment: weld an object to the calibrated TCP frame
# ---------------------------------------------------------------------------
#
# Frame conventions inside this section
# -------------------------------------
# At a canonical "grasp orientation" the gripper points DOWN at the table
# and tool +Z = task -Z (i.e. tool +Z points downward in the world).
# The TCP frame (``tcp_left`` / ``tcp_right``, added by scene/grippers.py
# as a FixedOffsetFrame on wrist_3) is the calibrated tool-tip frame:
#
#   * TCP +Z = tool axis (= "down" in world at canonical grasp orientation)
#   * TCP +X chosen by the per-grasp rotation as "radial outward" at the
#     canonical grasp angle (away from the held object's centre axis)
#   * TCP +Y completes the right-handed triad
#
# We weld attached objects to the TCP frame (not the gripper body
# frame). That decouples the in-hand object pose from any visual
# rotation applied to the gripper SDF — see ``add_hook_gripper`` for
# why the hook visual is rotated 90° CCW about its own z relative to
# the calibrated TCP.

_GRIPPER_TCP_FRAME = {
    "ur_left":  "tcp_left",
    "ur_right": "tcp_right",
}


def _find_tcp_frame(plant: MultibodyPlant, arm_name: str):
    return plant.GetFrameByName(_GRIPPER_TCP_FRAME[arm_name])


def _hook_rim_X_TCP_obj_body(*, outer_radius: float,
                             rim_z_in_obj: float,
                             full_height: float) -> RigidTransform:
    """X_TCP_(object body frame) for a HOOK gripper top-down rim grasp.

    Derived directly from ``grasps/_hook_rim.py::hook_rim_rotation`` at
    angle=0, tilt=0:
        R_obj_TCP = R_x(π)
        t_obj_TCP = (R, 0, rim_z_in_obj)        # rim point at canonical angle
    Therefore:
        X_TCP_obj_origin = (R_x(π)^T, -R_x(π)^T @ t)
                         = (R_x(π), -R_x(π) @ (R,0,h_rim))
                         = (R_x(π), (-R, 0, h_rim))
    Object body Drake origin = object frame [0, 0, full_height/2]; in TCP:
        body_origin_in_TCP = X_TCP_obj_origin @ [0, 0, full_height/2]
                           = (-R, 0, h_rim) + R_x(π) @ (0, 0, full_height/2)
                           = (-R, 0, h_rim - full_height/2)

    No extra rotation about TCP +Z — the held object sits at TCP -X
    (radially inward at the canonical grasp angle) with body extending
    in TCP +Z. (Earlier iterations applied R_z(-π/2) and R_z(-π); both
    reverted to 0° per the user's "all hook rotations 0°" instruction.)
    """
    return RigidTransform(
        RotationMatrix.MakeXRotation(np.pi),
        [-outer_radius, 0.0, rim_z_in_obj - full_height * 0.5],
    )


# R_TCP_obj_origin for ROBOTIQ top-down rim pinch — flat 3x3 derived from
# ``grasps/plate.py::_top_down_rim_rotation`` at angle=0:
#   R_obj_TCP = (R_z(π) * R_y(π)) * R_z(π/2) = [[0,-1,0],[-1,0,0],[0,0,-1]]
# (symmetric, so its transpose is itself).
_ROBOTIQ_RIM_R_TCP_OBJ = RotationMatrix(np.array([
    [ 0.0, -1.0,  0.0],
    [-1.0,  0.0,  0.0],
    [ 0.0,  0.0, -1.0],
]))


def _robotiq_rim_X_TCP_obj_body(*, outer_radius: float,
                                rim_z_in_obj: float,
                                full_height: float,
                                rotate_axis_to_radial: bool = False
                                ) -> RigidTransform:
    """X_TCP_(object body frame) for a ROBOTIQ top-down rim pinch
    (plate, cup). Differs from the hook variant by the ``tool_z_90``
    factor in the grasp rotation, which swaps the radial direction
    from tool -X to tool +Y.

    Object body Drake origin in TCP frame:
        t = X_TCP_obj_origin @ [0, 0, full_height/2]
          = (0, R, rim_z_in_obj) + R_TCP_obj @ (0, 0, full_height/2)
          = (0, R, rim_z_in_obj - full_height/2)

    ``rotate_axis_to_radial`` (used for the plate): rotate the whole
    assembly +π/2 about TCP +X so the object's symmetry axis aligns
    with TCP +Y → TCP +Z direction. After this, the object hangs
    "radially down" from TCP with its long axis along the gripper
    tool axis — matches how a flat plate is held by its rim.
    """
    base = RigidTransform(
        _ROBOTIQ_RIM_R_TCP_OBJ,
        [0.0, outer_radius, rim_z_in_obj - full_height * 0.5],
    )
    if rotate_axis_to_radial:
        return RigidTransform(RotationMatrix.MakeXRotation(np.pi / 2)) @ base
    return base


def _robotiq_side_pinch_X_TCP_obj_body(*, body_radius: float,
                                       grasp_z_in_obj: float,
                                       full_height: float) -> RigidTransform:
    """X_TCP_(object body frame) for a ROBOTIQ side body pinch (bottle).

    Derived from ``grasps/bottle.py::_side_body_rotation`` at angle=0:
      R_obj_TCP = [[0,0,-1],[0,-1,0],[-1,0,0]]
      t_obj_TCP = (body_radius, 0, grasp_z_in_obj)
    Therefore:
      X_TCP_obj_origin.translation = -R^T @ t = (grasp_z, 0, body_radius)
      X_TCP_obj_origin.rotation    = R^T = same matrix (symmetric)
    Body Drake origin in TCP:
      = X_TCP_obj_origin @ (0, 0, full_height/2)
      = (grasp_z, 0, body_radius) + R^T @ (0, 0, full_height/2)
      = (grasp_z, 0, body_radius) + (-full_height/2, 0, 0)
      = (grasp_z - full_height/2, 0, body_radius)

    Then user-requested -π rotation about TCP +Z (same as bowl) so the
    bottle ends up on the "long side of rim points out" direction.
    """
    R = RotationMatrix(np.array([
        [ 0.0,  0.0, -1.0],
        [ 0.0, -1.0,  0.0],
        [-1.0,  0.0,  0.0],
    ]))
    base = RigidTransform(
        R,
        [grasp_z_in_obj - full_height * 0.5, 0.0, body_radius],
    )
    return RigidTransform(RotationMatrix.MakeZRotation(-np.pi)) @ base


def attach_object_to_gripper(
    plant: MultibodyPlant,
    kind: str,
    arm_name: str,
    *,
    X_gripper_obj: Optional[RigidTransform] = None,
    name: Optional[str] = None,
) -> ObjectHandles:
    """Weld a primitive object to the named arm's gripper body.

    Default ``X_gripper_obj`` per kind matches the grasp style we
    actually use:

      * ``plate``, ``cup``, ``cup_with_stick``, ``bowl`` — top-down
        **rim grasp**: rim sits at TCP, object body hangs in its
        natural upright orientation just below the gripper, axis
        offset radially inward by the object's rim radius.
      * ``bottle`` — **side body pinch**: bottle held vertically with
        TCP touching the body midway up; bottle axis is vertical,
        offset radially by the body radius.
      * ``tray`` — **flat-on-fingers** (placeholder default): tray
        sits flat below the gripper with its long axis along gripper
        +X. Probably wrong for any real grasp; override with
        ``X_gripper_obj`` if you actually carry the tray.

    Pass ``X_gripper_obj`` explicitly to override the default for any
    kind. The "canonical orientation" assumption is that gripper +Z
    is the tool axis and gripper +X is radial outward at angle 0;
    see the ``In-hand attachment`` section comment for details.
    """
    if arm_name not in _GRIPPER_TCP_FRAME:
        raise KeyError(f"unknown arm {arm_name!r}")
    if name is None:
        name = f"{kind}_in_hand_{arm_name}"

    # Weld attached objects to the TCP frame (a FixedOffsetFrame on the
    # wrist) instead of the gripper body. That way the in-hand object
    # pose is independent of any rotation applied to the gripper SDF
    # — see ``add_hook_gripper`` for why the hook visual is rotated
    # 90° about wrist z relative to the calibrated TCP.
    grip_frame = _find_tcp_frame(plant, arm_name)

    inst = plant.AddModelInstance(name)

    # Pick the X_TCP_obj_body for this (kind, arm) combination — derived
    # from the actual grasp pose math in ``grasps/<kind>.py``.
    X_TCP_obj_body: Optional[RigidTransform] = None

    if kind == "plate":
        # Robotiq top-down rim pinch — radius = plate outer rim, rim z =
        # plate rim height in object frame. ``rotate_axis_to_radial``
        # tilts the plate so its symmetry axis ends up along the gripper
        # tool axis (i.e., tool +Z points along the plate's radius).
        X_TCP_obj_body = _robotiq_rim_X_TCP_obj_body(
            outer_radius=PLATE_RADIUS,
            rim_z_in_obj=PLATE_HEIGHT,
            full_height=PLATE_HEIGHT,
            rotate_axis_to_radial=True,
        )
    elif kind == "cup":
        # Robotiq top-down rim pinch — rim z = cup full height.
        X_TCP_obj_body = _robotiq_rim_X_TCP_obj_body(
            outer_radius=CUP_RADIUS,
            rim_z_in_obj=CUP_HEIGHT,
            full_height=CUP_HEIGHT,
        )
    elif kind == "cup_with_stick":
        # Cup body: same rim pinch as plain cup.
        X_TCP_obj_body = _robotiq_rim_X_TCP_obj_body(
            outer_radius=CUP_RADIUS,
            rim_z_in_obj=CUP_HEIGHT,
            full_height=CUP_HEIGHT,
        )
    elif kind == "bowl":
        # Bowl is grasped via HOOK (left) in the actual tasks. If the
        # caller specified the right arm, fall back to a Robotiq rim
        # pinch with the same geometry (it's a valid grasp style; just
        # not the one we use).
        if arm_name == "ur_left":
            X_TCP_obj_body = _hook_rim_X_TCP_obj_body(
                outer_radius=BOWL_RADIUS,
                rim_z_in_obj=BOWL_HEIGHT,
                full_height=BOWL_HEIGHT,
            )
        else:
            X_TCP_obj_body = _robotiq_rim_X_TCP_obj_body(
                outer_radius=BOWL_RADIUS,
                rim_z_in_obj=BOWL_HEIGHT,
                full_height=BOWL_HEIGHT,
            )
    elif kind == "bottle":
        # Robotiq side-body pinch (right arm) vs. hook rim grasp on the
        # bottle's neck opening (left arm) — both grasp styles exist in
        # the tasks; dispatch by arm.
        if arm_name == "ur_left":
            # Hook grasps the OPENING (top of bottle), not the body.
            # Opening is at full height; treat the cylindrical body as
            # the collision shape. This isn't perfect (the real bottle
            # has a neck taper) but is good enough for transit checks.
            X_TCP_obj_body = _hook_rim_X_TCP_obj_body(
                outer_radius=BOTTLE_RADIUS,
                rim_z_in_obj=BOTTLE_HEIGHT,
                full_height=BOTTLE_HEIGHT,
            )
        else:
            # Side body pinch at mid-height (matches BOTTLE_DEFAULT_GRASP_Z_M).
            X_TCP_obj_body = _robotiq_side_pinch_X_TCP_obj_body(
                body_radius=BOTTLE_RADIUS,
                grasp_z_in_obj=0.1,         # mid-body, see grasps/bottle.py
                full_height=BOTTLE_HEIGHT,
            )
    elif kind == "tray":
        # No real grasp defined for the tray. Placeholder: tray sits
        # flat below the gripper. Pass X_gripper_obj explicitly if you
        # actually carry it.
        X_TCP_obj_body = RigidTransform([0.0, 0.0, TRAY_HEIGHT * 0.5])
    else:
        raise KeyError(
            f"unknown object kind {kind!r}; expected one of: "
            f"plate, cup, cup_with_stick, bowl, bottle, tray"
        )

    # Welding directly to the TCP frame, so X_TCP_obj_body IS the weld
    # transform. No gripper-body offset adjustment needed.
    if X_gripper_obj is None:
        X_gripper_obj = X_TCP_obj_body

    # Build the geometry, welded at X_gripper_obj relative to the gripper body.
    if kind == "plate":
        _weld_cylinder(
            plant, inst, name,
            radius=PLATE_RADIUS, length=PLATE_HEIGHT,
            parent_frame=grip_frame, X_PF=X_gripper_obj, color=_PLATE_COLOR,
        )
    elif kind == "cup":
        _weld_cylinder(
            plant, inst, "body",
            radius=CUP_RADIUS, length=CUP_HEIGHT,
            parent_frame=grip_frame, X_PF=X_gripper_obj, color=_CUP_COLOR,
        )
    elif kind == "cup_with_stick":
        _weld_cylinder(
            plant, inst, "body",
            radius=CUP_RADIUS, length=CUP_HEIGHT,
            parent_frame=grip_frame, X_PF=X_gripper_obj, color=_CUP_COLOR,
        )
        # Stick: poking out the cup's rim (i.e. the END of the cup that
        # USED to face up when the cup was sitting on the table).
        # In the cup's object frame the stick centre sits at:
        #     z_obj = (CUP_HEIGHT - STICK_SUBMERGED) + STICK_LENGTH/2
        # Compose through the cup's own X_TCP_obj_body to get a pose for
        # the stick body relative to the gripper (so the stick rotates
        # with the cup).
        stick_in_cup = RigidTransform(
            [0.0, 0.0,
             (CUP_HEIGHT - STICK_SUBMERGED) + STICK_LENGTH * 0.5
             - CUP_HEIGHT * 0.5],   # subtract cup's own body-origin offset
        )
        # The cup's X_gripper_obj already maps the cup body frame to the
        # gripper body frame. The stick is offset from the cup body
        # along cup +Z; in gripper body frame that's X_gripper_obj @
        # stick_in_cup.
        X_gripper_stick = X_gripper_obj @ stick_in_cup
        _weld_cylinder(
            plant, inst, "stick",
            radius=STICK_RADIUS, length=STICK_LENGTH,
            parent_frame=grip_frame, X_PF=X_gripper_stick, color=_STICK_COLOR,
        )
    elif kind == "bowl":
        _weld_cylinder(
            plant, inst, name,
            radius=BOWL_RADIUS, length=BOWL_HEIGHT,
            parent_frame=grip_frame, X_PF=X_gripper_obj, color=_BOWL_COLOR,
        )
    elif kind == "bottle":
        _weld_cylinder(
            plant, inst, name,
            radius=BOTTLE_RADIUS, length=BOTTLE_HEIGHT,
            parent_frame=grip_frame, X_PF=X_gripper_obj, color=_BOTTLE_COLOR,
        )
    elif kind == "tray":
        _weld_box(
            plant, inst, name,
            size=(TRAY_LENGTH_X, TRAY_WIDTH_Y, TRAY_HEIGHT),
            parent_frame=grip_frame, X_PF=X_gripper_obj, color=_TRAY_COLOR,
        )

    return ObjectHandles(model_instance=inst)
