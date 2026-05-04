"""Programmatic Vention stand: one Box per structural element.

A Vention frame is overwhelmingly empty space — convex-hulling or
decimating the full mesh fills the gaps your arm passes through with
solid material, which is why earlier attempts produced collision
geometry that was 'completely incorrect'.

Each extrusion / plate is its own primitive Box, welded to the world
body at its measured pose. Visually true to shape (every beam is
exactly where it is, with the right cross-section) and collision-correct
(the negative space stays negative).

How to populate the stand
-------------------------
The default ``BIMANUAL_STAND`` below is a *starter* derived from
``calibration.py`` — it gets the arm-base positions right but the
truss topology is approximate. You must replace the truss/bracing
beams with what your physical Vention assembly actually has, by
measurement or by reading off Vention's MachineLogic CAD export.

The numbers that come from calibration and are well-grounded:
    - top_plate_z          0.740 m   (just below arm base height 0.753 m)
    - half_arm_offset_x    0.165 m   (x distance arm-base ↔ task origin)
    - mount_y              0.383 m   (y distance arm-base ↔ task origin)
    - leg_height           0.740 m   (table-top to top-plate)

The numbers that need confirmation against your real hardware:
    - extrusion cross-sections (defaults assume Vention 4040)
    - leg X/Y footprint (defaults to a tight rectangle around the plate)
    - which faces have cross-bracing

Anchoring
---------
``add_vention_stand`` welds every box to the world body. World frame is
the task frame (see ``planning/__init__.py``):

    +x → right-arm side    +y → microwave side    +z → up

Returns the ``ModelInstanceIndex`` for the stand and the world-frame
pose of the top-plate top surface (where the arm bases bolt on).
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import List, Tuple

import numpy as np
from pydrake.geometry import Box
from pydrake.math import RigidTransform, RollPitchYaw, RotationMatrix
from pydrake.multibody.plant import CoulombFriction, MultibodyPlant
from pydrake.multibody.tree import ModelInstanceIndex


# -- Vention extrusion presets (cross-section in metres) --------------------
# Common Vention sizes. Cross-section is (width, height) of the rectangular
# bounding box. Internal slots and cutouts are not modelled — for collision
# checking the bounding rectangle is conservative and accurate to the order
# of millimetres.
VENTION_2020 = (0.020, 0.020)
VENTION_4040 = (0.040, 0.040)
VENTION_4080 = (0.040, 0.080)


# -- Visual style -----------------------------------------------------------
_VENTION_GREY = np.array([0.55, 0.58, 0.60, 1.0])
_VENTION_PLATE = np.array([0.45, 0.48, 0.50, 1.0])
_NO_FRICTION = CoulombFriction(0.4, 0.4)


@dataclass(frozen=True)
class Extrusion:
    """One structural beam or plate of a Vention assembly.

    ``size_m`` is (length, width, height) — full extents along the
    extrusion's local x, y, z axes. ``X_world`` places the centroid of
    the box in world frame; rotate it to align the extrusion's long
    axis (local x) with the direction of the beam.
    """

    name: str
    size_m: Tuple[float, float, float]
    X_world: RigidTransform
    color: np.ndarray = field(default_factory=lambda: _VENTION_GREY.copy())


def _post(
    name: str,
    cross_section: Tuple[float, float],
    height: float,
    xy: Tuple[float, float],
    z_bottom: float,
) -> Extrusion:
    """Vertical post with bottom-face at ``z_bottom`` and centre at ``xy``."""
    cx, cy = cross_section
    return Extrusion(
        name=name,
        size_m=(cx, cy, height),
        X_world=RigidTransform([xy[0], xy[1], z_bottom + height * 0.5]),
    )


def _beam_x(
    name: str,
    cross_section: Tuple[float, float],
    length: float,
    centre: Tuple[float, float, float],
) -> Extrusion:
    """Horizontal beam aligned with world +x."""
    cx, cy = cross_section
    return Extrusion(
        name=name,
        size_m=(length, cx, cy),
        X_world=RigidTransform(centre),
    )


def _beam_y(
    name: str,
    cross_section: Tuple[float, float],
    length: float,
    centre: Tuple[float, float, float],
) -> Extrusion:
    """Horizontal beam aligned with world +y."""
    cx, cy = cross_section
    return Extrusion(
        name=name,
        size_m=(cx, length, cy),
        X_world=RigidTransform(centre),
    )


def _plate(
    name: str,
    size_xy: Tuple[float, float],
    thickness: float,
    centre_xy: Tuple[float, float],
    z_top: float,
) -> Extrusion:
    """Flat plate with top face at ``z_top``."""
    return Extrusion(
        name=name,
        size_m=(size_xy[0], size_xy[1], thickness),
        X_world=RigidTransform([centre_xy[0], centre_xy[1], z_top - thickness * 0.5]),
        color=_VENTION_PLATE,
    )


def _beam_between(
    name: str,
    p_start,
    p_end,
    cross_section: Tuple[float, float],
    color=None,
) -> Extrusion:
    """Beam between two points, with arbitrary tilt.

    The box's long axis (local +x) runs from ``p_start`` to ``p_end``;
    its length equals ``|p_end - p_start|``. ``cross_section`` is the
    pair (y-extent, perpendicular-thickness) — the perpendicular axis
    is whatever direction completes a right-handed frame after the
    box's local +y is taken to be the world +y projected perpendicular
    to the long axis.

    This convention covers every slanted member in the 212 Vention
    stand: each slope lies in the x-z plane, so local +y == world +y
    cleanly. For a beam whose long axis is parallel to world +y the
    rule is undefined and the helper raises.
    """
    if color is None:
        color = _VENTION_GREY.copy()
    p_start = np.asarray(p_start, dtype=float)
    p_end = np.asarray(p_end, dtype=float)
    delta = p_end - p_start
    length = float(np.linalg.norm(delta))
    if length < 1e-9:
        raise ValueError(f"_beam_between: zero-length beam {name!r}")
    centre = 0.5 * (p_start + p_end)

    new_x = delta / length
    new_y = np.array([0.0, 1.0, 0.0]) - float(new_x[1]) * new_x
    new_y_norm = float(np.linalg.norm(new_y))
    if new_y_norm < 1e-6:
        raise ValueError(
            f"_beam_between: long axis of {name!r} is parallel to world +y; "
            "local +y is undefined under the world-+y projection rule."
        )
    new_y /= new_y_norm
    new_z = np.cross(new_x, new_y)
    R = np.column_stack([new_x, new_y, new_z])

    return Extrusion(
        name=name,
        size_m=(length, float(cross_section[0]), float(cross_section[1])),
        X_world=RigidTransform(RotationMatrix(R), centre),
        color=color,
    )


# ---------------------------------------------------------------------------
#  Default bimanual stand — STARTER VALUES, calibrate against your CAD.
# ---------------------------------------------------------------------------

# ---------------------------------------------------------------------------
#  CAD geometry — measured at the 212 lab (latest pass: 2026-05-03).
# ---------------------------------------------------------------------------
# Five layers, no bottom beam — the lower trap sits directly on the table.
# All values in metres.
#
# Heights (z direction):
_H_LOWER_TRAP    = 0.3592    # sqrt(50.5² − ((80 − 9)/2)²) cm — fans wider going DOWN
_H_MID_BEAM      = 0.325     # 32.5 cm — vertical neck
_H_UPPER_TRAP    = 0.16544   # sqrt(22.5² − ((49 − 18.5)/2)²) cm — fans wider going UP
_H_TOP_BEAM      = 0.075     # 7.5 cm  — caps the top

# y-direction depth (uniform across all layers in the new measurements):
_THICK_Y         = 0.222     # 22.2 cm — both sections share the same y depth

# x-direction lengths:
_LOWER_TOP_X     = 0.09      # 9 cm  — x length at top of lower trap (narrow)
_LOWER_BOT_X     = 0.80      # 80 cm — x length at bottom of lower trap (wide)
_MID_X           = 0.09      # 9 cm  — x length of vertical mid beam
_UPPER_TOP_X     = 0.49      # 49 cm — x length at top of upper trap (wide)
_UPPER_BOT_X     = 0.185     # 18.5 cm — x length at bottom of upper trap (narrow)
_TOP_X           = 0.56      # 56 cm — x length of top beam

# Slab thickness for the four slanted trap faces. The real trap is
# solid; we model only its slanted faces as ~20 mm slabs (interior
# hollow). The upper-trap slabs span the full 22.2 cm y-depth, so
# arm collisions with the trap are caught even though the interior
# isn't filled.
_TRAP_SLAB_THICK = 0.020

# Centre of the stand in world (= task) frame. Anchored to the
# calibration's arm-base y (−0.3825) so the arm welds land on the
# upper-trap slope midpoint (within a few mm). The stand centre y
# differs from the geometrically-natural "stand on operator-edge of
# table" placement (≈ −0.274) by ~11 cm — the same calibration vs
# CAD mismatch flagged elsewhere; reconcile in person.
_PLATE_CENTRE = (0.0, -0.3825)

# Top of the stand in world z. With the lower trap sitting flush on
# the table top at z = 0, the top of the top beam lands at:
#     0 + 0.3592 + 0.325 + 0.16544 + 0.075 ≈ 0.9246 m
# Upper-trap slope midpoint sits ~14 mm above the calibrated arm
# base z (0.753) — measurement-error scale, see header comment.
_TOP_PLATE_TOP_Z = (
    _H_LOWER_TRAP + _H_MID_BEAM + _H_UPPER_TRAP + _H_TOP_BEAM
)


def _default_bimanual_stand() -> List[Extrusion]:
    """The 212 lab Vention rig — 6 boxes from CAD measurements.

    Layout (front view, looking from operator side toward microwave):

        ┌──────────────────────┐                  z = 0.925 (top of stand)
        │      TOP BEAM        │
        └─┬──────────────────┬─┘                  z = 0.850
          ╲   upper trap    ╱
           ╲  fans wider   ╱                      arms weld on slanted faces
            ╲   going UP  ╱                       at midpoint of slope
             ╲___________╱                        z = 0.684
             │ MID BEAM  │
             │  9 × 22.2 │                        vertical neck
             │___________│                        z = 0.359
              ╱         ╲
             ╱           ╲
            ╱  lower trap ╲                       fans wider going DOWN
           ╱ fans wider    ╲
          ╱  going DOWN     ╲
         ╱___________________╲                    z = 0.000  (workspace table top)

    Both trapezoids slant in the x direction only — their flat faces
    point toward operator (−y) and toward microwave (+y), so the rig
    looks like a column from the side. Front-back stability comes from
    the lower trap being bolted directly to the table top.

    Six boxes total: 2 slanted slabs per trapezoid + the mid beam +
    the top beam. Each slanted slab is a 20 mm thick approximation of
    the trap's slanted face. The trap interiors are hollow; that's
    fine because the upper-trap interior is occupied by the arm bases
    and the lower-trap interior sits below the workspace.

    Anchored to ``_PLATE_CENTRE = (0, -0.3825)`` in task frame, which
    is the calibration's arm-base y. The stand visually overhangs the
    operator-side edge of the workspace table by ~11 cm; that's the
    calibration vs CAD mismatch flagged in the header.
    """
    cx, cy = _PLATE_CENTRE

    # z of each boundary between layers (no bottom beam — lower trap
    # sits flush on the table top).
    z0 = 0.0
    z1 = z0 + _H_LOWER_TRAP       # top of lower trap (= bottom of mid beam)
    z2 = z1 + _H_MID_BEAM         # top of mid beam   (= bottom of upper trap)
    z3 = z2 + _H_UPPER_TRAP       # top of upper trap (= bottom of top beam)
    z4 = z3 + _H_TOP_BEAM         # top of top beam

    # Lower trap: top edge 9 cm, bottom edge 80 cm, fanning along x.
    lower_trap_pos = _beam_between(
        "lower_trap_+x",
        p_start=(cx + _LOWER_TOP_X * 0.5, cy, z1),    # top-inner
        p_end=(cx + _LOWER_BOT_X * 0.5, cy, z0),      # bottom-outer
        cross_section=(_THICK_Y, _TRAP_SLAB_THICK),
    )
    lower_trap_neg = _beam_between(
        "lower_trap_-x",
        p_start=(cx - _LOWER_TOP_X * 0.5, cy, z1),
        p_end=(cx - _LOWER_BOT_X * 0.5, cy, z0),
        cross_section=(_THICK_Y, _TRAP_SLAB_THICK),
    )

    mid_beam = Extrusion(
        name="mid_beam",
        size_m=(_MID_X, _THICK_Y, _H_MID_BEAM),
        X_world=RigidTransform([cx, cy, 0.5 * (z1 + z2)]),
    )

    # Upper trap: top edge 49 cm, bottom edge 18.5 cm, fanning along x.
    # Fan direction inverts vs lower trap: wide at TOP this time.
    # Arms weld at the geometric midpoint of each slanted face.
    upper_trap_pos = _beam_between(
        "upper_trap_+x",
        p_start=(cx + _UPPER_TOP_X * 0.5, cy, z3),    # top-outer
        p_end=(cx + _UPPER_BOT_X * 0.5, cy, z2),      # bottom-inner
        cross_section=(_THICK_Y, _TRAP_SLAB_THICK),
    )
    upper_trap_neg = _beam_between(
        "upper_trap_-x",
        p_start=(cx - _UPPER_TOP_X * 0.5, cy, z3),
        p_end=(cx - _UPPER_BOT_X * 0.5, cy, z2),
        cross_section=(_THICK_Y, _TRAP_SLAB_THICK),
    )

    top_beam = Extrusion(
        name="top_beam",
        size_m=(_TOP_X, _THICK_Y, _H_TOP_BEAM),
        X_world=RigidTransform([cx, cy, 0.5 * (z3 + z4)]),
        color=_VENTION_PLATE,
    )

    return [
        lower_trap_pos, lower_trap_neg,
        mid_beam,
        upper_trap_pos, upper_trap_neg,
        top_beam,
    ]


BIMANUAL_STAND: List[Extrusion] = _default_bimanual_stand()


# ---------------------------------------------------------------------------
#  Plumbing into MultibodyPlant
# ---------------------------------------------------------------------------

@dataclass
class VentionHandles:
    model_instance: ModelInstanceIndex
    top_plate_top_z: float
    """World-frame Z of the top plate's top surface — where arm bases weld."""


def add_vention_stand(
    plant: MultibodyPlant,
    extrusions: List[Extrusion] = BIMANUAL_STAND,
    *,
    name: str = "vention",
) -> VentionHandles:
    """Add every ``Extrusion`` in ``extrusions`` as a welded box.

    Each box gets visual + collision geometry and is welded directly to
    the world body — the stand has no joints. Returns a handle whose
    ``top_plate_top_z`` is the surface to weld arm bases against.
    """
    model_instance = plant.AddModelInstance(name)
    world_body = plant.world_body()
    world_frame = plant.world_frame()

    for ext in extrusions:
        body = plant.AddRigidBody(
            ext.name,
            model_instance,
        )
        # Weld each beam to world (a Vention frame is rigid).
        plant.WeldFrames(world_frame, body.body_frame(), ext.X_world)

        box = Box(*ext.size_m)
        plant.RegisterVisualGeometry(
            body, RigidTransform(), box, f"{ext.name}_visual", ext.color,
        )
        plant.RegisterCollisionGeometry(
            body, RigidTransform(), box, f"{ext.name}_collision", _NO_FRICTION,
        )

    return VentionHandles(
        model_instance=model_instance,
        top_plate_top_z=_TOP_PLATE_TOP_Z,
    )
