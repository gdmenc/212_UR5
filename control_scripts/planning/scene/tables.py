"""Workspace table — programmatic boxes (4 legs + top), task origin at top.

Using boxes (rather than the existing ``extra_heavy_duty_table.sdf``)
because:
  - The SDF auto-welds its single link at a baked-in offset, fighting
    explicit ``WeldFrames`` calls.
  - The dimensions are easier to tune from one place when calibration
    needs to follow the lab.
  - Same primitive-box philosophy as Vention and microwave.

Default geometry: a 1.2 m × 0.6 m × 0.74 m table, top centred on the
task-frame origin (top surface at world z = 0).
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Tuple

import numpy as np
from pydrake.geometry import Box
from pydrake.math import RigidTransform
from pydrake.multibody.plant import CoulombFriction, MultibodyPlant
from pydrake.multibody.tree import ModelInstanceIndex


# Physical dimensions — measured at the 212 lab (latest pass: 2026-05-03).
# 'length' = 90 cm along the operator's left-right axis.
# 'depth'  = 77 cm toward the microwave.
# Height not yet measured; 0.74 m is a typical lab-bench placeholder.
_TOP_W = 0.90      # x extent (left-right)        — 90 cm
_TOP_D = 0.77      # y extent (+y toward microwave)— 77 cm
_TOP_T = 0.030     # top-slab thickness
_LEG_X = 0.040     # leg cross-section (x)        — placeholder
_LEG_Y = 0.040     # leg cross-section (y)        — placeholder
_TABLE_H = 0.740   # floor → top surface          — placeholder until measured

_LEG_INSET = 0.040 # legs offset inwards from the slab edges

# The task origin is the centre of the **55 cm clear portion** of the
# table (the part not covered by the Vention stand). The geometric
# centre of the full 77 cm table is offset by (77-55)/2 = 11 cm toward
# the operator (-y). This default lines the box up with that convention
# so the calibration constants in ``calibration.py`` (which place the
# arm bases at task y = -0.385) land correctly above the Vention end.
_VENTION_COVERAGE_Y = 0.220
_CLEAR_AREA_Y = _TOP_D - _VENTION_COVERAGE_Y         # = 0.55, lab-measured
_TABLE_CENTRE_Y_TASK = -0.5 * (_TOP_D - _CLEAR_AREA_Y)   # = -0.11

_TOP_COLOR = np.array([0.55, 0.40, 0.30, 1.0])
_LEG_COLOR = np.array([0.30, 0.30, 0.30, 1.0])
_NO_FRICTION = CoulombFriction(0.5, 0.5)


@dataclass
class TableHandles:
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


def add_workspace_table(
    plant: MultibodyPlant,
    *,
    name: str = "workspace_table",
    top_centre_world: Tuple[float, float, float] = (0.0, _TABLE_CENTRE_Y_TASK, 0.0),
    width_x: float = _TOP_W,
    depth_y: float = _TOP_D,
    height: float = _TABLE_H,
) -> TableHandles:
    """Add a 4-legged workspace table.

    Default places the table top so its top *surface* sits at world z=0
    and the **clear-area centre** (not the full-table centre) is at the
    task origin in xy. This matches the lab calibration convention —
    see the ``_TABLE_CENTRE_Y_TASK`` derivation above. The full table
    spans task y ∈ [−0.495, +0.275]; the Vention covers task y ∈
    [−0.495, −0.275]; the clear area is task y ∈ [−0.275, +0.275],
    centered on the task origin.
    """
    model_instance = plant.AddModelInstance(name)
    cx, cy, cz_top = top_centre_world

    # Top slab: centre at z = top_surface − thickness/2.
    _add_box(
        plant, model_instance, "top",
        size=(width_x, depth_y, _TOP_T),
        X_world=RigidTransform([cx, cy, cz_top - _TOP_T * 0.5]),
        color=_TOP_COLOR,
    )

    # Four legs: symmetric inset from the slab edges.
    leg_h = height - _TOP_T
    leg_z = cz_top - _TOP_T - leg_h * 0.5
    half_x = width_x * 0.5 - _LEG_INSET - _LEG_X * 0.5
    half_y = depth_y * 0.5 - _LEG_INSET - _LEG_Y * 0.5

    for sign_x, sign_y, suffix in [
        (-1, -1, "FL"), (+1, -1, "FR"), (-1, +1, "BL"), (+1, +1, "BR"),
    ]:
        _add_box(
            plant, model_instance, f"leg_{suffix}",
            size=(_LEG_X, _LEG_Y, leg_h),
            X_world=RigidTransform([cx + sign_x * half_x, cy + sign_y * half_y, leg_z]),
            color=_LEG_COLOR,
        )

    return TableHandles(model_instance=model_instance)
