"""Single source of environment truth shared across planning, sim, and real.

A ``World`` bundles the kwargs that ``planning.build_scene.build_scene``
and ``planning.rrt.build_planning_scene`` already accept, plus the
partner-arm pin that ``plan_transit`` reads. Constructed once per task,
passed wherever a sim or planning scene is needed.

Why this exists
---------------
Today the same env values (microwave door angle, in-hand object,
partner-arm joint configuration) flow through several call sites:
each task hand-builds its planning scene, the dryruns build their own
visualization scene, and ``other_arm_q`` is passed separately to
``plan_transit``. Easy to drift. ``World`` makes those values one
object so the planner, the sim, and the rig all read from the same
fields.

Scope
-----
Object pose customization is intentionally out of scope here. Static
objects use the default poses baked into ``planning/scene/objects.py``;
``World`` owns *selection* (include/skip/in_hand) and *door angle*,
not custom static placement. Adding per-object pose overrides is a
separate change to ``scene/objects.py``.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Dict, Optional, Tuple

import numpy as np


@dataclass
class World:
    """Typed bag of env values + builders for sim/planning scenes."""

    microwave_door_open_rad: float = 0.0
    """Welded door angle. 0 = closed; ~np.pi/2 to ~7π/12 for typical open."""

    include_microwave: bool = True
    include_objects: bool = True
    robotiq_mode: str = "closed"
    skip_static_objects: Tuple[str, ...] = ()
    """Object kinds to drop from the default static placement.
    Use together with ``in_hand`` so a held object isn't double-loaded."""

    in_hand: Dict[str, Tuple[str, Optional[Any]]] = field(default_factory=dict)
    """Per-arm in-hand object: ``arm_name`` → ``(kind, X_gripper_obj_or_None)``.
    Welded into both scenes as an extra collision body. ``X`` is a
    ``pydrake.math.RigidTransform`` or ``None`` for the default mount pose
    in ``scene.objects.attach_object_to_gripper``."""

    partner_arm_q: Dict[str, np.ndarray] = field(default_factory=dict)
    """Per-arm joint configuration to pin during planning (passed as
    ``other_arm_q`` to ``plan_transit``) AND to apply to a sim plant
    context so meshcat shows the partner arm in the correct pose."""

    object_xyz_overrides: Dict[str, Tuple[float, float, float]] = field(
        default_factory=dict,
    )
    """Per-object task-frame xyz that overrides the default placement in
    ``planning/scene/objects.py``. Use this when a task wants a static
    obstacle at a non-default location — e.g. swapping out
    ``cup_with_stick`` for plain ``cup`` while keeping the obstacle in
    the same spot. Only affects objects that are actually included
    (``include_objects=True`` and not in ``skip_static_objects``)."""

    # ------------------------------------------------------------------
    #  Internal: convert in_hand dict to the tuple form build_scene uses
    # ------------------------------------------------------------------

    def _attached_tuples(self) -> tuple:
        return tuple(
            (kind, arm, X)
            for arm, (kind, X) in self.in_hand.items()
        )

    # ------------------------------------------------------------------
    #  Builders — both feed _compose_scene_fragments under the hood
    # ------------------------------------------------------------------

    def build_sim_scene(
        self,
        meshcat=None,
        *,
        show_visual: bool = True,
        show_collision: bool = False,
    ):
        """Build a meshcat-capable scene (``SceneHandles``).

        Pass ``meshcat=StartMeshcat()`` to attach a visualizer; omit for
        a headless plant suitable for FK / IK / animation without a
        browser."""
        from .planning.build_scene import build_scene

        return build_scene(
            include_microwave=self.include_microwave,
            include_objects=self.include_objects,
            robotiq_mode=self.robotiq_mode,
            microwave_door_open_angle_rad=self.microwave_door_open_rad,
            skip_static_objects=self.skip_static_objects,
            attached_objects=self._attached_tuples(),
            object_xyz_overrides=dict(self.object_xyz_overrides),
            meshcat=meshcat,
            show_visual=show_visual,
            show_collision=show_collision,
        )

    def build_planning_scene(self):
        """Build a ``RobotDiagram`` + plant for ``SceneGraphCollisionChecker``.

        Returns ``(diagram, plant, arms, grippers)`` matching
        ``planning.rrt.build_planning_scene``'s signature."""
        from .planning.rrt import build_planning_scene

        return build_planning_scene(
            include_microwave=self.include_microwave,
            include_objects=self.include_objects,
            robotiq_mode=self.robotiq_mode,
            microwave_door_open_angle_rad=self.microwave_door_open_rad,
            skip_static_objects=self.skip_static_objects,
            attached_objects=self._attached_tuples(),
            object_xyz_overrides=dict(self.object_xyz_overrides),
        )

    # ------------------------------------------------------------------
    #  Apply partner-arm pin to a plant context
    # ------------------------------------------------------------------

    def apply_partner_pin(self, plant, plant_context) -> None:
        """Set every arm listed in ``partner_arm_q`` on the given context.

        Call after ``plant.CreateDefaultContext()`` so meshcat shows the
        partner arm where it really is on the rig, and so a subsequent
        ``plant.GetPositions(plant_context)`` returns a seed q that
        already encodes the pin (handy as ``current_q`` for
        ``plan_transit``).
        """
        from .planning.transit import _arm_model_instance

        for arm_name, q in self.partner_arm_q.items():
            try:
                inst = _arm_model_instance(plant, arm_name)
            except KeyError:
                continue
            plant.SetPositions(
                plant_context, inst, np.asarray(q, dtype=float),
            )
