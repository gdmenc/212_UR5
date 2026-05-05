"""Compose every scene fragment into one MultibodyPlant.

Single entry point: ``build_scene()``. Returns the diagram, plant,
scene-graph, and a dict of body / model handles needed by downstream
planners.

Fragment order:

    1. workspace table     — anchors world z = 0 at table top
    2. Vention stand       — sits on the table, provides arm-mount surface
    3. UR5e arms           — welded to world (= task) frame at calibrated bases
    4. grippers            — welded to each arm's wrist
    5. microwave           — five-wall housing + closed door (placeholder dims)
    6. tabletop objects    — plate, cup(s), bowl, bottle, tray (collision only)

The actual body adds are factored into ``_compose_scene_fragments`` so
``rrt.build_planning_scene`` (which uses ``RobotDiagramBuilder``) can
share the same composition path. ``build_scene`` itself uses a regular
``DiagramBuilder`` and optionally attaches a Meshcat visualizer so all
visualizer scripts can call it directly instead of duplicating the
fragment list inline.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Optional

from pydrake.geometry import (
    MeshcatVisualizer,
    MeshcatVisualizerParams,
    Role,
    SceneGraph,
)
from pydrake.multibody.plant import AddMultibodyPlantSceneGraph, MultibodyPlant
from pydrake.systems.framework import Diagram, DiagramBuilder

from pydrake.math import RigidTransform

from .scene.arms import ArmHandles, add_both_arms
from .scene.grippers import GripperHandles, add_grippers
from .scene.microwave import MicrowaveHandles, add_microwave
from .scene.objects import (
    ObjectHandles,
    add_bottle,
    add_bowl,
    add_cup,
    add_plate,
    add_tray,
    attach_object_to_gripper,
)
from .scene.tables import TableHandles, add_workspace_table
from .scene.vention import VentionHandles, add_vention_stand


# (kind, arm_name, X_gripper_obj_or_None) — see attach_object_to_gripper.
AttachedObjectSpec = tuple  # Tuple[str, str, Optional[RigidTransform]]


@dataclass
class SceneFragments:
    """Handles returned by ``_compose_scene_fragments``."""

    table: TableHandles
    vention: VentionHandles
    arms: dict[str, ArmHandles]
    grippers: dict[str, GripperHandles]
    microwave: Optional[MicrowaveHandles] = None
    objects: dict[str, ObjectHandles] = field(default_factory=dict)
    attached_objects: dict[str, ObjectHandles] = field(default_factory=dict)


@dataclass
class SceneHandles:
    """Everything a planner needs to address parts of the scene by name."""

    plant: MultibodyPlant
    scene_graph: SceneGraph
    diagram: Diagram

    table: TableHandles
    vention: VentionHandles
    arms: dict[str, ArmHandles]
    grippers: dict[str, GripperHandles]
    microwave: Optional[MicrowaveHandles] = None
    objects: dict[str, ObjectHandles] = field(default_factory=dict)
    attached_objects: dict[str, ObjectHandles] = field(default_factory=dict)


_DEFAULT_OBJECT_KINDS = (
    "plate", "cup", "cup_with_stick", "bowl", "bottle", "tray",
)


def _compose_scene_fragments(
    plant: MultibodyPlant,
    *,
    include_microwave: bool,
    include_grippers: bool,
    include_objects: bool,
    robotiq_mode: str,
    microwave_door_open_angle_rad: float = 0.0,
    skip_static_objects: tuple = (),
    attached_objects: tuple = (),
) -> SceneFragments:
    """Add every welded body / fixture to ``plant``.

    Caller is responsible for ``Finalize`` and any ``Build``. Pulling
    this out lets both ``build_scene`` (DiagramBuilder + optional
    Meshcat) and ``rrt.build_planning_scene`` (RobotDiagramBuilder)
    share one composition path so the scene can't drift between them.

    ``skip_static_objects`` (tuple of kind names) suppresses default
    static adds — use this in conjunction with ``attached_objects`` to
    say "the bowl is no longer on the table because we picked it up".

    ``attached_objects`` is an iterable of ``(kind, arm_name,
    X_gripper_obj_or_None)`` tuples. Each entry welds an extra copy of
    that object kind to the named arm's gripper body for collision
    purposes. ``X_gripper_obj=None`` uses the hardcoded default pose
    from ``attach_object_to_gripper``.
    """
    table = add_workspace_table(plant)
    vention = add_vention_stand(plant)
    arms = add_both_arms(plant)
    grippers = (
        add_grippers(plant, arms, robotiq_mode=robotiq_mode)
        if include_grippers else {}
    )
    microwave = (
        add_microwave(plant, door_open_angle_rad=microwave_door_open_angle_rad)
        if include_microwave else None
    )

    skip = set(skip_static_objects)
    objects: dict[str, ObjectHandles] = {}
    if include_objects:
        if "plate" not in skip:
            objects["plate"] = add_plate(plant)
        if "cup" not in skip:
            objects["cup"] = add_cup(plant)
        if "cup_with_stick" not in skip:
            objects["cup_with_stick"] = add_cup(plant, with_stick=True)
        if "bowl" not in skip:
            objects["bowl"] = add_bowl(plant)
        if "bottle" not in skip:
            objects["bottle"] = add_bottle(plant)
        if "tray" not in skip:
            objects["tray"] = add_tray(plant)

    attached: dict[str, ObjectHandles] = {}
    for entry in attached_objects:
        kind, arm_name, X = entry
        handle = attach_object_to_gripper(
            plant, kind, arm_name, X_gripper_obj=X,
        )
        # Keyed by the model-instance name so multiple of the same kind
        # across arms don't collide.
        key = f"{kind}_in_hand_{arm_name}"
        attached[key] = handle

    return SceneFragments(
        table=table,
        vention=vention,
        arms=arms,
        grippers=grippers,
        microwave=microwave,
        objects=objects,
        attached_objects=attached,
    )


def build_scene(
    *,
    include_microwave: bool = True,
    include_grippers: bool = True,
    include_objects: bool = True,
    robotiq_mode: str = "closed",
    microwave_door_open_angle_rad: float = 0.0,
    skip_static_objects: tuple = (),
    attached_objects: tuple = (),
    time_step: float = 0.0,
    meshcat=None,
    show_visual: bool = True,
    show_collision: bool = False,
) -> SceneHandles:
    """Build and finalize the bimanual rig scene.

    ``time_step=0.0`` makes the plant continuous-time (planner-friendly);
    set a positive value (e.g. 1e-3) only if you need physical simulation.

    ``microwave_door_open_angle_rad``: 0 (default) builds the microwave
    with the door closed. Pass ``np.pi/2`` (90°) or ``≈ 7π/12`` (105°)
    to weld the door at an open angle for planning into an open cavity.
    Static at scene-build time — for door-state changes online, build
    a separate scene.

    Visualizer flags (no-ops when ``meshcat`` is None):

      ``show_visual``    (default True)  — attach an illustration-role
        visualizer that renders the URDF visual meshes. Looks pretty but
        ships ~12 MB of UR5e geometry per arm to the browser.
      ``show_collision`` (default False) — attach a proximity-role
        visualizer that renders the collision geometry (pre-decimated
        OBJs in ``ur_description/meshes/ur5e/collision`` — ~27× lighter
        than the visuals). Useful as a debug overlay alongside the
        visuals.

    Set ``show_visual=False, show_collision=True`` to render arms as
    lightweight collision bodies only — Meshcat-friendly when the
    full visual mesh count makes the browser sluggish.
    """
    builder = DiagramBuilder()
    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=time_step)

    fragments = _compose_scene_fragments(
        plant,
        include_microwave=include_microwave,
        include_grippers=include_grippers,
        include_objects=include_objects,
        robotiq_mode=robotiq_mode,
        microwave_door_open_angle_rad=microwave_door_open_angle_rad,
        skip_static_objects=skip_static_objects,
        attached_objects=attached_objects,
    )

    plant.Finalize()

    # Seed the plant's default positions at HOME so any
    # ``plant.CreateDefaultContext()`` (planner, viz, smoke tests)
    # starts from a sane pose instead of all-zero joints. Done after
    # Finalize because SetDefaultPositions touches the finalized
    # position vector layout.
    from . import default_home_q
    plant.SetDefaultPositions(default_home_q(plant))

    if meshcat is not None:
        if show_visual:
            illustration_params = MeshcatVisualizerParams()
            illustration_params.role = Role.kIllustration
            MeshcatVisualizer.AddToBuilder(
                builder, scene_graph, meshcat, illustration_params,
            )
        if show_collision:
            proximity_params = MeshcatVisualizerParams()
            proximity_params.role = Role.kProximity
            # Only nest under "collision" if the illustration layer is
            # also present (otherwise users have to dig through an
            # extra prefix path to find the only geometry that's there).
            if show_visual:
                proximity_params.prefix = "collision"
            MeshcatVisualizer.AddToBuilder(
                builder, scene_graph, meshcat, proximity_params,
            )

    diagram = builder.Build()

    return SceneHandles(
        plant=plant,
        scene_graph=scene_graph,
        diagram=diagram,
        table=fragments.table,
        vention=fragments.vention,
        arms=fragments.arms,
        grippers=fragments.grippers,
        microwave=fragments.microwave,
        objects=fragments.objects,
        attached_objects=fragments.attached_objects,
    )
