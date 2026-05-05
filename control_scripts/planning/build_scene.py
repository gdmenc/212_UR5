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
)
from .scene.tables import TableHandles, add_workspace_table
from .scene.vention import VentionHandles, add_vention_stand


@dataclass
class SceneFragments:
    """Handles returned by ``_compose_scene_fragments``."""

    table: TableHandles
    vention: VentionHandles
    arms: dict[str, ArmHandles]
    grippers: dict[str, GripperHandles]
    microwave: Optional[MicrowaveHandles] = None
    objects: dict[str, ObjectHandles] = field(default_factory=dict)


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


def _compose_scene_fragments(
    plant: MultibodyPlant,
    *,
    include_microwave: bool,
    include_grippers: bool,
    include_objects: bool,
    robotiq_mode: str,
) -> SceneFragments:
    """Add every welded body / fixture to ``plant``.

    Caller is responsible for ``Finalize`` and any ``Build``. Pulling
    this out lets both ``build_scene`` (DiagramBuilder + optional
    Meshcat) and ``rrt.build_planning_scene`` (RobotDiagramBuilder)
    share one composition path so the scene can't drift between them.
    """
    table = add_workspace_table(plant)
    vention = add_vention_stand(plant)
    arms = add_both_arms(plant)
    grippers = (
        add_grippers(plant, arms, robotiq_mode=robotiq_mode)
        if include_grippers else {}
    )
    microwave = add_microwave(plant) if include_microwave else None

    objects: dict[str, ObjectHandles] = {}
    if include_objects:
        objects["plate"] = add_plate(plant)
        objects["cup"] = add_cup(plant)
        objects["cup_with_stick"] = add_cup(plant, with_stick=True)
        objects["bowl"] = add_bowl(plant)
        objects["bottle"] = add_bottle(plant)
        objects["tray"] = add_tray(plant)

    return SceneFragments(
        table=table,
        vention=vention,
        arms=arms,
        grippers=grippers,
        microwave=microwave,
        objects=objects,
    )


def build_scene(
    *,
    include_microwave: bool = True,
    include_grippers: bool = True,
    include_objects: bool = True,
    robotiq_mode: str = "closed",
    time_step: float = 0.0,
    meshcat=None,
    show_collision: bool = False,
) -> SceneHandles:
    """Build and finalize the bimanual rig scene.

    ``time_step=0.0`` makes the plant continuous-time (planner-friendly);
    set a positive value (e.g. 1e-3) only if you need physical simulation.

    ``meshcat`` (optional): an instance from ``StartMeshcat()``. When
    supplied, an illustration-role visualizer is attached before the
    diagram is built so the visualizer captures every fragment. Set
    ``show_collision=True`` to also attach a proximity-role visualizer
    (green wireframes for collision geometry).
    """
    builder = DiagramBuilder()
    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=time_step)

    fragments = _compose_scene_fragments(
        plant,
        include_microwave=include_microwave,
        include_grippers=include_grippers,
        include_objects=include_objects,
        robotiq_mode=robotiq_mode,
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
        illustration_params = MeshcatVisualizerParams()
        illustration_params.role = Role.kIllustration
        MeshcatVisualizer.AddToBuilder(
            builder, scene_graph, meshcat, illustration_params,
        )
        if show_collision:
            proximity_params = MeshcatVisualizerParams()
            proximity_params.role = Role.kProximity
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
    )
