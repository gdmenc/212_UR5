"""Compose every scene fragment into one MultibodyPlant.

Single entry point: ``build_scene()``. Returns the diagram, plant,
scene-graph, and a dict of body / model handles needed by downstream
planners.

Fragment order:

    1. workspace table   — anchors world z = 0 at table top
    2. Vention stand     — sits on the table, provides arm-mount surface
    3. UR5e arms         — welded to world (= task) frame at calibrated bases
    4. microwave         — five-wall housing + closed door (placeholder dims)

Movable objects (cup, bowl, ...) come later via ``planning/scene/objects.py``.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Optional

from pydrake.geometry import SceneGraph
from pydrake.multibody.plant import AddMultibodyPlantSceneGraph, MultibodyPlant
from pydrake.systems.framework import Diagram, DiagramBuilder

from .scene.arms import ArmHandles, add_both_arms
from .scene.grippers import GripperHandles, add_grippers
from .scene.microwave import MicrowaveHandles, add_microwave
from .scene.tables import TableHandles, add_workspace_table
from .scene.vention import VentionHandles, add_vention_stand


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


def build_scene(
    *,
    include_microwave: bool = True,
    include_grippers: bool = True,
    robotiq_mode: str = "closed",
    time_step: float = 0.0,
) -> SceneHandles:
    """Build and finalize the bimanual rig scene.

    ``time_step=0.0`` makes the plant continuous-time (planner-friendly);
    set a positive value (e.g. 1e-3) only if you need physical simulation.
    """
    builder = DiagramBuilder()
    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=time_step)

    table = add_workspace_table(plant)
    vention = add_vention_stand(plant)
    arms = add_both_arms(plant)
    grippers = (
        add_grippers(plant, arms, robotiq_mode=robotiq_mode)
        if include_grippers else {}
    )
    microwave = add_microwave(plant) if include_microwave else None

    plant.Finalize()

    # Seed the plant's default positions at HOME so any
    # ``plant.CreateDefaultContext()`` (planner, viz, smoke tests)
    # starts from a sane pose instead of all-zero joints. Done after
    # Finalize because SetDefaultPositions touches the finalized
    # position vector layout.
    from . import default_home_q
    plant.SetDefaultPositions(default_home_q(plant))

    diagram = builder.Build()

    return SceneHandles(
        plant=plant,
        scene_graph=scene_graph,
        diagram=diagram,
        table=table,
        vention=vention,
        arms=arms,
        grippers=grippers,
        microwave=microwave,
    )
