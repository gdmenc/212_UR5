"""CLI entry point: open the bimanual scene in Meshcat.

Usage::

    python3.11 -m control_scripts.planning.visualize
    python3.11 -m control_scripts.planning.visualize --no-microwave
    python3.11 -m control_scripts.planning.visualize --duration 60
    python3.11 -m control_scripts.planning.visualize --show-collision

Opens a Meshcat browser window (URL printed to stdout). Holds the
scene open for ``--duration`` seconds, then exits cleanly.

By default only the **illustration** role is rendered (the solid
visual geometry). Pass ``--show-collision`` to also render proximity
geometry as green wireframes — useful when verifying that collision
boxes line up with the visible mesh, but expensive: it doubles the
geometry uploaded to Meshcat.

First milestone: confirm the geometry is correct (Vention truss is
true-to-shape, arms sit on the top plate, microwave is in the right
place).
"""

from __future__ import annotations

import argparse
import time

from pydrake.geometry import (
    MeshcatVisualizer,
    MeshcatVisualizerParams,
    Role,
    StartMeshcat,
)
from pydrake.multibody.plant import MultibodyPlant
from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import DiagramBuilder

from .build_scene import build_scene


def _build_visualized_scene(
    meshcat,
    *,
    include_microwave: bool,
    show_collision: bool,
):
    """Like ``build_scene`` but with a Meshcat visualizer wired in.

    We can't reuse ``build_scene`` directly because it calls
    ``builder.Build()`` before any visualizer is attached. The minor
    duplication is worth keeping ``build_scene`` planner-only.

    Adds an illustration-role visualizer; if ``show_collision`` is true,
    additionally adds a proximity-role visualizer (green wireframes).
    Skips the contact / inertia visualizers that ``AddDefaultVisualization``
    would otherwise wire in — those just slow the browser render and are
    not useful for static scene preview.
    """
    from pydrake.multibody.plant import AddMultibodyPlantSceneGraph

    from .scene.arms import add_both_arms
    from .scene.grippers import add_grippers
    from .scene.microwave import add_microwave
    from .scene.tables import add_workspace_table
    from .scene.vention import add_vention_stand

    builder = DiagramBuilder()
    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=0.0)

    add_workspace_table(plant)
    add_vention_stand(plant)
    arms = add_both_arms(plant)
    add_grippers(plant, arms, robotiq_mode="closed")
    if include_microwave:
        add_microwave(plant)
    plant.Finalize()

    # Match build_scene(): start at sim HOME instead of all-zero joints
    # so the Meshcat preview shows the actual park pose.
    from . import default_home_q
    plant.SetDefaultPositions(default_home_q(plant))

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
    return diagram, plant


def main() -> None:
    ap = argparse.ArgumentParser(
        description="Render the bimanual planning scene in Meshcat."
    )
    ap.add_argument(
        "--no-microwave",
        action="store_true",
        help="Skip the microwave (useful while debugging Vention placement).",
    )
    ap.add_argument(
        "--show-collision",
        action="store_true",
        help=(
            "Also render collision geometry as green wireframes. "
            "Useful for verifying box placement; doubles the geometry "
            "Meshcat has to draw."
        ),
    )
    ap.add_argument(
        "--duration",
        type=float,
        default=600.0,
        help="Seconds to keep the Meshcat session open (default 10 min).",
    )
    args = ap.parse_args()

    meshcat = StartMeshcat()
    diagram, plant = _build_visualized_scene(
        meshcat,
        include_microwave=not args.no_microwave,
        show_collision=args.show_collision,
    )

    # Step the diagram once so geometry is published.
    simulator = Simulator(diagram)
    simulator.Initialize()
    diagram.ForcedPublish(simulator.get_context())

    print()
    print("=" * 60)
    print("  bodies in plant   :", plant.num_bodies())
    print("  positions (DOF)   :", plant.num_positions())
    print("  collision pairs   :", "registered (run a query to count)")
    print(f"  Meshcat URL       : {meshcat.web_url()}")
    print("=" * 60)
    print(f"\nHolding scene open for {args.duration:.0f} s — Ctrl-C to exit.")

    try:
        t_end = time.time() + args.duration
        while time.time() < t_end:
            time.sleep(0.5)
    except KeyboardInterrupt:
        print("\ninterrupted — closing.")


if __name__ == "__main__":
    main()
