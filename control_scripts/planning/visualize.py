"""CLI entry point: open the bimanual scene in Meshcat.

Usage::

    python3.11 -m control_scripts.planning.visualize
    python3.11 -m control_scripts.planning.visualize --no-microwave
    python3.11 -m control_scripts.planning.visualize --no-objects
    python3.11 -m control_scripts.planning.visualize --duration 60
    python3.11 -m control_scripts.planning.visualize --show-collision

Opens a Meshcat browser window (URL printed to stdout). Holds the
scene open for ``--duration`` seconds, then exits cleanly.

By default only the **illustration** role is rendered (the solid
visual geometry). Pass ``--show-collision`` to also render proximity
geometry as green wireframes — useful when verifying that collision
boxes line up with the visible mesh, but expensive: it doubles the
geometry uploaded to Meshcat.
"""

from __future__ import annotations

import argparse
import time

from pydrake.geometry import StartMeshcat
from pydrake.systems.analysis import Simulator

from .build_scene import build_scene


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
        "--no-objects",
        action="store_true",
        help="Skip tabletop objects (plate/cup/bowl/bottle/tray).",
    )
    ap.add_argument(
        "--gripper-mode",
        choices=["closed", "open"],
        default="closed",
        help="Robotiq 2F-85 finger configuration to load (default: closed).",
    )
    ap.add_argument(
        "--in-hand",
        action="append",
        default=[],
        metavar="KIND:ARM",
        help=(
            "Weld a copy of object KIND to ARM's gripper instead of "
            "placing it on the table. Repeat for multiple. "
            "Example: --in-hand bowl:ur_right --in-hand cup:ur_left"
        ),
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
        "--microwave-door-open-deg",
        type=float,
        default=0.0,
        help=(
            "Weld the microwave door at the given open angle (degrees). "
            "0 = closed (default). 90 = perpendicular to front face. "
            "~105 = typical mechanical hinge stop. The door swings "
            "toward the operator (-y) about the outer-left edge."
        ),
    )
    ap.add_argument(
        "--duration",
        type=float,
        default=600.0,
        help="Seconds to keep the Meshcat session open (default 10 min).",
    )
    args = ap.parse_args()

    # Parse --in-hand pairs (kind:arm) into the build_scene kwargs.
    attached: list = []
    skip: list = []
    for raw in args.in_hand:
        if ":" not in raw:
            ap.error(f"--in-hand requires KIND:ARM, got {raw!r}")
        kind, arm = raw.split(":", 1)
        attached.append((kind, arm, None))   # None → default mount pose
        # Also drop the default static placement so we don't have two
        # copies of the same kind in the scene.
        if kind not in skip:
            skip.append(kind)

    import math

    meshcat = StartMeshcat()
    scene = build_scene(
        include_microwave=not args.no_microwave,
        include_objects=not args.no_objects,
        robotiq_mode=args.gripper_mode,
        skip_static_objects=tuple(skip),
        attached_objects=tuple(attached),
        microwave_door_open_angle_rad=math.radians(args.microwave_door_open_deg),
        meshcat=meshcat,
        show_collision=args.show_collision,
    )

    # Step the diagram once so geometry is published.
    simulator = Simulator(scene.diagram)
    simulator.Initialize()
    scene.diagram.ForcedPublish(simulator.get_context())

    print()
    print("=" * 60)
    print("  bodies in plant   :", scene.plant.num_bodies())
    print("  positions (DOF)   :", scene.plant.num_positions())
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
