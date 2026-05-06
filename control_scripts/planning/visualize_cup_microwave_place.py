"""Preview the candidate cup-with-stick place pose on top of the microwave.

One-shot Meshcat render. Strips the table objects to just the
``cup_with_stick`` (welded at the candidate pose) and the microwave so
the geometry is easy to eyeball. A red marker sits at the microwave's
right-front outer-top corner; a green marker sits at the cup-base pose.

Usage::

    python3.11 -m control_scripts.planning.visualize_cup_microwave_place
    python3.11 -m control_scripts.planning.visualize_cup_microwave_place --variant centre
"""

from __future__ import annotations

import argparse
import time

from pydrake.geometry import Rgba, Sphere, StartMeshcat
from pydrake.math import RigidTransform
from pydrake.systems.analysis import Simulator

from ..lab_landmarks import CUP_MICROWAVE_TOP_XYZ_TASK
from .build_scene import build_scene


# Right-front corner of the microwave outer top (max +x, min y), task frame.
# Matches the math in planning/scene/microwave.py (cavity centre + wall-
# thickness asymmetry + outer height).
RIGHT_FRONT_CORNER_TASK = (0.060, 0.375, 0.280)

# 3 cm wall-clearance: cup outer wall sits 3 cm clear of each microwave edge.
CUP_PLACE_WALL_CLEARANCE = tuple(CUP_MICROWAVE_TOP_XYZ_TASK)

# 3 cm centre-offset: cup centre 3 cm from each edge — cup body overhangs by
# (CUP_RADIUS − 0.030) ≈ 1.4 cm on the +x and −y edges. A/B comparison
# variant; not the canonical pose.
CUP_PLACE_CENTRE_OFFSET = (0.030, 0.405, 0.280)


def main() -> None:
    ap = argparse.ArgumentParser(
        description="Preview the cup-with-stick place pose on the microwave top."
    )
    ap.add_argument(
        "--variant",
        choices=["wall", "centre"],
        default="wall",
        help=(
            "Which 3 cm interpretation to render. 'wall' (default): cup wall "
            "sits 3 cm clear of each microwave edge — recommended. 'centre': "
            "cup centre 3 cm from each edge (cup body overhangs ~1.4 cm)."
        ),
    )
    ap.add_argument(
        "--duration",
        type=float,
        default=600.0,
        help="Seconds to keep the Meshcat session open (default 10 min).",
    )
    args = ap.parse_args()

    cup_xyz = (CUP_PLACE_WALL_CLEARANCE if args.variant == "wall"
               else CUP_PLACE_CENTRE_OFFSET)

    meshcat = StartMeshcat()
    scene = build_scene(
        include_microwave=True,
        include_objects=True,
        robotiq_mode="closed",
        # Drop everything except cup_with_stick so the rendering is clean.
        skip_static_objects=("plate", "cup", "bowl", "bottle", "tray"),
        attached_objects=(),
        object_xyz_overrides={"cup_with_stick": cup_xyz},
        microwave_door_open_angle_rad=0.0,
        meshcat=meshcat,
    )

    simulator = Simulator(scene.diagram)
    simulator.Initialize()
    scene.diagram.ForcedPublish(simulator.get_context())

    meshcat.SetObject(
        "/markers/right_front_corner",
        Sphere(0.012),
        Rgba(0.95, 0.2, 0.2, 0.9),
    )
    meshcat.SetTransform(
        "/markers/right_front_corner",
        RigidTransform(list(RIGHT_FRONT_CORNER_TASK)),
    )
    meshcat.SetObject(
        "/markers/cup_base",
        Sphere(0.010),
        Rgba(0.2, 0.95, 0.4, 0.9),
    )
    meshcat.SetTransform(
        "/markers/cup_base",
        RigidTransform(list(cup_xyz)),
    )

    print()
    print("=" * 60)
    print(f"  variant            : {args.variant}")
    print(f"  cup-base xyz_task  : {cup_xyz}")
    print(f"  right-front corner : {RIGHT_FRONT_CORNER_TASK}  (red marker)")
    print(f"  microwave top z    : 0.280")
    print(f"  Meshcat URL        : {meshcat.web_url()}")
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
