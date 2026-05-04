"""Visualize recorded waypoints in the Drake/Meshcat scene.

Loads JSON snapshots from ``logs/waypoints/`` (the format written by
``examples.record_waypoints``), sets the named arm to the recorded
joint angles, and drops a sphere marker at the recorded task-frame
TCP position so you can visually verify Drake's FK matches what the
real arm reported.

Usage::

    python3.11 -m control_scripts.planning.visualize_waypoints
    python3.11 -m control_scripts.planning.visualize_waypoints --list
    python3.11 -m control_scripts.planning.visualize_waypoints \
        --snapshot "press microwave button"
    python3.11 -m control_scripts.planning.visualize_waypoints \
        --snapshot pick_plate_1 tray_edge_1
    python3.11 -m control_scripts.planning.visualize_waypoints \
        --files logs/waypoints/ur_left_20260501_005726.json
    python3.11 -m control_scripts.planning.visualize_waypoints \
        --seconds-per-pose 4

``--snapshot`` does case-insensitive substring matching against the
snapshot's ``name`` field; pass multiple to keep more than one. With a
single match the visualizer sets that pose and just holds it.

The non-active arm stays parked at the sim HOME pose. Markers are
green when Drake FK agrees with the recorded TCP within 1 cm, red
otherwise — anything red points at a bad joint→pose record or a
calibration drift bigger than expected hand-measurement noise.
"""

from __future__ import annotations

import argparse
import glob
import json
import time
from pathlib import Path
from typing import List, Optional

import numpy as np
from pydrake.geometry import (
    MeshcatVisualizer,
    MeshcatVisualizerParams,
    Rgba,
    Role,
    Sphere,
    StartMeshcat,
)
from pydrake.math import RigidTransform
from pydrake.multibody.plant import AddMultibodyPlantSceneGraph
from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import DiagramBuilder

from . import default_home_q
from .scene.arms import add_both_arms
from .scene.grippers import add_grippers
from .scene.microwave import add_microwave
from .scene.tables import add_workspace_table
from .scene.vention import add_vention_stand
from .transit import _arm_model_instance


_DEFAULT_WP_DIR = (
    Path(__file__).resolve().parents[2] / "logs" / "waypoints"
)
_TOLERANCE_M = 0.01


def _build_scene(meshcat):
    builder = DiagramBuilder()
    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=0.0)

    add_workspace_table(plant)
    add_vention_stand(plant)
    arms = add_both_arms(plant)
    add_grippers(plant, arms, robotiq_mode="closed")
    add_microwave(plant)
    plant.Finalize()
    plant.SetDefaultPositions(default_home_q(plant))

    params = MeshcatVisualizerParams()
    params.role = Role.kIllustration
    MeshcatVisualizer.AddToBuilder(builder, scene_graph, meshcat, params)

    diagram = builder.Build()
    return diagram, plant


def _set_marker(meshcat, name: str, xyz, color: Rgba, radius: float = 0.012):
    """Drop / move a small sphere at task-frame xyz."""
    meshcat.SetObject(name, Sphere(radius), color)
    meshcat.SetTransform(name, RigidTransform(np.asarray(xyz, dtype=float)))


def _collect_snapshots(file_paths: List[Path]):
    """Flatten all (file, snapshot) pairs from the given JSON files."""
    out = []
    for p in file_paths:
        d = json.loads(p.read_text())
        for s in d["snapshots"]:
            out.append((p.name, d["arm"], s))
    return out


def main(file_paths: Optional[List[Path]] = None,
         seconds_per_pose: float = 3.0,
         snapshot_filters: Optional[List[str]] = None,
         list_only: bool = False) -> int:
    if file_paths is None:
        file_paths = sorted(_DEFAULT_WP_DIR.glob("*.json"))
    if not file_paths:
        print(f"No waypoint files found in {_DEFAULT_WP_DIR}")
        return 1

    snapshots = _collect_snapshots(file_paths)
    snapshots = [s for s in snapshots if s[2].get("joints_rad")]
    if not snapshots:
        print("All waypoint files were empty (no snapshots).")
        return 1

    if snapshot_filters:
        needles = [n.lower() for n in snapshot_filters]
        snapshots = [
            t for t in snapshots
            if any(n in t[2]["name"].lower() for n in needles)
        ]
        if not snapshots:
            print(f"No snapshot names matched any of: {snapshot_filters}")
            return 1

    if list_only:
        print(f"{'#':>3}  {'file':<35} {'arm':<9}  snapshot")
        for i, (file_name, arm, snap) in enumerate(snapshots, 1):
            print(f"{i:>3}  {file_name:<35} {arm:<9}  {snap['name']}")
        return 0

    meshcat = StartMeshcat()
    diagram, plant = _build_scene(meshcat)
    sim = Simulator(diagram)
    sim.Initialize()
    plant_ctx = plant.GetMyMutableContextFromRoot(sim.get_mutable_context())

    home_q = default_home_q(plant)

    print()
    print("=" * 78)
    print(f"  Meshcat URL          : {meshcat.web_url()}")
    print(f"  Waypoint files       : {len(file_paths)}  (snapshots: {len(snapshots)})")
    print(f"  Seconds per pose     : {seconds_per_pose:.1f}")
    print(f"  Marker tolerance     : {_TOLERANCE_M*1000:.0f} mm  "
          f"(green if FK within tolerance, red otherwise)")
    print("=" * 78)
    print()
    print(f"{'#':>3}  {'file':<35} {'snapshot':<35} "
          f"{'arm':<9} {'Δ (mm)':>10}")

    for i, (file_name, arm, snap) in enumerate(snapshots, 1):
        # Reset both arms to HOME, then set the recorded arm's joints.
        plant.SetPositions(plant_ctx, home_q)
        inst = _arm_model_instance(plant, arm)
        plant.SetPositions(plant_ctx, inst, np.asarray(snap["joints_rad"]))

        # Drake-side TCP via the gripper-mounted ``tcp_left/right`` frame.
        tcp_frame = plant.GetFrameByName(f"tcp_{arm.removeprefix('ur_')}")
        drake_xyz = np.asarray(tcp_frame.CalcPoseInWorld(plant_ctx).translation())
        rec_xyz = np.asarray(snap["task_pose"]["translation"])
        delta_mm = float(np.linalg.norm(drake_xyz - rec_xyz)) * 1000

        ok = delta_mm < _TOLERANCE_M * 1000
        color = Rgba(0.1, 0.9, 0.2, 0.9) if ok else Rgba(0.95, 0.2, 0.2, 0.9)
        _set_marker(meshcat, "/recorded_tcp", rec_xyz, color)
        _set_marker(meshcat, "/drake_tcp", drake_xyz,
                    Rgba(0.2, 0.4, 0.95, 0.6), radius=0.008)

        diagram.ForcedPublish(sim.get_context())

        print(f"{i:>3}  {file_name:<35} {snap['name'][:34]:<35} "
              f"{arm:<9} {delta_mm:>9.1f}{'  OK' if ok else '  X'}")

        time.sleep(seconds_per_pose)

    print()
    print(f"Done. Holding final pose — Ctrl-C to exit.")
    try:
        while True:
            time.sleep(0.5)
    except KeyboardInterrupt:
        print("Closing.")
    return 0


if __name__ == "__main__":
    ap = argparse.ArgumentParser(
        description="Step through recorded waypoints in the Drake scene.",
    )
    ap.add_argument(
        "--files", nargs="*", type=Path, default=None,
        help=f"Specific JSON files to load. Default: all in {_DEFAULT_WP_DIR}.",
    )
    ap.add_argument(
        "--seconds-per-pose", type=float, default=3.0,
        help="How long to hold each pose before advancing to the next.",
    )
    ap.add_argument(
        "--snapshot", "-s", nargs="+", default=None,
        help=(
            "Filter to snapshots whose name contains any of these "
            "(case-insensitive substring match). With one match, the "
            "visualizer just sets the pose and holds."
        ),
    )
    ap.add_argument(
        "--list", action="store_true",
        help="Print available snapshot names and exit (no rendering).",
    )
    args = ap.parse_args()
    raise SystemExit(main(
        file_paths=args.files,
        seconds_per_pose=args.seconds_per_pose,
        snapshot_filters=args.snapshot,
        list_only=args.list,
    ))
