"""Visualize recorded waypoints in the Drake/Meshcat scene.

Loads JSON snapshots from ``logs/waypoints/`` (the format written by
``examples.record_waypoints``), sets the named arm to the recorded
joint angles, and drops a sphere marker at the recorded task-frame
TCP position so you can visually verify Drake's FK matches what the
real arm reported.

Usage::

    python3.11 -m control_scripts.planning.visualize_waypoints           # interactive (default)
    python3.11 -m control_scripts.planning.visualize_waypoints --list
    python3.11 -m control_scripts.planning.visualize_waypoints --snapshot "press"
    python3.11 -m control_scripts.planning.visualize_waypoints --cycle --seconds-per-pose 4

Default mode is **interactive**: the scene loads once (one mesh upload
to the browser), then a slider + Prev/Next buttons in the Meshcat
controls panel let you flip between snapshots without restarting the
process. Switching poses is then near-instant — just SetPositions and
a publish, no geometry re-upload.

Use ``--cycle`` for the old auto-advance behaviour. ``--snapshot``
filters the loaded set in either mode (case-insensitive substring
match on the snapshot ``name``).

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
    Cylinder,
    MeshcatVisualizer,
    MeshcatVisualizerParams,
    Rgba,
    Role,
    Sphere,
    StartMeshcat,
)
from pydrake.math import RigidTransform, RotationMatrix
from scipy.spatial.transform import Rotation as ScipyRotation
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
_TOLERANCE_DEG = 5.0   # rotation tolerance for "OK" tag


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


def _set_triad(meshcat, prefix: str, xyz, R_world,
               length: float = 0.06, radius: float = 0.003):
    """Draw an RGB triad at ``xyz`` whose axes follow the columns of ``R_world``.

    Drake's ``Cylinder`` sits on its own +z axis centred at origin. To
    draw the X axis (red), rotate the cylinder so its z-axis aligns
    with R_world's first column, and shift it +length/2 along that
    column so its base sits at ``xyz``.
    """
    xyz = np.asarray(xyz, dtype=float)
    colors = (Rgba(1.0, 0.2, 0.2, 0.9),
              Rgba(0.2, 1.0, 0.2, 0.9),
              Rgba(0.2, 0.4, 1.0, 0.9))
    z_hat = np.array([0.0, 0.0, 1.0])
    for axis_idx, label in enumerate("xyz"):
        axis_dir = np.asarray(R_world[:, axis_idx], dtype=float)
        # Rotation that maps z_hat → axis_dir (the cylinder is along its own z).
        R_align = _rotation_between(z_hat, axis_dir)
        center = xyz + axis_dir * (length / 2.0)
        X = RigidTransform(RotationMatrix(R_align), center)
        path = f"{prefix}/{label}"
        meshcat.SetObject(path, Cylinder(radius, length), colors[axis_idx])
        meshcat.SetTransform(path, X)


def _rotation_between(a, b) -> np.ndarray:
    """Smallest rotation that maps unit vector ``a`` onto unit vector ``b``."""
    a = a / max(np.linalg.norm(a), 1e-12)
    b = b / max(np.linalg.norm(b), 1e-12)
    c = float(np.dot(a, b))
    if c > 0.9999:
        return np.eye(3)
    if c < -0.9999:
        # 180° flip — pick any axis perpendicular to a.
        helper = np.array([1.0, 0.0, 0.0]) if abs(a[0]) < 0.9 else np.array([0.0, 1.0, 0.0])
        axis = np.cross(a, helper)
        axis /= np.linalg.norm(axis)
        K = np.array([[0, -axis[2], axis[1]],
                      [axis[2], 0, -axis[0]],
                      [-axis[1], axis[0], 0]])
        return np.eye(3) + 2 * (K @ K)
    v = np.cross(a, b)
    K = np.array([[0, -v[2], v[1]],
                  [v[2], 0, -v[0]],
                  [-v[1], v[0], 0]])
    return np.eye(3) + K + K @ K * (1.0 / (1.0 + c))


def _rotation_angle_deg(R_a: np.ndarray, R_b: np.ndarray) -> float:
    """Geodesic distance (deg) between two rotation matrices."""
    R_rel = R_a.T @ R_b
    cos_t = (np.trace(R_rel) - 1.0) / 2.0
    return float(np.degrees(np.arccos(np.clip(cos_t, -1.0, 1.0))))


def _collect_snapshots(file_paths: List[Path]):
    """Flatten all (file, snapshot) pairs from the given JSON files."""
    out = []
    for p in file_paths:
        d = json.loads(p.read_text())
        for s in d["snapshots"]:
            out.append((p.name, d["arm"], s))
    return out


def _apply_snapshot(plant, plant_ctx, diagram, sim_ctx, meshcat,
                    home_q, snap_tuple) -> tuple:
    """Set the plant to a snapshot's joint config, drop markers + triad, publish.

    Returns (delta_mm, delta_deg, ok) for caller logging. ``ok`` is true
    when both translation and rotation are within tolerance.
    """
    file_name, arm, snap = snap_tuple

    plant.SetPositions(plant_ctx, home_q)
    inst = _arm_model_instance(plant, arm)
    plant.SetPositions(plant_ctx, inst, np.asarray(snap["joints_rad"]))

    tcp_frame = plant.GetFrameByName(f"tcp_{arm.removeprefix('ur_')}")
    X_drake = tcp_frame.CalcPoseInWorld(plant_ctx)
    drake_xyz = np.asarray(X_drake.translation())
    R_drake = np.asarray(X_drake.rotation().matrix())

    rec_xyz = np.asarray(snap["task_pose"]["translation"])
    R_rec = ScipyRotation.from_rotvec(snap["task_pose"]["rotvec"]).as_matrix()

    delta_mm = float(np.linalg.norm(drake_xyz - rec_xyz)) * 1000
    delta_deg = _rotation_angle_deg(R_drake, R_rec)

    ok = (delta_mm < _TOLERANCE_M * 1000) and (delta_deg < _TOLERANCE_DEG)
    color = Rgba(0.1, 0.9, 0.2, 0.9) if ok else Rgba(0.95, 0.2, 0.2, 0.9)
    _set_marker(meshcat, "/recorded_tcp", rec_xyz, color)
    _set_marker(meshcat, "/drake_tcp", drake_xyz,
                Rgba(0.2, 0.4, 0.95, 0.6), radius=0.008)
    # Recorded TCP frame as RGB triad — eyeball this against the rendered
    # hook/gripper geometry to confirm rotation is right (not just position).
    _set_triad(meshcat, "/recorded_triad", rec_xyz, R_rec)
    _set_triad(meshcat, "/drake_triad", drake_xyz, R_drake,
               length=0.04, radius=0.0018)

    diagram.ForcedPublish(sim_ctx)
    return delta_mm, delta_deg, ok


def _run_interactive(meshcat, plant, plant_ctx, diagram, sim_ctx,
                     home_q, snapshots) -> int:
    """Hold the visualizer alive; switch snapshots via Meshcat controls.

    Wires three Meshcat widgets in the controls panel:
      * a slider ``snapshot`` (1..N step 1) for direct index pick
      * a ``Prev`` button (decrement, wraps)
      * a ``Next`` button (increment, wraps)

    Polls ~10 Hz, only re-publishes when the index changes — pose
    switching is near-instant because no geometry is re-uploaded.
    """
    n = len(snapshots)
    meshcat.AddSlider("snapshot", min=1, max=n, step=1, value=1)
    meshcat.AddButton("Prev")
    meshcat.AddButton("Next")
    meshcat.AddButton("Quit")

    last_index = 0   # forces an apply on first iteration
    last_prev = 0
    last_next = 0

    print()
    print("Interactive mode — use the Meshcat controls panel:")
    print("  - Slider 'snapshot' picks index 1..{N}".format(N=n))
    print("  - Prev / Next buttons step through")
    print("  - Quit button (or Ctrl-C) exits")
    print()
    print(f"{'#':>3}  {'file':<35} {'snapshot':<35} "
          f"{'arm':<9} {'Δxyz (mm)':>10}  {'ΔR (deg)':>9}")

    try:
        while True:
            # Buttons report cumulative click counts; diff against last seen.
            prev_clicks = meshcat.GetButtonClicks("Prev")
            next_clicks = meshcat.GetButtonClicks("Next")
            quit_clicks = meshcat.GetButtonClicks("Quit")
            if quit_clicks > 0:
                break

            cur = int(round(meshcat.GetSliderValue("snapshot")))

            if next_clicks > last_next:
                cur = ((cur - 1 + 1) % n) + 1
                meshcat.SetSliderValue("snapshot", cur)
            elif prev_clicks > last_prev:
                cur = ((cur - 1 - 1) % n) + 1
                meshcat.SetSliderValue("snapshot", cur)
            last_prev, last_next = prev_clicks, next_clicks

            if cur != last_index:
                file_name, arm, snap = snapshots[cur - 1]
                delta_mm, delta_deg, ok = _apply_snapshot(
                    plant, plant_ctx, diagram, sim_ctx, meshcat,
                    home_q, snapshots[cur - 1],
                )
                tag = "OK" if ok else "X"
                print(f"{cur:>3}  {file_name:<35} {snap['name'][:34]:<35} "
                      f"{arm:<9} {delta_mm:>9.1f}  {delta_deg:>8.2f}  {tag}")
                last_index = cur

            time.sleep(0.05)
    except KeyboardInterrupt:
        print("Closing.")
    finally:
        # Best-effort cleanup of widgets so a tab refresh doesn't leave
        # orphan controls in the Meshcat panel.
        for name in ("Prev", "Next", "Quit"):
            try:
                meshcat.DeleteButton(name)
            except Exception:
                pass
        try:
            meshcat.DeleteSlider("snapshot")
        except Exception:
            pass
    return 0


def main(file_paths: Optional[List[Path]] = None,
         seconds_per_pose: float = 3.0,
         snapshot_filters: Optional[List[str]] = None,
         list_only: bool = False,
         cycle: bool = False) -> int:
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
    sim_ctx = sim.get_mutable_context()
    plant_ctx = plant.GetMyMutableContextFromRoot(sim_ctx)

    home_q = default_home_q(plant)

    print()
    print("=" * 78)
    print(f"  Meshcat URL          : {meshcat.web_url()}")
    print(f"  Waypoint files       : {len(file_paths)}  (snapshots: {len(snapshots)})")
    print(f"  Mode                 : {'cycle' if cycle else 'interactive'}")
    print(f"  Marker tolerance     : {_TOLERANCE_M*1000:.0f} mm  "
          f"(green if FK within tolerance, red otherwise)")
    print("=" * 78)

    if cycle:
        print()
        print(f"{'#':>3}  {'file':<35} {'snapshot':<35} "
              f"{'arm':<9} {'Δxyz (mm)':>10}  {'ΔR (deg)':>9}")
        for i, snap_tuple in enumerate(snapshots, 1):
            file_name, arm, snap = snap_tuple
            delta_mm, delta_deg, ok = _apply_snapshot(
                plant, plant_ctx, diagram, sim_ctx, meshcat,
                home_q, snap_tuple,
            )
            tag = "OK" if ok else "X"
            print(f"{i:>3}  {file_name:<35} {snap['name'][:34]:<35} "
                  f"{arm:<9} {delta_mm:>9.1f}  {delta_deg:>8.2f}  {tag}")
            time.sleep(seconds_per_pose)
        print()
        print("Done cycling. Holding final pose — Ctrl-C to exit.")
        try:
            while True:
                time.sleep(0.5)
        except KeyboardInterrupt:
            print("Closing.")
        return 0

    return _run_interactive(
        meshcat, plant, plant_ctx, diagram, sim_ctx, home_q, snapshots,
    )


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
    ap.add_argument(
        "--cycle", action="store_true",
        help=(
            "Auto-advance through snapshots with --seconds-per-pose "
            "between each. Default is interactive (slider + Prev/Next "
            "buttons in the Meshcat controls panel)."
        ),
    )
    args = ap.parse_args()
    raise SystemExit(main(
        file_paths=args.files,
        seconds_per_pose=args.seconds_per_pose,
        snapshot_filters=args.snapshot,
        list_only=args.list,
        cycle=args.cycle,
    ))
