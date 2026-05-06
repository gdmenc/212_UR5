"""Interactive viewer for the IKFast goal branches at a given TCP target.

Loads the planning scene in Meshcat (with collision geometry rendered as
green wireframes) and lets you flip through every IKFast branch for a
chosen task-frame TCP pose using a slider in the controls panel. For
each branch the script prints the colliding body pairs so you can see
*exactly* which Vention beam / object / arm link is the obstruction.

Usage::

    # Default: left arm, bottle hover
    python3.11 -m control_scripts.planning.visualize_ik_branches

    # Pick another preset target
    python3.11 -m control_scripts.planning.visualize_ik_branches \\
        --target plate --arm ur_right --dz 0.12

    # Custom: explicit task xyz + tilt
    python3.11 -m control_scripts.planning.visualize_ik_branches \\
        --arm ur_left --xyz=-0.32,-0.125,0.11
"""

from __future__ import annotations

import argparse
import time
from typing import List, Tuple

import numpy as np
from pydrake.geometry import Rgba, Sphere, StartMeshcat
from pydrake.math import RigidTransform
from pydrake.systems.analysis import Simulator

from . import default_home_q
from ..util.poses import Pose
from ..util.rotations import Rotation
from .build_scene import build_scene
from .rrt import (
    ikfast_goal_branches, make_collision_checker, _arm_position_indices,
)
from .scene.objects import (
    BOTTLE_DEFAULT_TASK_XYZ,
    BOWL_DEFAULT_TASK_XYZ,
    CUP_DEFAULT_TASK_XYZ,
    CUP_WITH_STICK_DEFAULT_TASK_XYZ,
    PLATE_DEFAULT_TASK_XYZ,
    TRAY_DEFAULT_TASK_XYZ,
)
from .transit import _arm_model_instance


_TARGETS = {
    "plate":          PLATE_DEFAULT_TASK_XYZ,
    "cup":            CUP_DEFAULT_TASK_XYZ,
    "cup_with_stick": CUP_WITH_STICK_DEFAULT_TASK_XYZ,
    "bowl":           BOWL_DEFAULT_TASK_XYZ,
    "bottle":         BOTTLE_DEFAULT_TASK_XYZ,
    "tray":           TRAY_DEFAULT_TASK_XYZ,
}

_R_DOWN = Rotation.from_matrix(np.array([
    [1.0,  0.0,  0.0],
    [0.0, -1.0,  0.0],
    [0.0,  0.0, -1.0],
]))


def _print_pairs(plant, scene_graph, diag_ctx) -> int:
    """Print all penetrating geometry pairs at the current state."""
    sg_ctx = scene_graph.GetMyMutableContextFromRoot(diag_ctx)
    qo = scene_graph.get_query_output_port().Eval(sg_ctx)
    pairs = qo.ComputePointPairPenetration()
    inspector = scene_graph.model_inspector()
    if not pairs:
        print(f"    no collisions")
        return 0
    for p in pairs[:6]:
        ba = plant.GetBodyFromFrameId(inspector.GetFrameId(p.id_A)).name()
        bb = plant.GetBodyFromFrameId(inspector.GetFrameId(p.id_B)).name()
        print(f"    {ba:>22} ↔ {bb:<22}  depth={p.depth*1000:.1f} mm")
    if len(pairs) > 6:
        print(f"    ...and {len(pairs) - 6} more")
    return len(pairs)


def main(arm: str, xyz_task, max_branch_dist, gripper_mode: str) -> int:
    target = Pose(translation=np.asarray(xyz_task), rotation=_R_DOWN)
    print(f"\nTarget (task xyz): {np.round(target.translation, 3)}  arm={arm}")

    meshcat = StartMeshcat()
    scene = build_scene(
        meshcat=meshcat,
        show_collision=True,
        robotiq_mode=gripper_mode,
    )
    diag, plant = scene.diagram, scene.plant
    sim = Simulator(diag)
    sim.Initialize()
    sim_ctx = sim.get_mutable_context()
    plant_ctx = plant.GetMyMutableContextFromRoot(sim_ctx)

    home = default_home_q(plant)
    plant.SetPositions(plant_ctx, home)
    diag.ForcedPublish(sim_ctx)

    inst = _arm_model_instance(plant, arm)
    arm_idx = _arm_position_indices(plant, inst)
    seed = home[arm_idx]

    branches = ikfast_goal_branches(
        arm, target, seed_arm_q=seed, max_branch_dist=max_branch_dist,
    )
    if not branches:
        print("ikfast returned 0 branches — target unreachable. Nothing to view.")
        return 1

    checker = make_collision_checker(diag, plant, arm)

    # Drop a marker at the requested TCP target.
    meshcat.SetObject("/target/tcp", Sphere(0.014), Rgba(0.95, 0.6, 0.1, 0.9))
    meshcat.SetTransform("/target/tcp", RigidTransform(np.asarray(target.translation)))

    print(f"\n{len(branches)} IK branches enumerated.")
    print(f"\nMeshcat URL: {meshcat.web_url()}")
    print("Use the 'branch' slider in the Meshcat controls panel.")
    print("Each step prints the colliding bodies (if any) for that config.\n")

    n = len(branches)
    meshcat.AddSlider("branch", min=1, max=n, step=1, value=1)
    meshcat.AddButton("Quit")
    last_idx = 0
    try:
        while meshcat.GetButtonClicks("Quit") == 0:
            cur = int(round(meshcat.GetSliderValue("branch")))
            if cur != last_idx:
                b = branches[cur - 1]
                full = home.copy(); full[arm_idx] = b
                plant.SetPositions(plant_ctx, full)
                free = checker.CheckConfigCollisionFree(full)
                # Sneaky: SceneGraph is owned by `scene.scene_graph` not diag,
                # so we use scene.scene_graph for queries.
                diag.ForcedPublish(sim_ctx)
                print(f"branch #{cur-1}/{n-1}  Δseed={np.linalg.norm(b - seed):.2f} rad  "
                      f"q={np.round(np.degrees(b)).astype(int)}°  "
                      f"{'COLLISION-FREE' if free else 'in collision:'}")
                if not free:
                    _print_pairs(plant, scene.scene_graph, sim_ctx)
                last_idx = cur
            time.sleep(0.05)
    except KeyboardInterrupt:
        print("\nClosing.")
    finally:
        for nm in ("Quit",):
            try: meshcat.DeleteButton(nm)
            except Exception: pass
        try: meshcat.DeleteSlider("branch")
        except Exception: pass
    return 0


if __name__ == "__main__":
    ap = argparse.ArgumentParser(
        description="Interactive viewer for IKFast branches at a TCP target."
    )
    ap.add_argument("--arm", choices=["ur_left", "ur_right"], default="ur_left")
    ap.add_argument(
        "--target", choices=list(_TARGETS), default="bottle",
        help="One of the default object positions. Ignored if --xyz is given.",
    )
    ap.add_argument(
        "--dz", type=float, default=0.12,
        help="Hover offset above the named target (default 12 cm).",
    )
    ap.add_argument(
        "--xyz", default=None,
        help="Custom target as 'x,y,z' (task frame). Overrides --target/--dz.",
    )
    ap.add_argument(
        "--max-branch-dist", type=float, default=None,
        help="Drop branches farther than this (rad) from HOME's seed.",
    )
    ap.add_argument(
        "--gripper-mode", choices=["closed", "open"], default="closed",
    )
    args = ap.parse_args()

    if args.xyz:
        xyz = tuple(float(x) for x in args.xyz.split(","))
        if len(xyz) != 3:
            raise SystemExit(f"--xyz must be 'x,y,z', got {args.xyz!r}")
    else:
        x, y, z = _TARGETS[args.target]
        xyz = (x, y, z + args.dz)

    raise SystemExit(main(
        arm=args.arm, xyz_task=xyz,
        max_branch_dist=args.max_branch_dist,
        gripper_mode=args.gripper_mode,
    ))
