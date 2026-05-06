"""Visualize the cup-microwave pick descent for a chosen grasp angle.

Companion to ``sweep_cup_grasp_angle``. Loads the same planning scene
(microwave + all default static objects + cup_with_stick skipped),
walks the descent from hover to grasp at the requested ``--angle-deg``
and ``--base-xyz``, and pops up a meshcat preview with collision
geometry rendered. A meshcat slider lets you scrub through the descent
samples; another puts the worst (most-penetrating) step on screen.

Useful for confirming WHICH body actually clips the arm at a given
angle — the URDF's collision boxes (e.g. Vention ``lower_trap_+x``)
show up alongside the visual mesh, so you can see if the box is the
right shape or "fatter" than the real lab geometry.

Usage
-----
    python3.11 -m control_scripts.planning.visualize_cup_grasp_angle
    python3.11 -m control_scripts.planning.visualize_cup_grasp_angle \\
        --angle-deg 45
    python3.11 -m control_scripts.planning.visualize_cup_grasp_angle \\
        --angle-deg 0 --base-xyz -0.20 -0.125 0 --door-open-deg 0
"""

from __future__ import annotations

import argparse
import time
from typing import List, Optional

import numpy as np

from pydrake.geometry import Rgba, Sphere, StartMeshcat
from pydrake.math import RigidTransform
from pydrake.systems.analysis import Simulator

from ..grasps.cup import CUP_HEIGHT_M, cup_hook_grasp, cup_rim_grasp
from ..planning import SIM_HOME_Q_LEFT, SIM_HOME_Q_RIGHT
from ..planning.rrt import ikfast_goal_branches, make_collision_checker
from ..planning.scene.objects import CUP_WITH_STICK_DEFAULT_TASK_XYZ
from ..planning.transit import _arm_model_instance, _arm_position_indices
from ..util.poses import Pose
from ..world import World


_HOME_Q = {"ur_left": SIM_HOME_Q_LEFT, "ur_right": SIM_HOME_Q_RIGHT}
_OTHER_ARM = {"ur_right": "ur_left", "ur_left": "ur_right"}


def _interp_descent(hover: Pose, grasp: Pose, t: float) -> Pose:
    z = (1.0 - t) * hover.translation[2] + t * grasp.translation[2]
    return Pose(
        translation=np.array([hover.translation[0], hover.translation[1], z]),
        rotation=hover.rotation,
    )


def _walk_descent(
    arm_name: str,
    hover: Pose,
    grasp: Pose,
    n_samples: int,
    branch_max_dist: float,
) -> List[np.ndarray]:
    """Pick the IK branch that stays feasible the longest along the
    descent (matches the sweep's "best branch" selection)."""
    hover_branches = ikfast_goal_branches(arm_name, hover, seed_arm_q=None)
    if not hover_branches:
        raise RuntimeError("No IK branches at hover pose.")

    best_path: List[np.ndarray] = []
    best_len = -1
    for start_q in hover_branches:
        path: List[np.ndarray] = [np.asarray(start_q, dtype=float)]
        seed = path[0]
        for i in range(1, n_samples + 1):
            t = i / n_samples
            tcp = _interp_descent(hover, grasp, t)
            sols = ikfast_goal_branches(
                arm_name, tcp, seed_arm_q=seed,
                max_branch_dist=branch_max_dist,
            )
            if not sols:
                break
            path.append(np.asarray(sols[0], dtype=float))
            seed = sols[0]
        if len(path) > best_len:
            best_len = len(path)
            best_path = path
    return best_path


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--arm", choices=["ur_left", "ur_right"], default="ur_right")
    ap.add_argument(
        "--grasp", choices=["rim", "hook"], default="rim",
        help="Which cup grasp factory to use. 'rim' = Robotiq top-down "
             "rim pinch (default, used by pick_place_cup_microwave). "
             "'hook' = hook-on-rim grasp (used by pick_place_cup_hook_"
             "microwave). Pair with --arm ur_left for the hook variant.",
    )
    ap.add_argument(
        "--rim-leeway-mm", type=float, default=0.0,
        help="Lift the TCP above the rim by this many mm (hook only "
             "— the rim_z_offset_m kwarg). Mirrors TCP_RIM_LEEWAY_M "
             "in pick_place_cup_hook_microwave.py. Default 0 (flush "
             "rim grasp).",
    )
    ap.add_argument("--angle-deg", type=float, default=45.0,
                    help="Grasp angle to visualize (default 45°).")
    ap.add_argument(
        "--base-xyz", nargs=3, type=float,
        default=list(CUP_WITH_STICK_DEFAULT_TASK_XYZ),
        metavar=("X", "Y", "Z"),
    )
    ap.add_argument("--transit-z", type=float, default=0.55)
    ap.add_argument(
        "--stop-z", type=float, default=None,
        help="Override the bottom of the descent (TCP z, m). Default "
             "is the cup-rim grasp z = base_xyz[2] + CUP_HEIGHT_M = "
             f"{CUP_HEIGHT_M:.3f}. Set higher to stop the descent early "
             "(e.g. --stop-z 0.275 to halt 12 cm above the rim). xy and "
             "rotation are still taken from the rim grasp at the chosen "
             "angle so the gripper points at the same target.",
    )
    ap.add_argument("--door-open-deg", type=float, default=0.0)
    ap.add_argument("--descent-samples", type=int, default=12,
                    help="Descent steps from hover to grasp (default 12).")
    ap.add_argument("--branch-max-dist", type=float, default=0.6)
    ap.add_argument(
        "--show-collision", action="store_true", default=True,
        help="Render collision geometry (default ON for diagnostic value). "
             "Use --no-show-collision for a cleaner visual scene.",
    )
    ap.add_argument("--no-show-collision", action="store_false", dest="show_collision")
    ap.add_argument("--duration", type=float, default=600.0)
    args = ap.parse_args()

    base_xyz = np.array(args.base_xyz, dtype=float)
    door_open_rad = float(np.radians(args.door_open_deg))
    angle_rad = float(np.radians(args.angle_deg))

    # Two worlds: the PLANNING world skips cup_with_stick so the IK +
    # clearance math doesn't penalise the gripper for being next to its
    # own pickup target. The VISUALIZATION world keeps cup_with_stick
    # at the pick pose (base_xyz) so the user can see the cup the arm
    # is descending toward.
    plan_world = World(
        include_microwave=True,
        include_objects=True,
        skip_static_objects=("cup_with_stick",),
        robotiq_mode="closed",
        microwave_door_open_rad=door_open_rad,
        partner_arm_q={
            _OTHER_ARM[args.arm]: np.asarray(_HOME_Q[_OTHER_ARM[args.arm]], dtype=float),
        },
    )
    viz_world = World(
        include_microwave=True,
        include_objects=True,
        skip_static_objects=(),
        object_xyz_overrides={"cup_with_stick": tuple(float(v) for v in base_xyz)},
        robotiq_mode="closed",
        microwave_door_open_rad=door_open_rad,
        partner_arm_q=plan_world.partner_arm_q,
    )

    # 1. Compute the descent path q's via the planning scene + ikfast.
    plan_diagram, plan_plant, _, _ = plan_world.build_planning_scene()
    plan_ctx = plan_plant.GetMyMutableContextFromRoot(
        plan_diagram.CreateDefaultContext()
    )
    arm_inst = _arm_model_instance(plan_plant, args.arm)
    arm_idx = _arm_position_indices(plan_plant, arm_inst)
    other_inst = _arm_model_instance(plan_plant, _OTHER_ARM[args.arm])
    other_idx = _arm_position_indices(plan_plant, other_inst)

    cup_pose = Pose(translation=base_xyz)
    if args.grasp == "hook":
        leeway_m = args.rim_leeway_mm / 1000.0
        grasp = cup_hook_grasp(
            cup_pose,
            angle_rad=angle_rad,
            rim_z_offset_m=CUP_HEIGHT_M + leeway_m,
        )
    else:
        grasp = cup_rim_grasp(cup_pose, angle_rad=angle_rad)
    # Optional early-stop: override the descent endpoint's z while
    # keeping xy + rotation from the rim grasp factory.
    if args.stop_z is not None:
        descent_end = Pose(
            translation=np.array([
                grasp.grasp_pose.translation[0],
                grasp.grasp_pose.translation[1],
                float(args.stop_z),
            ]),
            rotation=grasp.grasp_pose.rotation,
        )
    else:
        descent_end = grasp.grasp_pose
    hover = Pose(
        translation=np.array([
            grasp.grasp_pose.translation[0],
            grasp.grasp_pose.translation[1],
            args.transit_z,
        ]),
        rotation=grasp.grasp_pose.rotation,
    )

    print(f"[viz] computing IK descent at angle = {args.angle_deg:+.1f}°")
    print(f"      cup base xyz   = {base_xyz}")
    print(f"      hover xyz      = {np.round(hover.translation, 3)}")
    print(f"      descent end xyz= {np.round(descent_end.translation, 3)}"
          + ("  (rim grasp pose)" if args.stop_z is None else f"  (--stop-z {args.stop_z})"))
    print(f"      rim grasp z    = {grasp.grasp_pose.translation[2]:.3f} m  "
          f"(gap from descent end: {(descent_end.translation[2] - grasp.grasp_pose.translation[2])*100:+.1f} cm)")

    path_q = _walk_descent(
        args.arm, hover, descent_end,
        args.descent_samples, args.branch_max_dist,
    )
    print(f"[viz] descent path: {len(path_q)} configs (hover + {len(path_q)-1} steps)")

    # 2. Compute clearance per step using the planning checker.
    checker = make_collision_checker(plan_diagram, plan_plant, args.arm)
    other_q = np.asarray(_HOME_Q[_OTHER_ARM[args.arm]], dtype=float)
    clearances_mm: List[float] = []
    for q_arm in path_q:
        q_full = np.asarray(plan_plant.GetPositions(plan_ctx)).copy()
        q_full[arm_idx] = q_arm
        q_full[other_idx] = other_q
        rc = checker.CalcRobotClearance(q_full, 0.05)
        d = float(np.min(np.asarray(rc.distances()))) if rc.distances().size > 0 else 0.05
        clearances_mm.append(d * 1000.0)

    worst_idx = int(np.argmin(clearances_mm))
    print(f"[viz] per-step clearance (mm):")
    for i, mm in enumerate(clearances_mm):
        marker = " ←WORST" if i == worst_idx else ""
        z = (1.0 - i / max(1, len(path_q) - 1)) * hover.translation[2] + (
            i / max(1, len(path_q) - 1)
        ) * descent_end.translation[2]
        print(f"        step {i:2d}  z={z:.3f}m  clear={mm:+7.2f} mm{marker}")

    # 3. Build a sim scene (with optional collision rendering) and show.
    meshcat = StartMeshcat()
    scene = viz_world.build_sim_scene(
        meshcat=meshcat,
        show_visual=True,
        show_collision=args.show_collision,
    )
    sim_diagram = scene.diagram
    sim_plant = scene.plant
    simulator = Simulator(sim_diagram)
    simulator.Initialize()
    sim_ctx = simulator.get_mutable_context()
    sim_plant_ctx = sim_plant.GetMyMutableContextFromRoot(sim_ctx)

    sim_arm_inst = _arm_model_instance(sim_plant, args.arm)
    sim_other_inst = _arm_model_instance(sim_plant, _OTHER_ARM[args.arm])

    # Add markers for hover, grasp, and worst-step TCP positions.
    meshcat.SetObject(
        "/markers/hover", Sphere(0.012), Rgba(0.2, 0.7, 0.95, 0.9),
    )
    meshcat.SetTransform(
        "/markers/hover", RigidTransform(list(hover.translation)),
    )
    meshcat.SetObject(
        "/markers/grasp", Sphere(0.012), Rgba(0.95, 0.2, 0.4, 0.9),
    )
    meshcat.SetTransform(
        "/markers/grasp", RigidTransform(list(grasp.grasp_pose.translation)),
    )
    # If --stop-z, also mark the descent end (orange) so the user can
    # see the gap between where the gripper stops and where the rim is.
    if args.stop_z is not None:
        meshcat.SetObject(
            "/markers/descent_end", Sphere(0.012), Rgba(0.95, 0.55, 0.1, 0.9),
        )
        meshcat.SetTransform(
            "/markers/descent_end",
            RigidTransform(list(descent_end.translation)),
        )

    # Slider: descent step (0 = hover, len-1 = grasp).
    n_steps = len(path_q)
    meshcat.AddSlider("descent_step", min=0, max=n_steps - 1, step=1, value=worst_idx)

    def _set_step(idx: int) -> None:
        sim_plant.SetPositions(sim_plant_ctx, sim_arm_inst, path_q[idx])
        sim_plant.SetPositions(sim_plant_ctx, sim_other_inst, other_q)
        sim_diagram.ForcedPublish(sim_ctx)

    _set_step(worst_idx)

    print()
    print("=" * 70)
    print(f"  Meshcat URL : {meshcat.web_url()}")
    print(f"  Showing step {worst_idx} (worst, clearance {clearances_mm[worst_idx]:+.2f} mm)")
    print(f"  Drag the 'descent_step' slider to scrub through the path.")
    print(f"  Blue sphere  = hover TCP  ({np.round(hover.translation,3)})")
    print(f"  Red sphere   = grasp TCP  ({np.round(grasp.grasp_pose.translation,3)})")
    if args.show_collision:
        print(f"  Collision boxes (e.g. Vention lower_trap) shown alongside visual mesh.")
    print(f"  Holding scene open for {args.duration:.0f} s — Ctrl-C to exit.")
    print("=" * 70)

    last_step = worst_idx
    try:
        t_end = time.time() + args.duration
        while time.time() < t_end:
            cur = int(round(meshcat.GetSliderValue("descent_step")))
            if cur != last_step:
                _set_step(cur)
                print(f"  step {cur:2d}  clearance = {clearances_mm[cur]:+7.2f} mm")
                last_step = cur
            time.sleep(0.05)
    except KeyboardInterrupt:
        print("\n[viz] interrupted — closing.")
    finally:
        try:
            meshcat.DeleteSlider("descent_step")
        except Exception:
            pass
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
