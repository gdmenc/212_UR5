"""Sweep ``GRASP_ANGLE_RAD`` to find a robust cup-rim grasp for the
cup-microwave task.

Picks the cup-with-stick at a base task-frame pose (defaults to
``CUP_WITH_STICK_DEFAULT_TASK_XYZ``). For every (angle, dx, dy) triple
in a rectangular tolerance window, it samples the descent path
hover → pregrasp → grasp at fixed xy + rotation, and for each IK
branch walks the descent with nearest-neighbor IK and checks every
sample against ``SceneGraphCollisionChecker``. A cell is "feasible"
when at least one IK branch keeps the arm collision-free for the
entire descent.

Coverage = #feasible cells / total cells per angle. The angle that
hits 100% coverage AND the most resilient nearest-neighbor branch
chain wins.

Scene
-----
Uses the cup-microwave task's WORLD shape: microwave on, all default
static objects (plate, plain cup, bowl, bottle, tray) at their default
poses, ``cup_with_stick`` skipped (it's the target). Partner arm is
pinned at its ``SIM_HOME_Q``.

Running
-------
    python3.11 -m control_scripts.planning.sweep_cup_grasp_angle
    python3.11 -m control_scripts.planning.sweep_cup_grasp_angle \\
        --angle-step-deg 5 --grid-step 0.0025 --descent-samples 12
    python3.11 -m control_scripts.planning.sweep_cup_grasp_angle \\
        --base-xyz -0.20 -0.125 0.0 --tol-x 0.02 --tol-y 0.005
    python3.11 -m control_scripts.planning.sweep_cup_grasp_angle \\
        --door-open-deg 103   # microwave door open (after open_microwave)
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from typing import List, Optional

import numpy as np

from ..grasps.cup import CUP_HEIGHT_M, cup_rim_grasp
from ..planning import SIM_HOME_Q_LEFT, SIM_HOME_Q_RIGHT
from ..planning.rrt import ikfast_goal_branches, make_collision_checker
from ..planning.scene.objects import CUP_WITH_STICK_DEFAULT_TASK_XYZ
from ..planning.transit import _arm_model_instance, _arm_position_indices
from ..util.poses import Pose
from ..world import World


_OTHER_ARM = {"ur_right": "ur_left", "ur_left": "ur_right"}
_HOME_Q = {"ur_left": SIM_HOME_Q_LEFT, "ur_right": SIM_HOME_Q_RIGHT}


@dataclass
class CellResult:
    feasible: bool
    n_branches_tried: int
    n_branches_feasible: int
    # Best min-clearance (mm) across all hover branches' descents.
    # Negative = penetration. Higher = more comfortable grasp.
    best_min_clearance_mm: float
    # Body pair (geometry names) that produced the best branch's
    # worst step. Lets you see WHAT is actually limiting the descent.
    worst_body_pair: str


@dataclass
class AngleResult:
    angle_deg: float
    n_cells: int
    n_feasible: int
    coverage: float
    avg_branches_feasible: float
    worst_min_clearance_mm: float
    avg_min_clearance_mm: float
    # Most-common limiting body pair across cells (for the worst step
    # of each cell's best branch).
    dominant_body_pair: str


def _interp_descent(hover: Pose, grasp: Pose, t: float) -> Pose:
    """Linear z interpolation between hover and grasp; xy + rotation
    held constant (matches ``approach_to`` Cartesian descent)."""
    z = (1.0 - t) * hover.translation[2] + t * grasp.translation[2]
    return Pose(
        translation=np.array([hover.translation[0], hover.translation[1], z]),
        rotation=hover.rotation,
    )


def _build_q_full(
    plant,
    plant_context,
    arm_idx: np.ndarray,
    arm_q: np.ndarray,
    other_idx: Optional[np.ndarray],
    other_q: Optional[np.ndarray],
) -> np.ndarray:
    q_full = np.asarray(plant.GetPositions(plant_context), dtype=float).copy()
    q_full[arm_idx] = arm_q
    if other_idx is not None and other_q is not None:
        q_full[other_idx] = other_q
    return q_full


def _min_robot_clearance_m(checker, q_full: np.ndarray) -> float:
    """Min signed clearance of the robot to the rest of the scene at
    ``q_full``. Honors Drake's collision filter. Negative = penetration."""
    rc = checker.CalcRobotClearance(q_full, 0.05)
    dists = np.asarray(rc.distances())
    if dists.size == 0:
        return 0.05
    return float(np.min(dists))


def _limiting_body_for(plant, plant_context, q_full: np.ndarray) -> str:
    """Name of the closest non-robot body to the planning arm at ``q_full``.
    Uses ComputeSignedDistancePairwiseClosestPoints (NOT filtered for
    self-collision; we filter manually below by skipping pairs whose
    geometries belong to the same model instance)."""
    plant.SetPositions(plant_context, q_full)
    sg = plant.get_geometry_query_input_port().Eval(plant_context)
    inspector = sg.inspector()
    pairs = sg.ComputeSignedDistancePairwiseClosestPoints(0.05)
    if not pairs:
        return "(none within 5 cm)"

    def _frame_model(geom_id):
        frame_id = inspector.GetFrameId(geom_id)
        body = plant.GetBodyFromFrameId(frame_id)
        return body.model_instance() if body is not None else None

    best_d = float("inf")
    best_name = "(unknown)"
    for p in pairs:
        ma = _frame_model(p.id_A)
        mb = _frame_model(p.id_B)
        if ma == mb:
            continue  # same model instance — adjacent-link self-pair
        if p.distance < best_d:
            best_d = p.distance
            nA = inspector.GetName(p.id_A).split("::")[-1]
            nB = inspector.GetName(p.id_B).split("::")[-1]
            best_name = f"{nA} ↔ {nB}"
    return best_name


def _evaluate_descent(
    *,
    checker,
    plant,
    plant_context,
    arm_name: str,
    arm_idx: np.ndarray,
    other_idx: Optional[np.ndarray],
    other_q: Optional[np.ndarray],
    hover_pose: Pose,
    grasp_pose: Pose,
    n_samples: int,
    branch_max_dist: float,
    feasibility_threshold_m: float,
) -> CellResult:
    """For each IK branch at hover, walk down with nearest-neighbor IK
    and at every sample compute the min signed distance across all
    geometry pairs. A branch's score = min over the descent. The cell
    "feasible" if any branch's score ≥ ``feasibility_threshold_m``."""
    hover_branches = ikfast_goal_branches(
        arm_name, hover_pose, seed_arm_q=None,
    )
    if not hover_branches:
        return CellResult(False, 0, 0, -np.inf, "(no IK at hover)")

    best_min_clear = -np.inf
    best_q_full_at_worst: Optional[np.ndarray] = None
    n_feasible = 0
    for start_q in hover_branches:
        path_q: List[np.ndarray] = [np.asarray(start_q, dtype=float)]
        seed = path_q[0]
        broke = False
        for i in range(1, n_samples + 1):
            t = i / n_samples
            tcp = _interp_descent(hover_pose, grasp_pose, t)
            sols = ikfast_goal_branches(
                arm_name, tcp, seed_arm_q=seed,
                max_branch_dist=branch_max_dist,
            )
            if not sols:
                broke = True
                break
            path_q.append(np.asarray(sols[0], dtype=float))
            seed = sols[0]
        if broke:
            continue

        branch_min_clear = np.inf
        branch_worst_q: Optional[np.ndarray] = None
        for q_arm in path_q:
            q_full = _build_q_full(
                plant, plant_context, arm_idx, q_arm, other_idx, other_q,
            )
            d = _min_robot_clearance_m(checker, q_full)
            if d < branch_min_clear:
                branch_min_clear = d
                branch_worst_q = q_full.copy()
        if branch_min_clear > best_min_clear:
            best_min_clear = branch_min_clear
            best_q_full_at_worst = branch_worst_q
        if branch_min_clear >= feasibility_threshold_m:
            n_feasible += 1

    body_pair = (
        _limiting_body_for(plant, plant_context, best_q_full_at_worst)
        if best_q_full_at_worst is not None else "(none)"
    )
    return CellResult(
        feasible=n_feasible > 0,
        n_branches_tried=len(hover_branches),
        n_branches_feasible=n_feasible,
        best_min_clearance_mm=float(best_min_clear * 1000.0),
        worst_body_pair=body_pair,
    )


def _build_world(door_open_rad: float) -> World:
    """Mirror the cup-microwave task's WORLD: microwave on, all default
    static objects, cup_with_stick skipped (it's the target)."""
    return World(
        include_microwave=True,
        include_objects=True,
        skip_static_objects=("cup_with_stick",),
        robotiq_mode="closed",
        microwave_door_open_rad=door_open_rad,
    )


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument(
        "--arm", choices=["ur_left", "ur_right"], default="ur_right",
        help="Which arm picks the cup (default ur_right).",
    )
    ap.add_argument(
        "--base-xyz", nargs=3, type=float,
        default=list(CUP_WITH_STICK_DEFAULT_TASK_XYZ),
        metavar=("X", "Y", "Z"),
        help="Cup-with-stick base pose in task frame "
             f"(default {CUP_WITH_STICK_DEFAULT_TASK_XYZ}).",
    )
    ap.add_argument(
        "--tol-x", type=float, default=0.02,
        help="±x tolerance window in metres (default 0.02 = 2 cm).",
    )
    ap.add_argument(
        "--tol-y", type=float, default=0.005,
        help="±y tolerance window in metres (default 0.005 = 0.5 cm).",
    )
    ap.add_argument(
        "--grid-step", type=float, default=0.005,
        help="Tolerance grid step in metres (default 0.005 = 5 mm).",
    )
    ap.add_argument(
        "--angle-step-deg", type=float, default=10.0,
        help="Angle sweep step in degrees (default 10°). 5° gives a "
             "finer search but ~2× the runtime.",
    )
    ap.add_argument(
        "--descent-samples", type=int, default=8,
        help="Number of intermediate descent samples (default 8).",
    )
    ap.add_argument(
        "--transit-z", type=float, default=0.55,
        help="TCP altitude at hover, matching pick_place_cup_microwave "
             "(default 0.55 m).",
    )
    ap.add_argument(
        "--door-open-deg", type=float, default=0.0,
        help="Microwave door angle in degrees (default 0 = closed).",
    )
    ap.add_argument(
        "--branch-max-dist", type=float, default=0.6,
        help="Reject IK branches whose joint distance from the previous "
             "step exceeds this (radians, default 0.6 ≈ 34°). Keeps "
             "the descent on a single kinematic branch.",
    )
    ap.add_argument(
        "--feasibility-threshold-mm", type=float, default=0.0,
        help="Cells with best-branch min-clearance below this are "
             "marked infeasible (default 0 mm = no penetration). Set "
             "negative (e.g. -2) to allow up to 2 mm of model "
             "interference — useful when the rig's actual collision "
             "geometry is less padded than the URDF.",
    )
    ap.add_argument(
        "--top-n", type=int, default=10,
        help="How many best angles to print at the end (default 10).",
    )
    args = ap.parse_args()

    base_xyz = np.array(args.base_xyz, dtype=float)
    door_open_rad = float(np.radians(args.door_open_deg))
    world = _build_world(door_open_rad)

    diagram, plant, _, _ = world.build_planning_scene()
    root_ctx = diagram.CreateDefaultContext()
    plant_ctx = plant.GetMyMutableContextFromRoot(root_ctx)

    arm_inst = _arm_model_instance(plant, args.arm)
    arm_idx = _arm_position_indices(plant, arm_inst)

    other_arm_name = _OTHER_ARM[args.arm]
    other_idx = None
    other_q = None
    if plant.HasModelInstanceNamed(other_arm_name) or plant.HasModelInstanceNamed(f"{other_arm_name}::ur5e"):
        try:
            other_inst = _arm_model_instance(plant, other_arm_name)
            other_idx = _arm_position_indices(plant, other_inst)
            other_q = np.asarray(_HOME_Q[other_arm_name], dtype=float)
        except KeyError:
            pass

    checker = make_collision_checker(
        diagram, plant, args.arm,
        edge_step_size=0.05,
        env_padding=0.005,
    )

    # Build offset grid (centered, includes endpoints).
    def _grid_axis(half: float) -> np.ndarray:
        if half <= 0:
            return np.array([0.0])
        n = max(1, int(round(2 * half / args.grid_step)))
        return np.linspace(-half, +half, n + 1)

    dxs = _grid_axis(args.tol_x)
    dys = _grid_axis(args.tol_y)
    angles_deg = np.arange(0.0, 360.0, args.angle_step_deg)

    print("=" * 72)
    print(f"  arm                : {args.arm}")
    print(f"  cup base xyz_task  : {base_xyz}")
    print(f"  tolerance window   : x ±{args.tol_x*100:.1f} cm, y ±{args.tol_y*100:.1f} cm")
    print(f"  grid step          : {args.grid_step*1000:.1f} mm")
    print(f"  angle sweep        : {args.angle_step_deg:.1f}° step ({len(angles_deg)} angles)")
    print(f"  cells per angle    : {len(dxs) * len(dys)} ({len(dxs)} dx × {len(dys)} dy)")
    print(f"  descent samples    : {args.descent_samples}")
    print(f"  transit z          : {args.transit_z:.3f} m")
    print(f"  door open          : {args.door_open_deg:.0f}°")
    print(f"  partner arm pin    : {other_arm_name} @ SIM_HOME"
          if other_q is not None else f"  partner arm        : not present in scene")
    print("=" * 72)

    results: List[AngleResult] = []
    n_cells_total = len(dxs) * len(dys)
    fthr_m = args.feasibility_threshold_mm / 1000.0
    print(f"  feasibility thr    : {args.feasibility_threshold_mm:+.1f} mm")
    print("=" * 72)

    from collections import Counter

    for ang_deg in angles_deg:
        ang_rad = float(np.radians(ang_deg))
        n_feasible = 0
        sum_branches_feasible = 0
        cell_min_clears: List[float] = []
        body_counter: Counter = Counter()

        for dx in dxs:
            for dy in dys:
                cup_pose = Pose(translation=base_xyz + np.array([dx, dy, 0.0]))
                grasp = cup_rim_grasp(cup_pose, angle_rad=ang_rad)
                hover = Pose(
                    translation=np.array([
                        grasp.grasp_pose.translation[0],
                        grasp.grasp_pose.translation[1],
                        args.transit_z,
                    ]),
                    rotation=grasp.grasp_pose.rotation,
                )
                cell = _evaluate_descent(
                    checker=checker,
                    plant=plant,
                    plant_context=plant_ctx,
                    arm_name=args.arm,
                    arm_idx=arm_idx,
                    other_idx=other_idx,
                    other_q=other_q,
                    hover_pose=hover,
                    grasp_pose=grasp.grasp_pose,
                    n_samples=args.descent_samples,
                    branch_max_dist=args.branch_max_dist,
                    feasibility_threshold_m=fthr_m,
                )
                if cell.feasible:
                    n_feasible += 1
                    sum_branches_feasible += cell.n_branches_feasible
                cell_min_clears.append(cell.best_min_clearance_mm)
                body_counter[cell.worst_body_pair] += 1

        coverage = n_feasible / n_cells_total
        avg_branches_feasible = (
            sum_branches_feasible / n_feasible if n_feasible > 0 else 0.0
        )
        worst_clear = float(min(cell_min_clears))
        avg_clear = float(np.mean(cell_min_clears))
        dominant_body = body_counter.most_common(1)[0][0] if body_counter else "?"
        results.append(AngleResult(
            angle_deg=float(ang_deg),
            n_cells=n_cells_total,
            n_feasible=n_feasible,
            coverage=coverage,
            avg_branches_feasible=avg_branches_feasible,
            worst_min_clearance_mm=worst_clear,
            avg_min_clearance_mm=avg_clear,
            dominant_body_pair=dominant_body,
        ))
        print(
            f"  angle = {ang_deg:+6.1f}°  "
            f"worst-min = {worst_clear:+7.2f} mm  "
            f"limited by: {dominant_body}"
        )

    # Primary rank: worst-case clearance (the bottleneck across the
    # tolerance grid). Tie-break on coverage and average clearance.
    ranked = sorted(
        results,
        key=lambda r: (r.worst_min_clearance_mm, r.coverage, r.avg_min_clearance_mm),
        reverse=True,
    )

    print()
    print("=" * 90)
    print(f"  Top {args.top_n} angles by worst-case descent clearance "
          f"(threshold {args.feasibility_threshold_mm:+.1f} mm):")
    print("=" * 90)
    print(f"  {'angle':>7} | {'worst-min':>10} | {'avg-min':>9} | limiting body pair")
    print("  " + "-" * 88)
    for r in ranked[: args.top_n]:
        print(
            f"  {r.angle_deg:+5.1f}° | "
            f"{r.worst_min_clearance_mm:+8.2f} mm | "
            f"{r.avg_min_clearance_mm:+7.2f} mm | "
            f"{r.dominant_body_pair}"
        )

    print()
    best = ranked[0]
    if best.worst_min_clearance_mm > 0:
        print(f"  ✓ RECOMMENDED  : GRASP_ANGLE_RAD = float(np.radians({best.angle_deg:.1f}))")
        print(f"     coverage    : 100% across the ±{args.tol_x*100:.1f}/±{args.tol_y*100:.1f} cm window")
        print(f"     worst clear : {best.worst_min_clearance_mm:+.2f} mm "
              f"(positive = no penetration in the URDF model)")
        print(f"     avg clear   : {best.avg_min_clearance_mm:+.2f} mm")
    elif best.worst_min_clearance_mm > args.feasibility_threshold_mm:
        print(f"  ✓ BEST ANGLE   : GRASP_ANGLE_RAD = float(np.radians({best.angle_deg:.1f}))")
        print(f"     coverage    : 100% under the chosen threshold ({args.feasibility_threshold_mm:+.1f} mm)")
        print(f"     worst clear : {best.worst_min_clearance_mm:+.2f} mm "
              f"(URDF model penetration; rig may be more lenient)")
        print(f"     avg clear   : {best.avg_min_clearance_mm:+.2f} mm")
        print(f"     Note: URDF says the arm overlaps something by "
              f"{abs(best.worst_min_clearance_mm):.1f} mm somewhere in the descent.")
        print(f"     If the rig actually picks the cup at the current "
              f"setup, the URDF Vention/microwave model is more conservative")
        print(f"     than reality. The relative ranking is still valid — "
              f"this angle has the most clearance margin.")
    else:
        print(f"  ⚠  Best angle still penetrates beyond the chosen threshold:")
        print(f"       angle      : {best.angle_deg:+.1f}°")
        print(f"       coverage   : {best.n_feasible}/{best.n_cells} cells")
        print(f"       worst clear: {best.worst_min_clearance_mm:+.2f} mm")
        print(f"     Try one of:")
        print(f"       • lower --feasibility-threshold-mm further")
        print(f"       • shift --base-xyz away from the colliding obstacle")
        print(f"       • check if the partner arm pin or door angle is at fault")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
