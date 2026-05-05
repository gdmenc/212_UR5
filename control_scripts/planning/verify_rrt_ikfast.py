"""End-to-end test for the IKFast-seeded RRT goal-set planner.

Verifies that ``rrt_connect_to_tcp_pose``:
  1. Runs without raising for a reachable, sensible TCP pose target.
  2. Drops the IK branches that land in collision (e.g., the
     "elbow-down through the table" mirror solution at most plate
     positions).
  3. Returns a path whose final config FKs back to the requested
     TCP pose within tight tolerances (≤ 5 mm position, ≤ 1 deg
     rotation).

Run::

    python3.11 -m control_scripts.planning.verify_rrt_ikfast
    python3.11 -m control_scripts.planning.verify_rrt_ikfast --pick shortest
    python3.11 -m control_scripts.planning.verify_rrt_ikfast --visualize

With ``--visualize`` opens Meshcat and animates each test case.
"""

from __future__ import annotations

import argparse
import time
from dataclasses import dataclass
from typing import List, Optional, Tuple

import numpy as np
from pydrake.geometry import Rgba, Sphere, StartMeshcat
from pydrake.math import RigidTransform
from pydrake.systems.analysis import Simulator

from . import default_home_q
from ..util.poses import Pose
from ..util.rotations import Rotation
from .build_scene import build_scene
from .rrt import (
    GoalSetReport,
    RRTFailure,
    RRTPlan,
    build_planning_scene,
    ikfast_goal_branches,
    make_collision_checker,
    path_length,
    rrt_connect_to_tcp_pose,
    shortcut_path,
    _arm_position_indices,
)
from .scene.objects import (
    BOTTLE_DEFAULT_TASK_XYZ,
    BOWL_DEFAULT_TASK_XYZ,
    CUP_DEFAULT_TASK_XYZ,
    PLATE_DEFAULT_TASK_XYZ,
)
from .transit import _arm_model_instance


# Tool +Z = task -Z (gripper points straight down). Used for grasp-style
# poses where the wrist hangs above the object.
_R_DOWN = Rotation.from_matrix(np.array([
    [1.0,  0.0,  0.0],
    [0.0, -1.0,  0.0],
    [0.0,  0.0, -1.0],
]))


@dataclass
class CaseSpec:
    name: str
    arm: str
    tcp_target_task: Pose


def _make_cases() -> List[CaseSpec]:
    """Two regimes:

    * **tabletop hovers** — TCP 12 cm above an object with tool +Z = task -Z.
      Right-arm objects work directly (elbow stays on the +x side of the
      Vention). Left-arm hovers over operator-side objects FAIL because the
      cup sits near the centerline and the left elbow has to fold back
      *through* the Vention truss to reach there with the TCP pointing
      straight down. This is a real rig constraint, not a planner bug.

    * **free-space hovers** — TCP at a point off to the arm's own side at
      mid altitude. No table reach, no Vention conflict. Both arms succeed
      — confirms the IKFast + RRT pipeline works end to end on both arms.
    """
    def hover(xyz, dz=0.12):
        return Pose(translation=np.array([xyz[0], xyz[1], xyz[2] + dz]),
                    rotation=_R_DOWN)
    return [
        # Tabletop hovers (table-reach regime)
        CaseSpec("right_hover_plate",  "ur_right",
                 hover(PLATE_DEFAULT_TASK_XYZ)),
        CaseSpec("right_hover_bowl",   "ur_right",
                 hover(BOWL_DEFAULT_TASK_XYZ)),
        CaseSpec("left_hover_cup",     "ur_left",
                 hover(CUP_DEFAULT_TASK_XYZ)),
        CaseSpec("left_hover_bottle",  "ur_left",
                 hover(BOTTLE_DEFAULT_TASK_XYZ)),
        # Free-space hovers (no Vention conflict)
        CaseSpec("right_freespace",    "ur_right",
                 Pose(translation=np.array([+0.93, 0.03, 0.40]),
                      rotation=_R_DOWN)),
        CaseSpec("left_freespace",     "ur_left",
                 Pose(translation=np.array([-0.93, 0.03, 0.40]),
                      rotation=_R_DOWN)),
    ]


def _fk_tcp_world(plant, plant_ctx, arm_name) -> RigidTransform:
    short = arm_name.removeprefix("ur_")
    return plant.GetFrameByName(f"tcp_{short}").CalcPoseInWorld(plant_ctx)


def _verdict(ok: bool, msg: str) -> str:
    return f"{'OK  ' if ok else 'FAIL'}  {msg}"


def _run_case(case: CaseSpec, *, pick: str, max_iters: int) -> Tuple[
    Optional[RRTPlan], Optional[GoalSetReport], dict
]:
    """Plan one case. Returns (plan, report, info_dict)."""
    diag, plant, _, _ = build_planning_scene(include_objects=True)
    home = default_home_q(plant)
    checker = make_collision_checker(diag, plant, case.arm)

    # Diagnostic: how many ikfast branches before any collision filter?
    arm_inst = _arm_model_instance(plant, case.arm)
    arm_idx = _arm_position_indices(plant, arm_inst)
    seed = home[arm_idx]
    raw_branches = ikfast_goal_branches(
        case.arm, case.tcp_target_task, seed_arm_q=seed,
    )

    info = {"raw_branches": len(raw_branches)}

    t0 = time.time()
    try:
        plan, report = rrt_connect_to_tcp_pose(
            checker,
            q_start_full=home,
            tcp_pose_task=case.tcp_target_task,
            planning_arm_name=case.arm,
            max_iters=max_iters,
            pick=pick,
            seed=0,
        )
    except RRTFailure as exc:
        info["error"] = str(exc)
        info["wall_s"] = time.time() - t0
        return None, None, info
    info["wall_s"] = time.time() - t0
    info["nodes"] = len(plan.path_full)

    # FK the path's last config to confirm we landed at the requested TCP.
    diag_ctx = diag.CreateDefaultContext()
    plant_ctx = plant.GetMyMutableContextFromRoot(diag_ctx)
    plant.SetPositions(plant_ctx, plan.path_full[-1])
    X_world_tcp_actual = _fk_tcp_world(plant, plant_ctx, case.arm)

    # Requested TCP in world frame: world == task in this scene.
    requested_xyz = np.asarray(case.tcp_target_task.translation)
    actual_xyz = np.asarray(X_world_tcp_actual.translation())
    info["pos_err_mm"] = float(np.linalg.norm(actual_xyz - requested_xyz)) * 1000

    R_req = case.tcp_target_task.rotation.as_matrix()
    R_act = np.asarray(X_world_tcp_actual.rotation().matrix())
    R_rel = R_req.T @ R_act
    cos_t = (np.trace(R_rel) - 1.0) / 2.0
    info["rot_err_deg"] = float(np.degrees(np.arccos(np.clip(cos_t, -1, 1))))

    return plan, report, info


def _print_row(case: CaseSpec, plan, report, info: dict) -> None:
    if plan is None:
        print(f"  {case.name:<22} {case.arm:<9} → "
              f"FAIL ({info.get('error', 'unknown')})")
        return
    pos_ok = info["pos_err_mm"] <= 5.0
    rot_ok = info["rot_err_deg"] <= 1.0
    fk_status = "OK" if (pos_ok and rot_ok) else "X"
    print(
        f"  {case.name:<22} {case.arm:<9}  "
        f"raw={info['raw_branches']:<2} "
        f"col={report.n_in_collision:<2} "
        f"goal#={report.chosen_goal_index:<2} "
        f"planner={report.chosen_planner:<14} "
        f"nodes={info['nodes']:<3} "
        f"FK Δ={info['pos_err_mm']:>5.1f}mm/{info['rot_err_deg']:>4.2f}° {fk_status}  "
        f"({info['wall_s']*1000:>5.0f} ms)"
    )


def _animate_path(diagram, plant, sim_ctx, path_full, *, seconds: float = 4.0):
    plant_ctx = plant.GetMyMutableContextFromRoot(sim_ctx)
    if len(path_full) < 2:
        plant.SetPositions(plant_ctx, path_full[0])
        diagram.ForcedPublish(sim_ctx)
        time.sleep(seconds)
        return
    # Densify by linear interpolation between waypoints.
    dense: List[np.ndarray] = []
    n_per = 25
    for q0, q1 in zip(path_full[:-1], path_full[1:]):
        for s in np.linspace(0.0, 1.0, n_per, endpoint=False):
            dense.append(q0 + s * (q1 - q0))
    dense.append(path_full[-1])
    t0 = time.time()
    while True:
        elapsed = time.time() - t0
        if elapsed >= seconds:
            break
        s = min(1.0, elapsed / seconds)
        idx = min(int(s * (len(dense) - 1)), len(dense) - 1)
        plant.SetPositions(plant_ctx, dense[idx])
        diagram.ForcedPublish(sim_ctx)
        time.sleep(0.02)


def _visualize(cases_with_plans, *, animate_seconds: float = 4.0,
               gripper_mode: str = "closed") -> int:
    meshcat = StartMeshcat()
    print(f"\n[verify_rrt_ikfast] Meshcat → {meshcat.web_url()}")

    scene = build_scene(meshcat=meshcat, robotiq_mode=gripper_mode)
    diagram, plant = scene.diagram, scene.plant
    sim = Simulator(diagram)
    sim.Initialize()
    sim_ctx = sim.get_mutable_context()

    # Drop a marker per case at the requested TCP target.
    for i, (case, _plan, _info) in enumerate(cases_with_plans):
        xyz = np.asarray(case.tcp_target_task.translation)
        meshcat.SetObject(f"/{case.name}/target", Sphere(0.014),
                          Rgba(0.95, 0.6, 0.1, 0.9))
        meshcat.SetTransform(f"/{case.name}/target",
                             RigidTransform(xyz))

    case_names = [c.name for c, p, _ in cases_with_plans if p is not None]
    if not case_names:
        print("No successful plans to animate.")
        return 1

    n = len(case_names)
    meshcat.AddSlider("case", min=1, max=n, step=1, value=1)
    meshcat.AddButton("Replay")
    meshcat.AddButton("Quit")
    last_replay = meshcat.GetButtonClicks("Replay")
    last_idx = 0

    by_name = {c.name: (c, p, info) for c, p, info in cases_with_plans
               if p is not None}

    print("\nInteractive: pick case via slider, Replay to re-animate.")
    try:
        while True:
            if meshcat.GetButtonClicks("Quit") > 0:
                break
            cur = int(round(meshcat.GetSliderValue("case")))
            replay_clicks = meshcat.GetButtonClicks("Replay")
            if cur != last_idx or replay_clicks > last_replay:
                name = case_names[cur - 1]
                _case, plan, _info = by_name[name]
                print(f"[play] {name}  ({len(plan.path_full)} nodes)")
                _animate_path(diagram, plant, sim_ctx, plan.path_full,
                              seconds=animate_seconds)
                last_idx, last_replay = cur, replay_clicks
            time.sleep(0.05)
    except KeyboardInterrupt:
        print("Closing.")
    finally:
        for nm in ("Replay", "Quit"):
            try:
                meshcat.DeleteButton(nm)
            except Exception:
                pass
        try:
            meshcat.DeleteSlider("case")
        except Exception:
            pass
    return 0


def main(*, pick: str, max_iters: int, visualize: bool,
         animate_seconds: float, gripper_mode: str) -> int:
    cases = _make_cases()
    print()
    print("=" * 110)
    print(f"  IKFast-seeded RRT  (pick={pick}, max_iters={max_iters})")
    print("=" * 110)
    print(f"  {'case':<22} {'arm':<9}  raw col goal# planner         "
          f"nodes  FK error          wall")
    results: List[Tuple[CaseSpec, Optional[RRTPlan], dict]] = []
    for case in cases:
        plan, report, info = _run_case(case, pick=pick, max_iters=max_iters)
        _print_row(case, plan, report, info)
        results.append((case, plan, info))

    if visualize:
        return _visualize(results, animate_seconds=animate_seconds,
                          gripper_mode=gripper_mode)
    return 0


if __name__ == "__main__":
    ap = argparse.ArgumentParser(
        description="End-to-end test for ikfast-seeded multi-goal RRT.",
    )
    ap.add_argument(
        "--pick", choices=["first", "shortest"], default="first",
        help="Whether to return the first reachable goal or the shortest path.",
    )
    ap.add_argument(
        "--max-iters", type=int, default=2000,
        help="RRT iteration budget per goal candidate.",
    )
    ap.add_argument(
        "--visualize", action="store_true",
        help="Open Meshcat and animate each successful plan.",
    )
    ap.add_argument(
        "--animate-seconds", type=float, default=4.0,
        help="Seconds per animation playthrough.",
    )
    ap.add_argument(
        "--gripper-mode", choices=["closed", "open"], default="closed",
    )
    args = ap.parse_args()
    raise SystemExit(main(
        pick=args.pick,
        max_iters=args.max_iters,
        visualize=args.visualize,
        animate_seconds=args.animate_seconds,
        gripper_mode=args.gripper_mode,
    ))
