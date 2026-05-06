"""Push the microwave door closed with the left arm (no grasp).

Uses the hand-recorded task-frame waypoints from the provided
``saved_at=2026-05-05T01:14:19`` close snapshot. Three phases:

  1. Motion-planned approach — current rig q → preclosedoor pose,
     planning against an OPEN microwave (door panel modeled at +90°)
     plus a bottle welded on the microwave roof.
  2. Recorded close motion — preclosedoor moveJ → closedoor moveJ
     (the existing ``close_microwave_door`` primitive).
  3. Motion-planned return — closedoor pose → start q, planning against
     a CLOSED microwave plus the bottle.

Running
-------
    python3.11 -m control_scripts.tasks.close_microwave_full
    python3.11 -m control_scripts.tasks.close_microwave_full --dry
    python3.11 -m control_scripts.tasks.close_microwave_full --mode sim

First-run checklist
-------------------
1. ``--dry`` first — confirm the two joint waypoints match the latest
   ``preclosedoor`` and ``closedoor`` snapshots.
2. ``--mode sim`` next — preview phase 1 + phase 3 in meshcat.
3. Confirm the door is open enough for the preclose pose before running.
4. Keep joint speed modest so the final push does not slam the door.
"""

from __future__ import annotations

import argparse

import numpy as np

from ...config import PickPlaceConfig
from ...session import default_session
from ...world import World
from ...lab_landmarks import BOTTLE_MICROWAVE_TOP_XYZ_TASK
from ...planning.execute import execute_plan
from ...planning.transit import (
    InfeasiblePlanError,
    make_rtde_ik,
    plan_transit,
    plan_transit_chained,
)
from .close_microwave import (
    CloseMicrowaveDoorSpec,
    close_microwave_door,
)
from ...util.poses import Pose
from ...util.rotations import Rotation
from ...util.rtde_convert import rtde_to_pose


# ---------------------------------------------------------------------------
#  Waypoints — sourced from the provided 2026-05-05T01:14:19 close snapshot
# ---------------------------------------------------------------------------

# Source: snapshot "preclosedoor"
PRECLOSE_DOOR_POSE_TASK = Pose(
    translation=np.array([-0.6093772150461308, 0.27488770991396877, 0.41131672797896235]),
    rotation=Rotation.from_rotvec(
        [-1.0250496228801607, 1.3970250241161535, 0.8022438222398867]
    ),
)

# Source: snapshot "closedoor"
CLOSED_DOOR_POSE_TASK = Pose(
    translation=np.array([-0.27459578866963485, 0.25997937217088424, 0.41819012355144713]),
    rotation=Rotation.from_rotvec(
        [-0.5442464470297064, 1.4736026545243532, 0.3871995361438228]
    ),
)

PRECLOSE_DOOR_JOINTS_RAD = [
    1.5467852354049683,
    -1.1462249618819733,
    0.7166374365435999,
    -1.3493216794780274,
    -2.396539036427633,
    -2.1319807211505335,
]
CLOSED_DOOR_JOINTS_RAD = [
    1.897124171257019,
    -1.0298115772059937,
    1.0604260603534144,
    -1.3474183839610596,
    -2.403875176106588,
    -2.132052723561422,
]

ARM = "ur_left"

DOOR_SPEC = CloseMicrowaveDoorSpec(
    handle_closed_pose_task=CLOSED_DOOR_POSE_TASK,
    hinge_position_task=np.zeros(3),
    pull_direction_task=np.array([-1.0, -1.0, 0.0]),
    preclose_pose_task=PRECLOSE_DOOR_POSE_TASK,
    preclose_joints_rad=PRECLOSE_DOOR_JOINTS_RAD,
    closed_pose_task=CLOSED_DOOR_POSE_TASK,
    closed_joints_rad=CLOSED_DOOR_JOINTS_RAD,
    joint_speed=0.8,          # rad/s — matches open_microwave
    joint_accel=0.5,          # rad/s²
)

CONFIG = PickPlaceConfig(
    transit_z=0.25,         # same as open_microwave
    transit_speed=0.1,
    transit_accel=0.2,
    approach_speed=0.04,    # slow push so door doesn't slam
    approach_accel=0.1,
    retract_speed=0.10,
    retract_accel=0.2,
)


# ---------------------------------------------------------------------------
#  Motion-planning worlds
# ---------------------------------------------------------------------------
# Phase 1 (approach) plans against an OPEN microwave so the planner avoids
# the open door panel. Phase 3 (return) plans against a CLOSED microwave —
# the door has just been pushed shut. Both worlds weld a bottle on top of
# the microwave so the planner avoids it while routing free-space transits.
MICROWAVE_DOOR_OPEN_ANGLE_RAD = np.deg2rad(95.0)
"""Welded door angle for Phase 1's planning scene. Slightly past 90° so the
modeled panel sits a little farther from the closed-door volume than the
arm's actual recorded open angle (~69°) — gives the planner extra clearance
margin when routing around the open door."""

# Bottle stashed on the microwave roof — same landmark used by
# pour_bottle_hook, so the close-microwave planner sees the same
# obstacle the bottle-stash task placed there.
# Drop every default static object except the bottle — close-microwave
# doesn't interact with the cup/plate/bowl/tray, so leaving them in the
# scene only inflates collision-checker work and risks IK rejection at
# the recorded preclose/closed waypoints.
_BOTTLE_ONLY_KW = dict(
    include_objects=True,
    skip_static_objects=("plate", "cup", "cup_with_stick", "bowl", "tray"),
    object_xyz_overrides={"bottle": tuple(BOTTLE_MICROWAVE_TOP_XYZ_TASK)},
)

WORLD_DOOR_OPEN = World(
    include_microwave=True,
    robotiq_mode="closed",
    microwave_door_open_rad=MICROWAVE_DOOR_OPEN_ANGLE_RAD,
    **_BOTTLE_ONLY_KW,
)
WORLD_DOOR_CLOSED = World(
    include_microwave=True,
    robotiq_mode="closed",
    microwave_door_open_rad=0.0,
    **_BOTTLE_ONLY_KW,
)

MOTION_PLAN_N_WAYPOINTS = 30
MOTION_PLAN_BLEND_R_M = 0.005
MOTION_PLAN_RRT_FALLBACK = True

SUPPORTS_SIM = True


# ---------------------------------------------------------------------------
#  Run
# ---------------------------------------------------------------------------

def run(dry: bool) -> None:
    print("=" * 60)
    print("  Arm               :", ARM)
    print("  Preclose (task)   :", np.round(PRECLOSE_DOOR_POSE_TASK.translation, 4))
    print("  Closed (task)     :", np.round(CLOSED_DOOR_POSE_TASK.translation, 4))
    print("  Joint speed       :", DOOR_SPEC.joint_speed, "rad/s")
    print("-" * 60)
    print("  Close waypoints:")
    print("    [1] preclosedoor", np.round(PRECLOSE_DOOR_JOINTS_RAD, 4))
    print("    [2] closedoor   ", np.round(CLOSED_DOOR_JOINTS_RAD, 4))
    print("=" * 60)

    if dry:
        print("[dry run] skipping RTDE connection. No motion commanded.")
        return

    with default_session(left=True, right=False) as session:
        arm = session.arms[ARM]

        # Capture start state — phase 3 returns here.
        start_q = np.array(arm.receive.getActualQ(), dtype=float)
        start_pose_task = arm.to_task(rtde_to_pose(arm.receive.getActualTCPPose()))

        # --- Phase 1: motion-planned approach (door OPEN) ---
        print("\n→ phase 1: motion-planned approach to preclosedoor (world: door open)")
        p1_diagram, p1_plant, _, _ = WORLD_DOOR_OPEN.build_planning_scene()
        p1_ctx = p1_plant.GetMyMutableContextFromRoot(p1_diagram.CreateDefaultContext())
        try:
            plan1 = plan_transit(
                plant=p1_plant, arm=ARM,
                waypoints=[start_pose_task, PRECLOSE_DOOR_POSE_TASK],
                plant_context=p1_ctx,
                current_q={ARM: start_q},
                use_rrt_fallback=MOTION_PLAN_RRT_FALLBACK,
                rrt_diagram=p1_diagram,
                rtde_ik=make_rtde_ik(arm),
            )
        except InfeasiblePlanError as exc:
            print(f"  ✗ phase 1 plan infeasible: {exc}")
            return
        execute_plan(
            plan1, session,
            method="moveJ_path",
            n_waypoints=MOTION_PLAN_N_WAYPOINTS,
            blend_r_m=MOTION_PLAN_BLEND_R_M,
        )

        # --- Phase 2: existing close motion (preclosedoor → closedoor) ---
        print("\n→ phase 2: close microwave door (preclosedoor → closedoor)")
        result = close_microwave_door(arm, DOOR_SPEC, CONFIG)
        if not result.success:
            print(f"  ✗ FAILED: {result.reason}")
            return
        print("  ✓ door closed.")

        # --- Phase 3: motion-planned return to start q (door CLOSED) ---
        # Use plan_transit_chained so the planner can try every ikfast
        # branch at the closed-door pose as an IK seed — matches the
        # canonical chained-leg pattern used by the other _full tasks.
        # min_clearance_m=0 because the TCP starts at the closed-door
        # surface; full clearance would reject the start state.
        print("\n→ phase 3: motion-planned return to start q (world: door closed)")
        cur_q = np.array(arm.receive.getActualQ(), dtype=float)
        p3_diagram, p3_plant, _, _ = WORLD_DOOR_CLOSED.build_planning_scene()
        p3_ctx = p3_plant.GetMyMutableContextFromRoot(p3_diagram.CreateDefaultContext())
        try:
            plan3 = plan_transit_chained(
                arm=ARM,
                log_label="phase 3 return",
                prev_terminal_pose=CLOSED_DOOR_POSE_TASK,
                chained_arm_q=np.asarray(CLOSED_DOOR_JOINTS_RAD, dtype=float),
                plant=p3_plant,
                waypoints=[CLOSED_DOOR_POSE_TASK, start_pose_task],
                plant_context=p3_ctx,
                current_q={ARM: cur_q},
                use_rrt_fallback=MOTION_PLAN_RRT_FALLBACK,
                rrt_diagram=p3_diagram,
                min_clearance_m=0.0,
                rtde_ik=make_rtde_ik(arm),
            )
        except InfeasiblePlanError as exc:
            print(f"  ✗ phase 3 plan infeasible: {exc}")
            return
        execute_plan(
            plan3, session,
            method="moveJ_path",
            n_waypoints=MOTION_PLAN_N_WAYPOINTS,
            blend_r_m=MOTION_PLAN_BLEND_R_M,
        )
        print("  ✓ returned to start q.")


# ---------------------------------------------------------------------------
#  Sim mode — plan phase 1 + phase 3 and replay them in meshcat.
# ---------------------------------------------------------------------------
# Phase 2 (the recorded preclose→closed moveJ pair) is hand-coded and
# cannot be simulated meaningfully — sim mode covers only the motion-
# planned legs, just like ``open_microwave_full``.

def _run_sim() -> int:
    from pydrake.geometry import StartMeshcat
    from pydrake.systems.analysis import Simulator

    from ...planning import preview
    from ...planning.transit import _arm_model_instance, _tcp_frame

    legs: list = []

    # --- Phase 1: HOME (FK) → preclosedoor, world door OPEN ---
    p1_diagram, p1_plant, _, _ = WORLD_DOOR_OPEN.build_planning_scene()
    p1_ctx = p1_plant.GetMyMutableContextFromRoot(p1_diagram.CreateDefaultContext())
    home_X = _tcp_frame(p1_plant, _arm_model_instance(p1_plant, ARM)).CalcPoseInWorld(p1_ctx)
    home_pose = Pose(
        translation=np.asarray(home_X.translation()),
        rotation=Rotation.from_matrix(home_X.rotation().matrix()),
    )
    print("\n[sim] planning phase 1 (HOME → preclosedoor, door open)...")
    try:
        plan1 = plan_transit(
            plant=p1_plant, arm=ARM,
            waypoints=[home_pose, PRECLOSE_DOOR_POSE_TASK],
            plant_context=p1_ctx,
            use_rrt_fallback=MOTION_PLAN_RRT_FALLBACK,
            rrt_diagram=p1_diagram,
        )
        legs.append(("phase 1: HOME → preclosedoor", plan1))
    except InfeasiblePlanError as exc:
        print(f"  ✗ phase 1 plan infeasible: {exc}")

    # --- Phase 3: closedoor → HOME, world door CLOSED ---
    # Sim mode skips Phase 2's hand-coded moveJs, so the post-phase-2
    # joint config we'd feed Phase 3 isn't available. Approximate it
    # with CLOSED_DOOR_JOINTS_RAD (which is exactly where Phase 2 ends
    # on the rig).
    p3_diagram, p3_plant, _, _ = WORLD_DOOR_CLOSED.build_planning_scene()
    p3_ctx = p3_plant.GetMyMutableContextFromRoot(p3_diagram.CreateDefaultContext())
    print("\n[sim] planning phase 3 (closedoor → HOME, door closed)...")
    try:
        plan3 = plan_transit_chained(
            arm=ARM,
            log_label="phase 3 return",
            prev_terminal_pose=CLOSED_DOOR_POSE_TASK,
            chained_arm_q=np.asarray(CLOSED_DOOR_JOINTS_RAD, dtype=float),
            plant=p3_plant,
            waypoints=[CLOSED_DOOR_POSE_TASK, home_pose],
            plant_context=p3_ctx,
            current_q={ARM: np.asarray(CLOSED_DOOR_JOINTS_RAD, dtype=float)},
            use_rrt_fallback=MOTION_PLAN_RRT_FALLBACK,
            rrt_diagram=p3_diagram,
            min_clearance_m=0.0,
        )
        legs.append(("phase 3: closedoor → HOME", plan3))
    except InfeasiblePlanError as exc:
        print(f"  ✗ phase 3 plan infeasible: {exc}")

    if not legs:
        print("[sim] no legs planned; aborting")
        return 1

    print("\n[sim] plan summary:")
    for label, plan in legs:
        clr = (plan.min_clearance_m * 1000
               if np.isfinite(plan.min_clearance_m) else float("nan"))
        print(f"  - {label}: planner={plan.metadata.get('planner')}  "
              f"duration={plan.duration_s:.2f}s  clearance={clr:.1f}mm")

    meshcat = StartMeshcat()
    print(f"\n[sim] meshcat → {meshcat.web_url()}")
    sim_scene = WORLD_DOOR_CLOSED.build_sim_scene(meshcat=meshcat)
    simulator = Simulator(sim_scene.diagram)
    simulator.Initialize()
    sim_ctx = simulator.get_mutable_context()

    return preview.run_interactive_legs(
        meshcat,
        sim_scene.diagram,
        sim_scene.plant,
        sim_ctx,
        legs,
    )


def main() -> None:
    ap = argparse.ArgumentParser(
        description="Push microwave door closed with the left arm. Phase 1 and "
                    "phase 3 are motion-planned; phase 2 replays the recorded "
                    "preclose→closed moveJ snapshots."
    )
    ap.add_argument(
        "--dry",
        action="store_true",
        help="Print planned arc waypoints without connecting to RTDE.",
    )
    ap.add_argument(
        "--mode",
        choices=["real", "sim"],
        default="real",
        help="'real' (default) runs on the rig via RTDE. 'sim' plans the "
             "motion-planned legs (phase 1 + phase 3) and replays them in "
             "meshcat with a leg-by-leg stepper; the hand-coded close motion "
             "(phase 2) is NOT simulated.",
    )
    args = ap.parse_args()

    if args.mode == "sim":
        if args.dry:
            print("[sim] --dry has no effect in sim mode (no RTDE involved)")
        raise SystemExit(_run_sim())
    run(dry=args.dry)


if __name__ == "__main__":
    main()
