"""End-to-end pick-and-place of a cup on the right arm.

Mirrors ``pick_place_plate.py``: pick the cup with a top-down rim pinch,
transit, and place it. The grasp is a vertical (top-down) rim grasp —
fingers straddle the rim and close radially across the ~2 mm wall.

Hardware assumptions
--------------------
  - The right arm has a Robotiq 2F-85 attached and a pre-calibrated TCP
    offset (set in ``control_scripts/calibration.py``).
  - A cup sitting at task-frame ``CUP_PICK_POSE_TASK``. The cup's task-z
    is the resting surface — the rim ends up at task-z + CUP_HEIGHT_M.
  - ``CUP_PLACE_POSE_TASK`` is the intended cup base pose after placement.

Running
-------
Standalone:
    python -m control_scripts.tasks.pick_place_cup [--dry]

Via the unified entrypoint:
    python -m control_scripts.run task pick_place_cup [--dry]
"""

from __future__ import annotations

import argparse
from dataclasses import replace
from typing import Optional

import numpy as np

from ..arm import ArmHandle
from ..config import PickPlaceConfig
from ..grasps.cup import cup_rim_grasp
from ..moves import transit_xy
from ..pick import pick
from ..place import place
from ..session import Session, default_session
from ..util.poses import Pose, pose_at_altitude
from ..util.rtde_convert import rtde_to_pose
from ..world import World


# --- Tunables (edit to match your physical layout) ------------------------
CUP_PICK_POSE_TASK = Pose(translation=[-0.08, -0.125, 0.0])
# CUP_PICK_POSE_TASK = Pose(translation=[0.3, 0.075, 0.0])
"""Cup base at PICK location, expressed in task frame. Edit to match the
table layout. Z is the resting surface; the rim is at z + CUP_HEIGHT_M."""

CUP_PLACE_POSE_TASK = Pose(translation=[0, 0, 0.0])
# CUP_PLACE_POSE_TASK = Pose(translation=[0.33, 0.075, 0.0])
"""Cup base at PLACE location, task frame. Defaults to the same as the
pick pose (set it back down) — set to a different translation to relocate."""

FINAL_XY = np.array([0.35, 0.0])
"""Task-frame XY to move the arm to after placing, at ``CONFIG.transit_z``."""

GRASP_ANGLE_RAD = 0.0
"""Which rim angle to grasp at, in cup frame."""

PLACE_ANGLE_RAD = 0.0
"""Which rim angle to release at. Keeping it equal to GRASP_ANGLE_RAD
means the gripper's orientation is unchanged through the whole motion —
easier on the wrist than re-solving a new orientation at the destination."""

ARM = "ur_right"

SUPPORTS_SIM = True
"""``--mode sim`` plans and visualizes the three motion-planned segments
(pre-pick approach, post-pick carry, post-place return). The hand-coded
``pick`` / ``place`` primitives — descend, close/release, retract — are
not simulated; sim jumps the arm to the next leg's start state between
legs to represent those skipped contact moves."""

WORLD = World(
    include_microwave=True,
    include_objects=False,        # cup is welded via in_hand for the carry leg
    robotiq_mode="closed",
    microwave_door_open_rad=0.0,  # microwave is in the rig but not interacted with here
)
"""Single source of env truth for this task's planning + sim scenes."""

USE_MOTION_PLANNING = True
"""Use Drake ``plan_transit`` + ``execute_plan`` for free-space transits.
Override at the CLI with ``--no-motion-planning`` to disable entirely."""

MOTION_PLAN_PRE_PICK_APPROACH = True
"""When True, the lift+transit_xy steps inside ``pick`` are preceded by a
motion-planned transit from the rig's current TCP to the grasp hover.
Pick's own lift+transit_xy then become near-no-ops."""

MOTION_PLAN_RRT_FALLBACK = True
"""Try RRT after KTO when the optimizer cannot find a planned transit."""

MOTION_PLAN_AUTO_FALLBACK = True
"""On planner failure (``plan_transit`` raises or ``execute_plan`` reports
failure), fall back to the sequential moveL path with a loud warning
instead of failing the task."""

CARRY_MIN_CLEARANCE_M = 0.005
"""Minimum clearance for the in-hand carry."""

MOTION_PLAN_N_WAYPOINTS = 30
MOTION_PLAN_BLEND_R_M = 0.005

CONFIG = PickPlaceConfig(
    # Transit clearance above all workspace objects. Cup is 15.4 cm tall;
    # 30 cm leaves ~15 cm overhead for the held cup during transit.
    transit_z=0.30,
    place_use_contact_descent=False,  # no force-seeking; no table contact
    transit_speed=0.15,
    transit_accel=0.3,

    # Release aperture. Rim wall is ~2 mm; opening to 30 mm puts each
    # finger ~14 mm from TCP center, well clear of the rim wall (and the
    # cup's interior at the rim height ≈ 4.2 cm radius).
    release_aperture_mm=30,

    # Gripper speeds. Slower than 100% — gentle close on the thin rim,
    # controlled release so the cup doesn't get knocked when fingers part.
    gripper_open_speed_pct=40,
    gripper_close_speed_pct=30,
)


def plan_pick():
    return cup_rim_grasp(CUP_PICK_POSE_TASK, angle_rad=GRASP_ANGLE_RAD)


def plan_place() -> Pose:
    """TCP pose at release. Uses the same rim-grasp factory at the place
    location so the gripper's orientation stays consistent between pick
    and place."""
    grasp_at_dest = cup_rim_grasp(
        CUP_PLACE_POSE_TASK, angle_rad=PLACE_ANGLE_RAD
    )
    return grasp_at_dest.grasp_pose


def plan_final_pose(reference_pose: Pose) -> Pose:
    """Pose used only for final XY transit after the cup is released."""
    return Pose(
        translation=[FINAL_XY[0], FINAL_XY[1], 0.0],
        rotation=reference_pose.rotation,
    )


def _print_plan(grasp, place_pose: Pose) -> None:
    print("=" * 60)
    print("  Pick location  :", CUP_PICK_POSE_TASK.translation, "(task frame)")
    print("  Place location :", CUP_PLACE_POSE_TASK.translation, "(task frame)")
    print("  Grasp angle    :", f"{np.degrees(GRASP_ANGLE_RAD):+.0f}°")
    print("  Place angle    :", f"{np.degrees(PLACE_ANGLE_RAD):+.0f}°")
    print("  Grasp pose     :", grasp.grasp_pose.translation, "(task frame)")
    print("  Place pose     :", place_pose.translation, "(task frame)")
    print("  Final XY       :", FINAL_XY, f"(task frame @ z={CONFIG.transit_z})")
    print("  Transit Z      :", CONFIG.transit_z, "m")
    print("  Release aperture:", CONFIG.release_aperture_mm, "mm")
    print("  Gripper speeds :",
          f"open={CONFIG.gripper_open_speed_pct}%,",
          f"close={CONFIG.gripper_close_speed_pct}%")
    print("  Grasp force    :", grasp.grasp_force, "N")
    print("=" * 60)


def _current_q(session: Session) -> dict[str, np.ndarray]:
    """Current connected-arm joint positions for seeding Drake IK."""
    return {
        name: np.asarray(arm.receive.getActualQ(), dtype=float)
        for name, arm in session.arms.items()
    }


def _current_tcp_pose_task(arm: ArmHandle) -> Pose:
    """Read the real TCP pose and convert it into the shared task frame."""
    return arm.to_task(rtde_to_pose(arm.receive.getActualTCPPose()))


def _hover_before_pick(grasp, config: PickPlaceConfig) -> Pose:
    """The transit-altitude pose ``pick`` will target before descending."""
    return pose_at_altitude(grasp.grasp_pose, config.transit_z)


def _hover_before_place(place_pose: Pose, config: PickPlaceConfig) -> Pose:
    """The transit-altitude pose ``place`` will target before descending."""
    return pose_at_altitude(place_pose, config.transit_z)


def _build_planning_context(*, attached_cup: bool):
    """Build a planning scene for a single live planned transit.

    During the post-pick carry, weld a cup to the active TCP so collision
    checks include the object in hand."""
    leg_world = (
        replace(WORLD, in_hand={ARM: ("cup", None)})
        if attached_cup else WORLD
    )
    diagram, plant, _, _ = leg_world.build_planning_scene()
    root_context = diagram.CreateDefaultContext()
    plant_context = plant.GetMyMutableContextFromRoot(root_context)
    return diagram, plant, plant_context


def _planned_or_linear_transit(
    session: Session,
    arm: ArmHandle,
    label: str,
    waypoints: list,
    config: PickPlaceConfig,
    *,
    attached_cup: bool,
) -> bool:
    """Plan a free-space transit through ``waypoints`` and execute it.

    Routes through Drake ``plan_transit`` + ``execute_plan`` when
    ``USE_MOTION_PLANNING`` is True; falls back to sequential moveL via
    ``transit_xy`` otherwise OR on planner/exec failure when
    ``MOTION_PLAN_AUTO_FALLBACK`` is True.
    """
    print(f"\n→ planned transit: {label}")
    for i, wp in enumerate(waypoints):
        print(f"  wp {i}: xyz={np.round(wp.translation, 3)}")

    def _run_movel_fallback(reason: str) -> bool:
        print(f"  ➜ moveL fallback ({reason}); routing through "
              f"sequential transit_xy")
        for wp in waypoints[1:]:
            transit_xy(
                arm, wp, config.transit_z,
                config.transit_speed, config.transit_accel,
            )
        return True

    if not USE_MOTION_PLANNING:
        return _run_movel_fallback("USE_MOTION_PLANNING=False")

    from ..planning.execute import execute_plan
    from ..planning.transit import (
        InfeasiblePlanError, make_rtde_ik, plan_transit,
    )

    diagram, plant, plant_context = _build_planning_context(
        attached_cup=attached_cup,
    )
    try:
        plan = plan_transit(
            plant=plant,
            arm=ARM,
            waypoints=waypoints,
            plant_context=plant_context,
            current_q=_current_q(session),
            use_rrt_fallback=MOTION_PLAN_RRT_FALLBACK,
            rrt_diagram=diagram,
            min_clearance_m=CARRY_MIN_CLEARANCE_M if attached_cup else 0.01,
            rtde_ik=make_rtde_ik(arm),
        )
    except InfeasiblePlanError as exc:
        print(f"  ✗ motion plan infeasible: {exc}")
        if MOTION_PLAN_AUTO_FALLBACK:
            return _run_movel_fallback("planner raised InfeasiblePlanError")
        return False

    print(f"  planner={plan.metadata.get('planner')}  "
          f"duration={plan.duration_s:.2f}s  "
          f"clearance={plan.min_clearance_m * 1000:.1f}mm")

    result = execute_plan(
        plan, session,
        method="moveJ_path",
        n_waypoints=MOTION_PLAN_N_WAYPOINTS,
        blend_r_m=MOTION_PLAN_BLEND_R_M,
    )
    if not result.success:
        print(f"  ✗ planned transit execution failed: {result.reason}")
        if MOTION_PLAN_AUTO_FALLBACK:
            return _run_movel_fallback(
                f"execute_plan failed: {result.reason}",
            )
        return False
    print("  ✓ planned transit reached.")
    return True


def _plan_sim_legs(grasp, place_pose: Pose, config: PickPlaceConfig):
    """Plan the same motion-planned segments the real path plans, but
    from synthetic start states (no live RTDE).

    Three legs (when ``MOTION_PLAN_PRE_PICK_APPROACH=True``):
      1. pre-pick approach — HOME (FK) → cup grasp hover.
      2. post-pick carry   — grasp hover (cup in hand) → place hover.
      3. post-place return — place hover → final-XY hover.
    """
    from ..planning.transit import (
        InfeasiblePlanError,
        _arm_model_instance,
        _arm_position_indices,
        _tcp_frame,
        plan_transit,
        plan_transit_chained,
    )
    from ..util.rotations import Rotation as RotUtil

    legs: list[tuple[str, object]] = []
    chained_arm_q = None
    prev_terminal_pose: Optional[Pose] = None
    home_pose = None  # captured during the approach leg, reused on return fallback

    # ---- Leg 1: pre-pick approach (no cup) --------------------------
    if MOTION_PLAN_PRE_PICK_APPROACH:
        approach_diagram, approach_plant, _, _ = WORLD.build_planning_scene()
        approach_root = approach_diagram.CreateDefaultContext()
        approach_plant_ctx = approach_plant.GetMyMutableContextFromRoot(
            approach_root,
        )
        approach_arm_idx = _arm_position_indices(
            approach_plant, _arm_model_instance(approach_plant, ARM),
        )
        approach_tcp_frame = _tcp_frame(
            approach_plant, _arm_model_instance(approach_plant, ARM),
        )
        X = approach_tcp_frame.CalcPoseInWorld(approach_plant_ctx)
        home_pose = Pose(
            translation=np.asarray(X.translation()),
            rotation=RotUtil.from_matrix(X.rotation().matrix()),
        )
        approach_waypoints = [home_pose, _hover_before_pick(grasp, config)]
        print("\n[sim] planning pre-pick approach...")
        try:
            approach_plan = plan_transit(
                plant=approach_plant,
                arm=ARM,
                waypoints=approach_waypoints,
                plant_context=approach_plant_ctx,
                use_rrt_fallback=MOTION_PLAN_RRT_FALLBACK,
                rrt_diagram=approach_diagram,
                min_clearance_m=0.01,
            )
            legs.append(("pre-pick approach to grasp hover", approach_plan))
            chained_arm_q = np.asarray(
                approach_plan.trajectory.value(
                    approach_plan.trajectory.end_time(),
                )
            ).flatten()[approach_arm_idx]
            prev_terminal_pose = _hover_before_pick(grasp, config)
        except InfeasiblePlanError as exc:
            print(f"  ✗ pre-pick approach infeasible: {exc}")
            return legs

    # ---- Leg 2: post-pick carry (cup in hand) -----------------------
    carry_world = replace(WORLD, in_hand={ARM: ("cup", None)})
    carry_diagram, carry_plant, _, _ = carry_world.build_planning_scene()
    carry_root = carry_diagram.CreateDefaultContext()
    carry_plant_ctx = carry_plant.GetMyMutableContextFromRoot(carry_root)
    carry_arm_idx = _arm_position_indices(
        carry_plant, _arm_model_instance(carry_plant, ARM),
    )

    carry_waypoints = [
        _hover_before_pick(grasp, config),
        _hover_before_place(place_pose, config),
    ]
    try:
        carry_plan = plan_transit_chained(
            arm=ARM,
            log_label="post-pick carry",
            prev_terminal_pose=prev_terminal_pose,
            chained_arm_q=chained_arm_q,
            plant=carry_plant,
            waypoints=carry_waypoints,
            plant_context=carry_plant_ctx,
            current_q={ARM: chained_arm_q} if chained_arm_q is not None else None,
            use_rrt_fallback=MOTION_PLAN_RRT_FALLBACK,
            rrt_diagram=carry_diagram,
            min_clearance_m=CARRY_MIN_CLEARANCE_M,
        )
    except InfeasiblePlanError as exc:
        print(f"  ✗ post-pick carry plan infeasible: {exc}")
        return legs
    legs.append(("post-pick carry to place hover", carry_plan))
    chained_arm_q = np.asarray(
        carry_plan.trajectory.value(carry_plan.trajectory.end_time())
    ).flatten()[carry_arm_idx]
    prev_terminal_pose = carry_waypoints[-1]

    # ---- Leg 3: post-place return (no cup) → final XY hover ---------
    return_diagram, return_plant, _, _ = WORLD.build_planning_scene()
    return_root = return_diagram.CreateDefaultContext()
    return_plant_ctx = return_plant.GetMyMutableContextFromRoot(return_root)

    final_pose = pose_at_altitude(
        plan_final_pose(place_pose), config.transit_z,
    )
    return_waypoints = [
        _hover_before_place(place_pose, config),
        final_pose,
    ]

    try:
        return_plan = plan_transit_chained(
            arm=ARM,
            log_label="post-place return to final XY",
            prev_terminal_pose=prev_terminal_pose,
            chained_arm_q=chained_arm_q,
            plant=return_plant,
            waypoints=return_waypoints,
            plant_context=return_plant_ctx,
            current_q={ARM: chained_arm_q} if chained_arm_q is not None else None,
            use_rrt_fallback=MOTION_PLAN_RRT_FALLBACK,
            rrt_diagram=return_diagram,
            min_clearance_m=0.01,
        )
    except InfeasiblePlanError as exc:
        print(f"  ✗ post-place return plan infeasible: {exc}")
        return legs
    legs.append(("post-place return to final XY", return_plan))

    return legs


def _run_sim(grasp, place_pose: Pose, config: PickPlaceConfig) -> int:
    """Plan the carry legs, build a meshcat scene, hand off to the
    leg-by-leg stepper. The hand-coded ``pick`` / ``place`` /
    final-transit primitives are not simulated."""
    from pydrake.geometry import StartMeshcat
    from pydrake.systems.analysis import Simulator

    from ..planning import preview

    legs = _plan_sim_legs(grasp, place_pose, config)
    if not legs:
        print("[sim] no legs planned; aborting")
        return 1

    print()
    print("[sim] plan summary:")
    for label, plan in legs:
        clr = plan.min_clearance_m
        s = (f"{clr * 1000:.1f}mm" if np.isfinite(clr) else "n/a")
        print(f"  - {label}: planner={plan.metadata.get('planner')}  "
              f"duration={plan.duration_s:.2f}s  clearance={s}")

    sim_world = replace(WORLD, in_hand={ARM: ("cup", None)})
    meshcat = StartMeshcat()
    print(f"\n[sim] meshcat → {meshcat.web_url()}")
    scene = sim_world.build_sim_scene(meshcat=meshcat)

    simulator = Simulator(scene.diagram)
    simulator.Initialize()
    sim_ctx = simulator.get_mutable_context()

    return preview.run_interactive_legs(
        meshcat,
        scene.diagram,
        scene.plant,
        sim_ctx,
        legs,
    )


def run_on_arm(
    session: Session,
    arm: ArmHandle,
    grasp,
    place_pose: Pose,
    config: PickPlaceConfig = CONFIG,
) -> bool:
    """Execute the pick + place on a live ArmHandle. Returns True on success.

    Use this from a routine that owns the Session — it does not connect
    or disconnect on its own."""
    if MOTION_PLAN_PRE_PICK_APPROACH:
        if not _planned_or_linear_transit(
            session, arm,
            "pre-pick approach to grasp hover",
            [_current_tcp_pose_task(arm), _hover_before_pick(grasp, config)],
            config,
            attached_cup=False,
        ):
            return False

    print(f"\n→ pick: {grasp.description}")
    pick_result = pick(arm, grasp, config)
    if not pick_result.success:
        print(f"  ✗ pick FAILED: {pick_result.reason}")
        return False
    print("  ✓ pick succeeded.")

    if not _planned_or_linear_transit(
        session, arm,
        "post-pick carry to place hover",
        [_current_tcp_pose_task(arm), _hover_before_place(place_pose, config)],
        config,
        attached_cup=True,
    ):
        return False

    print(f"\n→ place @ {CUP_PLACE_POSE_TASK.translation}")
    place_result = place(arm, place_pose, config)
    if not place_result.success:
        print(f"  ✗ place FAILED: {place_result.reason}")
        return False
    print("  ✓ place succeeded.")

    final_pose = plan_final_pose(place_pose)
    if not _planned_or_linear_transit(
        session, arm,
        "post-place return to final XY",
        [_current_tcp_pose_task(arm),
         pose_at_altitude(final_pose, config.transit_z)],
        config,
        attached_cup=False,
    ):
        return False

    print("\nDone — arm retracted to transit altitude.")
    return True


def main(
    dry: bool = False,
    mode: str = "real",
    motion_planning: bool = True,
) -> int:
    """CLI / registry entry point. Returns 0 on success, 1 on task
    failure, leaves exceptions to the caller."""
    if not motion_planning:
        global USE_MOTION_PLANNING
        USE_MOTION_PLANNING = False
        print("[CLI] motion planning DISABLED — every transit will use the "
              "sequential moveL fallback (the original pre-planner flow).")

    grasp = plan_pick()
    place_pose = plan_place()
    _print_plan(grasp, place_pose)

    if dry:
        print("[dry run] skipping RTDE connection. No motion commanded.")
        return 0

    if mode == "sim":
        if not SUPPORTS_SIM:
            print("[sim] this task does not support sim mode")
            return 1
        return _run_sim(grasp, place_pose, CONFIG)
    if mode != "real":
        raise ValueError(f"unknown mode {mode!r}; choose 'real' or 'sim'")

    left = ARM == "ur_left"
    right = ARM == "ur_right"
    with default_session(left=left, right=right) as session:
        arm = session.arms[ARM]
        return 0 if run_on_arm(session, arm, grasp, place_pose, CONFIG) else 1


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument(
        "--dry",
        action="store_true",
        help="Plan and print the grasp/place poses without connecting to RTDE.",
    )
    ap.add_argument(
        "--mode",
        choices=["real", "sim"],
        default="real",
        help=(
            "Execution mode. 'real' (default) runs on the rig via RTDE. "
            "'sim' plans the planner-driven carry segments and replays them "
            "in meshcat with a leg-by-leg stepper; the hand-coded pick / "
            "place primitives are not simulated."
        ),
    )
    ap.add_argument(
        "--no-motion-planning",
        action="store_true",
        help=(
            "Disable Drake motion planning entirely. All transits route "
            "through sequential moveL Cartesian primitives — the pre-"
            "planner flow. ``MOTION_PLAN_AUTO_FALLBACK`` already does "
            "this automatically on per-segment plan failures; this flag "
            "forces it for every segment up front."
        ),
    )
    args = ap.parse_args()
    raise SystemExit(main(
        dry=args.dry, mode=args.mode,
        motion_planning=not args.no_motion_planning,
    ))
