"""End-to-end pick-and-place of a bowl using the HOOK arm (ur_left).

Parallel of ``pick_place_cup_full`` / ``pick_place_plate``: uses Drake motion
planning for free-space transits when enabled, then the standard ``pick`` /
``place`` contact primitives.

Hardware assumptions and running notes match the non-full
``pick_place_bowl_hook`` task; see that module's docstring for checklist.
"""

from __future__ import annotations

import argparse
from dataclasses import replace

import numpy as np

from ...arm import ArmHandle
from ...config import PickPlaceConfig
from ...grasps.bowl import bowl_hook_grasp
from ...pick import pick
from ...place import place
from ...session import Session, default_session
from ...moves import transit_xy
from ...util.poses import Pose, pose_at_altitude
from ...util.rtde_convert import rtde_to_pose
from ...world import World


# --- Tunables (edit to match your physical layout) ------------------------
BOWL_PICK_POSE_TASK = Pose(translation=[0.1, 0, 0.01])
BOWL_PLACE_POSE_TASK = Pose(translation=[-0.1, 0.2, 0.01])

GRASP_ANGLE_RAD = float(np.radians(180))
PLACE_ANGLE_RAD = float(np.radians(180 + 45))
APPROACH_TILT_RAD = float(np.radians(10))

ARM = "ur_left"

WORLD = World(
    include_microwave=True,
    include_objects=False,
    robotiq_mode="closed",
    microwave_door_open_rad=0.0,
)

USE_MOTION_PLANNING = True
MOTION_PLAN_PRE_PICK_APPROACH = True
MOTION_PLAN_RRT_FALLBACK = True
MOTION_PLAN_AUTO_FALLBACK = True
CARRY_MIN_CLEARANCE_M = 0.005
MOTION_PLAN_N_WAYPOINTS = 30
MOTION_PLAN_BLEND_R_M = 0.005

CONFIG = PickPlaceConfig(
    transit_z=0.3,
    place_use_contact_descent=False,
    transit_speed=0.1,
    transit_accel=0.2,
    approach_speed=0.05,
    approach_accel=0.2,
    retract_speed=0.1,
    retract_accel=0.2,
    release_aperture_mm=None,
    gripper_open_speed_pct=40,
    gripper_close_speed_pct=30,
)


def plan_pick():
    return bowl_hook_grasp(
        BOWL_PICK_POSE_TASK,
        angle_rad=GRASP_ANGLE_RAD,
        approach_tilt_rad=APPROACH_TILT_RAD,
    )


def plan_place() -> Pose:
    grasp_at_dest = bowl_hook_grasp(
        BOWL_PLACE_POSE_TASK,
        angle_rad=PLACE_ANGLE_RAD,
        approach_tilt_rad=APPROACH_TILT_RAD,
    )
    return grasp_at_dest.grasp_pose


def _print_plan(grasp, place_pose: Pose) -> None:
    print("=" * 60)
    print("  Arm            :", ARM, "(hook gripper)")
    print("  Pick location  :", BOWL_PICK_POSE_TASK.translation, "(task frame)")
    print("  Place location :", BOWL_PLACE_POSE_TASK.translation, "(task frame)")
    print("  Grasp angle    :", f"{np.degrees(GRASP_ANGLE_RAD):+.0f}°")
    print("  Place angle    :", f"{np.degrees(PLACE_ANGLE_RAD):+.0f}°")
    print("  Approach tilt  :", f"{np.degrees(APPROACH_TILT_RAD):+.1f}°")
    print("  Grasp pose     :", grasp.grasp_pose.translation, "(task frame)")
    print("  Place pose     :", place_pose.translation, "(task frame)")
    print("  Pregrasp offset:", f"{grasp.pregrasp_offset*100:.1f} cm")
    print("  Transit Z      :", CONFIG.transit_z, "m")
    print("=" * 60)


def _current_q(session: Session) -> dict[str, np.ndarray]:
    return {
        name: np.asarray(arm.receive.getActualQ(), dtype=float)
        for name, arm in session.arms.items()
    }


def _current_tcp_pose_task(arm: ArmHandle) -> Pose:
    return arm.to_task(rtde_to_pose(arm.receive.getActualTCPPose()))


def _hover_before_pick(grasp, config: PickPlaceConfig) -> Pose:
    return pose_at_altitude(grasp.grasp_pose, config.transit_z)


def _hover_before_place(place_pose: Pose, config: PickPlaceConfig) -> Pose:
    return pose_at_altitude(place_pose, config.transit_z)


def _build_planning_context(*, attached_bowl: bool):
    leg_world = (
        replace(WORLD, in_hand={ARM: ("bowl", None)})
        if attached_bowl else WORLD
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
    attached_bowl: bool,
) -> bool:
    print(f"\n→ planned transit: {label}")
    for i, wp in enumerate(waypoints):
        print(f"  wp {i}: xyz={np.round(wp.translation, 3)}")

    def _run_movel_fallback(reason: str) -> bool:
        print(f"  ➜ moveL fallback ({reason}); routing through sequential transit_xy")
        for wp in waypoints[1:]:
            transit_xy(
                arm, wp, config.transit_z,
                config.transit_speed, config.transit_accel,
            )
        return True

    if not USE_MOTION_PLANNING:
        return _run_movel_fallback("USE_MOTION_PLANNING=False")

    from ...planning.execute import execute_plan
    from ...planning.transit import (
        InfeasiblePlanError, make_rtde_ik, plan_transit,
    )

    diagram, plant, plant_context = _build_planning_context(
        attached_bowl=attached_bowl,
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
            min_clearance_m=CARRY_MIN_CLEARANCE_M if attached_bowl else 0.01,
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
            return _run_movel_fallback(f"execute_plan failed: {result.reason}")
        return False
    print("  ✓ planned transit reached.")
    return True


def run_on_arm(
    session: Session,
    arm: ArmHandle,
    grasp,
    place_pose: Pose,
    config: PickPlaceConfig = CONFIG,
    *,
    pick_only: bool = False,
) -> bool:
    if MOTION_PLAN_PRE_PICK_APPROACH:
        if not _planned_or_linear_transit(
            session, arm,
            "pre-pick approach to grasp hover",
            [_current_tcp_pose_task(arm), _hover_before_pick(grasp, config)],
            config,
            attached_bowl=False,
        ):
            return False

    print(f"\n→ pick: {grasp.description}")
    pick_result = pick(arm, grasp, config)
    if not pick_result.success:
        print(f"  ✗ pick FAILED: {pick_result.reason}")
        return False
    print("  ✓ pick succeeded.")

    if pick_only:
        print("\nDone — pick only (no place).")
        return True

    if not _planned_or_linear_transit(
        session, arm,
        "post-pick carry to place hover",
        [_current_tcp_pose_task(arm), _hover_before_place(place_pose, config)],
        config,
        attached_bowl=True,
    ):
        return False

    print(f"\n→ place @ {BOWL_PLACE_POSE_TASK.translation}")
    place_result = place(arm, place_pose, config)
    if not place_result.success:
        print(f"  ✗ place FAILED: {place_result.reason}")
        return False
    print("  ✓ place succeeded.")

    print("\nDone — arm retracted to transit altitude.")
    return True


def main(dry: bool = False, motion_planning: bool = True) -> int:
    if not motion_planning:
        global USE_MOTION_PLANNING
        USE_MOTION_PLANNING = False
        print("[CLI] motion planning DISABLED — transits use sequential moveL.")

    grasp = plan_pick()
    place_pose = plan_place()
    _print_plan(grasp, place_pose)

    if dry:
        print("[dry run] skipping RTDE connection. No motion commanded.")
        return 0

    left = ARM == "ur_left"
    right = ARM == "ur_right"
    with default_session(left=left, right=right) as session:
        arm = session.arms[ARM]
        return 0 if run_on_arm(session, arm, grasp, place_pose, CONFIG) else 1


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--dry", action="store_true")
    ap.add_argument(
        "--no-motion-planning",
        action="store_true",
        help="Disable Drake planning; use moveL transits only.",
    )
    args = ap.parse_args()
    raise SystemExit(main(
        dry=args.dry,
        motion_planning=not args.no_motion_planning,
    ))
