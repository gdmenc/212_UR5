"""Send one or both arms to a hard-coded home joint configuration.

By default uses the standard motion-planning workflow (``plan_transit``
+ ``execute_plan``) so the swing to home is collision-aware — important
after tasks that end with the arm inside the microwave cavity. Falls
back to a direct ``moveJ`` if the planner fails or you pass
``--no-motion-planning``.

After a successful planned transit, a final ``moveJ`` snaps the arm to
exact home q (the planner reaches the home POSE in some IK branch,
which may not be exactly the configured home q — the snap is a small
correction that brings the joint config home).

Defaults to ``SIM_HOME_Q_LEFT`` / ``SIM_HOME_Q_RIGHT`` from
``control_scripts/planning/__init__.py``; edit the ``HOME_Q_*``
constants below for a different home.

Running
-------
    python3.11 -m control_scripts.tasks.go_home                       # both arms, planned
    python3.11 -m control_scripts.tasks.go_home --arm right
    python3.11 -m control_scripts.tasks.go_home --no-motion-planning  # bare moveJ
    python3.11 -m control_scripts.tasks.go_home --door-open-deg 103   # plan with door open
    python3.11 -m control_scripts.tasks.go_home --dry                 # print plan only
    python3.11 -m control_scripts.tasks.go_home --yes --speed 0.5     # auto, faster
"""

from __future__ import annotations

import argparse
from typing import Optional

import numpy as np

from ..arm import ArmHandle
from ..planning import SIM_HOME_Q_LEFT, SIM_HOME_Q_RIGHT
from ..planning.execute import execute_plan
from ..planning.transit import (
    InfeasiblePlanError,
    _arm_model_instance,
    _tcp_frame,
    make_rtde_ik,
    plan_transit,
)
from ..session import Session, default_session
from ..util.poses import Pose
from ..util.rotations import Rotation
from ..util.rtde_convert import rtde_to_pose
from ..world import World


# --- Tunables (edit to change the home joint configuration) ----------------
HOME_Q_LEFT = np.asarray(SIM_HOME_Q_LEFT, dtype=float)
HOME_Q_RIGHT = np.asarray(SIM_HOME_Q_RIGHT, dtype=float)

DEFAULT_SPEED_RAD_S = 0.3
DEFAULT_ACCEL_RAD_S2 = 0.5

# Planning / execution knobs (mirror the cup-task defaults).
MOTION_PLAN_RRT_FALLBACK = True
MOTION_PLAN_RRT_MAX_ITERS = 2000
MOTION_PLAN_RRT_SHORTCUT_ATTEMPTS = 20
MOTION_PLAN_N_WAYPOINTS = 30
MOTION_PLAN_BLEND_R_M = 0.005
MOTION_PLAN_MIN_CLEARANCE_M = 0.01


# --- Helpers ---------------------------------------------------------------

def _confirm(arm_label: str, q_target: np.ndarray, q_current: np.ndarray) -> bool:
    print(f"\n  arm     : {arm_label}")
    print(f"  current : {np.round(np.degrees(q_current), 1)}°")
    print(f"  target  : {np.round(np.degrees(q_target), 1)}°")
    print(f"  delta   : {np.round(np.degrees(q_target - q_current), 1)}°")
    ans = input("  proceed? [Enter = yes / n = abort] ").strip().lower()
    return ans in ("", "y", "yes")


def _build_world(door_open_rad: float, partner_q: dict[str, np.ndarray]) -> World:
    """Planning scene for the go-home transit. Includes microwave + all
    default static objects so the planner avoids them. ``partner_q``
    pins non-planning arms at their actual current configuration."""
    return World(
        include_microwave=True,
        microwave_door_open_rad=door_open_rad,
        include_objects=True,
        robotiq_mode="closed",
        partner_arm_q=partner_q,
    )


def _fk_target_pose(world: World, arm_name: str, target_q: np.ndarray) -> Pose:
    """FK ``target_q`` through a fresh planning scene to get the home TCP
    pose in task frame. Throwaway scene — only used to read the FK."""
    diagram, plant, _, _ = world.build_planning_scene()
    plant_ctx = plant.GetMyMutableContextFromRoot(diagram.CreateDefaultContext())
    inst = _arm_model_instance(plant, arm_name)
    plant.SetPositions(plant_ctx, inst, np.asarray(target_q, dtype=float))
    X = _tcp_frame(plant, inst).CalcPoseInWorld(plant_ctx)
    return Pose(
        translation=np.asarray(X.translation()),
        rotation=Rotation.from_matrix(X.rotation().matrix()),
    )


def _moveJ_to_home(
    arm: ArmHandle, target_q: np.ndarray, speed: float, accel: float,
) -> None:
    arm.control.moveJ(list(target_q), speed=speed, acceleration=accel)


def _planned_to_home(
    session: Session,
    arm: ArmHandle,
    arm_name: str,
    target_q: np.ndarray,
    *,
    door_open_rad: float,
) -> tuple[bool, Optional[str]]:
    """Plan a collision-aware transit from current TCP to FK(target_q)
    and execute it. Returns (success, reason_if_failed)."""
    # Partner pin from any other connected arms' actual q.
    partner_q = {
        name: np.asarray(a.receive.getActualQ(), dtype=float)
        for name, a in session.arms.items()
        if name != arm_name
    }
    world = _build_world(door_open_rad, partner_q)

    current_pose = arm.to_task(rtde_to_pose(arm.receive.getActualTCPPose()))
    target_pose = _fk_target_pose(world, arm_name, target_q)

    diagram, plant, _, _ = world.build_planning_scene()
    plant_ctx = plant.GetMyMutableContextFromRoot(diagram.CreateDefaultContext())

    # Seed the planning arm with its actual q so the planner's start IK
    # matches reality; partner pins are baked into the world.
    current_q = {
        name: np.asarray(a.receive.getActualQ(), dtype=float)
        for name, a in session.arms.items()
    }

    print(f"  planning transit: current → home ...")
    try:
        plan = plan_transit(
            plant=plant,
            arm=arm_name,
            waypoints=[current_pose, target_pose],
            plant_context=plant_ctx,
            current_q=current_q,
            use_rrt_fallback=MOTION_PLAN_RRT_FALLBACK,
            rrt_diagram=diagram,
            rrt_max_iters=MOTION_PLAN_RRT_MAX_ITERS,
            rrt_shortcut_attempts=MOTION_PLAN_RRT_SHORTCUT_ATTEMPTS,
            min_clearance_m=MOTION_PLAN_MIN_CLEARANCE_M,
            rtde_ik=make_rtde_ik(arm),
        )
    except InfeasiblePlanError as exc:
        return False, f"planner: {exc}"

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
        return False, f"execute: {result.reason}"
    return True, None


def _send_arm_home(
    session: Session,
    arm_name: str,
    target_q: np.ndarray,
    *,
    door_open_rad: float,
    speed: float,
    accel: float,
    motion_planning: bool,
    dry: bool,
    skip_confirm: bool,
) -> bool:
    arm = session.arms[arm_name]
    q_current = np.asarray(arm.receive.getActualQ(), dtype=float)

    if not skip_confirm:
        if not _confirm(arm_name, target_q, q_current):
            print(f"  [{arm_name}] aborted by user.")
            return False

    if dry:
        print(f"  [{arm_name}] [dry] would "
              f"{'plan + execute' if motion_planning else 'moveJ directly'} "
              f"to home (no motion commanded).")
        return True

    print(f"\n→ go_home {arm_name}")
    if motion_planning:
        ok, reason = _planned_to_home(
            session, arm, arm_name, target_q,
            door_open_rad=door_open_rad,
        )
        if ok:
            # Planner reaches the home POSE; the IK branch may differ
            # slightly from target_q. Snap to exact home q with a small
            # moveJ — bounded delta, no collision risk at this point.
            print("  ✓ planned transit reached. Snapping to exact home q ...")
            _moveJ_to_home(arm, target_q, speed, accel)
            print("  ✓ at home q.")
            return True
        print(f"  ✗ planning failed ({reason}); falling back to direct moveJ")

    _moveJ_to_home(arm, target_q, speed, accel)
    print("  ✓ at home q (moveJ).")
    return True


def main(
    arm: str = "both",
    dry: bool = False,
    speed: float = DEFAULT_SPEED_RAD_S,
    accel: float = DEFAULT_ACCEL_RAD_S2,
    skip_confirm: bool = False,
    motion_planning: bool = True,
    door_open_rad: float = 0.0,
) -> int:
    if arm not in ("left", "right", "both"):
        raise ValueError(f"--arm must be left, right, or both; got {arm!r}")

    left = arm in ("left", "both")
    right = arm in ("right", "both")

    print("=" * 50)
    print("  go_home")
    print(f"  arm(s)         : {'left+right' if arm == 'both' else arm}")
    print(f"  motion plan    : {'ON' if motion_planning else 'OFF (direct moveJ)'}")
    print(f"  door open      : {np.degrees(door_open_rad):.0f}°")
    print(f"  fallback speed : {speed} rad/s")
    print(f"  fallback accel : {accel} rad/s²")
    print(f"  mode           : {'DRY' if dry else 'REAL'}")
    print("=" * 50)

    if dry:
        # Skip the RTDE connection entirely — print intended plan and exit.
        if left:
            print(f"  [ur_left] target  : "
                  f"{np.round(np.degrees(HOME_Q_LEFT), 1)}°")
        if right:
            print(f"  [ur_right] target : "
                  f"{np.round(np.degrees(HOME_Q_RIGHT), 1)}°")
        print("[dry run] skipping RTDE connection. No motion commanded.")
        return 0

    with default_session(left=left, right=right) as session:
        ok = True
        if left:
            ok &= _send_arm_home(
                session, "ur_left", HOME_Q_LEFT,
                door_open_rad=door_open_rad,
                speed=speed, accel=accel,
                motion_planning=motion_planning,
                dry=False, skip_confirm=skip_confirm,
            )
        if right:
            ok &= _send_arm_home(
                session, "ur_right", HOME_Q_RIGHT,
                door_open_rad=door_open_rad,
                speed=speed, accel=accel,
                motion_planning=motion_planning,
                dry=False, skip_confirm=skip_confirm,
            )

    return 0 if ok else 1


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument(
        "--arm", choices=["left", "right", "both"], default="both",
        help="Which arm(s) to send home (default: both).",
    )
    ap.add_argument(
        "--dry", action="store_true",
        help="Print target q + delta but don't connect or command motion.",
    )
    ap.add_argument(
        "--speed", type=float, default=DEFAULT_SPEED_RAD_S,
        help=(
            f"moveJ joint speed (rad/s, default {DEFAULT_SPEED_RAD_S}). "
            "Used for the snap-to-exact-q after a planned transit, and "
            "as the fallback if the planner fails."
        ),
    )
    ap.add_argument(
        "--accel", type=float, default=DEFAULT_ACCEL_RAD_S2,
        help=(
            f"moveJ joint accel (rad/s², default {DEFAULT_ACCEL_RAD_S2})."
        ),
    )
    ap.add_argument(
        "--yes", action="store_true",
        help="Skip the per-arm confirmation prompt.",
    )
    ap.add_argument(
        "--no-motion-planning", action="store_true",
        help=(
            "Disable Drake motion planning entirely. Use a direct moveJ "
            "(NOT collision-aware). Default is to plan with auto-fallback "
            "to moveJ on planner failure."
        ),
    )
    ap.add_argument(
        "--door-open-deg", type=float, default=0.0,
        help=(
            "Microwave door angle (degrees) to model in the planning "
            "scene. Default 0 (closed). Set to ~103 if returning home "
            "with the door open (matches MICROWAVE_DOOR_OPEN_ANGLE_RAD "
            "= 1.8 in the bowl/plate microwave tasks)."
        ),
    )
    args = ap.parse_args()
    raise SystemExit(main(
        arm=args.arm,
        dry=args.dry,
        speed=args.speed,
        accel=args.accel,
        skip_confirm=args.yes,
        motion_planning=not args.no_motion_planning,
        door_open_rad=float(np.radians(args.door_open_deg)),
    ))
