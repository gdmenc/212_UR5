"""End-to-end pick-and-place of a bowl using the HOOK arm, with the
pick or place pose optionally inside the microwave.

Same hook-on-rim grasp as ``pick_place_bowl_hook.py``. The microwave-
side half routes through ``pick_from_box`` / ``place_into_box`` so the
wrist stays below the cavity ceiling during transit inside.

Toggling sides
--------------
Edit ``PICK_FROM`` / ``PLACE_TO`` at the top of the file to choose
which side is inside the microwave. ``"outside"`` uses the standard
``pick`` / ``place`` flow with ``BOWL_PICK_POSE_TASK`` /
``BOWL_PLACE_POSE_TASK``. ``"microwave"`` overrides the location to
the microwave's interior center (from ``microwave.py``) at the glass-
tray height + 1 cm.

Approach angle
--------------
The microwave door faces task -Y. For the wrist to exit cleanly back
through the door, the hook's forearm should point in -Y, which means
GRASP_ANGLE_RAD = -π/2 (hook approaches the bowl from the -Y side).
The current value GRASP_ANGLE_RAD = π was set when the door was
assumed to face -X — RE-TUNE before running a microwave-side leg
with the bowl. Plate task is the priority for now; this task is wired
up for end-to-end test once the bowl is being staged.

Approach tilt
-------------
``APPROACH_TILT_RAD_OUTSIDE = +15°`` keeps the wrist clear of the
table for free-standing bowls. Inside the microwave the +15° tilt
would lift the wrist by ~2.7 cm into the 23 cm ceiling — so
``APPROACH_TILT_RAD_MICROWAVE = 0`` (pure vertical descent).

Running
-------
Standalone:
    python -m control_scripts.tasks.pick_place_bowl_hook_microwave [--dry]
"""

from __future__ import annotations

import argparse
from typing import Literal

import numpy as np

from ..arm import ArmHandle
from ..config import PickPlaceConfig
from ..grasps.bowl import bowl_hook_grasp
from ..microwave import (
    MICROWAVE_CEILING_Z,
    MICROWAVE_CENTER_XY_TASK,
    MICROWAVE_FLOOR_Z,
    entry_xy_for_pose,
)
from ..moves import transit_xy
from ..pick import pick, pick_from_box
from ..place import place, place_into_box
from ..session import Session, default_session
from ..util.poses import Pose, offset_along_tool_z, pose_at_altitude
from ..util.rtde_convert import rtde_to_pose


# --- Tunables --------------------------------------------------------------

PICK_FROM: Literal["outside", "microwave"] = "outside"
PLACE_TO: Literal["outside", "microwave"] = "microwave"

# Free-standing bowl poses (used when the corresponding side is "outside").
BOWL_PICK_POSE_TASK = Pose(translation=[0.05, -0.125, -0.01])
BOWL_PLACE_POSE_TASK = Pose(translation=[0.05, -0.125, -0.01])

# Intermediate waypoint between pick and place. The transit_z is what
# actually sets the carry altitude; the Z below is ignored. Picked at a
# Cartesian location that breaks long pick→place swings into two
# shorter, predictable legs — moveL between far-apart poses can over-
# extend through awkward joint configurations even when the start and
# end are reachable. Move this until the path looks clean on the rig.
BOWL_MIDPOINT_POSE_TASK = Pose(translation=[-0.1, 0.1, 0.0])
USE_MIDPOINT = True
"""Whether to carry the held bowl through ``BOWL_MIDPOINT_POSE_TASK``
between pick and place. Off-by-default would replicate the failure mode
the user just saw on the rig — leave True unless you've shortened the
pick↔place geometry."""

# Bowl-frame angle at which to engage the rim. π = approach from −X
# side; forearm exits back through the −X microwave door. Keep at π
# for any microwave-side leg.
GRASP_ANGLE_RAD = float(np.radians(180-30)) 
PLACE_ANGLE_RAD = float(-np.radians(90 + 10))  
MIDPOINT_ANGLE_RAD = PLACE_ANGLE_RAD
"""Bowl-frame angle used at the midpoint. Mirrors the plate microwave task:
the midpoint has an explicit orientation knob rather than inferring from
PICK_FROM / PLACE_TO branches. Default matches the place leg so wrist
rotation happens before the final entry/place move."""

# Tilt of the descent off pure-vertical (rotation around tool +Y).
# +15° lifts the wrist ~2.7 cm to clear the table for free-standing bowls.
# Inside the microwave that lift would scrape the 23 cm ceiling — so 0.
APPROACH_TILT_RAD_OUTSIDE = float(np.radians(15))
APPROACH_TILT_RAD_MICROWAVE = 0.0

# Constrained altitude inside the microwave cavity. Bowl rim sits at
# task z 8 cm (tray) + 7.2 cm (bowl height) = 15.2 cm. Hook TCP at the
# rim grasp is at 15.2 cm; the wrist sits at the same task z (offset is
# horizontal, see calibration.TCP_OFFSET_HOOK). 18 cm gives ~3 cm
# clearance over the rim during in-cavity transit and ~5 cm clearance
# below the 23 cm ceiling.
MICROWAVE_ENTRY_Z = 0.165

BOWL_ENTRY_CLEARANCE = 0.15
"""Distance outside the microwave door before lowering to ``MICROWAVE_ENTRY_Z``.
Keep this larger than the hook TCP envelope plus the bowl rim diameter so
the bowl/hook clears the front lip before descending."""

# Bowl center sits 1 cm above the glass tray to avoid scraping it on
# the descent — same convention as the free-standing pose.
MICROWAVE_BOWL_Z_OFFSET = 0.0

# In-cavity pregrasp standoff override for the microwave-side leg.
# Default HOOK_RIM_PREGRASP_OFFSET (5 cm) puts the pregrasp at task z
# 21.2 cm — only 1.8 cm under the 23 cm ceiling, tight. 3 cm gives
# 19.2 cm pregrasp, 3.8 cm of margin. Outside the microwave the global
# 5 cm default still applies (better friction-break before ascent).
MICROWAVE_HOOK_PREGRASP_OFFSET = 0.03

ARM = "ur_left"

CONFIG = PickPlaceConfig(
    transit_z=0.35,
    # 3 cm preplace standoff for the microwave place leg. The 10 cm
    # default would land preplace at task z 26.2 cm — ABOVE the 23 cm
    # ceiling. 3 cm matches MICROWAVE_HOOK_PREGRASP_OFFSET for symmetric
    # behaviour between pick and place. Plenty for an open-loop drop;
    # outside the cavity the moveL distance is harmless.
    preplace_offset=0.03,
    place_use_contact_descent=False,
    transit_speed=0.2,
    transit_accel=0.4,
    approach_speed=0.1,
    approach_accel=0.25,
    retract_speed=0.1,
    retract_accel=0.25,
    release_aperture_mm=None,  # hook has no continuous aperture
    gripper_open_speed_pct=40,
    gripper_close_speed_pct=30,
)

MICROWAVE_DOOR_OPEN_ANGLE_RAD = 1.8
"""Door angle used by planning scenes for bowl-to-microwave motions.
Matches the tuned open-microwave task value (~103°), so the door does not
appear as an obstacle to the carry-to-entry plan."""

USE_MOTION_PLANNING = True
"""Use Drake ``plan_transit`` + ``execute_plan`` for free-space carry segments.
``False`` falls back to the original sequential ``transit_xy`` moveL routing."""

MOTION_PLAN_RRT_FALLBACK = True
"""Try RRT after KTO when the optimizer cannot find a planned transit."""

KEEP_BOWL_LEVEL_DURING_CARRY = True
"""Constrain the post-pick planned carry so the bowl orientation does not
drift much from the orientation it had at the start of the carry. This is the
hook-bowl analog of ``KEEP_PLATE_LEVEL_DURING_CARRY`` in the plate task."""

CARRY_BOWL_LEVEL_TOLERANCE_RAD = float(np.radians(3.0))
"""Allowed bowl-up-axis tilt away from task +Z during the in-hand carry."""

CARRY_MIN_CLEARANCE_M = 0.005
"""Minimum clearance for the in-hand carry. The default planner clearance is
1 cm but the welded bowl + cup-with-stick obstacle leave only millimetres of
margin in places — keep this positive so penetration still fails, but allow a
tighter corridor."""

INCLUDE_CUP_WITH_STICK_OBSTACLE = True
"""When True, the cup-with-stick is treated as a static obstacle on the table
during planning so the bowl carry routes around it. Other tabletop objects are
still skipped because the bowl itself is welded to the gripper for planning."""

MOTION_PLAN_N_WAYPOINTS = 30
MOTION_PLAN_BLEND_R_M = 0.005
MOTION_PLAN_EXECUTION_METHOD = "servoJ"
"""Execution method for planned transits.

``servoJ`` streams dense setpoints from the planned trajectory and is smoother
for KTO carries. Set to ``"moveJ_path"`` to use sparse blended waypoints.
"""
MOTION_PLAN_SERVO_DT_S = 0.008
MOTION_PLAN_SERVO_TIME_SCALE = 2.0
"""Stretch servoJ execution in time. 2.0 means half-speed playback."""


# --- Pose / grasp planning -------------------------------------------------

def _bowl_pose_for(side: str, default: Pose) -> Pose:
    if side == "microwave":
        return Pose(translation=[
            MICROWAVE_CENTER_XY_TASK[0],
            MICROWAVE_CENTER_XY_TASK[1],
            MICROWAVE_FLOOR_Z + MICROWAVE_BOWL_Z_OFFSET,
        ])
    return default


def _tilt_for(side: str) -> float:
    return (
        APPROACH_TILT_RAD_MICROWAVE
        if side == "microwave"
        else APPROACH_TILT_RAD_OUTSIDE
    )


def plan_pick():
    grasp = bowl_hook_grasp(
        _bowl_pose_for(PICK_FROM, BOWL_PICK_POSE_TASK),
        angle_rad=GRASP_ANGLE_RAD,
        approach_tilt_rad=_tilt_for(PICK_FROM),
    )
    if PICK_FROM == "microwave":
        # Tighter pregrasp standoff so the in-cavity moveL endpoint
        # stays under the cavity ceiling. See MICROWAVE_HOOK_PREGRASP_OFFSET
        # docstring above.
        grasp.pregrasp_offset = MICROWAVE_HOOK_PREGRASP_OFFSET
    return grasp


def plan_place() -> Pose:
    grasp_at_dest = bowl_hook_grasp(
        _bowl_pose_for(PLACE_TO, BOWL_PLACE_POSE_TASK),
        angle_rad=PLACE_ANGLE_RAD,
        approach_tilt_rad=_tilt_for(PLACE_TO),
    )
    return grasp_at_dest.grasp_pose


def plan_midpoint(
    angle_rad: float = MIDPOINT_ANGLE_RAD,
    approach_tilt_rad: float | None = None,
) -> Pose:
    """TCP pose for carrying the held bowl through the midpoint XY at
    transit_z. Orientation is explicit, matching the plate microwave task's
    current midpoint planning style. By default the tilt still follows the
    place side so microwave-side entries stay ceiling-safe."""
    if approach_tilt_rad is None:
        approach_tilt_rad = _tilt_for(PLACE_TO)
    grasp_at_midpoint = bowl_hook_grasp(
        BOWL_MIDPOINT_POSE_TASK,
        angle_rad=angle_rad,
        approach_tilt_rad=approach_tilt_rad,
    )
    return grasp_at_midpoint.grasp_pose


def _current_q(session: Session) -> dict[str, np.ndarray]:
    """Current connected-arm joint positions for seeding Drake IK."""
    return {
        name: np.asarray(arm.receive.getActualQ(), dtype=float)
        for name, arm in session.arms.items()
    }


def _current_tcp_pose_task(arm: ArmHandle) -> Pose:
    """Read the real TCP pose and convert it into the shared task frame."""
    return arm.to_task(rtde_to_pose(arm.receive.getActualTCPPose()))


def _bowl_up_axis_in_tcp(carry_start_pose: Pose) -> np.ndarray:
    """Task-frame +Z expressed in the TCP frame at the start of the carry.

    Constraining this body-fixed axis to stay near task +Z during the carry
    keeps the bowl orientation roughly the same as it was right after the
    pick (up to yaw around world up). The hook grasp leaves the bowl tilted
    by the approach tilt angle, so this is "preserve the carry-start tilt"
    rather than "force the bowl level" — which is what we want anyway for
    not splashing whatever is in the bowl.
    """
    return carry_start_pose.rotation.inv().apply([0.0, 0.0, 1.0])


def _build_planning_context(*, attached_bowl: bool):
    """Build a planning scene for a single live planned transit.

    Tabletop demo objects are skipped so stale defaults do not block a live
    run, except for the cup-with-stick which we deliberately keep as an
    obstacle (the user setup has it remaining on the table during the bowl
    pick-and-place). During the post-pick carry, weld a bowl to the active
    TCP so collision checks include the object in hand.
    """
    from ..planning.rrt import build_planning_scene

    if INCLUDE_CUP_WITH_STICK_OBSTACLE:
        include_objects = True
        skip_static_objects = ("plate", "cup", "bowl", "bottle", "tray")
    else:
        include_objects = False
        skip_static_objects = ()

    attached_objects = (("bowl", ARM, None),) if attached_bowl else ()
    diagram, plant, _, _ = build_planning_scene(
        include_objects=include_objects,
        skip_static_objects=skip_static_objects,
        robotiq_mode="closed",
        microwave_door_open_angle_rad=MICROWAVE_DOOR_OPEN_ANGLE_RAD,
        attached_objects=attached_objects,
    )
    root_context = diagram.CreateDefaultContext()
    plant_context = plant.GetMyMutableContextFromRoot(root_context)
    return diagram, plant, plant_context


def _hover_before_place(place_pose: Pose, config: PickPlaceConfig) -> Pose:
    """The transit-altitude pose that ``place`` / ``place_into_box`` will
    target before descending."""
    if PLACE_TO == "microwave":
        entry_xy = entry_xy_for_pose(
            place_pose,
            clearance=BOWL_ENTRY_CLEARANCE,
        )
        entry_pose = Pose(
            translation=np.array([entry_xy[0], entry_xy[1], MICROWAVE_ENTRY_Z]),
            rotation=place_pose.rotation,
        )
        return pose_at_altitude(entry_pose, config.transit_z)

    preplace = offset_along_tool_z(place_pose, config.preplace_offset)
    return pose_at_altitude(preplace, config.transit_z)


def _planned_or_linear_transit(
    session: Session,
    arm: ArmHandle,
    label: str,
    waypoints: list[Pose],
    config: PickPlaceConfig,
    *,
    attached_bowl: bool,
) -> bool:
    """Move through hover/free-space waypoints.

    ``USE_MOTION_PLANNING=False`` preserves the old moveL behavior while still
    honoring multiple waypoints.
    """
    print(f"\n→ planned transit: {label}")
    for i, wp in enumerate(waypoints):
        print(f"  wp {i}: xyz={np.round(wp.translation, 3)}")

    if not USE_MOTION_PLANNING:
        print("  motion planning disabled; using sequential transit_xy moveL")
        for wp in waypoints[1:]:
            transit_xy(
                arm, wp, config.transit_z,
                config.transit_speed, config.transit_accel,
            )
        return True

    from ..planning.execute import execute_plan
    from ..planning.transit import InfeasiblePlanError, plan_transit

    bowl_up_tcp = (
        _bowl_up_axis_in_tcp(waypoints[0])
        if attached_bowl and KEEP_BOWL_LEVEL_DURING_CARRY
        else None
    )
    if bowl_up_tcp is not None:
        print("  carry bowl level: bowl-up axis within "
              f"±{np.degrees(CARRY_BOWL_LEVEL_TOLERANCE_RAD):.1f}° of task +Z")

    current_tcp = _current_tcp_pose_task(arm)
    tcp_xyz_err = np.asarray(current_tcp.translation) - np.asarray(waypoints[0].translation)
    tcp_rot_delta = waypoints[0].rotation.inv() * current_tcp.rotation
    print("  live TCP vs wp0: "
          f"dxyz={np.round(tcp_xyz_err * 1000.0, 1)} mm, "
          f"drot={np.degrees(np.linalg.norm(tcp_rot_delta.as_rotvec())):.2f}°")
    if bowl_up_tcp is not None:
        live_bowl_up = current_tcp.rotation.apply(bowl_up_tcp)
        live_bowl_up = live_bowl_up / np.linalg.norm(live_bowl_up)
        live_tilt = np.degrees(np.arccos(np.clip(
            float(np.dot(live_bowl_up, np.array([0.0, 0.0, 1.0]))),
            -1.0,
            1.0,
        )))
        print(f"  live bowl tilt estimate: {live_tilt:.2f}° from task +Z")

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
            align_tcp_axis=bowl_up_tcp,
            align_tcp_axis_world=np.array([0.0, 0.0, 1.0]),
            align_tcp_axis_tolerance_rad=CARRY_BOWL_LEVEL_TOLERANCE_RAD,
            min_clearance_m=CARRY_MIN_CLEARANCE_M if attached_bowl else 0.01,
        )
    except InfeasiblePlanError as exc:
        print(f"  ✗ motion plan infeasible: {exc}")
        return False

    print(f"  planner={plan.metadata.get('planner')}  "
          f"duration={plan.duration_s:.2f}s  "
          f"clearance={plan.min_clearance_m * 1000:.1f}mm")
    fallback = plan.metadata.get("spline_fallback_reason")
    if fallback:
        print(f"  (spline fell back to KTO: {fallback})")
    kto_fallback = plan.metadata.get("kto_fallback_reason")
    if kto_fallback:
        print(f"  (KTO fell back to RRT: {kto_fallback})")

    result = execute_plan(
        plan,
        session,
        method=MOTION_PLAN_EXECUTION_METHOD,
        n_waypoints=MOTION_PLAN_N_WAYPOINTS,
        dt=MOTION_PLAN_SERVO_DT_S,
        servo_time_scale=MOTION_PLAN_SERVO_TIME_SCALE,
        blend_r_m=MOTION_PLAN_BLEND_R_M,
    )
    if not result.success:
        print(f"  ✗ planned transit execution failed: {result.reason}")
        return False
    print("  ✓ planned transit reached.")
    return True


def _check_in_cavity_clearance(grasp, place_pose: Pose) -> None:
    """Loudly warn if any in-cavity TCP pose (place, grasp, preplace,
    pregrasp) sits above the cavity ceiling minus a small margin.
    For the hook at angle π / tilt 0 the wrist Z equals the TCP Z
    (offset is horizontal), so checking TCP Z against the ceiling
    is the same as checking wrist Z."""
    margin = 0.01  # 1 cm safety margin under the ceiling
    ceiling_safe = MICROWAVE_CEILING_Z - margin
    poses_to_check = []
    if PICK_FROM == "microwave":
        pregrasp = offset_along_tool_z(grasp.grasp_pose, grasp.pregrasp_offset)
        poses_to_check.append(("grasp", grasp.grasp_pose.translation[2]))
        poses_to_check.append(("pregrasp", pregrasp.translation[2]))
    if PLACE_TO == "microwave":
        preplace = offset_along_tool_z(place_pose, CONFIG.preplace_offset)
        poses_to_check.append(("place", place_pose.translation[2]))
        poses_to_check.append(("preplace", preplace.translation[2]))
    overshoots = [(name, z) for name, z in poses_to_check if z > ceiling_safe]
    if not overshoots:
        return
    print("!" * 60)
    print(" IN-CAVITY CLEARANCE WARNING — pose(s) above ceiling minus margin")
    print(f"  ceiling z (- {margin*100:.0f} cm margin) = {ceiling_safe*100:.1f} cm")
    for name, z in overshoots:
        print(f"  {name:>10} z = {z*100:.1f} cm  (overshoot {(z - ceiling_safe)*100:+.1f} cm)")
    print(" Reduce MICROWAVE_HOOK_PREGRASP_OFFSET or CONFIG.preplace_offset.")
    print("!" * 60)


def _print_plan(grasp, place_pose: Pose) -> None:
    print("=" * 60)
    print(f"  Arm           : {ARM} (hook gripper)")
    print(f"  Pick from     : {PICK_FROM}")
    print(f"  Place to      : {PLACE_TO}")
    print(f"  Grasp pose    : {grasp.grasp_pose.translation} (task)")
    print(f"  Place pose    : {place_pose.translation} (task)")
    print(f"  Grasp angle   : {np.degrees(GRASP_ANGLE_RAD):+.0f}°")
    print(f"  Place angle   : {np.degrees(PLACE_ANGLE_RAD):+.0f}°")
    print(f"  Tilt (pick)   : {np.degrees(_tilt_for(PICK_FROM)):+.1f}°")
    print(f"  Tilt (place)  : {np.degrees(_tilt_for(PLACE_TO)):+.1f}°")
    print(f"  Midpoint      : "
          f"{BOWL_MIDPOINT_POSE_TASK.translation if USE_MIDPOINT else 'disabled'}"
          f"{' (task)' if USE_MIDPOINT else ''}")
    if USE_MIDPOINT:
        print(f"  Midpoint angle: {np.degrees(MIDPOINT_ANGLE_RAD):+.0f}°")
    print(f"  Transit Z     : {CONFIG.transit_z} m")
    if PICK_FROM == "microwave" or PLACE_TO == "microwave":
        print(f"  Microwave entry Z : {MICROWAVE_ENTRY_Z} m")
        print(f"  Entry clearance   : {BOWL_ENTRY_CLEARANCE} m")
        if PICK_FROM == "microwave":
            xy = entry_xy_for_pose(
                grasp.grasp_pose,
                clearance=BOWL_ENTRY_CLEARANCE,
            )
            pregrasp = offset_along_tool_z(grasp.grasp_pose, grasp.pregrasp_offset)
            print(f"  Entry XY (pick)   : {xy}")
            print(f"  Pregrasp Z (pick) : {pregrasp.translation[2]:.3f} m"
                  f"  (offset {grasp.pregrasp_offset*100:.0f} cm)")
        if PLACE_TO == "microwave":
            xy = entry_xy_for_pose(
                place_pose,
                clearance=BOWL_ENTRY_CLEARANCE,
            )
            preplace = offset_along_tool_z(place_pose, CONFIG.preplace_offset)
            print(f"  Entry XY (place)  : {xy}")
            print(f"  Preplace Z (place): {preplace.translation[2]:.3f} m"
                  f"  (offset {CONFIG.preplace_offset*100:.0f} cm)")
    print("=" * 60)
    _check_in_cavity_clearance(grasp, place_pose)


# --- Execution -------------------------------------------------------------

def run_on_arm(
    session: Session,
    arm: ArmHandle,
    grasp,
    place_pose: Pose,
    config: PickPlaceConfig = CONFIG,
) -> bool:
    print(f"\n→ pick: {grasp.description}  (from {PICK_FROM})")
    if PICK_FROM == "microwave":
        entry_xy = entry_xy_for_pose(
            grasp.grasp_pose,
            clearance=BOWL_ENTRY_CLEARANCE,
        )
        pick_result = pick_from_box(
            arm, grasp, entry_xy, MICROWAVE_ENTRY_Z, config
        )
    else:
        pick_result = pick(arm, grasp, config)
    if not pick_result.success:
        print(f"  ✗ pick FAILED: {pick_result.reason}")
        return False
    print("  ✓ pick succeeded.")

    carry_waypoints = [_current_tcp_pose_task(arm)]
    if USE_MIDPOINT:
        midpoint_pose = plan_midpoint(angle_rad=MIDPOINT_ANGLE_RAD)
        carry_waypoints.append(pose_at_altitude(midpoint_pose, config.transit_z))
    carry_waypoints.append(_hover_before_place(place_pose, config))
    if not _planned_or_linear_transit(
        session,
        arm,
        "post-pick carry to place hover",
        carry_waypoints,
        config,
        attached_bowl=True,
    ):
        return False

    if USE_MIDPOINT:
        print("  ✓ carried through midpoint.")

    # ``place`` / ``place_into_box`` will re-issue its own transit-to-hover
    # command. Because the planned transit ends at that same hover pose, that
    # call becomes a short alignment/no-op before the straight Cartesian
    # descent/contact sequence.
    print(f"\n→ place @ {place_pose.translation}  (to {PLACE_TO})")
    if PLACE_TO == "microwave":
        entry_xy = entry_xy_for_pose(
            place_pose,
            clearance=BOWL_ENTRY_CLEARANCE,
        )
        place_result = place_into_box(
            arm, place_pose, entry_xy, MICROWAVE_ENTRY_Z, config
        )
    else:
        place_result = place(arm, place_pose, config)
    if not place_result.success:
        print(f"  ✗ place FAILED: {place_result.reason}")
        return False
    print("  ✓ place succeeded.")

    print("\nDone — arm retracted to transit altitude.")

    if USE_MIDPOINT:
        midpoint_pose = plan_midpoint(angle_rad=MIDPOINT_ANGLE_RAD)
        if not _planned_or_linear_transit(
            session,
            arm,
            "post-place return to midpoint",
            [
                _current_tcp_pose_task(arm),
                pose_at_altitude(midpoint_pose, config.transit_z),
            ],
            config,
            attached_bowl=False,
        ):
            return False
    return True


def main(dry: bool = False) -> int:
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
    ap.add_argument(
        "--dry",
        action="store_true",
        help="Plan and print the grasp/place poses without connecting to RTDE.",
    )
    args = ap.parse_args()
    raise SystemExit(main(dry=args.dry))
