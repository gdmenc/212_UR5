"""End-to-end pick-and-place of a plate (2F-85 arm), with the pick or
place pose optionally inside the microwave.

Same plate rim grasp as ``pick_place_plate.py``. The microwave-side
half routes through ``pick_from_box`` / ``place_into_box`` so the wrist
stays below the cavity ceiling during transit inside.

============================================================
WARNING — vertical 2F-85 plate place may NOT fit the cavity
============================================================
With ``TCP_OFFSET_ROBOTIQ_2F85`` ≈ 0.184 m, the wrist sits ~18.4 cm
above the TCP at a top-down rim grasp. With the plate sitting on the
glass tray:

    TCP at plate rim   ≈ task z 0.10 m   (8 cm tray + 2 cm plate rim)
    Wrist at descent   ≈ task z 0.284 m
    Microwave ceiling  = task z 0.230 m  (5.4 cm overshoot)

That means a vertical descent of the plate INTO the microwave will
hit the cavity ceiling with the wrist before the TCP reaches the tray.
``plate_rim_grasp_edge`` adds a small wrist tilt (rim slope, ~10°)
which buys back ~3 cm of vertical, but it's not enough to clear.

Options if you actually need plate-into-microwave on the rig:
  - Re-measure ``MICROWAVE_CEILING_Z`` — if the real cavity is taller
    than 23 cm (e.g. 28-30 cm), this script works as-is.
  - Mount the 2F-85 differently (shorter TCP offset).
  - Switch to a side-pinch grasp (would be a new ``grasps/plate.py``
    factory). Out of scope here.

This script is wired up for the geometrically-valid case (plate stays
outside the microwave, or the cavity is taller than the placeholder
spec). It will print a loud warning at plan time if the wrist will
exceed the cavity ceiling.

Toggling sides
--------------
``PICK_FROM`` and ``PLACE_TO`` flags at the top — same convention as
``pick_place_bowl_hook_microwave.py``. ``"outside"`` uses the standard
``pick`` / ``place`` flow; ``"microwave"`` overrides location to the
cavity center at tray-z + plate-rim-height and routes through the box
helpers.

Approach angle
--------------
The 2F-85 closes along tool +Y. ``plate_rim_grasp_edge`` rotates the
TCP yaw by ``angle_rad + π``, so picking the rim angle orients the
wrist on the OPPOSITE side of the plate. The microwave door faces
task -Y, so for a microwave-side leg the wrist should sit on the +Y
side of the plate (forearm exits back through the -Y door). The user-
tuned ``PLACE_ANGLE_RAD = -0.88 rad`` (~-50°) places the wrist at
~+130° around the plate from center, which is mostly +Y from the
plate's center — consistent with -Y door exit. Re-tune if door
geometry changes.

Running
-------
Standalone:
    python -m control_scripts.tasks.pick_place_plate_microwave [--dry]
"""

from __future__ import annotations

import argparse
from typing import Literal

import numpy as np

from ..arm import ArmHandle
from ..calibration import TCP_OFFSET_ROBOTIQ_2F85
from ..config import PickPlaceConfig
from ..grasps.plate import PLATE_RIM_HEIGHT, plate_rim_grasp_edge
from ..microwave import (
    MICROWAVE_CEILING_Z,
    MICROWAVE_CENTER_XY_TASK,
    MICROWAVE_FLOOR_Z,
    entry_xy_for_motion_direction,
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
# PICK_FROM: Literal["outside", "microwave"] = "microwave"
# PLACE_TO: Literal["outside", "microwave"] = "outside"

# Free-standing plate poses (used when the corresponding side is "outside").
# 0.025 m is the height of the plate rim that is reasonable for pickup /
# setdown — same convention as ``pick_place_plate.py``.

# These are the same given these are the outside poses to set these plates down
PLATE_PICK_POSE_TASK = Pose(translation=[0.295521, -0.1, 0.01])
PLATE_PLACE_POSE_TASK = Pose(translation=[0.295521, -0.1, 0.01])

# Intermediate waypoint between pick and place. Z is ignored —
# transit_z sets the carry altitude. Picked at a Cartesian location
# that breaks long pick→place swings into two shorter, predictable
# legs. Same value as ``examples/pick_place_plate.py`` since the pick/
# place geometry is similar.
PLATE_MIDPOINT_POSE_TASK = Pose(translation=[0.0, 0.0, 0.0])
USE_MIDPOINT = True
MIDPOINT_ANGLE_RAD = np.radians(0)
"""Whether to carry the held plate through ``PLATE_MIDPOINT_POSE_TASK``
between pick and place. Mirrors ``examples/pick_place_plate.py`` —
disabling this is what caused the rig over-extend on a long direct
pick→entry swing."""

# Plate-frame angle at which to engage the rim. Choose so the wrist
# exits back through the -X microwave door for any microwave-side leg.
# 2F-85 closes along tool +Y, so the rim-radial axis is tool +X. With
# ``plate_rim_grasp_edge`` the TCP yaw = angle_rad + π.

GRASP_ANGLE_RAD = 0.0
PLACE_ANGLE_RAD = -np.radians(55)
# GRASP_ANGLE_RAD = -np.radians(55)
# PLACE_ANGLE_RAD = 0 # ~-50.6° — same as the non-microwave plate task

# Approach angles — used ONLY for the cavity-aware microwave legs
# (pick_from_box / place_into_box). These are the **physical xy motion
# direction** the arm moves in during the lateral entry through the
# door — NOT a plate-rim azimuth. Tool orientation throughout the
# entry stays at the corresponding GRASP/PLACE orientation; only the
# placement of the entry point relative to the target changes.
#
# Convention: angle measured CCW from task +x.
#   +90°  = motion in +y (straight through the door, perpendicular)
#     0°  = motion in +x  (would never reach door — see fallback below)
#   +60°  = motion mostly +y, biased +x  (entering from lower-left)
#  +120°  = motion mostly +y, biased -x  (entering from lower-right)
#
# +90° (perpendicular through door) is the natural default. Set to
# None to fall back to ``entry_xy_for_pose``, which derives the entry
# heading from the tool axis instead of an explicit motion angle.
APPROACH_ANGLE_RAD = np.radians(90)
RELEASE_APPROACH_ANGLE_RAD = np.radians(90)

# Constrained altitude inside the cavity. Plate on tray sits at task z
# ~10 cm (8 + 2 cm rim). Top of plate rim ~ 11 cm. 14 cm gives ~3 cm
# clearance over the rim during in-cavity transit. Wrist will be at
# 14 + 18.4 = 32.4 cm — ABOVE the 23 cm ceiling. See top-of-file
# warning; this constant is correct for an enlarged real cavity but
# does not magically fit a 23 cm one.
MICROWAVE_ENTRY_Z = 0.12

PLATE_ENTRY_CLEARANCE = 0.2
"""Distance outside the microwave door before lowering to ``MICROWAVE_ENTRY_Z``.
Keep this larger than the 2F-85 TCP/finger envelope plus the plate diameter
so the plate clears the front lip before descending."""

MICROWAVE_DOOR_OPEN_ANGLE_RAD = 1.8
"""Door angle used by planning scenes for plate-to-microwave motions.

Matches the tuned open-microwave task value (~103°), so the door is no longer
treated as a closed obstacle during the carry-to-entry plan.
"""

# Plate center at task z when sitting on tray = tray + plate rim height.
MICROWAVE_PLATE_Z = MICROWAVE_FLOOR_Z + PLATE_RIM_HEIGHT - 0.02  # 0.10 m

ARM = "ur_right"

CONFIG = PickPlaceConfig(
    transit_z=0.22,
    place_use_contact_descent=False,
    transit_speed=0.1,
    transit_accel=0.2,

    # 25 mm release aperture — clears the plate rim but keeps fingers
    # narrow enough to stay inside the cavity. Same value as the
    # non-microwave plate task.
    release_aperture_mm=25,

    gripper_open_speed_pct=40,
    gripper_close_speed_pct=30,
)

USE_MOTION_PLANNING = True
"""Use Drake ``plan_transit`` + ``execute_plan`` for broad free-space
transits. Precise pick/place approach, in-cavity entry, and release moves
remain Cartesian ``moveL`` primitives."""

MOTION_PLAN_RRT_FALLBACK = True
"""Try RRT after KTO when the optimizer cannot find a planned transit."""

KEEP_PLATE_LEVEL_DURING_CARRY = True
"""Constrain the post-pick planned carry to keep the plate normal near +Z.

This is intentionally only applied to the free-space carry with the plate in
hand, not to the pick/place approach primitives. Unlike a full TCP orientation
lock, this still lets the gripper yaw/rotate as long as the plate does not tip
much relative to task-frame vertical.
"""

CARRY_PLATE_LEVEL_TOLERANCE_RAD = float(np.radians(3.0))
"""Allowed plate-normal tilt away from task +Z during the in-hand carry."""

CARRY_MIN_CLEARANCE_M = 0.005
"""Minimum clearance for the in-hand carry.

The default planner clearance is 1 cm, but the welded plate model passes within
a few millimetres of the microwave/table in this task geometry. Keep this
positive so actual penetration still fails, but allow a tighter corridor.
"""

MOTION_PLAN_N_WAYPOINTS = 30
MOTION_PLAN_BLEND_R_M = 0.005


# --- Pose / grasp planning -------------------------------------------------

def _plate_pose_for(side: str, default: Pose) -> Pose:
    if side == "microwave":
        return Pose(translation=[
            MICROWAVE_CENTER_XY_TASK[0],
            MICROWAVE_CENTER_XY_TASK[1],
            MICROWAVE_PLATE_Z,
        ])
    return default


def plan_pick():
    return plate_rim_grasp_edge(
        _plate_pose_for(PICK_FROM, PLATE_PICK_POSE_TASK),
        angle_rad=GRASP_ANGLE_RAD,
    )


def plan_place() -> Pose:
    grasp_at_dest = plate_rim_grasp_edge(
        _plate_pose_for(PLACE_TO, PLATE_PLACE_POSE_TASK),
        angle_rad=PLACE_ANGLE_RAD,
    )
    return grasp_at_dest.grasp_pose


def _pick_entry_xy(grasp_pose: Pose) -> np.ndarray:
    """Entry XY for the microwave pick leg. Uses APPROACH_ANGLE_RAD
    (motion direction) when set; falls back to the tool-axis-derived
    ``entry_xy_for_pose`` otherwise."""
    if APPROACH_ANGLE_RAD is not None:
        return entry_xy_for_motion_direction(
            grasp_pose.translation[:2],
            APPROACH_ANGLE_RAD,
            clearance=PLATE_ENTRY_CLEARANCE,
        )
    return entry_xy_for_pose(grasp_pose, clearance=PLATE_ENTRY_CLEARANCE)


def _place_entry_xy(place_pose: Pose) -> np.ndarray:
    """Entry XY for the microwave place leg. Uses RELEASE_APPROACH_ANGLE_RAD
    (motion direction) when set; falls back to ``entry_xy_for_pose``."""
    if RELEASE_APPROACH_ANGLE_RAD is not None:
        return entry_xy_for_motion_direction(
            place_pose.translation[:2],
            RELEASE_APPROACH_ANGLE_RAD,
            clearance=PLATE_ENTRY_CLEARANCE,
        )
    return entry_xy_for_pose(place_pose, clearance=PLATE_ENTRY_CLEARANCE)


def plan_midpoint(angle_rad: float = GRASP_ANGLE_RAD) -> Pose:
    """TCP pose for carrying the held plate through the midpoint XY at
    transit_z. Orientation matches the upcoming PLACE leg so the wrist
    rotation happens during the pick→midpoint leg, not the longer
    midpoint→place transit."""
    grasp_at_midpoint = plate_rim_grasp_edge(
        PLATE_MIDPOINT_POSE_TASK,
        angle_rad=angle_rad,
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


def _plate_normal_axis_in_tcp(carry_start_pose: Pose) -> np.ndarray:
    """Plate +Z, expressed in the TCP frame at the start of carry.

    The grasp factories assume the plate itself is level in task frame; food
    safety means keeping that object-frame +Z close to task +Z while allowing
    yaw around it.
    """
    return carry_start_pose.rotation.inv().apply([0.0, 0.0, 1.0])


def _build_planning_context(*, attached_plate: bool):
    """Build a planning scene for a single live planned transit.

    The demo tabletop objects are omitted so stale/default object poses do not
    block a live run. During the post-pick carry, weld a plate to the active
    TCP so collision checks include the object in hand.
    """
    from ..planning.rrt import build_planning_scene

    attached_objects = (("plate", ARM, None),) if attached_plate else ()
    diagram, plant, _, _ = build_planning_scene(
        include_objects=False,
        robotiq_mode="closed",
        microwave_door_open_angle_rad=MICROWAVE_DOOR_OPEN_ANGLE_RAD,
        attached_objects=attached_objects,
    )
    root_context = diagram.CreateDefaultContext()
    plant_context = plant.GetMyMutableContextFromRoot(root_context)
    return diagram, plant, plant_context


def _planned_or_linear_transit(
    session: Session,
    arm: ArmHandle,
    label: str,
    waypoints: list[Pose],
    config: PickPlaceConfig,
    *,
    attached_plate: bool,
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

    plate_normal_tcp = (
        _plate_normal_axis_in_tcp(waypoints[0])
        if attached_plate and KEEP_PLATE_LEVEL_DURING_CARRY
        else None
    )
    if plate_normal_tcp is not None:
        print("  carry plate level: plate normal within "
              f"±{np.degrees(CARRY_PLATE_LEVEL_TOLERANCE_RAD):.1f}° of task +Z")

    current_tcp = _current_tcp_pose_task(arm)
    tcp_xyz_err = np.asarray(current_tcp.translation) - np.asarray(waypoints[0].translation)
    tcp_rot_delta = waypoints[0].rotation.inv() * current_tcp.rotation
    print("  live TCP vs wp0: "
          f"dxyz={np.round(tcp_xyz_err * 1000.0, 1)} mm, "
          f"drot={np.degrees(np.linalg.norm(tcp_rot_delta.as_rotvec())):.2f}°")
    if plate_normal_tcp is not None:
        live_plate_normal = current_tcp.rotation.apply(plate_normal_tcp)
        live_plate_normal = live_plate_normal / np.linalg.norm(live_plate_normal)
        live_tilt = np.degrees(np.arccos(np.clip(
            float(np.dot(live_plate_normal, np.array([0.0, 0.0, 1.0]))),
            -1.0,
            1.0,
        )))
        print(f"  live plate tilt estimate: {live_tilt:.2f}° from task +Z")

    diagram, plant, plant_context = _build_planning_context(
        attached_plate=attached_plate,
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
            align_tcp_axis=plate_normal_tcp,
            align_tcp_axis_world=np.array([0.0, 0.0, 1.0]),
            align_tcp_axis_tolerance_rad=CARRY_PLATE_LEVEL_TOLERANCE_RAD,
            min_clearance_m=CARRY_MIN_CLEARANCE_M if attached_plate else 0.01,
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
        method="moveJ_path",
        n_waypoints=MOTION_PLAN_N_WAYPOINTS,
        blend_r_m=MOTION_PLAN_BLEND_R_M,
    )
    if not result.success:
        print(f"  ✗ planned transit execution failed: {result.reason}")
        return False
    print("  ✓ planned transit reached.")
    return True


def _hover_before_place(place_pose: Pose, config: PickPlaceConfig) -> Pose:
    """The transit-altitude pose that ``place`` / ``place_into_box`` will
    target before descending."""
    if PLACE_TO == "microwave":
        entry_xy = _place_entry_xy(place_pose)
        entry_pose = Pose(
            translation=np.array([entry_xy[0], entry_xy[1], MICROWAVE_ENTRY_Z]),
            rotation=place_pose.rotation,
        )
        return pose_at_altitude(entry_pose, config.transit_z)

    preplace = offset_along_tool_z(place_pose, config.preplace_offset)
    return pose_at_altitude(preplace, config.transit_z)


def _check_wrist_clearance() -> None:
    """Loudly warn at plan time if the 2F-85 wrist will exceed the cavity
    ceiling on a microwave-side leg. Conservative: assumes pure-vertical
    descent (ignores the small rim-slope tilt)."""
    if PICK_FROM != "microwave" and PLACE_TO != "microwave":
        return
    tcp_offset_z = TCP_OFFSET_ROBOTIQ_2F85[2]
    wrist_at_descent_z = MICROWAVE_PLATE_Z + tcp_offset_z
    wrist_at_entry_z = MICROWAVE_ENTRY_Z + tcp_offset_z
    margin_descent = MICROWAVE_CEILING_Z - wrist_at_descent_z
    margin_entry = MICROWAVE_CEILING_Z - wrist_at_entry_z
    if margin_descent < 0 or margin_entry < 0:
        print("!" * 60)
        print(" GEOMETRY WARNING — 2F-85 wrist exceeds microwave ceiling")
        print(f"  ceiling z          = {MICROWAVE_CEILING_Z:.3f} m")
        print(f"  wrist z @ entry_z  = {wrist_at_entry_z:.3f} m  "
              f"(margin {margin_entry*100:+.1f} cm)")
        print(f"  wrist z @ plate    = {wrist_at_descent_z:.3f} m  "
              f"(margin {margin_descent*100:+.1f} cm)")
        print(" Re-measure ceiling, switch to a side grasp, or skip "
              "microwave leg.")
        print("!" * 60)


def _print_plan(grasp, place_pose: Pose) -> None:
    print("=" * 60)
    print(f"  Arm           : {ARM} (Robotiq 2F-85)")
    print(f"  Pick from     : {PICK_FROM}")
    print(f"  Place to      : {PLACE_TO}")
    print(f"  Grasp pose    : {grasp.grasp_pose.translation} (task)")
    print(f"  Place pose    : {place_pose.translation} (task)")
    print(f"  Grasp angle   : {np.degrees(GRASP_ANGLE_RAD):+.0f}°  (plate-rim azimuth)")
    print(f"  Place angle   : {np.degrees(PLACE_ANGLE_RAD):+.0f}°  (plate-rim azimuth)")
    if PICK_FROM == "microwave":
        a = APPROACH_ANGLE_RAD
        print(f"  Approach (pick) : "
              f"{('%+.0f° (motion dir)' % np.degrees(a)) if a is not None else 'tool-axis (entry_xy_for_pose)'}")
    if PLACE_TO == "microwave":
        a = RELEASE_APPROACH_ANGLE_RAD
        print(f"  Approach (place): "
              f"{('%+.0f° (motion dir)' % np.degrees(a)) if a is not None else 'tool-axis (entry_xy_for_pose)'}")
    print(f"  Midpoint      : "
          f"{PLATE_MIDPOINT_POSE_TASK.translation if USE_MIDPOINT else 'disabled'}"
          f"{' (task)' if USE_MIDPOINT else ''}")
    print(f"  Transit Z     : {CONFIG.transit_z} m")
    print(f"  Release aper. : {CONFIG.release_aperture_mm} mm")
    if PICK_FROM == "microwave" or PLACE_TO == "microwave":
        print(f"  Microwave entry Z : {MICROWAVE_ENTRY_Z} m")
        print(f"  Entry clearance   : {PLATE_ENTRY_CLEARANCE} m")
        if PICK_FROM == "microwave":
            print(f"  Entry XY (pick)   : {_pick_entry_xy(grasp.grasp_pose)}")
        if PLACE_TO == "microwave":
            print(f"  Entry XY (place)  : {_place_entry_xy(place_pose)}")
    print("=" * 60)
    _check_wrist_clearance()


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
        entry_xy = _pick_entry_xy(grasp.grasp_pose)
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
        attached_plate=True,
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
        entry_xy = _place_entry_xy(place_pose)
        place_result = place_into_box(
            arm, place_pose, entry_xy, MICROWAVE_ENTRY_Z, config,
        )
    else:
        place_result = place(arm, place_pose, config)
    if not place_result.success:
        print(f"  ✗ place FAILED: {place_result.reason}")
        return False
    print("  ✓ place succeeded.")

    print("\nDone — arm retracted to transit altitude.")

    # return to midpoint at the very end
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
        attached_plate=False,
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
