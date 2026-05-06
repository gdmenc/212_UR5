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
from dataclasses import replace
from typing import Literal, Optional

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
from ..world import World


# --- Tunables --------------------------------------------------------------
# PICK_FROM: Literal["outside", "microwave"] = "outside"
# PLACE_TO: Literal["outside", "microwave"] = "microwave"
PICK_FROM: Literal["outside", "microwave"] = "microwave"
PLACE_TO: Literal["outside", "microwave"] = "outside"

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

SUPPORTS_SIM = True
"""``--mode sim`` plans the planner-driven carry segments and replays them
in meshcat with a leg-by-leg stepper. The hand-coded pick / place / release
primitives go through RTDE directly and are NOT simulated; sim mode skips
them. Set False on tasks that have no planner-driven segments."""

WORLD = World(
    include_microwave=True,
    include_objects=False,                         # task uses live grasp pose, not the demo plate
    robotiq_mode="closed",
    microwave_door_open_rad=MICROWAVE_DOOR_OPEN_ANGLE_RAD,
)
"""Single source of env truth for this task's planning + sim scenes.
``in_hand`` is overridden per-leg via ``dataclasses.replace`` (plate welded
to the gripper for the post-pick carry, empty for the post-place return)."""

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

MOTION_PLAN_PRE_PICK_APPROACH = True
"""When True, the lift+transit_xy steps inside ``pick``/``pick_from_box``
are preceded by a motion-planned transit from the rig's current TCP to
the pick-side hover pose. The pick primitive's own lift+transit_xy then
become near-no-ops (TCP is already at the target). When False, pick
runs unchanged with its hand-coded Cartesian moves only."""

USE_MOTION_PLANNING = True
"""Use Drake ``plan_transit`` + ``execute_plan`` for broad free-space
transits. Precise pick/place approach, in-cavity entry, and release moves
remain Cartesian ``moveL`` primitives.

Override at the CLI with ``--no-motion-planning`` (sets this to False at
task start) to fall back to the pure-moveL flow — the original behavior
before the planner was added. Useful when the planner can't be trusted
(e.g., new geometry, suspect calibration) and you want a known-safe path."""

MOTION_PLAN_RRT_FALLBACK = True
"""Try RRT after KTO when the optimizer cannot find a planned transit."""

MOTION_PLAN_AUTO_FALLBACK = True
"""When the motion planner fails (``plan_transit`` raises, or
``execute_plan`` reports a failure), fall back to the sequential moveL
path with a loud warning instead of failing the task. Default ON because
on a real-rig deployment we'd rather keep moving (degraded but safe)
than abort. Set False to make planner failures fatal — useful for
debugging or if you want strict planning."""

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
    leg_world = (
        replace(WORLD, in_hand={ARM: ("plate", None)})
        if attached_plate else WORLD
    )
    diagram, plant, _, _ = leg_world.build_planning_scene()
    root_context = diagram.CreateDefaultContext()
    plant_context = plant.GetMyMutableContextFromRoot(root_context)
    return diagram, plant, plant_context


def _plan_sim_legs(grasp, place_pose: Pose, config: PickPlaceConfig):
    """Plan the same motion-planned segments the real path plans, but
    from synthetic start states (no live RTDE).

    Three legs (when ``MOTION_PLAN_PRE_PICK_APPROACH=True``, otherwise
    skip the first):

      1. pre-pick approach — HOME (FK) → grasp hover. No plate in hand.
         Mirrors the planned transit run before ``pick`` on the rig.
      2. post-pick carry   — grasp hover (plate in hand) → optional
         midpoint → place hover. Starting altitude mirrors what
         ``pick``/``pick_from_box`` actually leaves on the rig:
         descend to grasp, close, ascend back to ``config.transit_z``.
      3. post-place return — place hover (no plate) → midpoint hover.
    """
    from ..planning.transit import (
        InfeasiblePlanError,
        plan_transit,
        plan_transit_chained,
    )

    from ..planning.transit import _arm_model_instance, _arm_position_indices, _tcp_frame
    from ..util.rotations import Rotation as RotUtil

    legs: list[tuple[str, object]] = []

    # The planning arm's joint config at the END of the most recently
    # planned leg, used to seed the IK chain of the next leg. Mirrors
    # what the real rig has via ``arm.receive.getActualQ()`` after each
    # primitive completes — each leg starts from the previous leg's
    # endpoint, so IK branches stay consistent across the task.
    chained_arm_q: Optional[np.ndarray] = None
    # Terminal task-frame pose of the previous leg, used by
    # ``plan_transit_chained`` to enumerate ikfast branches as IK seed
    # candidates for the next leg.
    prev_terminal_pose: Optional[Pose] = None

    # ---- Leg 1: pre-pick approach (no plate) ------------------------
    if MOTION_PLAN_PRE_PICK_APPROACH:
        approach_diagram, approach_plant, _, _ = WORLD.build_planning_scene()
        approach_root = approach_diagram.CreateDefaultContext()
        approach_plant_ctx = approach_plant.GetMyMutableContextFromRoot(
            approach_root,
        )
        # Read HOME pose via FK from plant defaults so the approach
        # starts where the rig would at task entry (sim seeds from the
        # plant's default_home_q which both arms land at).
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
            # Chain: the carry leg starts where the approach ended.
            chained_arm_q = np.asarray(
                approach_plan.trajectory.value(
                    approach_plan.trajectory.end_time(),
                )
            ).flatten()[approach_arm_idx]
            prev_terminal_pose = _hover_before_pick(grasp, config)
        except InfeasiblePlanError as exc:
            print(f"  ✗ pre-pick approach infeasible: {exc}")
            return legs

    # ---- Leg 2: post-pick carry (plate in hand) ---------------------
    carry_world = replace(WORLD, in_hand={ARM: ("plate", None)})
    carry_diagram, carry_plant, _, _ = carry_world.build_planning_scene()
    carry_root = carry_diagram.CreateDefaultContext()
    carry_plant_ctx = carry_plant.GetMyMutableContextFromRoot(carry_root)
    carry_arm_idx = _arm_position_indices(
        carry_plant, _arm_model_instance(carry_plant, ARM),
    )

    carry_waypoints: list[Pose] = [
        pose_at_altitude(grasp.grasp_pose, config.transit_z),
    ]
    if USE_MIDPOINT:
        midpoint_pose = plan_midpoint(angle_rad=MIDPOINT_ANGLE_RAD)
        carry_waypoints.append(pose_at_altitude(midpoint_pose, config.transit_z))
    carry_waypoints.append(_hover_before_place(place_pose, config))

    plate_normal_tcp = (
        _plate_normal_axis_in_tcp(carry_waypoints[0])
        if KEEP_PLATE_LEVEL_DURING_CARRY else None
    )

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
            align_tcp_axis=plate_normal_tcp,
            align_tcp_axis_world=np.array([0.0, 0.0, 1.0]),
            align_tcp_axis_tolerance_rad=CARRY_PLATE_LEVEL_TOLERANCE_RAD,
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

    return_diagram, return_plant, _, _ = WORLD.build_planning_scene()
    return_root = return_diagram.CreateDefaultContext()
    return_plant_ctx = return_plant.GetMyMutableContextFromRoot(return_root)
    return_arm_idx = _arm_position_indices(
        return_plant, _arm_model_instance(return_plant, ARM),
    )

    # Return to HOME (the FK of plant defaults computed above for the
    # approach leg's start) rather than the midpoint hover. HOME is the
    # rig's neutral pose by construction — IK is trivial there, and the
    # elbow/wrist quadrants match the carry's terminal joint config so
    # the q3/q5 singularity-branch lock passes. Going to a midpoint
    # at the workspace edge can force an elbow flip that the lock
    # forbids (was the failure mode for bowl).
    if MOTION_PLAN_PRE_PICK_APPROACH:
        return_target_pose = home_pose
        return_target_label = "HOME hover"
    else:
        # Fall back to the midpoint when we don't have a cached
        # home_pose (approach leg disabled).
        midpoint_pose = plan_midpoint(angle_rad=MIDPOINT_ANGLE_RAD)
        return_target_pose = pose_at_altitude(midpoint_pose, config.transit_z)
        return_target_label = "midpoint hover"
    return_waypoints = [
        _hover_before_place(place_pose, config),
        return_target_pose,
    ]

    try:
        return_plan = plan_transit_chained(
            arm=ARM,
            log_label=f"post-place return to {return_target_label}",
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
    legs.append((f"post-place return to {return_target_label}", return_plan))

    return legs


def _run_sim(grasp, place_pose: Pose, config: PickPlaceConfig) -> int:
    """Plan the carry legs, build a meshcat scene, hand off to the
    leg-by-leg stepper.

    The hand-coded ``pick`` / ``place`` primitives are NOT simulated.
    The visualization scene includes the welded plate so the carry leg
    looks right; the return leg also plays in this scene, so the plate
    will visually stay in the gripper across the release boundary.
    Acceptable for trajectory inspection; not a literal task replay.
    """
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
        print(f"  - {label}: planner={plan.metadata.get('planner')}  "
              f"duration={plan.duration_s:.2f}s  "
              f"clearance={plan.min_clearance_m * 1000:.1f}mm")

    sim_world = replace(WORLD, in_hand={ARM: ("plate", None)})
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

    def _run_movel_fallback(reason: str) -> bool:
        """Sequential transit_xy moveL through the waypoints — the
        original pre-planner flow. Used when motion planning is
        disabled OR (with MOTION_PLAN_AUTO_FALLBACK) when the planner
        fails at runtime."""
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
        if MOTION_PLAN_AUTO_FALLBACK:
            return _run_movel_fallback(
                f"execute_plan failed: {result.reason}",
            )
        return False
    print("  ✓ planned transit reached.")
    return True


def _hover_before_pick(grasp, config: PickPlaceConfig) -> Pose:
    """The transit-altitude pose ``pick`` / ``pick_from_box`` will target
    before descending. Mirrors ``_hover_before_place`` for the pick side.

    Used to land the motion-planned pre-pick approach exactly where the
    hand-coded ``lift_to_transit + transit_xy`` chain would have ended,
    so the pick primitive's own lift+transit_xy become near-no-ops."""
    if PICK_FROM == "microwave":
        entry_xy = _pick_entry_xy(grasp.grasp_pose)
        entry_pose = Pose(
            translation=np.array([entry_xy[0], entry_xy[1], MICROWAVE_ENTRY_Z]),
            rotation=grasp.grasp_pose.rotation,
        )
        return pose_at_altitude(entry_pose, config.transit_z)

    return pose_at_altitude(grasp.grasp_pose, config.transit_z)


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
    if MOTION_PLAN_PRE_PICK_APPROACH:
        if not _planned_or_linear_transit(
            session,
            arm,
            "pre-pick approach to grasp hover",
            [_current_tcp_pose_task(arm), _hover_before_pick(grasp, config)],
            config,
            attached_plate=False,
        ):
            return False

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


def main(
    dry: bool = False,
    mode: str = "real",
    motion_planning: bool = True,
) -> int:
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
            "planner flow. Use as a safety fallback when the planner "
            "can't be trusted (new geometry, suspect calibration, etc.). "
            "Note: ``MOTION_PLAN_AUTO_FALLBACK`` already does this "
            "automatically on per-segment plan failures; this flag forces "
            "it for every segment up front."
        ),
    )
    args = ap.parse_args()
    raise SystemExit(main(
        dry=args.dry, mode=args.mode,
        motion_planning=not args.no_motion_planning,
    ))
