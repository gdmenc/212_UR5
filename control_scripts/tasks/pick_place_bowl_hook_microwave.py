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
from dataclasses import replace
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
from ..world import World


# --- Tunables --------------------------------------------------------------

PICK_FROM: Literal["outside", "microwave"] = "microwave"
PLACE_TO: Literal["outside", "microwave"] = "outside"

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

BOWL_ENTRY_CLEARANCE = 0.200
"""Distance outside the microwave door before lowering to ``MICROWAVE_ENTRY_Z``.
Keep this larger than the hook TCP envelope plus the bowl rim diameter so
the bowl/hook clears the front lip before descending.

0.150 m was the original value, but the welded bowl model (mounted at the
default ``attach_object_to_gripper`` pose) extends ~7.7 cm in +y from the
TCP — at 0.150 m clearance the bowl's far edge sat at the door face
(y=0.375) and clipped the microwave's top body by 1–3 mm at every IK
branch, blocking sim planning. Diagnostic sweep with the planner's actual
5 mm env_padding plus q3/q5 singularity branch lock:

    0.150 m  →  0/8 IK branches clean   (broken)
    0.175 m  →  2/8 (still fails singularity filter)
    0.200 m  →  3/8 — first value with planner-acceptable goals

Affects both real and sim — slightly more conservative pre-descent
traverse (5 cm farther from the door before descending), well within
the cell. Bump higher (0.225–0.250) if more headroom is desired."""

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

SUPPORTS_SIM = True
"""``--mode sim`` plans the planner-driven carry segments and replays them
in meshcat with a leg-by-leg stepper. The hand-coded pick / place / release
primitives go through RTDE directly and are NOT simulated; sim mode skips
them. Set False on tasks that have no planner-driven segments."""

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

MOTION_PLAN_PRE_PICK_APPROACH = True
"""When True, the lift+transit_xy steps inside ``pick``/``pick_from_box``
are preceded by a motion-planned transit from the rig's current TCP to
the pick-side hover pose. The pick primitive's own lift+transit_xy then
become near-no-ops (TCP is already at the target). When False, pick
runs unchanged with its hand-coded Cartesian moves only."""

USE_MOTION_PLANNING = True
"""Use Drake ``plan_transit`` + ``execute_plan`` for free-space carry segments.
``False`` falls back to the original sequential ``transit_xy`` moveL routing.

Override at the CLI with ``--no-motion-planning`` (sets this to False at
task start). Useful when the planner can't be trusted and you want a
known-safe path."""

MOTION_PLAN_RRT_FALLBACK = True
"""Try RRT after KTO when the optimizer cannot find a planned transit."""

MOTION_PLAN_AUTO_FALLBACK = True
"""When the motion planner fails (``plan_transit`` raises, or
``execute_plan`` reports a failure), fall back to the sequential moveL
path with a loud warning instead of failing the task. Default ON because
on a real-rig deployment we'd rather keep moving (degraded but safe)
than abort. Set False to make planner failures fatal — useful for
debugging or strict planning."""

KEEP_BOWL_LEVEL_DURING_CARRY = True
"""Constrain the post-pick planned carry so the bowl orientation does not
drift much from the orientation it had at the start of the carry. This is the
hook-bowl analog of ``KEEP_PLATE_LEVEL_DURING_CARRY`` in the plate task."""

CARRY_BOWL_LEVEL_TOLERANCE_RAD = float(np.radians(20.0))
"""Allowed bowl-up-axis tilt away from task +Z during the in-hand carry.

Must be at least the wrist-rotation magnitude between the carry's start
and goal orientations, otherwise the constraint is geometrically
infeasible at the goal. With ``APPROACH_TILT_RAD_OUTSIDE = 15°`` at
grasp and ``APPROACH_TILT_RAD_MICROWAVE = 0°`` at place, the wrist
rotates ~15° during the carry, which would violate any tolerance < 15°.
20° gives a 5° margin and still bounds the worst-case spill tilt.

Was 3° originally — that value was geometrically incompatible with the
pick→place tilt mismatch and made every IK branch at the place hover
fail validation. Same incompatibility exists in the real-path planning
(use_motion_planning=True), so this fix is shared by both."""

CARRY_MIN_CLEARANCE_M = 0.005
"""Minimum clearance for the in-hand carry. The default planner clearance is
1 cm but the welded bowl + cup-with-stick obstacle leave only millimetres of
margin in places — keep this positive so penetration still fails, but allow a
tighter corridor."""

INCLUDE_CUP_OBSTACLE = True
"""When True, the **plain cup** (no stick) is treated as a static obstacle on
the table during planning so the bowl carry routes around it. Other tabletop
objects are still skipped because the bowl itself is welded to the gripper
for planning.

Was previously ``INCLUDE_CUP_WITH_STICK_OBSTACLE`` keyed to ``cup_with_stick``,
but the stick body extends up into the workspace and triggered IK-branch
rejections on far branches (``forearm_link↔stick``, ``wrist_1_link↔stick``).
Plain ``cup`` is short and stays under the typical wrist altitude — same
table-footprint obstacle for the carry without the spurious stick collisions."""

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


WORLD = World(
    include_microwave=True,
    include_objects=INCLUDE_CUP_OBSTACLE,
    skip_static_objects=(
        # Skip everything except plain ``cup`` — that's the single static
        # obstacle the bowl carry must route around. The bowl itself is
        # welded to the gripper per-leg via ``in_hand``.
        ("plate", "cup_with_stick", "bowl", "bottle", "tray")
        if INCLUDE_CUP_OBSTACLE else ()
    ),
    object_xyz_overrides=(
        # Place plain ``cup`` at the cup-with-stick rig location so the
        # planning obstacle sits where the actual cup-with-stick is on
        # the table. ``CUP_WITH_STICK_DEFAULT_TASK_XYZ = (-0.20, -0.125,
        # 0.0)`` from ``planning/scene/objects.py``; hardcoded here so
        # the bowl task is self-contained and doesn't import from
        # private scene constants.
        {"cup": (-0.20, -0.125, 0.0)}
        if INCLUDE_CUP_OBSTACLE else {}
    ),
    robotiq_mode="closed",
    microwave_door_open_rad=MICROWAVE_DOOR_OPEN_ANGLE_RAD,
)
"""Single source of env truth. ``include_objects=True`` keeps the
plain cup as an obstacle on the table at the cup-with-stick location
(the actual physical placement on the rig). The rest are skipped
because the bowl itself is welded to the gripper during the carry leg
via ``in_hand`` per-leg overrides."""


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
    leg_world = (
        replace(WORLD, in_hand={ARM: ("bowl", None)})
        if attached_bowl else WORLD
    )
    diagram, plant, _, _ = leg_world.build_planning_scene()
    root_context = diagram.CreateDefaultContext()
    plant_context = plant.GetMyMutableContextFromRoot(root_context)
    return diagram, plant, plant_context


def _hover_before_pick(grasp, config: PickPlaceConfig) -> Pose:
    """The transit-altitude pose ``pick`` / ``pick_from_box`` will target
    before descending. Mirrors ``_hover_before_place`` for the pick side.

    Used to land the motion-planned pre-pick approach exactly where the
    hand-coded ``lift_to_transit + transit_xy`` chain would have ended,
    so the pick primitive's own lift+transit_xy become near-no-ops."""
    if PICK_FROM == "microwave":
        entry_xy = entry_xy_for_pose(
            grasp.grasp_pose, clearance=BOWL_ENTRY_CLEARANCE,
        )
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


def _plan_sim_legs(grasp, place_pose: Pose, config: PickPlaceConfig):
    """Plan the same motion-planned segments the real path plans, but
    from synthetic start states (no live RTDE).

    Three legs (when ``MOTION_PLAN_PRE_PICK_APPROACH=True``, otherwise
    skip the first):

      1. pre-pick approach — HOME (FK) → grasp hover. No bowl in hand.
      2. post-pick carry   — grasp hover (bowl in hand) → optional
         midpoint → place hover.
      3. post-place return — place hover (no bowl) → midpoint hover.
    """
    from ..planning.transit import InfeasiblePlanError, plan_transit

    from ..planning.transit import _arm_model_instance, _arm_position_indices, _tcp_frame
    from ..util.rotations import Rotation as RotUtil

    legs: list[tuple[str, object]] = []

    # The planning arm's joint config at the END of the most recently
    # planned leg, used to seed the IK chain of the next leg. Mirrors
    # what the real rig has via ``arm.receive.getActualQ()`` after each
    # primitive completes.
    chained_arm_q: Optional[np.ndarray] = None

    # ---- Leg 1: pre-pick approach (no bowl) -------------------------
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
        except InfeasiblePlanError as exc:
            print(f"  ✗ pre-pick approach infeasible: {exc}")
            return legs

    # ---- Leg 2: post-pick carry (bowl in hand) ----------------------
    carry_world = replace(WORLD, in_hand={ARM: ("bowl", None)})
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

    bowl_up_tcp = (
        _bowl_up_axis_in_tcp(carry_waypoints[0])
        if KEEP_BOWL_LEVEL_DURING_CARRY else None
    )

    # Multi-branch carry-seed search.
    #
    # The carry's start config (= seed) is normally the pre-pick leg's
    # terminal joint config. That terminal pose has up to 8 IK branches;
    # ikfast picks one ("closest-to-seed in the matching wrist quadrant"),
    # which works for the pre-pick itself but may put the carry's start
    # in a wrist quadrant whose only collision-clean place-hover branches
    # cross the q3/q5 singularity — the lock then rejects them.
    #
    # Fix: enumerate ALL ikfast branches at the pre-pick terminal pose
    # (regardless of seed quadrant) and try each as the carry's seed.
    # The branch that produces a feasible carry plan is the right one.
    # ``avoid_arm_singularity`` stays True throughout — we never cross
    # a singularity inside any single leg; we just pick a leg-boundary
    # config that lets the next leg stay in one quadrant.
    from ..planning.rrt import ikfast_goal_branches

    pre_pick_terminal_pose = _hover_before_pick(grasp, config)
    seed_candidates: list[np.ndarray]
    if chained_arm_q is not None:
        # All ikfast branches at the pre-pick terminal pose, sorted by
        # joint distance to chained_arm_q. The first is what ikfast
        # would have picked; the rest are alternatives.
        seed_candidates = list(ikfast_goal_branches(
            ARM, pre_pick_terminal_pose,
            seed_arm_q=chained_arm_q,
            max_branch_dist=None,
        )) or [chained_arm_q]
    else:
        seed_candidates = [None]  # let plan_transit use plant defaults

    carry_plan = None
    for i, seed_arm in enumerate(seed_candidates):
        cur_q = {ARM: seed_arm} if seed_arm is not None else None
        if seed_arm is not None:
            wrap = (seed_arm + np.pi) % (2.0 * np.pi) - np.pi
            print(f"\n[sim] planning post-pick carry "
                  f"(seed option {i + 1}/{len(seed_candidates)}: "
                  f"q3 sign={'+1' if wrap[2] >= 0 else '-1'}, "
                  f"q5 sign={'+1' if wrap[4] >= 0 else '-1'})...")
        else:
            print("\n[sim] planning post-pick carry (no seed; plant defaults)...")
        try:
            carry_plan = plan_transit(
                plant=carry_plant,
                arm=ARM,
                waypoints=carry_waypoints,
                plant_context=carry_plant_ctx,
                current_q=cur_q,
                use_rrt_fallback=MOTION_PLAN_RRT_FALLBACK,
                rrt_diagram=carry_diagram,
                align_tcp_axis=bowl_up_tcp,
                align_tcp_axis_world=np.array([0.0, 0.0, 1.0]),
                align_tcp_axis_tolerance_rad=CARRY_BOWL_LEVEL_TOLERANCE_RAD,
                min_clearance_m=CARRY_MIN_CLEARANCE_M,
            )
            break
        except InfeasiblePlanError as exc:
            print(f"  ✗ seed {i + 1} infeasible: {exc!s:.180}")
            continue
    if carry_plan is None:
        print(f"  ✗ post-pick carry plan infeasible across all "
              f"{len(seed_candidates)} ikfast branch options")
        return legs
    legs.append(("post-pick carry to place hover", carry_plan))
    chained_arm_q = np.asarray(
        carry_plan.trajectory.value(carry_plan.trajectory.end_time())
    ).flatten()[carry_arm_idx]

    if USE_MIDPOINT:
        return_diagram, return_plant, _, _ = WORLD.build_planning_scene()
        return_root = return_diagram.CreateDefaultContext()
        return_plant_ctx = return_plant.GetMyMutableContextFromRoot(return_root)

        # Return to HOME (the FK of plant defaults computed above for
        # the approach leg's start) rather than the midpoint hover.
        # HOME is the rig's neutral pose by construction — IK is
        # trivial there, and the elbow/wrist quadrants match the
        # carry's terminal joint config so the q3/q5 singularity-branch
        # lock passes. Going to the workspace-edge midpoint forces an
        # elbow flip that the lock forbids.
        if MOTION_PLAN_PRE_PICK_APPROACH:
            return_target_pose = home_pose
            return_target_label = "HOME hover"
        else:
            midpoint_pose = plan_midpoint(angle_rad=MIDPOINT_ANGLE_RAD)
            return_target_pose = pose_at_altitude(
                midpoint_pose, config.transit_z,
            )
            return_target_label = "midpoint hover"
        return_waypoints = [
            _hover_before_place(place_pose, config),
            return_target_pose,
        ]

        # Same multi-branch seed search as the carry leg — try each
        # ikfast branch at the carry's terminal pose as the return's
        # seed, in case the exact carry-end branch causes ikfast's
        # 2π-wrapping to return 0 solutions at the return's goal.
        carry_terminal_pose = _hover_before_place(place_pose, config)
        return_seeds: list[np.ndarray]
        if chained_arm_q is not None:
            return_seeds = list(ikfast_goal_branches(
                ARM, carry_terminal_pose,
                seed_arm_q=chained_arm_q,
                max_branch_dist=None,
            )) or [chained_arm_q]
        else:
            return_seeds = [None]

        return_plan = None
        for i, seed_arm in enumerate(return_seeds):
            cur_q = {ARM: seed_arm} if seed_arm is not None else None
            if seed_arm is not None:
                wrap = (seed_arm + np.pi) % (2.0 * np.pi) - np.pi
                print(f"\n[sim] planning post-place return to "
                      f"{return_target_label} "
                      f"(seed option {i + 1}/{len(return_seeds)}: "
                      f"q3 sign={'+1' if wrap[2] >= 0 else '-1'}, "
                      f"q5 sign={'+1' if wrap[4] >= 0 else '-1'})...")
            else:
                print(f"\n[sim] planning post-place return to "
                      f"{return_target_label} (no seed)...")
            try:
                return_plan = plan_transit(
                    plant=return_plant,
                    arm=ARM,
                    waypoints=return_waypoints,
                    plant_context=return_plant_ctx,
                    current_q=cur_q,
                    use_rrt_fallback=MOTION_PLAN_RRT_FALLBACK,
                    rrt_diagram=return_diagram,
                    min_clearance_m=0.01,
                )
                break
            except InfeasiblePlanError as exc:
                print(f"  ✗ seed {i + 1} infeasible: {exc!s:.180}")
                continue
        if return_plan is None:
            print(f"  ✗ post-place return infeasible across all "
                  f"{len(return_seeds)} ikfast branch options")
            return legs
        legs.append((f"post-place return to {return_target_label}", return_plan))

    return legs


def _run_sim(grasp, place_pose: Pose, config: PickPlaceConfig) -> int:
    """Plan the carry legs, build a meshcat scene, hand off to the
    leg-by-leg stepper. Hand-coded pick / place primitives are not
    simulated; the visualization scene welds the bowl to the gripper
    so the carry leg looks right (the return leg also plays in this
    scene — the bowl will visually persist across the release boundary).
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

    sim_world = replace(WORLD, in_hand={ARM: ("bowl", None)})
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
    attached_bowl: bool,
) -> bool:
    """Move through hover/free-space waypoints.

    ``USE_MOTION_PLANNING=False`` preserves the old moveL behavior while still
    honoring multiple waypoints.
    """
    print(f"\n→ planned transit: {label}")
    for i, wp in enumerate(waypoints):
        print(f"  wp {i}: xyz={np.round(wp.translation, 3)}")

    def _run_movel_fallback(reason: str) -> bool:
        """Sequential transit_xy moveL — original pre-planner flow."""
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
        method=MOTION_PLAN_EXECUTION_METHOD,
        n_waypoints=MOTION_PLAN_N_WAYPOINTS,
        dt=MOTION_PLAN_SERVO_DT_S,
        servo_time_scale=MOTION_PLAN_SERVO_TIME_SCALE,
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
    if MOTION_PLAN_PRE_PICK_APPROACH:
        if not _planned_or_linear_transit(
            session,
            arm,
            "pre-pick approach to grasp hover",
            [_current_tcp_pose_task(arm), _hover_before_pick(grasp, config)],
            config,
            attached_bowl=False,
        ):
            return False

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
