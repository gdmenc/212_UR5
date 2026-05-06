"""End-to-end pick + pour + place of a water bottle using the HOOK arm.

Parallel of ``pour_bottle.py`` but using the welded hook on ur_left to
engage the bottle by its OPENING RIM rather than by the body. The hook
descends vertically over the rim, the moving finger threads through the
opening into the neck cavity, and the rim wall is clamped between the
finger (radially inside the rim) and the fixed jaw (radially outside).

Hardware assumptions
--------------------
  - The arm named ``ARM`` has the welded hook gripper attached and a
    pre-calibrated ``TCP_OFFSET_HOOK = [0, 0, 0.10275, 0, +π/2, 0]``
    (calibration.py).
  - A water bottle (matching grasps/bottle.py geometry) sits at
    BOTTLE_PICK_POSE_TASK with bottle +z pointing up. The CAP IS OFF —
    the rim must be exposed for the hook to engage.
  - A receiver (cup, bowl, drain, ...) below POUR_TARGET_XY_TASK. The
    bottle opening targets receiver rim z + POUR_HEIGHT_ABOVE_RIM_M so
    liquid falls cleanly.

Running
-------
Standalone:
    python -m control_scripts.tasks.pour_bottle_hook [--dry]

First-run checklist
-------------------
1. Run ``--dry`` and inspect the printed pour TCP pose. The opening's
   offset from TCP is only R_rim (~2 cm) for the rim grasp, so the
   pour TCP should be very close to POUR_TARGET_TASK.
2. Verify ``transit_z`` clears the suspended bottle's full height
   (17.5 cm hangs below the TCP at the rim grasp) plus a safety margin.
3. Confirm GRASP_ANGLE_RAD orients the tilt direction sensibly: at
   angle π (gripper on the -X side of the bottle), positive tilt sends
   the cap end toward +X, so POUR_TARGET should be on +X relative to
   the pickup. Default uses angle π to match the recorded teaching pose.
4. Start with a short ``POUR_DURATION_S`` (1-2 s) and a moderate tilt
   (≤ 110°) until you trust the geometry, then tune.
"""

from __future__ import annotations

import argparse
import time
from dataclasses import replace
from typing import Optional, Sequence

import numpy as np

from ...arm import ArmHandle
from ...config import PickPlaceConfig
from ...grasps.bottle import (
    bottle_hook_grasp,
    bottle_hook_pour_tcp_pose,
)
from ...moves import lift_to_transit, transit_xy
from ...pick import pick
from ...place import place
from ...session import Session, default_session
from ...util.poses import Pose, pose_at_altitude
from ...util.rtde_convert import rtde_to_pose
from ...world import World


# --- Tunables (edit to match your physical layout) ------------------------
# Note: descend 1 cm below the nominal rim target for a more secure hook grasp.
BOTTLE_PICK_POSE_TASK = Pose(translation=[-0.32, -0.125, -0.01])
"""Bottle base in task frame at PICK location. Identity rotation is
correct for any free-standing bottle on the table."""

BOTTLE_PLACE_POSE_TASK = Pose(translation=[-0.30, 0.42, 0.28])
"""Bottle base in task frame at PLACE location.

Default: on top of the microwave (microwave outer top is at task z=0.28),
in the front-left portion of the top so the LEFT arm can reach it
without overextending. With a hook bottle grasp at the rim, the TCP at
place ends up at task z = 0.28 + bottle_grasp_z, well within reach.

Bottle xy footprint = ``[-0.34, -0.26] × [0.38, 0.46]`` (radius ~4 cm).
Microwave top footprint = ``[-0.43, +0.01] × [0.36, 0.645]``. Bottle is
fully on top with ~9 cm side margin and ~6 cm front margin."""

POUR_TARGET_XY_TASK = np.array([0.0, 0.0])
"""xy of the receiver opening / pour target in task frame. The default +X
direction matches a GRASP_ANGLE_RAD of π (gripper on -X side, bottle tips
toward +X)."""

POUR_RECEIVER_RIM_Z_TASK = 0.154
"""Receiver rim height in task z (m). Measure this from the same task-frame
table origin used for the bottle pose."""

POUR_HEIGHT_ABOVE_RIM_M = 0.05
"""Vertical clearance from receiver rim to bottle opening during pour.
3-5 cm is a good starting range: high enough to avoid rim collisions, low
enough that the stream lands cleanly."""

POUR_TARGET_TASK = np.array([
    POUR_TARGET_XY_TASK[0],
    POUR_TARGET_XY_TASK[1],
    POUR_RECEIVER_RIM_Z_TASK + POUR_HEIGHT_ABOVE_RIM_M,
])
"""xyz of the bottle's OPENING during the pour, in task frame."""

POUR_TILT_RAD = float(np.radians(130))
"""Tilt about the gripper's tool +Y axis. > 0 tips the bottle's +z away
from the gripper. ~120° is a typical pour past horizontal."""

POUR_DESCENT_TILT_RAD = float(np.radians(85))
"""Intermediate tilt used for vertical clearance around the receiver.
The hook rotates to this angle at transit height before descending to pour,
then returns to this angle before ascending away from the target."""

POUR_DURATION_S = 3.0
"""Seconds to hold the tilt."""

POUR_MAX_TILT_STEP_RAD = float(np.radians(85))
"""Maximum relative tilt step per moveL — same rotvec-continuity reason
as in pour_bottle.py."""

GRASP_ANGLE_RAD = float(np.pi)
"""Bottle-frame angle at which to grasp the rim (radians). At angle π
the hook approaches from the -X side of the bottle frame; positive tilt
then pours toward +X. This matches the recorded teaching pose used to
verify TCP_OFFSET_HOOK; equivalent to -np.pi modulo 2π."""

ARM = "ur_left"

SUPPORTS_SIM = True
"""``--mode sim`` plans and visualizes four free-space transits:
pre-pick approach, post-pick to upright-at-pour hover, post-pour to
place hover, and post-place return to HOME. The pour primitive (descent-
tilt + tilt arc + hold + untilt + ascent-tilt) and the contact-sensitive
pick / place primitives are NOT simulated; sim jumps the arm to the next
leg's start state between legs to represent those skipped phases."""

WORLD = World(
    include_microwave=True,
    include_objects=False,
    robotiq_mode="closed",
    microwave_door_open_rad=0.0,
)
"""Single source of env truth for this task's planning + sim scenes."""

USE_MOTION_PLANNING = True
"""Use Drake ``plan_transit`` + ``execute_plan`` for free-space transits.
Override at the CLI with ``--no-motion-planning`` to disable entirely."""

MOTION_PLAN_PRE_PICK_APPROACH = True
"""When True, the lift+transit_xy steps inside ``pick`` are preceded by
a motion-planned transit from current TCP to the grasp hover."""

MOTION_PLAN_RRT_FALLBACK = True
"""Try RRT after KTO when the optimizer cannot find a planned transit."""

MOTION_PLAN_AUTO_FALLBACK = True
"""On planner failure, fall back to the original ``transit_xy`` moveL
sequence with a loud warning instead of failing the task."""

KEEP_BOTTLE_UPRIGHT = True
"""Constrain the post-pick → upright-at-pour and post-pour → place
free-space transits so the bottle's body-fixed +Z stays close to task
+Z. Prevents the planner from tilting the bottle mid-transit and
spilling. Tilt during the pour itself is the hand-coded ``_pour``
primitive — ``KEEP_BOTTLE_UPRIGHT`` does not constrain that."""

CARRY_BOTTLE_UPRIGHT_TOLERANCE_RAD = float(np.radians(4.0))
"""Allowed bottle-up-axis tilt away from task +Z during in-hand
free-space transits. 4° is tight (vs. bowl's 20°). The bottle should
stay near vertical during transit so the contents don't shift."""

CARRY_MIN_CLEARANCE_M = 0.005
"""Minimum clearance for the in-hand free-space transits."""

MOTION_PLAN_N_WAYPOINTS = 30
MOTION_PLAN_BLEND_R_M = 0.005

CONFIG = PickPlaceConfig(
    # transit_z must clear the microwave top with the bottle dangling
    # below the hook. Bottle is 0.175 m tall and the hook holds it by
    # the rim, so the bottle base sits ~17.5 cm below the TCP. Microwave
    # top is at task z=0.28, so TCP needs to be at >= 0.455 to keep the
    # bottle base above the top, plus margin. 0.55 gives the place TCP
    # (z=0.455) ~10 cm of headroom and the bottle base ~10 cm of
    # clearance over the microwave roof.
    transit_z=0.55,
    place_use_contact_descent=False,
    transit_speed=0.1,
    transit_accel=0.2,
    approach_speed=0.05,
    approach_accel=0.2,
    retract_speed=0.1,
    retract_accel=0.2,

    # Hook has no continuous aperture. ``None`` makes ``prepare_for_grasp``
    # dispatch to ``gripper.open()`` (extend the finger to clear the
    # throat) before the final descent.
    release_aperture_mm=None,

    gripper_open_speed_pct=40,   # no-op for the hook; kept for symmetry
    gripper_close_speed_pct=30,
)


def plan_pick():
    return bottle_hook_grasp(
        BOTTLE_PICK_POSE_TASK,
        angle_rad=GRASP_ANGLE_RAD,
    )


def plan_pour_pose() -> Pose:
    """Tilted TCP pose at which the bottle's opening sits at POUR_TARGET_TASK."""
    return bottle_hook_pour_tcp_pose(
        BOTTLE_PICK_POSE_TASK,
        POUR_TARGET_TASK,
        angle_rad=GRASP_ANGLE_RAD,
        tilt_rad=POUR_TILT_RAD,
    )


def _pour_tilt_segments(pour_tilt_rad: float, max_step_rad: float) -> int:
    return max(1, int(np.ceil(abs(pour_tilt_rad) / max_step_rad)))


def plan_upright_at_pour(bottle_pose: Pose | None = None) -> Pose:
    """Upright (un-tilted) TCP pose with the opening above POUR_TARGET.

    ``bottle_pose`` overrides ``BOTTLE_PICK_POSE_TASK`` — pass the
    perception-detected base when available."""
    _bp = bottle_pose if bottle_pose is not None else BOTTLE_PICK_POSE_TASK
    return bottle_hook_pour_tcp_pose(
        _bp,
        POUR_TARGET_TASK,
        angle_rad=GRASP_ANGLE_RAD,
        tilt_rad=0.0,
    )


def plan_place_pose() -> Pose:
    grasp_at_dest = bottle_hook_grasp(
        BOTTLE_PLACE_POSE_TASK,
        angle_rad=GRASP_ANGLE_RAD,
    )
    return grasp_at_dest.grasp_pose


def _print_plan(grasp, upright_at_pour: Pose, pour_pose: Pose, place_pose: Pose) -> None:
    print("=" * 60)
    print("  Arm                :", ARM, "(hook gripper)")
    print("  Pick (bottle base) :", BOTTLE_PICK_POSE_TASK.translation, "(task frame)")
    print("  Place (bottle base):", BOTTLE_PLACE_POSE_TASK.translation, "(task frame)")
    print("  Pour target (opening):", POUR_TARGET_TASK, "(task frame)")
    print("  Receiver rim z     :", f"{POUR_RECEIVER_RIM_Z_TASK:.3f} m")
    print("  Pour height        :", f"{POUR_HEIGHT_ABOVE_RIM_M*100:.1f} cm above rim")
    print("  Grasp angle        :", f"{np.degrees(GRASP_ANGLE_RAD):+.0f}°")
    print("  Descent tilt       :", f"{np.degrees(POUR_DESCENT_TILT_RAD):.0f}°")
    print("  Pour tilt          :", f"{np.degrees(POUR_TILT_RAD):.0f}°")
    print("  Pour segments      :", _pour_tilt_segments(
        POUR_TILT_RAD,
        POUR_MAX_TILT_STEP_RAD,
    ),
          f"(max step {np.degrees(POUR_MAX_TILT_STEP_RAD):.0f}°)")
    print("  Hold time          :", f"{POUR_DURATION_S:.1f} s")
    print("  Transit Z          :", CONFIG.transit_z, "m")
    print("  Pick grasp pose    :", grasp.grasp_pose.translation)
    print("  Upright @ pour XY  :", upright_at_pour.translation,
          "(z gets overridden to transit_z by transit_xy)")
    print("  Pour TCP pose      :", pour_pose.translation)
    print("  Place TCP pose     :", place_pose.translation)
    print("  Pregrasp offset    :", f"{grasp.pregrasp_offset*100:.1f} cm (vertical)")
    print("=" * 60)


def _nearest_equivalent_rotvec(rotvec: np.ndarray, reference: Optional[np.ndarray]) -> np.ndarray:
    """Choose an axis-angle representation closest to ``reference`` — same
    function as in pour_bottle.py, kept local to avoid cross-module imports."""
    rotvec = np.asarray(rotvec, dtype=float).reshape(3)
    if reference is None:
        return rotvec

    theta = float(np.linalg.norm(rotvec))
    if theta < 1e-9:
        return rotvec

    axis = rotvec / theta
    candidates = [
        axis * (theta + 2.0 * np.pi * k)
        for k in (-1, 0, 1)
    ]
    candidates.extend(
        -axis * ((2.0 * np.pi - theta) + 2.0 * np.pi * k)
        for k in (-1, 0, 1)
    )
    return min(candidates, key=lambda candidate: np.linalg.norm(candidate - reference))


def _move_l_continuous(
    arm: ArmHandle,
    pose_task: Pose,
    speed: float,
    accel: float,
    previous_rotvec: Optional[np.ndarray],
) -> np.ndarray:
    """moveL to ``pose_task`` using the rotvec branch nearest the last move."""
    pose_base = arm.to_base(pose_task)
    rotvec = _nearest_equivalent_rotvec(pose_base.rotation.as_rotvec(), previous_rotvec)
    arm.control.moveL([
        float(pose_base.translation[0]),
        float(pose_base.translation[1]),
        float(pose_base.translation[2]),
        float(rotvec[0]),
        float(rotvec[1]),
        float(rotvec[2]),
    ], speed, accel)
    return rotvec


def plan_pour_waypoints(
    bottle_pose_task: Pose,
    target_task: Sequence[float],
    pour_tilt_rad: float,
    max_step_rad: float = POUR_MAX_TILT_STEP_RAD,
    *,
    angle_rad: float = GRASP_ANGLE_RAD,
    start_tilt_rad: float = 0.0,
) -> list[Pose]:
    """Tilt waypoints from ``start_tilt_rad`` to the final pour pose."""
    waypoints = []
    abs_tilt = abs(pour_tilt_rad)
    direction = 1.0 if pour_tilt_rad >= 0.0 else -1.0
    start_abs_tilt = min(abs(start_tilt_rad), abs_tilt)
    if np.isclose(start_abs_tilt, abs_tilt):
        return []

    tilt = min(start_abs_tilt + max_step_rad, abs_tilt)
    while tilt < abs_tilt:
        waypoints.append(
            bottle_hook_pour_tcp_pose(
                bottle_pose_task,
                target_task,
                angle_rad=angle_rad,
                tilt_rad=direction * tilt,
            )
        )
        tilt += max_step_rad
    waypoints.append(
        bottle_hook_pour_tcp_pose(
            bottle_pose_task,
            target_task,
            angle_rad=angle_rad,
            tilt_rad=pour_tilt_rad,
        )
    )
    return waypoints


def _descent_tilt_rad(pour_tilt_rad: float) -> float:
    """Tilt used for descend/ascend, capped at the requested final tilt."""
    direction = 1.0 if pour_tilt_rad >= 0.0 else -1.0
    return direction * min(abs(POUR_DESCENT_TILT_RAD), abs(pour_tilt_rad))


def _pour(
    arm: ArmHandle,
    upright_at_pour: Pose,
    bottle_pose_task: Pose,
    target_task: Sequence[float],
    pour_tilt_rad: float,
    max_step_rad: float,
    grasp_angle_rad: float,
    config: PickPlaceConfig,
) -> None:
    """Tilt the held bottle so its opening reaches POUR_TARGET, hold for
    POUR_DURATION_S, return to upright at transit altitude."""
    lift_to_transit(arm, config.transit_z, config.transit_speed, config.transit_accel)
    transit_xy(
        arm,
        upright_at_pour,
        config.transit_z,
        config.transit_speed,
        config.transit_accel,
    )
    previous_rotvec = arm.to_base(upright_at_pour).rotation.as_rotvec()

    descent_tilt_rad = _descent_tilt_rad(pour_tilt_rad)
    tilted_at_pour = bottle_hook_pour_tcp_pose(
        bottle_pose_task,
        target_task,
        angle_rad=grasp_angle_rad,
        tilt_rad=descent_tilt_rad,
    )
    tilted_at_transit = pose_at_altitude(tilted_at_pour, config.transit_z)

    # Rotate while high above the receiver, then descend already tilted so
    # the bottle does not sweep through the cup rim during the pour approach.
    previous_rotvec = _move_l_continuous(
        arm,
        tilted_at_transit,
        config.transit_speed,
        config.transit_accel,
        previous_rotvec,
    )
    previous_rotvec = _move_l_continuous(
        arm,
        tilted_at_pour,
        config.approach_speed,
        config.approach_accel,
        previous_rotvec,
    )

    pour_waypoints = plan_pour_waypoints(
        bottle_pose_task,
        target_task,
        pour_tilt_rad,
        max_step_rad,
        angle_rad=grasp_angle_rad,
        start_tilt_rad=descent_tilt_rad,
    )
    for waypoint in pour_waypoints:
        previous_rotvec = _move_l_continuous(
            arm,
            waypoint,
            config.approach_speed,
            config.approach_accel,
            previous_rotvec,
        )

    print(f"  holding tilt for {POUR_DURATION_S:.1f} s ...")
    time.sleep(POUR_DURATION_S)

    untilt_waypoints = []
    if not np.isclose(abs(descent_tilt_rad), abs(pour_tilt_rad)):
        untilt_waypoints.append(tilted_at_pour)
    for waypoint in untilt_waypoints:
        previous_rotvec = _move_l_continuous(
            arm,
            waypoint,
            config.retract_speed,
            config.retract_accel,
            previous_rotvec,
        )

    # Ascend while still at the clearance tilt, then finish rotating upright
    # at transit height after the bottle has cleared the receiver.
    previous_rotvec = _move_l_continuous(
        arm,
        pose_at_altitude(tilted_at_pour, config.transit_z),
        config.retract_speed,
        config.retract_accel,
        previous_rotvec,
    )
    _move_l_continuous(
        arm,
        pose_at_altitude(upright_at_pour, config.transit_z),
        config.retract_speed,
        config.retract_accel,
        previous_rotvec,
    )


def _current_q(session: Session) -> dict[str, np.ndarray]:
    return {
        name: np.asarray(arm.receive.getActualQ(), dtype=float)
        for name, arm in session.arms.items()
    }


def _current_tcp_pose_task(arm: ArmHandle) -> Pose:
    return arm.to_task(rtde_to_pose(arm.receive.getActualTCPPose()))


def _hover_before_pick(grasp, config: PickPlaceConfig) -> Pose:
    """Transit-altitude pose ``pick`` will target before descending."""
    return pose_at_altitude(grasp.grasp_pose, config.transit_z)


def _hover_at_pour(config: PickPlaceConfig, *, bottle_base_pose: Pose | None = None) -> Pose:
    """Transit-altitude pose at the upright-at-pour xy. Pre-pour and
    post-pour planned transits both terminate here; the pour primitive
    descends through the descent-tilt and pour-tilt from this hover.

    ``bottle_base_pose`` overrides the module-global ``BOTTLE_PICK_POSE_TASK``
    when the bottle location was determined by perception."""
    return pose_at_altitude(plan_upright_at_pour(bottle_base_pose), config.transit_z)


def _hover_before_place(place_pose: Pose, config: PickPlaceConfig) -> Pose:
    """Transit-altitude pose ``place`` will target before descending."""
    return pose_at_altitude(place_pose, config.transit_z)


def _bottle_up_axis_in_tcp(carry_start_pose: Pose) -> np.ndarray:
    """Task-frame +Z expressed in the TCP frame at the start of the
    carry. Constraining this body-fixed vector to stay near task +Z
    during the transit keeps the bottle near-vertical."""
    return carry_start_pose.rotation.inv().apply([0.0, 0.0, 1.0])


def _build_planning_context(*, attached_bottle: bool):
    """Build a planning scene; when ``attached_bottle`` is True, weld a
    bottle to the active TCP for collision checks."""
    leg_world = (
        replace(WORLD, in_hand={ARM: ("bottle", None)})
        if attached_bottle else WORLD
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
    attached_bottle: bool,
) -> bool:
    """Plan a free-space transit through ``waypoints`` and execute it.

    Routes through Drake ``plan_transit`` + ``execute_plan`` when
    ``USE_MOTION_PLANNING`` is True; falls back to sequential moveL via
    ``transit_xy`` otherwise OR on planner / exec failure when
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

    from ...planning.execute import execute_plan
    from ...planning.transit import (
        InfeasiblePlanError, make_rtde_ik, plan_transit,
    )

    bottle_up_tcp = (
        _bottle_up_axis_in_tcp(waypoints[0])
        if attached_bottle and KEEP_BOTTLE_UPRIGHT
        else None
    )
    if bottle_up_tcp is not None:
        print("  carry bottle upright: bottle-up axis within "
              f"±{np.degrees(CARRY_BOTTLE_UPRIGHT_TOLERANCE_RAD):.1f}° of task +Z")

    diagram, plant, plant_context = _build_planning_context(
        attached_bottle=attached_bottle,
    )
    try:
        plan = plan_transit(
            plant=plant, arm=ARM,
            waypoints=waypoints,
            plant_context=plant_context,
            current_q=_current_q(session),
            use_rrt_fallback=MOTION_PLAN_RRT_FALLBACK,
            rrt_diagram=diagram,
            align_tcp_axis=bottle_up_tcp,
            align_tcp_axis_world=np.array([0.0, 0.0, 1.0]),
            align_tcp_axis_tolerance_rad=CARRY_BOTTLE_UPRIGHT_TOLERANCE_RAD,
            min_clearance_m=CARRY_MIN_CLEARANCE_M if attached_bottle else 0.01,
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


def run_on_arm(
    session: Session,
    arm: ArmHandle,
    grasp,
    pour_pose: Pose,
    place_pose: Pose,
    config: PickPlaceConfig = CONFIG,
    *,
    bottle_base_pose: Pose | None = None,
) -> bool:
    """Execute the full pick → pour → place sequence on a live arm.

    ``bottle_base_pose`` accepts a perception-detected bottle location.
    When ``None``, falls back to the module-global ``BOTTLE_PICK_POSE_TASK``
    for all intermediate calculations (upright-at-pour target, etc.)."""
    _bottle_pose = bottle_base_pose if bottle_base_pose is not None else BOTTLE_PICK_POSE_TASK

    # Leg 1: pre-pick approach (planned, no bottle).
    if MOTION_PLAN_PRE_PICK_APPROACH:
        if not _planned_or_linear_transit(
            session, arm,
            "pre-pick approach to grasp hover",
            [_current_tcp_pose_task(arm), _hover_before_pick(grasp, config)],
            config,
            attached_bottle=False,
        ):
            return False

    # Pick (hand-coded primitive).
    print(f"\n→ pick: {grasp.description}")
    pick_result = pick(arm, grasp, config)
    if not pick_result.success:
        print(f"  ✗ pick FAILED: {pick_result.reason}")
        return False
    print("  ✓ pick succeeded.")

    # Leg 2: post-pick → upright-at-pour (planned, bottle in hand,
    # upright). The pour primitive's internal lift+transit_xy then
    # become near-no-ops since we're already at upright_at_pour.
    if not _planned_or_linear_transit(
        session, arm,
        "post-pick → upright-at-pour hover",
        [_current_tcp_pose_task(arm), _hover_at_pour(config, bottle_base_pose=_bottle_pose)],
        config,
        attached_bottle=True,
    ):
        return False

    # Pour primitive (hand-coded — descent tilt + pour tilt + hold +
    # untilt + ascent tilt). Unchanged.
    print(f"\n→ pour @ {POUR_TARGET_TASK} "
          f"({POUR_HEIGHT_ABOVE_RIM_M*100:.1f} cm above rim, "
          f"tilt {np.degrees(POUR_TILT_RAD):.0f}°, {POUR_DURATION_S:.1f} s)")
    _pour(
        arm,
        plan_upright_at_pour(_bottle_pose),
        _bottle_pose,
        POUR_TARGET_TASK,
        POUR_TILT_RAD,
        POUR_MAX_TILT_STEP_RAD,
        GRASP_ANGLE_RAD,
        config,
    )
    print("  ✓ pour complete.")

    # Leg 3: post-pour → place hover (planned, bottle in hand, upright).
    if not _planned_or_linear_transit(
        session, arm,
        "post-pour → place hover",
        [_current_tcp_pose_task(arm), _hover_before_place(place_pose, config)],
        config,
        attached_bottle=True,
    ):
        return False

    # Place (hand-coded primitive).
    print(f"\n→ place @ {BOTTLE_PLACE_POSE_TASK.translation}")
    place_result = place(arm, place_pose, config)
    if not place_result.success:
        print(f"  ✗ place FAILED: {place_result.reason}")
        return False
    print("  ✓ place succeeded.")

    # Leg 4: post-place return to HOME hover (planned, no bottle).
    if MOTION_PLAN_PRE_PICK_APPROACH:
        from ...planning.transit import _arm_model_instance, _tcp_frame
        from ...util.rotations import Rotation as RotUtil
        diagram, plant, _, _ = WORLD.build_planning_scene()
        ctx = plant.GetMyMutableContextFromRoot(diagram.CreateDefaultContext())
        tcp_frame = _tcp_frame(plant, _arm_model_instance(plant, ARM))
        X = tcp_frame.CalcPoseInWorld(ctx)
        home_pose = Pose(
            translation=np.asarray(X.translation()),
            rotation=RotUtil.from_matrix(X.rotation().matrix()),
        )
        if not _planned_or_linear_transit(
            session, arm,
            "post-place return to HOME hover",
            [_current_tcp_pose_task(arm), home_pose],
            config,
            attached_bottle=False,
        ):
            return False

    print("\nDone — arm retracted to transit altitude.")
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
    upright_at_pour = plan_upright_at_pour()
    pour_pose = plan_pour_pose()
    place_pose = plan_place_pose()
    _print_plan(grasp, upright_at_pour, pour_pose, place_pose)

    if dry:
        print("[dry run] skipping RTDE connection. No motion commanded.")
        return 0

    if mode == "sim":
        print("[sim] full Meshcat visualisation not yet wired for this task; "
              "use --dry to preview the static plan offline.")
        return 0
    elif mode != "real":
        raise ValueError(f"unknown mode {mode!r}; choose 'real' or 'sim'")

    left = ARM == "ur_left"
    right = ARM == "ur_right"
    with default_session(left=left, right=right) as session:
        arm = session.arms[ARM]
        return 0 if run_on_arm(session, arm, grasp, pour_pose, place_pose, CONFIG) else 1


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument(
        "--dry",
        action="store_true",
        help="Plan and print the pick/pour/place poses without connecting to RTDE.",
    )
    ap.add_argument(
        "--mode",
        choices=["real", "sim"],
        default="real",
        help=(
            "Execution mode. 'real' (default) runs on the rig via RTDE. "
            "'sim' plans all transits and visualises them in Drake/Meshcat "
            "without executing on hardware."
        ),
    )
    ap.add_argument(
        "--no-motion-planning",
        action="store_true",
        help=(
            "Disable Drake motion planning entirely. Free-space transits "
            "route through sequential moveL Cartesian primitives."
        ),
    )
    args = ap.parse_args()
    raise SystemExit(main(
        dry=args.dry,
        mode=args.mode,
        motion_planning=not args.no_motion_planning,
    ))
