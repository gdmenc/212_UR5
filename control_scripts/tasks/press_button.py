"""Press a button with the right arm.

The press is fully specified by four task-frame numbers:

    BUTTON_TASK_XYZ                 — where the tip lands (m, task frame)
    PRESS_ANGLE_ABOVE_HORIZONTAL_RAD — elevation of press axis above horizontal
    PRESS_HORIZONTAL_DIR_TASK_XY    — which way the press points in xy plane
    PRESS_ROLL_RAD                  — gripper roll about the press axis

These constants are *paste-overrideable* — edit them at the top of this
file to retarget without re-recording a waypoint. They're populated with
defaults derived from a recorded waypoint
(``logs/waypoints/ur_right_20260430_223037.json``); rerun the derivation
at any time with::

    python -m control_scripts.tasks.press_button --derive

which loads ``WAYPOINT_PATH`` / ``WAYPOINT_NAME`` and prints the four
constants in paste-ready form.

Geometry
--------
The press direction (= flange +z, the direction the gripper points) is::

    press_dir = (cos(angle) * horizontal_xy.x,
                 cos(angle) * horizontal_xy.y,
                 sin(angle))

with ``angle`` = elevation above horizontal (negative = points down).
Negative elevation is the natural "press from above" configuration where
the wrist sits high and the tool tilts down toward the button.

The TCP target subtracts the tip-beyond-TCP offset along ``press_dir``
so the physical pressing tip lands exactly on ``BUTTON_TASK_XYZ``::

    press_tcp = button - (TIP_FLANGE_DIST_M - tcp_offset_z) * press_dir

with the gripper's roll about ``press_dir`` set by ``PRESS_ROLL_RAD``,
measured relative to a canonical reference frame (Tool +X = world +z
projected onto the plane perpendicular to ``press_dir``).

Sequence
--------
    1. Save current joints (return target at end).
    2. Close the gripper (lock any held press tool before motion).
    3. Lift to transit altitude (task z).
    4. moveL to the standoff pose (5 cm back along the press axis).
    5. move_until_contact along the press axis (threshold
       ``CONTACT_THRESHOLD_N``).
    6. forceMode at ``PRESS_FORCE_N`` for ``PRESS_HOLD_S`` seconds.
    7. forceModeStop, retract to standoff.
    8. Lift to transit, moveJ back to start joints.

Running
-------
    python -m control_scripts.tasks.press_button [--dry]
    python -m control_scripts.tasks.press_button --derive
"""

from __future__ import annotations

import argparse
import json
import os
import time
from typing import Tuple

import numpy as np
from scipy.spatial.transform import Rotation as ScipyRotation

from ..arm import ArmHandle
from ..calibration import TCP_OFFSET_ROBOTIQ_2F85
from ..config import PickPlaceConfig
from ..microwave import (
    MICROWAVE_HINGE_X,
    MICROWAVE_OUTER_W_X,
    WHITE_TABLE_TOP_Z,
    door_plane_y,
)
from ..moves import approach_to, lift_to_transit, move_until_contact, retract_to
from ..session import Session, default_session
from ..lab_landmarks import CUP_MICROWAVE_TOP_XYZ_TASK
from ..util.poses import Pose, pose_at_altitude
from ..util.rotations import Rotation
from ..util.rtde_convert import rtde_to_pose
from ..util.tray_layout import TRAY_DEFAULT_POSE_TASK, place_pose_on_tray
from ..world import World


# --- Button location, derived from microwave geometry ---------------------
# Source-of-truth for the microwave lives in ``control_scripts/microwave.py``.
# Edit there and these update automatically.
_BUTTON_FROM_OUTER_RIGHT_X = 0.0675 + 0.01
"""How far the button sits in (toward -x) from the microwave's outer
right edge. Measured: 6.75 cm."""

_BUTTON_HEIGHT_ABOVE_WHITE_TABLE = 0.105 - 0.0025#account for thickness of the fingers
"""How far the button sits above the white-table top surface.
Measured: 10.5 cm."""

_OUTER_RIGHT_X = MICROWAVE_HINGE_X + MICROWAVE_OUTER_W_X    # +0.060


# --- Tunables (edit to retarget the press) --------------------------------

BUTTON_TASK_XYZ = np.array([
    _OUTER_RIGHT_X - _BUTTON_FROM_OUTER_RIGHT_X,    # x = -0.0075
    door_plane_y(),                                  # y = +0.375
    WHITE_TABLE_TOP_Z + _BUTTON_HEIGHT_ABOVE_WHITE_TABLE,  # z = +0.075
])
"""Target tip location in task frame, xyz (m). DERIVED from the
microwave geometry constants (``control_scripts/microwave.py``) plus
the two measured button offsets above. Edit the microwave constants
or the button offsets — never this expression directly — so the source
of truth stays in one place.

Previous recorded value [-0.01244, +0.36312, +0.07298] was taken
against the old microwave position; re-derive via ``--derive`` if you
re-record waypoints under the updated geometry."""

PRESS_ANGLE_ABOVE_HORIZONTAL_RAD = -0.54827524
"""Elevation of the press axis above horizontal (radians). Sign:
    > 0 → tip points up (press into a ceiling-side button)
    = 0 → press axis horizontal
    < 0 → tip points down (the natural press-from-above configuration)
Default −0.5483 rad ≈ −31.4°, matching the recorded waypoint."""

PRESS_HORIZONTAL_DIR_TASK_XY = np.array([-0.09031991, +0.99591280])
"""Unit vector in task xy giving the press axis's horizontal direction.
press_dir = cos(angle)*horizontal_xy + sin(angle)*task_z. Default points
mostly +y (toward microwave); change to retarget to a button on a
different face. Length doesn't matter — re-normalised internally."""

PRESS_ROLL_RAD = +0.08227239
"""Roll about the press axis (radians). 0.0 = canonical orientation
where Tool +X points 'up' (world +z projected perpendicular to the
press axis). Positive = right-hand rotation about press_dir. Default
≈ +4.71°, matching the recorded waypoint."""

WAYPOINT_PATH = "logs/waypoints/ur_right_20260430_223037.json"
"""Recorded waypoint, used ONLY by --derive (not at runtime)."""

WAYPOINT_NAME = "press microwave button 1"
"""Snapshot inside WAYPOINT_PATH to use for --derive."""

TIP_FLANGE_DIST_M = 0.198894954
"""Distance from the wrist-3 flange face to the physical pressing tip,
along flange +z. The TCP itself is at ``TCP_OFFSET_ROBOTIQ_2F85[2]``
(0.184 m), so the tip is 14.9 mm beyond the calibrated TCP. This gap
is whatever the gripper holds (chopstick, finger, dowel, ...) — task
constant rather than a TCP recalibration."""

STANDOFF_M = 0.05
"""Standoff distance back along the press axis from the press TCP.
The arm goes here before move_until_contact."""

PRESS_FORCE_N = 20.0
"""Newtons applied along the press axis once contact is detected."""

PRESS_HOLD_S = 1.0
"""How long to hold ``PRESS_FORCE_N`` after contact, before retracting."""

CONTACT_THRESHOLD_N = 20.0
"""TCP force threshold (N) that ends move_until_contact."""

APPROACH_SPEED_M_S = 0.02
"""Linear speed (m/s) of the seek-to-contact move."""

ARM = "ur_right"

SUPPORTS_SIM = True
"""``--mode sim`` plans and visualizes the pre-press approach segment
(arm home → standoff hover). The contact-sensitive parts (descend until
contact, ``forceMode`` press, retract) are hand-coded ``moveL`` /
``forceMode`` and are not simulated; sim ends after the approach leg."""

_TRAY_POSE_TASK = TRAY_DEFAULT_POSE_TASK
_CUP_ON_TRAY_POSE_TASK = place_pose_on_tray("cup", tray=_TRAY_POSE_TASK)

WORLD = World(
    include_microwave=True,
    include_objects=True,
    # By the time press_button runs in the demo sequence, the cup is on
    # its tray slot, the cup-with-stick has been placed on the microwave
    # roof, and the tray is in its measured pose. plate / bowl / bottle
    # are out of scene at this point.
    skip_static_objects=("plate", "bowl", "bottle"),
    object_xyz_overrides={
        "cup": tuple(float(v) for v in _CUP_ON_TRAY_POSE_TASK.translation),
        "cup_with_stick": tuple(float(v) for v in CUP_MICROWAVE_TOP_XYZ_TASK),
        "tray": (_TRAY_POSE_TASK.x, _TRAY_POSE_TASK.y, _TRAY_POSE_TASK.z),
    },
    robotiq_mode="closed",
    microwave_door_open_rad=0.0,
)
"""Single source of env truth for this task's planning + sim scenes.
Models the demo state at button-press time: cup on the tray, cup-with-
stick on the microwave roof, tray at its measured pose."""

USE_MOTION_PLANNING = True
"""Use Drake ``plan_transit`` + ``execute_plan`` for the pre-press approach
(lift + transit_xy → standoff). Override at the CLI with
``--no-motion-planning`` to disable entirely."""

MOTION_PLAN_PRE_PRESS_APPROACH = True
"""When True, the lift+transit_xy steps (3 and 4 in the run sequence) are
preceded by a motion-planned transit from the rig's current TCP to the
standoff hover. The existing approach_to landing on the same hover then
becomes a near-no-op."""

MOTION_PLAN_POST_PRESS_RETURN = True
"""When True, after the press completes (steps 7–8: forceMode + retract),
plan a collision-checked joint-space transit from the post-retract hover
directly back to the task-entry joint configuration."""

MOTION_PLAN_RRT_FALLBACK = True
MOTION_PLAN_RRT_MAX_ITERS = 5000
MOTION_PLAN_RRT_SHORTCUT_ATTEMPTS = 50
"""RRT fallback budget. Lower these to cap worst-case fallback time."""

MOTION_PLAN_AUTO_FALLBACK = True
"""On planner failure, fall back to the original lift + approach_to
sequence with a loud warning rather than failing the task."""

MOTION_PLAN_N_WAYPOINTS = 30
MOTION_PLAN_BLEND_R_M = 0.005

CONFIG = PickPlaceConfig(
    transit_z=0.30,
    transit_speed=0.15,
    transit_accel=0.3,
    approach_speed=0.10,
    approach_accel=0.3,
    retract_speed=0.10,
    retract_accel=0.3,
    gripper_close_speed_pct=30,
)


# -------------------------------------------------------------------------
#  Geometry: build press direction + rotation from the four constants
# -------------------------------------------------------------------------

def _build_press_dir(
    angle_above_horizontal_rad: float,
    horizontal_dir_xy: np.ndarray,
) -> np.ndarray:
    """Press axis = unit vector in task frame, built from the elevation
    angle and the horizontal direction. Re-normalises horizontal_dir_xy
    in case the user edited it slightly off-unit."""
    h = np.asarray(horizontal_dir_xy, dtype=float).reshape(2)
    h_norm = np.linalg.norm(h)
    if h_norm < 1e-9:
        raise ValueError("PRESS_HORIZONTAL_DIR_TASK_XY has zero length.")
    h = h / h_norm
    c = np.cos(angle_above_horizontal_rad)
    s = np.sin(angle_above_horizontal_rad)
    return np.array([c * h[0], c * h[1], s])


def _canonical_axes(press_dir: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """Canonical (Tool +X, Tool +Y) basis perpendicular to press_dir.

    Tool +X = task +z projected onto the plane perpendicular to press_dir
              (so 'roll = 0' means the gripper's local +X points 'up').
    Tool +Y = press_dir × Tool +X (right-handed completion).

    Falls back to task +x as the up reference if press_dir is too close
    to vertical (avoids a degenerate projection)."""
    z = press_dir / np.linalg.norm(press_dir)
    up = np.array([0.0, 0.0, 1.0])
    if abs(np.dot(z, up)) > 0.99:
        up = np.array([1.0, 0.0, 0.0])
    x_canon = up - np.dot(up, z) * z
    x_canon = x_canon / np.linalg.norm(x_canon)
    y_canon = np.cross(z, x_canon)
    return x_canon, y_canon


def _build_press_rotation(
    press_dir: np.ndarray,
    roll_rad: float,
) -> Rotation:
    """Rotation whose Tool +Z is press_dir, with the gripper's roll
    about that axis set to ``roll_rad`` relative to the canonical frame
    (Tool +X = world +z projected perpendicular to press_dir)."""
    z = press_dir / np.linalg.norm(press_dir)
    x_canon, y_canon = _canonical_axes(z)
    cos_r, sin_r = np.cos(roll_rad), np.sin(roll_rad)
    x = cos_r * x_canon + sin_r * y_canon
    y = np.cross(z, x)
    R = np.column_stack([x, y, z])
    return Rotation.from_matrix(R)


def _press_tcp_pose(
    button_task: np.ndarray,
    press_dir: np.ndarray,
    press_rotation: Rotation,
    tip_flange_dist_m: float,
) -> Pose:
    """TCP pose that puts the tip at ``button_task`` under ``press_rotation``."""
    tcp_offset_z = TCP_OFFSET_ROBOTIQ_2F85[2]
    tip_beyond_tcp = tip_flange_dist_m - tcp_offset_z
    return Pose(
        translation=button_task - tip_beyond_tcp * press_dir,
        rotation=press_rotation,
    )


def _standoff_pose(
    press_tcp: Pose,
    press_dir: np.ndarray,
    standoff_m: float,
) -> Pose:
    """Standoff = press TCP backed off ``standoff_m`` along -press_dir."""
    return Pose(
        translation=press_tcp.translation - standoff_m * press_dir,
        rotation=press_tcp.rotation,
    )


# -------------------------------------------------------------------------
#  Waypoint loading + --derive support
# -------------------------------------------------------------------------

def _load_waypoint(path: str, name: str) -> Pose:
    """Read the recorded TCP task-frame pose from the JSON snapshot named
    ``name``. Used only by ``--derive``."""
    with open(path) as f:
        payload = json.load(f)
    for snap in payload.get("snapshots", []):
        if snap.get("name") == name:
            tp = snap["task_pose"]
            return Pose(
                translation=np.asarray(tp["translation"], dtype=float),
                rotation=Rotation.from_rotvec(np.asarray(tp["rotvec"], dtype=float)),
            )
    raise ValueError(
        f"snapshot {name!r} not found in {path}. "
        f"Available: {[s.get('name') for s in payload.get('snapshots', [])]}"
    )


def _derive_from_waypoint(path: str, name: str) -> int:
    """Load ``path/name`` and print the four press constants in
    paste-ready form. Inverse of the runtime path: from a recorded TCP
    pose, recover (button, angle, horizontal direction, roll)."""
    if not os.path.exists(path):
        print(f"ERROR: waypoint file not found: {path}")
        return 1

    pose = _load_waypoint(path, name)
    Rmat = ScipyRotation.from_rotvec(pose.rotation.as_rotvec()).as_matrix()
    flange_z = Rmat @ np.array([0.0, 0.0, 1.0])
    flange_x = Rmat @ np.array([1.0, 0.0, 0.0])

    tip_extra = TIP_FLANGE_DIST_M - TCP_OFFSET_ROBOTIQ_2F85[2]
    button = pose.translation + tip_extra * flange_z

    angle = float(np.arcsin(np.clip(flange_z[2], -1.0, 1.0)))

    h_xy = np.array([flange_z[0], flange_z[1]])
    h_norm = float(np.linalg.norm(h_xy))
    if h_norm < 1e-9:
        print("WARNING: recorded press axis is purely vertical — "
              "horizontal direction undefined.")
        h_xy = np.array([1.0, 0.0])
    else:
        h_xy = h_xy / h_norm

    z = flange_z / np.linalg.norm(flange_z)
    x_canon, y_canon = _canonical_axes(z)
    cos_roll = float(np.dot(flange_x, x_canon))
    sin_roll = float(np.dot(flange_x, y_canon))
    roll = float(np.arctan2(sin_roll, cos_roll))

    print()
    print(f"=== Derived from {path} :: {name!r} ===")
    print()
    print("# Paste these into the Tunables block at the top of press_button.py:")
    print(f"BUTTON_TASK_XYZ = np.array("
          f"[{button[0]:+.8f}, {button[1]:+.8f}, {button[2]:+.8f}])")
    print(f"PRESS_ANGLE_ABOVE_HORIZONTAL_RAD = {angle:+.8f}   "
          f"# = {np.degrees(angle):+.4f}°")
    print(f"PRESS_HORIZONTAL_DIR_TASK_XY = np.array("
          f"[{h_xy[0]:+.8f}, {h_xy[1]:+.8f}])")
    print(f"PRESS_ROLL_RAD = {roll:+.8f}   "
          f"# = {np.degrees(roll):+.4f}°")
    print()
    return 0


# -------------------------------------------------------------------------
#  Force-mode helper
# -------------------------------------------------------------------------

def _build_force_mode_task_frame(
    arm: ArmHandle,
    press_dir_task: np.ndarray,
) -> list:
    """Build the rtde_c.forceMode ``task_frame`` 6-vector in BASE frame
    coords. Orientation: +x along the press direction; origin at the
    current TCP. Mirrors ``open_microwave._build_task_frame_for_pull``."""
    press_dir_base = arm.X_base_task.rotation.apply(press_dir_task)
    press_dir_base = press_dir_base / np.linalg.norm(press_dir_base)

    world_up_base = np.array([0.0, 0.0, 1.0])
    if abs(np.dot(press_dir_base, world_up_base)) > 0.95:
        world_up_base = np.array([0.0, 1.0, 0.0])
    z_axis = world_up_base - press_dir_base * np.dot(press_dir_base, world_up_base)
    z_axis = z_axis / np.linalg.norm(z_axis)
    y_axis = np.cross(z_axis, press_dir_base)

    R = np.column_stack([press_dir_base, y_axis, z_axis])
    rotvec = ScipyRotation.from_matrix(R).as_rotvec()

    from ..util.rtde_convert import rtde_to_pose
    tcp_base = rtde_to_pose(arm.receive.getActualTCPPose())
    return [
        float(tcp_base.translation[0]),
        float(tcp_base.translation[1]),
        float(tcp_base.translation[2]),
        float(rotvec[0]),
        float(rotvec[1]),
        float(rotvec[2]),
    ]


# -------------------------------------------------------------------------
#  Plan / run
# -------------------------------------------------------------------------

def _print_plan(
    button: np.ndarray,
    press_dir: np.ndarray,
    press_tcp: Pose,
    standoff: Pose,
) -> None:
    print("=" * 70)
    print(f"  Button (task xyz)  : {button}")
    print(f"  Press angle        : {np.degrees(PRESS_ANGLE_ABOVE_HORIZONTAL_RAD):+.2f}° "
          f"above horizontal")
    print(f"  Horizontal dir     : {PRESS_HORIZONTAL_DIR_TASK_XY}")
    print(f"  Press roll         : {np.degrees(PRESS_ROLL_RAD):+.2f}° "
          f"({PRESS_ROLL_RAD:+.4f} rad)")
    print(f"  Press dir (task)   : {press_dir}")
    print(f"  Press TCP (task)   : {press_tcp.translation}")
    print(f"  Tip-beyond-TCP     : "
          f"{(TIP_FLANGE_DIST_M - TCP_OFFSET_ROBOTIQ_2F85[2])*1000:.1f} mm")
    print(f"  Standoff (task)    : {standoff.translation} "
          f"({STANDOFF_M*100:.0f} cm back)")
    print(f"  Press force        : {PRESS_FORCE_N} N for {PRESS_HOLD_S} s")
    print(f"  Contact threshold  : {CONTACT_THRESHOLD_N} N")
    print(f"  Approach speed     : {APPROACH_SPEED_M_S*100:.1f} cm/s")
    print(f"  Transit altitude   : {CONFIG.transit_z} m (task z)")
    print("=" * 70)


def _current_q(session: Session) -> dict[str, np.ndarray]:
    return {
        name: np.asarray(arm.receive.getActualQ(), dtype=float)
        for name, arm in session.arms.items()
    }


def _current_tcp_pose_task(arm: ArmHandle) -> Pose:
    return arm.to_task(rtde_to_pose(arm.receive.getActualTCPPose()))


def _planned_or_linear_press_transit(
    session: Session,
    arm: ArmHandle,
    label: str,
    target: Pose,
    config: PickPlaceConfig,
) -> bool:
    """Plan a free-space transit from the current TCP to ``target``.

    Used for the pre-press approach to the standoff hover. Falls back to
    the original ``approach_to`` moveL when motion planning is disabled,
    when the planner raises, or when execution fails (with
    ``MOTION_PLAN_AUTO_FALLBACK=True``)."""
    waypoints = [_current_tcp_pose_task(arm), target]
    print(f"\n→ planned transit: {label}")
    for i, wp in enumerate(waypoints):
        print(f"  wp {i}: xyz={np.round(wp.translation, 3)}")

    def _movel_fallback(reason: str) -> bool:
        print(f"  ➜ moveL fallback ({reason}); routing through approach_to")
        approach_to(
            arm, target,
            config.transit_speed, config.transit_accel,
        )
        return True

    if not USE_MOTION_PLANNING:
        return _movel_fallback("USE_MOTION_PLANNING=False")

    from ..planning.execute import execute_plan
    from ..planning.transit import (
        InfeasiblePlanError, make_rtde_ik, plan_transit,
    )

    diagram, plant, _, _ = WORLD.build_planning_scene()
    root_ctx = diagram.CreateDefaultContext()
    plant_ctx = plant.GetMyMutableContextFromRoot(root_ctx)
    try:
        plan = plan_transit(
            plant=plant, arm=ARM,
            waypoints=waypoints,
            plant_context=plant_ctx,
            current_q=_current_q(session),
            use_rrt_fallback=MOTION_PLAN_RRT_FALLBACK,
            rrt_diagram=diagram,
            rrt_max_iters=MOTION_PLAN_RRT_MAX_ITERS,
            rrt_shortcut_attempts=MOTION_PLAN_RRT_SHORTCUT_ATTEMPTS,
            min_clearance_m=0.01,
            rtde_ik=make_rtde_ik(arm),
        )
    except InfeasiblePlanError as exc:
        print(f"  ✗ motion plan infeasible: {exc}")
        if MOTION_PLAN_AUTO_FALLBACK:
            return _movel_fallback("planner raised InfeasiblePlanError")
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
            return _movel_fallback(
                f"execute_plan failed: {result.reason}",
            )
        return False
    print("  ✓ planned transit reached.")
    return True


def _planned_return_to_joint_start(
    session: Session,
    arm: ArmHandle,
    q_start: list[float],
    config: PickPlaceConfig,
) -> bool:
    """Plan and execute a joint-space return to the task-entry joints."""
    print("\n→ planned transit: post-press return to q_start")

    if not USE_MOTION_PLANNING:
        print("  ➜ moveJ fallback (USE_MOTION_PLANNING=False)")
        arm.control.moveJ(q_start)
        return True

    from ..planning.execute import execute_plan
    from ..planning.transit import (
        InfeasiblePlanError,
        _arm_model_instance,
        _arm_position_indices,
        _plan_simple_spline,
    )

    diagram, plant, _, _ = WORLD.build_planning_scene()
    root_ctx = diagram.CreateDefaultContext()
    plant_ctx = plant.GetMyMutableContextFromRoot(root_ctx)

    q_current = _current_q(session)
    q0_full = plant.GetPositions(plant_ctx).copy()
    for arm_name, q_arm in q_current.items():
        inst = _arm_model_instance(plant, arm_name)
        q0_full[_arm_position_indices(plant, inst)] = np.asarray(q_arm, dtype=float)

    arm_instance = _arm_model_instance(plant, ARM)
    arm_idx = _arm_position_indices(plant, arm_instance)
    q1_full = q0_full.copy()
    q1_full[arm_idx] = np.asarray(q_start, dtype=float)
    plant.SetPositions(plant_ctx, q0_full)

    max_joint_delta = float(np.max(np.abs(q1_full[arm_idx] - q0_full[arm_idx])))
    duration_s = max(1.0, max_joint_delta / 0.5 * 2)

    try:
        plan = _plan_simple_spline(
            plant=plant,
            plant_context=plant_ctx,
            arm=ARM,
            arm_instance=arm_instance,
            waypoints_q_full=[q0_full, q1_full],
            duration_s=duration_s,
            check_collisions=True,
            min_clearance_m=0.01,
            other_arm_q=None,
            other_arm_instance=None,
        )
    except InfeasiblePlanError as exc:
        print(f"  ✗ q_start return plan infeasible: {exc}")
        if MOTION_PLAN_AUTO_FALLBACK:
            print("  ➜ moveJ fallback (planner raised InfeasiblePlanError)")
            arm.control.moveJ(q_start)
            return True
        return False

    plan.metadata["planner"] = "joint_spline_to_q_start"
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
        print(f"  ✗ q_start return execution failed: {result.reason}")
        if MOTION_PLAN_AUTO_FALLBACK:
            print("  ➜ moveJ fallback (execute_plan failed)")
            arm.control.moveJ(q_start)
            return True
        return False
    print("  ✓ returned to q_start.")
    return True


def _expected_motion_leg_count() -> int:
    expected = 0
    if MOTION_PLAN_PRE_PRESS_APPROACH:
        expected += 1
    if MOTION_PLAN_POST_PRESS_RETURN:
        expected += 1
    return expected


def _plan_motion_legs(
    standoff: Pose,
    config: PickPlaceConfig,
    *,
    start_pose: Pose | None = None,
    start_q: dict[str, np.ndarray] | None = None,
    q_start: np.ndarray | None = None,
    log_prefix: str = "sim",
):
    """Plan all free-space press legs before any task motion is commanded.

    The contact phase itself is still live-only. The planned return assumes
    the real retract/lift lands back at the standoff hover before returning
    to the task-entry joint configuration.
    """
    from ..planning.transit import (
        InfeasiblePlanError,
        _arm_model_instance,
        _arm_position_indices,
        _plan_simple_spline,
        _tcp_frame,
        plan_transit_chained,
    )
    from ..util.rotations import Rotation as RotUtil

    legs: list[tuple[str, object]] = []
    diagram, plant, _, _ = WORLD.build_planning_scene()
    root = diagram.CreateDefaultContext()
    plant_ctx = plant.GetMyMutableContextFromRoot(root)
    arm_instance = _arm_model_instance(plant, ARM)
    arm_idx = _arm_position_indices(plant, arm_instance)

    q_seed_full = plant.GetPositions(plant_ctx).copy()
    if start_q is not None:
        for arm_name, q_arm in start_q.items():
            inst = _arm_model_instance(plant, arm_name)
            q_seed_full[_arm_position_indices(plant, inst)] = np.asarray(
                q_arm,
                dtype=float,
            )
    plant.SetPositions(plant_ctx, q_seed_full)

    q_start_arm = (
        np.asarray(q_start, dtype=float)
        if q_start is not None
        else q_seed_full[arm_idx].copy()
    )
    tcp_frame = _tcp_frame(plant, _arm_model_instance(plant, ARM))
    X = tcp_frame.CalcPoseInWorld(plant_ctx)
    default_start_pose = Pose(
        translation=np.asarray(X.translation()),
        rotation=RotUtil.from_matrix(X.rotation().matrix()),
    )
    chain_start_pose = start_pose if start_pose is not None else default_start_pose
    standoff_at_altitude = pose_at_altitude(standoff, config.transit_z)

    chained_arm_q = q_seed_full[arm_idx].copy()
    if MOTION_PLAN_PRE_PRESS_APPROACH:
        print(f"\n[{log_prefix}] planning pre-press approach...")
        try:
            plan = plan_transit_chained(
                arm=ARM,
                log_label="pre-press approach",
                prev_terminal_pose=chain_start_pose,
                chained_arm_q=chained_arm_q,
                plant=plant,
                waypoints=[chain_start_pose, standoff_at_altitude],
                plant_context=plant_ctx,
                current_q=start_q,
                use_rrt_fallback=MOTION_PLAN_RRT_FALLBACK,
                rrt_diagram=diagram,
                rrt_max_iters=MOTION_PLAN_RRT_MAX_ITERS,
                rrt_shortcut_attempts=MOTION_PLAN_RRT_SHORTCUT_ATTEMPTS,
                min_clearance_m=0.01,
            )
            legs.append(("pre-press approach to standoff hover", plan))
            chained_arm_q = np.asarray(
                plan.trajectory.value(plan.trajectory.end_time())
            ).flatten()[arm_idx]
        except InfeasiblePlanError as exc:
            print(f"  ✗ pre-press approach infeasible: {exc}")
            return legs

    if MOTION_PLAN_POST_PRESS_RETURN:
        try:
            q0_full = q_seed_full.copy()
            q0_full[arm_idx] = chained_arm_q
            q1_full = q0_full.copy()
            q1_full[arm_idx] = q_start_arm
            max_joint_delta = float(np.max(np.abs(q1_full[arm_idx] - q0_full[arm_idx])))
            duration_s = max(1.0, max_joint_delta / 0.5 * 2)
            plan2 = _plan_simple_spline(
                plant=plant,
                plant_context=plant_ctx,
                arm=ARM,
                arm_instance=_arm_model_instance(plant, ARM),
                waypoints_q_full=[q0_full, q1_full],
                duration_s=duration_s,
                check_collisions=True,
                min_clearance_m=0.01,
                other_arm_q=None,
                other_arm_instance=None,
            )
            plan2.metadata["planner"] = "joint_spline_to_q_start"
            legs.append(("post-press return to q_start", plan2))
        except InfeasiblePlanError as exc:
            print(f"  ✗ post-press return infeasible: {exc}")
    return legs


def _plan_sim_legs(standoff: Pose, config: PickPlaceConfig):
    """Plan the same free-space press legs used by real mode, from HOME."""
    return _plan_motion_legs(standoff, config, log_prefix="sim")


def _execute_planned_transit(
    session: Session,
    label: str,
    plan,
    config: PickPlaceConfig,
) -> bool:
    from ..planning.execute import execute_plan

    print(f"\n→ execute planned transit: {label}")
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


def _run_sim(standoff: Pose, config: PickPlaceConfig) -> int:
    """Plan and replay the pre-press approach in meshcat."""
    from pydrake.geometry import StartMeshcat
    from pydrake.systems.analysis import Simulator

    from ..planning import preview

    if not USE_MOTION_PLANNING:
        print("[sim] motion planning is disabled, but sim mode replays planned "
              "free-space legs. Run real mode with --no-motion-planning for "
              "the moveL/moveJ fallback.")
        return 1

    legs = _plan_sim_legs(standoff, config)
    expected_legs = _expected_motion_leg_count()
    if len(legs) != expected_legs:
        print(f"[sim] planned {len(legs)}/{expected_legs} required legs; aborting")
        return 1

    print()
    print("[sim] plan summary:")
    for label, plan in legs:
        clr = plan.min_clearance_m
        s = (f"{clr * 1000:.1f}mm" if np.isfinite(clr) else "n/a")
        print(f"  - {label}: planner={plan.metadata.get('planner')}  "
              f"duration={plan.duration_s:.2f}s  clearance={s}")

    meshcat = StartMeshcat()
    print(f"\n[sim] meshcat → {meshcat.web_url()}")
    scene = WORLD.build_sim_scene(meshcat=meshcat)
    simulator = Simulator(scene.diagram)
    simulator.Initialize()
    sim_ctx = simulator.get_mutable_context()
    return preview.run_interactive_legs(
        meshcat, scene.diagram, scene.plant, sim_ctx, legs,
    )


def run_on_arm(
    session: Session,
    arm: ArmHandle,
    standoff: Pose,
    press_dir_task: np.ndarray,
    config: PickPlaceConfig = CONFIG,
) -> bool:
    """Execute the press on a connected arm. Returns True on success."""
    # 1. Save start joints for the final return move.
    q_start = np.asarray(arm.receive.getActualQ(), dtype=float)

    planned_legs: list[tuple[str, object]] = []
    if USE_MOTION_PLANNING:
        print("\n[real] pre-planning all free-space press legs before execution...")
        planned_legs = _plan_motion_legs(
            standoff,
            config,
            start_pose=_current_tcp_pose_task(arm),
            start_q=_current_q(session),
            q_start=q_start,
            log_prefix="real",
        )
        expected_legs = _expected_motion_leg_count()
        if len(planned_legs) != expected_legs:
            print(f"[real] planned {len(planned_legs)}/{expected_legs} required "
                  "legs; aborting before commanding motion.")
            return False
        print("[real] all free-space press legs planned successfully.")

    leg_i = 0

    # 2. Close the gripper before any motion. If a press tool is held,
    # this locks it rigidly to the fingers so the tip's offset from the
    # TCP stays exactly TIP_FLANGE_DIST_M throughout the task. Calling
    # close() (not grasp(force)) drives to fully-closed/stalled — we
    # don't want a force-controlled close that might back off slightly
    # on contact and shift the tool. Defensive guard for arms without
    # a gripper attached (non-default sessions).
    if arm.gripper is None:
        print("→ no gripper attached — skipping close (warning: tool offset may drift)")
    else:
        print("→ close gripper (lock press tool before motion)")
        arm.gripper.set_speed_pct(config.gripper_close_speed_pct)
        arm.gripper.close()

    # 3. Lift to transit altitude (purely vertical in task z), unless the
    # planned approach owns the full free-space move from task-entry pose
    # to standoff hover.
    if not (USE_MOTION_PLANNING and MOTION_PLAN_PRE_PRESS_APPROACH):
        print("\n→ lift to transit altitude")
        lift_to_transit(arm, config.transit_z, config.transit_speed, config.transit_accel)

    # 4. Transit to standoff at altitude (motion-planned when enabled,
    # falls back to the original Cartesian approach_to otherwise or on
    # planner failure).
    standoff_at_altitude = pose_at_altitude(standoff, config.transit_z)
    if MOTION_PLAN_PRE_PRESS_APPROACH:
        if USE_MOTION_PLANNING:
            label, plan = planned_legs[leg_i]
            leg_i += 1
            if not _execute_planned_transit(session, label, plan, config):
                return False
        else:
            if not _planned_or_linear_press_transit(
                session, arm,
                "pre-press approach to standoff hover",
                standoff_at_altitude, config,
            ):
                return False
    else:
        print(f"→ transit to standoff XY at altitude {config.transit_z} m")
        approach_to(arm, standoff_at_altitude, config.transit_speed, config.transit_accel)
    print(f"→ descend to standoff: xyz={standoff.translation}")
    approach_to(arm, standoff, config.approach_speed, config.approach_accel)

    # 5. Drive forward along press direction until contact.
    v_task = [
        APPROACH_SPEED_M_S * float(press_dir_task[0]),
        APPROACH_SPEED_M_S * float(press_dir_task[1]),
        APPROACH_SPEED_M_S * float(press_dir_task[2]),
        0.0, 0.0, 0.0,
    ]
    print(f"→ move_until_contact: v={APPROACH_SPEED_M_S*100:.1f} cm/s along press axis, "
          f"threshold {CONTACT_THRESHOLD_N} N")
    move_until_contact(arm, v_task, config.approach_accel, CONTACT_THRESHOLD_N)
    print("  ✓ contact detected")

    # 6. Force mode: hold PRESS_FORCE_N along the press axis for
    # PRESS_HOLD_S seconds. Compliant only along press axis;
    # position-locked in the other 5 DOF.
    task_frame = _build_force_mode_task_frame(arm, press_dir_task)
    selection_vector = [1, 0, 0, 0, 0, 0]
    wrench = [PRESS_FORCE_N, 0.0, 0.0, 0.0, 0.0, 0.0]
    limits = [0.05, 0.05, 0.05, 0.5, 0.5, 0.5]
    FORCE_MODE_TYPE = 2

    print(f"→ apply {PRESS_FORCE_N} N for {PRESS_HOLD_S} s")
    try:
        arm.control.forceMode(task_frame, selection_vector, wrench,
                              FORCE_MODE_TYPE, limits)
        time.sleep(PRESS_HOLD_S)
    finally:
        arm.control.forceModeStop()
    print("  ✓ press complete")

    # 7. Retract to standoff (along -press direction, plain moveL).
    print(f"→ retract to standoff: xyz={standoff.translation}")
    retract_to(arm, standoff, config.retract_speed, config.retract_accel)

    # 8. Lift to transit altitude.
    print(f"→ lift to transit altitude {config.transit_z} m")
    lift_to_transit(arm, config.transit_z, config.transit_speed, config.transit_accel)

    # 9. Post-press return directly to the task-entry joint configuration.
    if MOTION_PLAN_POST_PRESS_RETURN:
        if USE_MOTION_PLANNING:
            label, plan = planned_legs[leg_i]
            leg_i += 1
            if not _execute_planned_transit(session, label, plan, config):
                return False
        else:
            if not _planned_return_to_joint_start(session, arm, list(q_start), config):
                return False
    else:
        print("→ moveJ back to start joints")
        arm.control.moveJ(list(q_start))

    print("\nDone.")
    return True


def main(
    dry: bool = False,
    derive: bool = False,
    mode: str = "real",
    motion_planning: bool = True,
) -> int:
    """CLI entry point. With --derive, prints the four constants derived
    from WAYPOINT_PATH/NAME and exits. Otherwise computes the plan and
    (unless --dry) drives the arm using the constants at the top of
    this file."""
    if not motion_planning:
        global USE_MOTION_PLANNING
        USE_MOTION_PLANNING = False
        print("[CLI] motion planning DISABLED — pre-press approach will use "
              "the original approach_to moveL.")

    if derive:
        return _derive_from_waypoint(WAYPOINT_PATH, WAYPOINT_NAME)

    press_dir = _build_press_dir(
        PRESS_ANGLE_ABOVE_HORIZONTAL_RAD, PRESS_HORIZONTAL_DIR_TASK_XY
    )
    press_rotation = _build_press_rotation(press_dir, PRESS_ROLL_RAD)
    press_tcp = _press_tcp_pose(
        BUTTON_TASK_XYZ, press_dir, press_rotation, TIP_FLANGE_DIST_M
    )
    standoff = _standoff_pose(press_tcp, press_dir, STANDOFF_M)

    _print_plan(BUTTON_TASK_XYZ, press_dir, press_tcp, standoff)

    if dry:
        print("[dry run] skipping RTDE connection. No motion commanded.")
        return 0

    if mode == "sim":
        if not SUPPORTS_SIM:
            print("[sim] this task does not support sim mode")
            return 1
        return _run_sim(standoff, CONFIG)
    if mode != "real":
        raise ValueError(f"unknown mode {mode!r}; choose 'real' or 'sim'")

    left = ARM == "ur_left"
    right = ARM == "ur_right"
    with default_session(left=left, right=right) as session:
        arm = session.arms[ARM]
        return 0 if run_on_arm(session, arm, standoff, press_dir, CONFIG) else 1


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument(
        "--dry",
        action="store_true",
        help="Plan and print the press geometry without connecting to RTDE.",
    )
    ap.add_argument(
        "--derive",
        action="store_true",
        help="Load WAYPOINT_PATH/NAME, derive the four press constants, "
             "and print them for paste-in. No motion.",
    )
    ap.add_argument(
        "--mode",
        choices=["real", "sim"],
        default="real",
        help=(
            "Execution mode. 'real' (default) runs on the rig via RTDE. "
            "'sim' plans the pre-press approach and replays it in meshcat."
        ),
    )
    ap.add_argument(
        "--no-motion-planning",
        action="store_true",
        help=(
            "Disable Drake motion planning for the pre-press approach. "
            "Falls back to the original ``approach_to`` moveL Cartesian "
            "transit. ``MOTION_PLAN_AUTO_FALLBACK`` already does this on "
            "per-segment plan failures; this flag forces it up front."
        ),
    )
    args = ap.parse_args()
    raise SystemExit(main(
        dry=args.dry, derive=args.derive, mode=args.mode,
        motion_planning=not args.no_motion_planning,
    ))
