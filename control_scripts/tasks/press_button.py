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
from ..session import default_session
from ..util.poses import Pose, pose_at_altitude
from ..util.rotations import Rotation


# --- Button location, derived from microwave geometry ---------------------
# Source-of-truth for the microwave lives in ``control_scripts/microwave.py``.
# Edit there and these update automatically.
_BUTTON_FROM_OUTER_RIGHT_X = 0.0675
"""How far the button sits in (toward -x) from the microwave's outer
right edge. Measured: 6.75 cm."""

_BUTTON_HEIGHT_ABOVE_WHITE_TABLE = 0.105 - 0.01#account for thickness of the fingers
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

PRESS_FORCE_N = 15.0
"""Newtons applied along the press axis once contact is detected."""

PRESS_HOLD_S = 1.0
"""How long to hold ``PRESS_FORCE_N`` after contact, before retracting."""

CONTACT_THRESHOLD_N = 10.0
"""TCP force threshold (N) that ends move_until_contact."""

APPROACH_SPEED_M_S = 0.02
"""Linear speed (m/s) of the seek-to-contact move."""

ARM = "ur_right"

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


def run_on_arm(
    arm: ArmHandle,
    standoff: Pose,
    press_dir_task: np.ndarray,
    config: PickPlaceConfig = CONFIG,
) -> bool:
    """Execute the press on a connected arm. Returns True on success."""
    # 1. Save start joints for the final return move.
    q_start = list(arm.receive.getActualQ())

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

    # 3. Lift to transit altitude (purely vertical in task z).
    print("\n→ lift to transit altitude")
    lift_to_transit(arm, config.transit_z, config.transit_speed, config.transit_accel)

    # 4. moveL to the standoff pose. Transit at altitude first so the
    # orientation interpolation happens up high, then descend.
    standoff_at_altitude = pose_at_altitude(standoff, config.transit_z)
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

    # 8. Lift to transit altitude, moveJ back to start joints.
    print(f"→ lift to transit altitude {config.transit_z} m")
    lift_to_transit(arm, config.transit_z, config.transit_speed, config.transit_accel)

    print("→ moveJ back to start joints")
    arm.control.moveJ(q_start)

    print("\nDone.")
    return True


def main(dry: bool = False, derive: bool = False) -> int:
    """CLI entry point. With --derive, prints the four constants derived
    from WAYPOINT_PATH/NAME and exits. Otherwise computes the plan and
    (unless --dry) drives the arm using the constants at the top of
    this file."""
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

    left = ARM == "ur_left"
    right = ARM == "ur_right"
    with default_session(left=left, right=right) as session:
        arm = session.arms[ARM]
        return 0 if run_on_arm(arm, standoff, press_dir, CONFIG) else 1


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
    args = ap.parse_args()
    raise SystemExit(main(dry=args.dry, derive=args.derive))
