"""Press a button with the right arm, using a recorded press pose as ground truth.

Source-of-truth pose
--------------------
The pose at which the *tip of the pressing tool* touches the button comes
from a recorded waypoint JSON in ``logs/waypoints/``. The waypoint was
captured by hand-moving the arm to the button until the tip just made
contact, then snapshotting joints + task pose via
``examples/record_waypoints.py``.

The recorded pose is the **TCP** pose. The TCP is at flange + 0.184 m
(``calibration.TCP_OFFSET_ROBOTIQ_2F85[2]``) along flange +z. The press
tool's physical tip extends ``TIP_FLANGE_DIST_M = 0.198894954`` m along
flange +z — i.e. ``TIP_BEYOND_TCP_M`` = 14.9 mm past the TCP. Subtracting
gives the location of the button itself in task frame:

    button_task = recorded_tcp_task + (TIP_FLANGE_DIST_M - tcp_offset_z)
                                       * flange_z_in_task

Orientation handling
--------------------
The recorded flange +z is **not** parallel to the ground — it points
~31° below horizontal in this example. ``PRESS_TILT_BELOW_HORIZONTAL_RAD``
controls how the press axis is oriented:

  - ``None`` (default): use the recorded orientation as-is. Press axis
    inherits whatever tilt the operator captured. The press TCP equals
    the recorded TCP exactly (the tip lands on the button without any
    re-orientation correction).
  - ``0.0``: horizontalize. Project flange +z onto the xy plane and
    rotate the recorded orientation by the minimum-angle correction.
    Press axis ends up parallel to the ground.
  - any other float: tilt the press axis by that many radians **below**
    horizontal, in the vertical plane containing the recorded flange
    +z's xy projection. Positive = tip-down (most natural for pressing
    a button from above-and-behind); negative = tip-up.

In all cases the gripper's roll about the press axis stays as close to
the recorded one as possible (we apply only the minimum rotation needed
to align flange +z with the target direction).

When we re-orient (anything other than ``None``), the press TCP differs
slightly from the recorded TCP — because the tip-vs-TCP offset is along
flange +z and that direction has changed. The button's task-frame
position is unchanged (it's a fixed physical point), but the TCP target
moves so the tip still lands on the button under the new orientation.

Sequence
--------
    1. Save current joints (return target at end).
    2. Close the gripper. Locks any held press tool rigidly to the
       fingers BEFORE any motion — important because the press relies
       on the tool sitting at a known offset from the TCP, and a loose
       grip would let the tool shift on contact.
    3. Lift to transit altitude (task z).
    4. moveL to the standoff pose (5 cm back along the press axis from
       the press TCP).
    5. move_until_contact: drive forward at ``APPROACH_SPEED_M_S`` along
       the press axis until TCP force exceeds ``CONTACT_THRESHOLD_N``.
    6. forceMode: apply ``PRESS_FORCE_N`` along the press axis for
       ``PRESS_HOLD_S`` seconds (compliant in press axis, position-locked
       in the other 5 DOF).
    7. forceModeStop, then moveL back to the standoff pose.
    8. Lift to transit altitude, then moveJ back to the start joints.

Running
-------
    python -m control_scripts.tasks.press_button [--dry]
"""

from __future__ import annotations

import argparse
import json
import os
import time
from typing import Optional, Tuple

import numpy as np
from scipy.spatial.transform import Rotation as ScipyRotation

from ..arm import ArmHandle
from ..calibration import TCP_OFFSET_ROBOTIQ_2F85
from ..config import PickPlaceConfig
from ..moves import approach_to, lift_to_transit, move_until_contact, retract_to
from ..session import default_session
from ..util.poses import Pose, pose_at_altitude
from ..util.rotations import Rotation


# --- Tunables (edit to match your physical layout) ------------------------
WAYPOINT_PATH = "logs/waypoints/ur_right_20260430_223037.json"
"""JSON file with the recorded press pose (tip touching the button)."""

WAYPOINT_NAME = "press microwave button 1"
"""Which snapshot inside the JSON to use, by name."""

TIP_FLANGE_DIST_M = 0.198894954
"""Distance from the wrist-3 flange face to the physical pressing tip,
along flange +z. The tip is an extension past the calibrated TCP — the
TCP itself sits at ``TCP_OFFSET_ROBOTIQ_2F85[2]`` (0.184 m). Difference
(14.9 mm) is the gap between the pinch point and whatever is held in
the gripper that does the pressing."""

STANDOFF_M = 0.05
"""Standoff distance back along the press axis from the press TCP. The
arm goes here before move_until_contact."""

PRESS_FORCE_N = 10.0
"""Newtons applied along the press axis once contact is detected."""

PRESS_HOLD_S = 1.0
"""How long to hold ``PRESS_FORCE_N`` after contact, before retracting."""

CONTACT_THRESHOLD_N = 5.0
"""TCP force threshold (N) that ends move_until_contact. Conservative
default — microwave buttons typically actuate at 2-5 N."""

APPROACH_SPEED_M_S = 0.02
"""Linear speed (m/s) of the seek-to-contact move. 2 cm/s — slow enough
to read force cleanly, fast enough that the 5 cm standoff is covered
in ~2.5 s."""

PRESS_TILT_BELOW_HORIZONTAL_RAD: Optional[float] = None
"""Press axis tilt, in radians below horizontal. See module docstring
for the full semantics. Quick reference:

    None        → use recorded orientation as-is (recorded tilt, ~31°
                  for the current waypoint).
    0.0         → horizontal press axis.
    np.radians(30) → 30° below horizontal (tip-down approach).
    np.radians(-15) → 15° above horizontal (tip-up approach).

Only the press axis (flange +z) is rotated; the gripper's roll about
that axis stays as close to the recorded value as the minimum-angle
correction allows."""

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
#  Loading / geometry helpers
# -------------------------------------------------------------------------

def _load_waypoint(path: str, name: str) -> Pose:
    """Read the recorded TCP task-frame pose from the JSON snapshot named
    ``name``. Raises if the file or the named snapshot is missing."""
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


def _apply_press_tilt(
    rotation: Rotation,
    tilt_below_horizontal_rad: Optional[float],
) -> Tuple[Rotation, float]:
    """Return ``(corrected_rotation, recorded_tilt_rad)``.

    ``recorded_tilt_rad`` is the angle the original flange +z made with
    horizontal — informational, useful for log output regardless of
    whether we re-orient.

    If ``tilt_below_horizontal_rad`` is ``None``, the rotation is
    returned unchanged (caller wanted the recorded orientation).
    Otherwise we rotate flange +z to a direction in the *same vertical
    plane* as the recorded press direction, but tilted by
    ``tilt_below_horizontal_rad`` below horizontal. The correction
    applied to the rotation is the minimum-angle one — so other tool
    axes (and the gripper's roll about the press axis) stay as close
    to the recorded ones as possible.
    """
    z_orig = rotation.apply(np.array([0.0, 0.0, 1.0]))
    recorded_tilt = float(np.arcsin(-z_orig[2]))  # +ve if flange +z points DOWN

    if tilt_below_horizontal_rad is None:
        return rotation, recorded_tilt

    # Horizontal direction the press axis points along — preserve this so
    # the press still aims at the same button in xy, regardless of tilt.
    xy_dir = np.array([z_orig[0], z_orig[1], 0.0])
    xy_norm = np.linalg.norm(xy_dir)
    if xy_norm < 1e-9:
        raise ValueError(
            "recorded flange +z has no horizontal component (purely "
            "vertical) — cannot define a tilted press direction relative "
            "to horizontal. Set PRESS_TILT_BELOW_HORIZONTAL_RAD=None "
            "or use a different waypoint."
        )
    xy_dir = xy_dir / xy_norm

    # Target flange +z = horizontal-component along xy_dir, vertical-
    # component pointing DOWN (tip-down) by sin(tilt).
    target_z = (
        np.cos(tilt_below_horizontal_rad) * xy_dir
        + np.array([0.0, 0.0, -np.sin(tilt_below_horizontal_rad)])
    )
    target_z = target_z / np.linalg.norm(target_z)

    # Minimum-angle rotation taking z_orig to target_z.
    axis = np.cross(z_orig, target_z)
    axis_norm = np.linalg.norm(axis)
    if axis_norm < 1e-9:
        # Already aligned (or anti-aligned). If anti-aligned, flip 180°
        # about any horizontal axis perpendicular to z_orig.
        if np.dot(z_orig, target_z) > 0:
            return rotation, recorded_tilt
        # Anti-aligned (180°): build a perpendicular axis manually.
        perp = np.array([1.0, 0.0, 0.0])
        if abs(np.dot(perp, z_orig)) > 0.9:
            perp = np.array([0.0, 1.0, 0.0])
        axis = np.cross(z_orig, perp)
        axis = axis / np.linalg.norm(axis)
        correction = Rotation.from_rotvec(axis * np.pi)
        return correction * rotation, recorded_tilt

    axis = axis / axis_norm
    angle = float(np.arccos(np.clip(np.dot(z_orig, target_z), -1.0, 1.0)))
    correction = Rotation.from_rotvec(axis * angle)
    return correction * rotation, recorded_tilt


def _press_geometry(
    recorded_tcp_task: Pose,
    tip_flange_dist_m: float,
    tilt_below_horizontal_rad: Optional[float],
) -> Tuple[np.ndarray, np.ndarray, Rotation, float]:
    """Compute the physical button position in task frame, plus the
    chosen press-axis direction and orientation.

    Returns
    -------
    button_task        : (3,) — fixed physical button location in task frame.
    press_dir_task     : (3,) — unit vector along the press direction
                          (= chosen flange +z in task frame).
    press_rotation     : Rotation — the orientation to use during the press.
    recorded_tilt_rad  : float — angle the RECORDED flange +z made with
                          horizontal (informational; positive = tip-down).
    """
    tcp_offset_z = TCP_OFFSET_ROBOTIQ_2F85[2]
    tip_beyond_tcp = tip_flange_dist_m - tcp_offset_z

    # Button position is fixed in space (set by the recorded pose).
    flange_z_recorded = recorded_tcp_task.rotation.apply(np.array([0.0, 0.0, 1.0]))
    button_task = recorded_tcp_task.translation + tip_beyond_tcp * flange_z_recorded

    press_rotation, recorded_tilt = _apply_press_tilt(
        recorded_tcp_task.rotation, tilt_below_horizontal_rad
    )
    press_dir_task = press_rotation.apply(np.array([0.0, 0.0, 1.0]))
    press_dir_task = press_dir_task / np.linalg.norm(press_dir_task)
    return button_task, press_dir_task, press_rotation, recorded_tilt


def _press_tcp_pose(
    button_task: np.ndarray,
    press_dir_task: np.ndarray,
    press_rotation: Rotation,
    tip_flange_dist_m: float,
) -> Pose:
    """TCP pose that puts the tip at the button under ``press_rotation``."""
    tcp_offset_z = TCP_OFFSET_ROBOTIQ_2F85[2]
    tip_beyond_tcp = tip_flange_dist_m - tcp_offset_z
    return Pose(
        translation=button_task - tip_beyond_tcp * press_dir_task,
        rotation=press_rotation,
    )


def _standoff_pose(
    press_tcp_task: Pose,
    press_dir_task: np.ndarray,
    standoff_m: float,
) -> Pose:
    """Standoff = press TCP backed off ``standoff_m`` along -press_dir."""
    return Pose(
        translation=press_tcp_task.translation - standoff_m * press_dir_task,
        rotation=press_tcp_task.rotation,
    )


# -------------------------------------------------------------------------
#  Force-mode helpers
# -------------------------------------------------------------------------

def _build_force_mode_task_frame(
    arm: ArmHandle,
    press_dir_task: np.ndarray,
) -> list:
    """Build the rtde_c.forceMode ``task_frame`` 6-vector in BASE frame
    coords (the controller's expected input). Orientation has +x along
    the press direction; origin sits at the current TCP.

    Mirrors ``open_microwave._build_task_frame_for_pull``."""
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
    recorded_tcp: Pose,
    button_task: np.ndarray,
    press_dir_task: np.ndarray,
    press_tcp: Pose,
    standoff: Pose,
    original_tilt_rad: float,
) -> None:
    print("=" * 70)
    print(f"  Waypoint           : {WAYPOINT_PATH} :: {WAYPOINT_NAME!r}")
    print(f"  Recorded TCP (task): xyz={recorded_tcp.translation}")
    print(f"  Tip at flange+{TIP_FLANGE_DIST_M:.6f} m (TCP_OFFSET_z={TCP_OFFSET_ROBOTIQ_2F85[2]} m)")
    print(f"  Tip-beyond-TCP     : {(TIP_FLANGE_DIST_M - TCP_OFFSET_ROBOTIQ_2F85[2])*1000:.1f} mm")
    print(f"  Button (task xyz)  : {button_task}")
    print(f"  Recorded tilt      : {np.degrees(original_tilt_rad):+.2f}° below horizontal")
    if PRESS_TILT_BELOW_HORIZONTAL_RAD is None:
        chosen = "(use recorded)"
    else:
        chosen = f"{np.degrees(PRESS_TILT_BELOW_HORIZONTAL_RAD):+.2f}° below horizontal"
    effective_tilt_deg = float(np.degrees(np.arcsin(-press_dir_task[2])))
    print(f"  Configured tilt    : {chosen}")
    print(f"  Effective tilt     : {effective_tilt_deg:+.2f}° below horizontal")
    print(f"  Press dir (task)   : {press_dir_task}")
    print(f"  Press TCP (task)   : {press_tcp.translation}")
    print(f"  Standoff (task)    : {standoff.translation} ({STANDOFF_M*100:.0f} cm back)")
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

    # 3. moveL to the standoff pose. We first transit to standoff at
    # transit altitude (preserves orientation interp at altitude), then
    # descend to the standoff itself. Both are plain moveL — approach_to
    # is just moveL with the right naming for prose.
    standoff_at_altitude = pose_at_altitude(standoff, config.transit_z)
    print(f"→ transit to standoff XY at altitude {config.transit_z} m")
    approach_to(arm, standoff_at_altitude, config.transit_speed, config.transit_accel)
    print(f"→ descend to standoff: xyz={standoff.translation}")
    approach_to(arm, standoff, config.approach_speed, config.approach_accel)

    # 4. Drive forward along press direction until contact.
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

    # 5. Force mode: hold PRESS_FORCE_N along the press axis for
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

    # 6. Retract to standoff (along -press direction, plain moveL).
    print(f"→ retract to standoff: xyz={standoff.translation}")
    retract_to(arm, standoff, config.retract_speed, config.retract_accel)

    # 7. Lift to transit altitude, moveJ back to start joints.
    print(f"→ lift to transit altitude {config.transit_z} m")
    lift_to_transit(arm, config.transit_z, config.transit_speed, config.transit_accel)

    print("→ moveJ back to start joints")
    arm.control.moveJ(q_start)

    print("\nDone.")
    return True


def main(dry: bool = False) -> int:
    """CLI entry point. Loads the waypoint, computes geometry, prints the
    plan, and (unless --dry) drives the arm."""
    if not os.path.exists(WAYPOINT_PATH):
        print(f"ERROR: waypoint file not found: {WAYPOINT_PATH}")
        return 1

    recorded_tcp = _load_waypoint(WAYPOINT_PATH, WAYPOINT_NAME)
    button_task, press_dir_task, press_rotation, tilt_rad = _press_geometry(
        recorded_tcp, TIP_FLANGE_DIST_M, PRESS_TILT_BELOW_HORIZONTAL_RAD
    )
    press_tcp = _press_tcp_pose(button_task, press_dir_task, press_rotation, TIP_FLANGE_DIST_M)
    standoff = _standoff_pose(press_tcp, press_dir_task, STANDOFF_M)

    _print_plan(recorded_tcp, button_task, press_dir_task,
                press_tcp, standoff, tilt_rad)

    if dry:
        print("[dry run] skipping RTDE connection. No motion commanded.")
        return 0

    left = ARM == "ur_left"
    right = ARM == "ur_right"
    with default_session(left=left, right=right) as session:
        arm = session.arms[ARM]
        return 0 if run_on_arm(arm, standoff, press_dir_task, CONFIG) else 1


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument(
        "--dry",
        action="store_true",
        help="Plan and print the press geometry without connecting to RTDE.",
    )
    args = ap.parse_args()
    raise SystemExit(main(dry=args.dry))
