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
from typing import Optional, Sequence

import numpy as np

from ..arm import ArmHandle
from ..config import PickPlaceConfig
from ..grasps.bottle import (
    bottle_hook_grasp,
    bottle_hook_pour_tcp_pose,
)
from ..moves import lift_to_transit, transit_xy
from ..pick import pick
from ..place import place
from ..session import default_session
from ..util.poses import Pose, pose_at_altitude


# --- Tunables (edit to match your physical layout) ------------------------
# Note: descend 1 cm below the nominal rim target for a more secure hook grasp.
BOTTLE_PICK_POSE_TASK = Pose(translation=[-0.1, 0, -0.01])
"""Bottle base in task frame at PICK location. Identity rotation is
correct for any free-standing bottle on the table."""

BOTTLE_PLACE_POSE_TASK = Pose(translation=[-0.1, 0, -0.01])
"""Bottle base in task frame at PLACE location. Same as pickup by default."""

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

POUR_TILT_RAD = float(np.radians(140))
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

CONFIG = PickPlaceConfig(
    transit_z=0.4,
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


def plan_upright_at_pour() -> Pose:
    """Upright (un-tilted) TCP pose with the opening above POUR_TARGET."""
    return bottle_hook_pour_tcp_pose(
        BOTTLE_PICK_POSE_TASK,
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


def run_on_arm(
    arm: ArmHandle,
    grasp,
    pour_pose: Pose,
    place_pose: Pose,
    config: PickPlaceConfig = CONFIG,
) -> bool:
    print(f"\n→ pick: {grasp.description}")
    pick_result = pick(arm, grasp, config)
    if not pick_result.success:
        print(f"  ✗ pick FAILED: {pick_result.reason}")
        return False
    print("  ✓ pick succeeded.")

    print(f"\n→ pour @ {POUR_TARGET_TASK} "
          f"({POUR_HEIGHT_ABOVE_RIM_M*100:.1f} cm above rim, "
          f"tilt {np.degrees(POUR_TILT_RAD):.0f}°, {POUR_DURATION_S:.1f} s)")
    _pour(
        arm,
        plan_upright_at_pour(),
        BOTTLE_PICK_POSE_TASK,
        POUR_TARGET_TASK,
        POUR_TILT_RAD,
        POUR_MAX_TILT_STEP_RAD,
        GRASP_ANGLE_RAD,
        config,
    )
    print("  ✓ pour complete.")

    print(f"\n→ place @ {BOTTLE_PLACE_POSE_TASK.translation}")
    place_result = place(arm, place_pose, config)
    if not place_result.success:
        print(f"  ✗ place FAILED: {place_result.reason}")
        return False
    print("  ✓ place succeeded.")

    print("\nDone — arm retracted to transit altitude.")
    return True


def main(dry: bool = False) -> int:
    grasp = plan_pick()
    upright_at_pour = plan_upright_at_pour()
    pour_pose = plan_pour_pose()
    place_pose = plan_place_pose()
    _print_plan(grasp, upright_at_pour, pour_pose, place_pose)

    if dry:
        print("[dry run] skipping RTDE connection. No motion commanded.")
        return 0

    left = ARM == "ur_left"
    right = ARM == "ur_right"
    with default_session(left=left, right=right) as session:
        arm = session.arms[ARM]
        return 0 if run_on_arm(arm, grasp, pour_pose, place_pose, CONFIG) else 1


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument(
        "--dry",
        action="store_true",
        help="Plan and print the pick/pour/place poses without connecting to RTDE.",
    )
    args = ap.parse_args()
    raise SystemExit(main(dry=args.dry))
