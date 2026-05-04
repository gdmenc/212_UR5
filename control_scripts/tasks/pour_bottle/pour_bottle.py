"""End-to-end pick + pour + place of a water bottle on one arm.

Picks the bottle from BOTTLE_PICK_POSE_TASK with a side body pinch,
transits over POUR_TARGET_TASK while keeping the bottle upright, tilts
it about the gripper's horizontal tool-Y / finger-close axis so the opening sits at
POUR_TARGET_TASK, holds the tilt for POUR_DURATION_S, returns to
upright, then places the bottle back at BOTTLE_PLACE_POSE_TASK.

Hardware assumptions
--------------------
  - The arm named ``ARM`` has a Robotiq 2F-85 attached and a
    pre-calibrated TCP offset (set in calibration.py).
  - A water bottle (matching grasps/bottle.py geometry) sitting at
    BOTTLE_PICK_POSE_TASK with bottle +z pointing up.
  - A receiver (cup, bowl, drain, ...) below POUR_TARGET_XY_TASK. The
    bottle opening targets receiver rim z + POUR_HEIGHT_ABOVE_RIM_M,
    NOT table height.

Running
-------
Standalone:
    python -m control_scripts.tasks.pour_bottle [--dry]

Via the unified entrypoint:
    python -m control_scripts.run task pour_bottle [--dry]

First-run checklist
-------------------
1. Run ``--dry`` and inspect the printed pour TCP pose. Verify it is
   reachable (not too close to the base, not above transit_z).
2. Verify ``transit_z`` clears the bottle's full height (17.5 cm) plus
   whatever the gripper adds (~10-15 cm) plus a safety margin.
3. Check ``GRASP_ANGLE_RAD`` orients the gripper so that 'tilt forward
   away from the gripper' (the pour direction) corresponds to where
   POUR_TARGET sits relative to the pickup.
4. Start with a short ``POUR_DURATION_S`` (1-2 s) and a moderate tilt
   (≤ 110°) until you trust the geometry, then tune.
"""

from __future__ import annotations

import argparse
import time
from typing import Optional, Sequence

import numpy as np

from ...arm import ArmHandle
from ...config import PickPlaceConfig
from ...grasps.bottle import (
    BOTTLE_DEFAULT_GRASP_Z_M,
    bottle_body_grasp,
    bottle_pour_tcp_pose,
)
from ...moves import lift_to_transit, transit_xy
from ...pick import pick
from ...place import place
from ...session import default_session
from ...util.poses import Pose


# --- Tunables (edit to match your physical layout) ------------------------
BOTTLE_PICK_POSE_TASK = Pose(translation=[0, 0, 0.0])
"""Bottle base in task frame at PICK location. The bottle's +z must point
up out of this pose (identity rotation is correct for any free-standing
bottle on the table)."""

BOTTLE_PLACE_POSE_TASK = Pose(translation=[0, 0, 0.0])
"""Bottle base in task frame at PLACE location. Same as pickup by default
— change to put the bottle down somewhere else after pouring."""

POUR_TARGET_XY_TASK = np.array([-0.1, 0.0])
"""xy of the receiver opening / pour target in task frame."""

POUR_RECEIVER_RIM_Z_TASK = 0.15
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
"""xyz of the bottle's OPENING during the pour, in task frame. The bottle's
opening will end up here exactly (closed-form computation in
bottle_pour_tcp_pose)."""

POUR_TILT_RAD = float(np.radians(120))
"""Tilt about the gripper's horizontal tool-Y / finger-close axis. > 0 tips
the bottle in the observed pour direction by the direct angle. 90° =
horizontal; ~120° = past horizontal, typical 'pouring' tilt; 180° =
inverted. Tune together with POUR_DURATION_S."""

POUR_DURATION_S = 3.0
"""Seconds to hold the tilt. Total dispensed volume scales non-linearly
with this and with POUR_TILT_RAD (depends on fill level + opening
size)."""

POUR_MAX_TILT_STEP_RAD = float(np.radians(85))
"""Maximum tilt step for each moveL segment.

UR RTDE poses use absolute axis-angle rotvecs. Around this bottle grasp the
absolute orientation lies near the 180° rotvec boundary, so a single large
tilt can be represented on the opposite branch and make the wrist travel
the long way. We use the fewest segments needed to keep each relative tilt
below this threshold, then choose the nearest equivalent rotvec at each
waypoint to keep the commanded branch continuous.
"""

GRASP_ANGLE_RAD = 0
"""Bottle-frame angle at which to grasp (radians). The gripper sits on the
+x side of the bottle frame at angle 0 and the bottle tips toward -x
when tilted. Pick GRASP_ANGLE_RAD so that 'tip toward angle+π' points
roughly toward POUR_TARGET — that way the wrist isn't fighting the
geometry. Default π = grasp from -x side, opening pours toward +x."""

GRASP_Z = BOTTLE_DEFAULT_GRASP_Z_M
"""Height in bottle frame at which to pinch the body (m). Mid-body
default keeps wrist torque low when tilted."""

ARM = "ur_right"

CONFIG = PickPlaceConfig(
    transit_z=0.35,
    place_use_contact_descent=False,
    transit_speed=0.15,
    transit_accel=0.3,
    approach_speed=0.07,
    approach_accel=0.25,
    retract_speed=0.25,
    retract_accel=0.45,

    # 2F-85 partial-aperture preset. The bottle body is ~73 mm diameter;
    # 82 mm leaves ~4.5 mm clearance per side while staying under the
    # gripper's ~85 mm hard-open aperture.
    release_aperture_mm=82,

    gripper_open_speed_pct=40,
    gripper_close_speed_pct=30,
)


def plan_pick():
    return bottle_body_grasp(
        BOTTLE_PICK_POSE_TASK,
        angle_rad=GRASP_ANGLE_RAD,
        grasp_z=GRASP_Z,
    )


def plan_pour_pose() -> Pose:
    """Tilted TCP pose at which the bottle's opening sits at POUR_TARGET_TASK."""
    return bottle_pour_tcp_pose(
        BOTTLE_PICK_POSE_TASK,
        POUR_TARGET_TASK,
        angle_rad=GRASP_ANGLE_RAD,
        grasp_z=GRASP_Z,
        tilt_rad=POUR_TILT_RAD,
    )


def _pour_tilt_segments(pour_tilt_rad: float, max_step_rad: float) -> int:
    """Fewest tilt segments whose relative step stays under the configured cap."""
    return max(1, int(np.ceil(abs(pour_tilt_rad) / max_step_rad)))


def plan_upright_at_pour() -> Pose:
    """Upright (un-tilted) TCP pose with the opening above POUR_TARGET. Used
    as the transit waypoint right before/after the tilt: same orientation
    as the pick grasp, positioned so the descent into the tilt is a single
    moveL that interpolates orientation along the way."""
    return bottle_pour_tcp_pose(
        BOTTLE_PICK_POSE_TASK,
        POUR_TARGET_TASK,
        angle_rad=GRASP_ANGLE_RAD,
        grasp_z=GRASP_Z,
        tilt_rad=0.0,
    )


def plan_place_pose() -> Pose:
    grasp_at_dest = bottle_body_grasp(
        BOTTLE_PLACE_POSE_TASK,
        angle_rad=GRASP_ANGLE_RAD,
        grasp_z=GRASP_Z,
    )
    return grasp_at_dest.grasp_pose


def _print_plan(grasp, upright_at_pour: Pose, pour_pose: Pose, place_pose: Pose) -> None:
    print("=" * 60)
    print("  Pick (bottle base) :", BOTTLE_PICK_POSE_TASK.translation, "(task frame)")
    print("  Place (bottle base):", BOTTLE_PLACE_POSE_TASK.translation, "(task frame)")
    print("  Pour target (opening):", POUR_TARGET_TASK, "(task frame)")
    print("  Receiver rim z     :", f"{POUR_RECEIVER_RIM_Z_TASK:.3f} m")
    print("  Pour height        :", f"{POUR_HEIGHT_ABOVE_RIM_M*100:.1f} cm above rim")
    print("  Grasp angle        :", f"{np.degrees(GRASP_ANGLE_RAD):+.0f}°")
    print("  Grasp z (bottle)   :", f"{GRASP_Z*100:.1f} cm")
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
    print("  Grasp force        :", grasp.grasp_force, "N")
    print("=" * 60)


def _nearest_equivalent_rotvec(rotvec: np.ndarray, reference: Optional[np.ndarray]) -> np.ndarray:
    """Choose an axis-angle representation closest to ``reference``.

    A rotation vector is not unique: ``axis * theta`` is equivalent to
    ``axis * (theta +/- 2*pi)`` and ``-axis * (2*pi - theta)``. UR receives
    these raw values and interpolates between them, so keeping consecutive
    rotvecs close avoids long-way wrist rotations.
    """
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
    grasp_z: float = GRASP_Z,
) -> list[Pose]:
    """Tilt waypoints from upright to the final pour pose."""
    waypoints = []
    abs_tilt = abs(pour_tilt_rad)
    direction = 1.0 if pour_tilt_rad >= 0.0 else -1.0
    tilt = min(max_step_rad, abs_tilt)
    while tilt < abs_tilt:
        waypoints.append(
            bottle_pour_tcp_pose(
                bottle_pose_task,
                target_task,
                angle_rad=angle_rad,
                grasp_z=grasp_z,
                tilt_rad=direction * tilt,
            )
        )
        tilt += max_step_rad
    waypoints.append(
        bottle_pour_tcp_pose(
            bottle_pose_task,
            target_task,
            angle_rad=angle_rad,
            grasp_z=grasp_z,
            tilt_rad=pour_tilt_rad,
        )
    )
    return waypoints


def plan_untilt_in_place_waypoints(
    bottle_pose_task: Pose,
    target_task: Sequence[float],
    pour_pose: Pose,
    pour_tilt_rad: float,
    max_step_rad: float = POUR_MAX_TILT_STEP_RAD,
    *,
    angle_rad: float = GRASP_ANGLE_RAD,
    grasp_z: float = GRASP_Z,
) -> list[Pose]:
    """Return to 0° tilt at the pour TCP xyz before translating away."""
    waypoints = []
    abs_tilt = abs(pour_tilt_rad)
    direction = 1.0 if pour_tilt_rad >= 0.0 else -1.0
    tilt = max(0.0, abs_tilt - max_step_rad)
    while tilt > 0.0:
        pose = bottle_pour_tcp_pose(
            bottle_pose_task,
            target_task,
            angle_rad=angle_rad,
            grasp_z=grasp_z,
            tilt_rad=direction * tilt,
        )
        waypoints.append(Pose(translation=pour_pose.translation, rotation=pose.rotation))
        tilt -= max_step_rad

    upright = bottle_pour_tcp_pose(
        bottle_pose_task,
        target_task,
        angle_rad=angle_rad,
        grasp_z=grasp_z,
        tilt_rad=0.0,
    )
    waypoints.append(Pose(translation=pour_pose.translation, rotation=upright.rotation))
    return waypoints


def _pour(
    arm: ArmHandle,
    upright_at_pour: Pose,
    bottle_pose_task: Pose,
    target_task: Sequence[float],
    pour_tilt_rad: float,
    max_step_rad: float,
    grasp_angle_rad: float,
    grasp_z: float,
    config: PickPlaceConfig,
) -> None:
    """Tilt the held bottle so its opening reaches POUR_TARGET, hold for
    POUR_DURATION_S, return to upright at transit altitude."""
    # 1. Lift to transit altitude (re-asserts; pick already left us there).
    lift_to_transit(arm, config.transit_z, config.transit_speed, config.transit_accel)
    # 2. Transit horizontally to over the pour target — STILL UPRIGHT. The
    #    bottle's opening sits above POUR_TARGET (transit_xy applies
    #    pose_at_altitude internally, so we ride at transit_z).
    transit_xy(
        arm,
        upright_at_pour,
        config.transit_z,
        config.transit_speed,
        config.transit_accel,
    )
    # 3. Descend and tilt through rotvec-continuous waypoints.
    previous_rotvec = arm.to_base(upright_at_pour).rotation.as_rotvec()
    pour_waypoints = plan_pour_waypoints(
        bottle_pose_task,
        target_task,
        pour_tilt_rad,
        max_step_rad,
        angle_rad=grasp_angle_rad,
        grasp_z=grasp_z,
    )
    for waypoint in pour_waypoints:
        previous_rotvec = _move_l_continuous(
            arm,
            waypoint,
            config.approach_speed,
            config.approach_accel,
            previous_rotvec,
        )

    # 4. Hold while the bottle pours.
    print(f"  holding tilt for {POUR_DURATION_S:.1f} s ...")
    time.sleep(POUR_DURATION_S)

    # 5. Reverse-tilt in place first, then translate once upright.
    untilt_waypoints = plan_untilt_in_place_waypoints(
        bottle_pose_task,
        target_task,
        pour_waypoints[-1],
        pour_tilt_rad,
        max_step_rad,
        angle_rad=grasp_angle_rad,
        grasp_z=grasp_z,
    )
    untilt_waypoints.append(upright_at_pour)
    for waypoint in untilt_waypoints:
        previous_rotvec = _move_l_continuous(
            arm,
            waypoint,
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
        GRASP_Z,
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
