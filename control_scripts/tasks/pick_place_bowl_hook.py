"""End-to-end pick-and-place of a bowl using the HOOK arm (ur_left).

Parallel of ``pick_place_plate.py`` but using the welded hook on ur_left
to engage the bowl by its outer rim. The hook descends vertically at the
selected rim angle, the moving finger threads inside the rim while the
fixed jaw stays outside, and the rim wall is clamped between them.

Hardware assumptions
--------------------
  - The arm named ``ARM`` has the welded hook gripper attached and a
    pre-calibrated ``TCP_OFFSET_HOOK = [0, 0, 0.10275, 0, +π/2, 0]``
    (calibration.py).
  - A bowl matching ``grasps/bowl.py`` geometry sitting at task-frame
    ``BOWL_PICK_POSE_TASK`` with bowl +z pointing up.
  - Task-frame ``BOWL_PLACE_POSE_TASK`` is the intended bowl center
    after placement.

Running
-------
Standalone:
    python -m control_scripts.tasks.pick_place_bowl_hook [--dry]

Caveats
-------
This grasp does NOT yet compensate for the bowl's sidewall slant. The
rim is treated as a horizontal circle; the wrist descends straight down.
A tilted-rim variant (mirroring ``plate_rim_grasp_edge``) is left for a
future pass when the simple version is known-good on the rig.

First-run checklist
-------------------
1. Verify ``transit_z`` clears the suspended bowl (7.2 cm hangs below
   the TCP at the rim grasp) plus a margin.
2. Use ``--dry`` to print the grasp/place TCP poses without commanding
   any motion.
3. Watch the home → pregrasp transit on first run — at angle π the
   commanded rotvec is at the 180° boundary; if the wrist takes the
   long way, switch ``GRASP_ANGLE_RAD`` from ``np.pi`` to ``-np.pi``.
"""

from __future__ import annotations

import argparse

import numpy as np

from ..arm import ArmHandle
from ..config import PickPlaceConfig
from ..grasps.bowl import bowl_hook_grasp
from ..pick import pick
from ..place import place
from ..session import default_session
from ..util.poses import Pose


# --- Tunables (edit to match your physical layout) ------------------------
# Note: ascend 1 cm above nominal rim z to not hit the ground
BOWL_PICK_POSE_TASK = Pose(translation=[0.1, 0, 0.01])
"""Bowl center at PICK location, expressed in task frame. Identity
rotation is correct for any free-standing bowl on the table."""

# Note: ascend 1 cm above nominal rim z to not hit the ground
BOWL_PLACE_POSE_TASK = Pose(translation=[-0.1, 0.2, 0.01])
"""Bowl center at PLACE location, task frame. Same as pickup by default."""

GRASP_ANGLE_RAD = float(np.radians(180))
"""Bowl-frame angle at which to grasp the rim. At angle π the hook
approaches from the −X side of the bowl. Matches the recorded teaching
pose used to verify TCP_OFFSET_HOOK; equivalent to -np.pi modulo 2π."""

PLACE_ANGLE_RAD = float(np.radians(180+45))
"""Same orientation at the place location — keeps the wrist consistent
between pick and place."""

APPROACH_TILT_RAD = float(np.radians(10))
"""Tilt of the descent off pure-vertical, around tool +Y. Positive value
"dives" the gripper into the bowl from above-and-outward — front
(throat/finger) tips down, back (flange/wrist) lifts up. At 15° the
wrist sits ~2.7 cm higher in task z than the no-tilt grasp, which buys
forearm clearance from the table at the bowl's low rim height (7.2 cm).
Set to 0 to fall back to pure vertical descent. See
``grasps._hook_rim.hook_rim_rotation`` for the full sign convention."""

ARM = "ur_left"

CONFIG = PickPlaceConfig(
    transit_z=0.3,
    place_use_contact_descent=False,  # open-loop descent to the place pose
    transit_speed=0.1,
    transit_accel=0.2,
    approach_speed=0.05,
    approach_accel=0.2,
    retract_speed=0.1,
    retract_accel=0.2,

    # Hook has no continuous aperture. ``None`` makes ``prepare_for_grasp``
    # dispatch to ``gripper.open()`` (extend the finger) before descent.
    release_aperture_mm=None,

    gripper_open_speed_pct=40,   # no-op for the hook; kept for symmetry
    gripper_close_speed_pct=30,
)


def plan_pick():
    return bowl_hook_grasp(
        BOWL_PICK_POSE_TASK,
        angle_rad=GRASP_ANGLE_RAD,
        approach_tilt_rad=APPROACH_TILT_RAD,
    )


def plan_place() -> Pose:
    """TCP pose at release. Uses the same rim-grasp factory at the place
    location so the gripper's orientation stays consistent between pick
    and place."""
    grasp_at_dest = bowl_hook_grasp(
        BOWL_PLACE_POSE_TASK,
        angle_rad=PLACE_ANGLE_RAD,
        approach_tilt_rad=APPROACH_TILT_RAD,
    )
    return grasp_at_dest.grasp_pose


def _print_plan(grasp, place_pose: Pose) -> None:
    print("=" * 60)
    print("  Arm            :", ARM, "(hook gripper)")
    print("  Pick location  :", BOWL_PICK_POSE_TASK.translation, "(task frame)")
    print("  Place location :", BOWL_PLACE_POSE_TASK.translation, "(task frame)")
    print("  Grasp angle    :", f"{np.degrees(GRASP_ANGLE_RAD):+.0f}°")
    print("  Place angle    :", f"{np.degrees(PLACE_ANGLE_RAD):+.0f}°")
    print("  Approach tilt  :", f"{np.degrees(APPROACH_TILT_RAD):+.1f}°"
          " (positive = wrist UP, gripper dives in from above-and-outward)")
    print("  Grasp pose     :", grasp.grasp_pose.translation, "(task frame)")
    print("  Place pose     :", place_pose.translation, "(task frame)")
    print("  Pregrasp offset:", f"{grasp.pregrasp_offset*100:.1f} cm "
          "(along tool -Z, follows the tilt)")
    print("  Transit Z      :", CONFIG.transit_z, "m")
    print("=" * 60)


def run_on_arm(
    arm: ArmHandle,
    grasp,
    place_pose: Pose,
    config: PickPlaceConfig = CONFIG,
) -> bool:
    """Execute the pick + place on a live ArmHandle. Returns True on success."""
    print(f"\n→ pick: {grasp.description}")
    pick_result = pick(arm, grasp, config)
    if not pick_result.success:
        print(f"  ✗ pick FAILED: {pick_result.reason}")
        return False
    print("  ✓ pick succeeded.")

    print(f"\n→ place @ {BOWL_PLACE_POSE_TASK.translation}")
    place_result = place(arm, place_pose, config)
    if not place_result.success:
        print(f"  ✗ place FAILED: {place_result.reason}")
        return False
    print("  ✓ place succeeded.")

    print("\nDone — arm retracted to transit altitude.")
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
        return 0 if run_on_arm(arm, grasp, place_pose, CONFIG) else 1


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument(
        "--dry",
        action="store_true",
        help="Plan and print the grasp/place poses without connecting to RTDE.",
    )
    args = ap.parse_args()
    raise SystemExit(main(dry=args.dry))
