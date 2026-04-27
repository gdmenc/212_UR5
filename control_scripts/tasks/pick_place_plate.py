"""End-to-end pick-and-place of a plate on one arm.

The first task to run at the lab once connections are known-good.
Demonstrates the full stack: session → grasp factory → pick → place →
teardown, all in ~40 lines of body code.

Hardware assumptions
--------------------
  - The arm named ``ARM`` has a Robotiq 2F-85 attached and a pre-calibrated
    TCP offset (set in ``control_scripts/calibration.py``).
  - An 8-inch plate sitting at task-frame ``PLATE_PICK_POSE_TASK``. Edit
    the constant below to match your table layout.
  - Task-frame ``PLATE_PLACE_POSE_TASK`` is the intended plate center
    after placement.

Running
-------
Standalone:
    python -m control_scripts.tasks.pick_place_plate [--dry]

Via the unified entrypoint:
    python -m control_scripts.run task pick_place_plate [--dry]

API
---
    run_on_arm(arm, grasp, place_pose, config) -> bool
        Execute the pick + place on a connected ArmHandle. Use this from
        a routine that owns the session.
    main(dry=False) -> int
        CLI entry. Connects via ``default_session``, calls run_on_arm,
        returns a process-style exit code (0 = success).

First-run checklist
-------------------
1. Verify ``PickPlaceConfig.transit_z`` is above every object in the
   workspace. 0.30 m (30 cm task-Z) is a conservative default.
2. Verify ``plate_rim_grasp``'s ``PLATE_PREGRASP_OFFSET=0.03`` clears
   the plate. If not, pass a larger offset via a custom Grasp.
3. Use ``--dry`` to print what the motion WOULD do without connecting
   to the robot.
"""

from __future__ import annotations

import argparse

import numpy as np

from ..arm import ArmHandle
from ..config import PickPlaceConfig
from ..grasps.plate import plate_rim_grasp_edge
from ..pick import pick
from ..place import place
from ..session import default_session
from ..util.poses import Pose


# --- Tunables (edit to match your physical layout) ------------------------
#0.025 is the height of the plate rim that is reasonable for pickup
PLATE_PICK_POSE_TASK = Pose(translation=[0.2, -0.1, 0.025])
"""Plate center at PICK location, expressed in task frame."""

#0.025 is the height of the plate rim that is reasonable for setting down
PLATE_PLACE_POSE_TASK = Pose(translation=[-0.25, 0.5, 0.09])
"""Plate center at PLACE location, task frame."""

GRASP_ANGLE_RAD = 0
"""Which rim angle to grasp at. π/2 = approach the rim from the
microwave-facing side (y+ side of the plate)."""

PLACE_ANGLE_RAD = -np.pi * 7 / 24
"""Which rim angle to place at. π/2 = approach the rim from the
microwave-facing side (y+ side of the plate)."""

ARM = "ur_right"

CONFIG = PickPlaceConfig(
    transit_z=0.2,
    place_use_contact_descent=False,  # no force-seeking; no table contact
    transit_speed=0.15,
    transit_accel=0.3,

    # Staged release / pre-grasp aperture. ~25 mm clears the plate rim
    # but keeps the fingers narrow enough to stay inside the microwave
    # cavity. release_clearance lifts 2 cm between the partial open and
    # the full open so the fingers don't drag the released plate (and
    # symmetrically, lifts 2 cm after grasp before the larger retract).
    release_aperture_mm=25,
    # release_clearance=0.02,

    # Gripper speeds. Slower than the 100% defaults — controlled release
    # inside an enclosed cavity; gentle close on the plate rim.
    gripper_open_speed_pct=40,
    gripper_close_speed_pct=30,

    # Conservative defaults everywhere else. Speed things up in a later
    # pass once the motion is known-safe.
)


def plan_pick():
    # NOTE We are using rim_grasp here instead of vertical grasp
    return plate_rim_grasp_edge(PLATE_PICK_POSE_TASK, angle_rad=GRASP_ANGLE_RAD)


def plan_place() -> Pose:
    """TCP pose at release. Uses the same rim-grasp factory at the place
    location so the gripper's orientation stays consistent between pick
    and place — easier on the wrist than re-solving a new orientation."""
    grasp_at_dest = plate_rim_grasp_edge(
        PLATE_PLACE_POSE_TASK, angle_rad=PLACE_ANGLE_RAD
    )
    return grasp_at_dest.grasp_pose


def _print_plan(grasp, place_pose: Pose) -> None:
    print("=" * 60)
    print("  Pick location  :", PLATE_PICK_POSE_TASK.translation, "(task frame)")
    print("  Place location :", PLATE_PLACE_POSE_TASK.translation, "(task frame)")
    print("  Grasp angle    :", f"{np.degrees(GRASP_ANGLE_RAD):+.0f}°")
    print("  Grasp pose     :", grasp.grasp_pose.translation, "(task frame)")
    print("  Transit Z      :", CONFIG.transit_z, "m")
    print("  Gripper speeds :",
          f"open={CONFIG.gripper_open_speed_pct}%,",
          f"close={CONFIG.gripper_close_speed_pct}%")
    print("  Grasp force    :", grasp.grasp_force, "N")
    print("=" * 60)


def run_on_arm(
    arm: ArmHandle,
    grasp,
    place_pose: Pose,
    config: PickPlaceConfig = CONFIG,
) -> bool:
    """Execute the pick + place on a live ArmHandle. Returns True on success.

    Use this from a routine that owns the Session — it does not connect
    or disconnect on its own."""
    print(f"\n→ pick: {grasp.description}")
    pick_result = pick(arm, grasp, config)
    if not pick_result.success:
        print(f"  ✗ pick FAILED: {pick_result.reason}")
        return False
    print("  ✓ pick succeeded.")

    print(f"\n→ place @ {PLATE_PLACE_POSE_TASK.translation}")
    place_result = place(arm, place_pose, config)
    if not place_result.success:
        print(f"  ✗ place FAILED: {place_result.reason}")
        return False
    print("  ✓ place succeeded.")

    print("\nDone — arm retracted to transit altitude.")
    return True


def main(dry: bool = False) -> int:
    """CLI / registry entry point. Returns 0 on success, 1 on task
    failure, leaves exceptions to the caller."""
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
        # Optional: home before starting. Uncomment if the arm is not
        # already at a safe configuration.
        # session.move_to_home()
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
