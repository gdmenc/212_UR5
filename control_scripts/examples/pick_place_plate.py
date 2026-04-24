"""End-to-end pick-and-place of a plate using the right arm.

The first script to run at the lab once connections are known-good.
Demonstrates the full stack: session → grasp factory → pick → place →
teardown, all in ~40 lines of body code.

Hardware assumptions
--------------------
  - ``ur_right`` at ``192.168.1.102`` with a Robotiq 2F-85 attached and
    pre-calibrated TCP offset (set in control_scripts/calibration.py).
  - An 8-inch plate sitting at task-frame ``PLATE_PICK_POSE_TASK``. This
    defaults to 10 cm left of task origin, 20 cm toward the microwave —
    edit the constant below to match your table layout.
  - Task-frame ``PLATE_PLACE_POSE_TASK`` is the intended plate center
    after placement. Default places the plate 20 cm to the right of the
    pick location on the same table line.

Running
-------
    python -m control_scripts.examples.pick_place_plate

First-run checklist
-------------------
1. Verify ``PickPlaceConfig.transit_z`` is above every object in the
   workspace. 0.30 m (30 cm task-Z) is a conservative default.
2. Verify ``plate_rim_grasp``'s ``PLATE_PREGRASP_OFFSET=0.03`` clears
   the plate. If not, pass a larger offset via a custom Grasp.
3. Use ``--dry`` to print what the motion WOULD do without connecting
   to the robot — useful for a first sanity check at home.
"""

from __future__ import annotations

import argparse

import numpy as np

from ..config import PickPlaceConfig
from ..grasps.plate import plate_rim_grasp, plate_rim_grasp_edge
from ..pick import pick
from ..place import place
from ..session import default_session
from ..util.poses import Pose


# --- Tunables (edit to match your physical layout) ------------------------
#0.025 is the height of the plate rim
PLATE_PICK_POSE_TASK = Pose(translation=[-0.3, -0.1, 0.025])
"""Plate center at PICK location, expressed in task frame."""

#0.025 is the height of the plate rim
PLATE_PLACE_POSE_TASK = Pose(translation=[0, 0, 0.025])
"""Plate center at PLACE location, task frame."""

GRASP_ANGLE_RAD = np.pi
"""Which rim angle to grasp at. π/2 = approach the rim from the
microwave-facing side (y+ side of the plate)."""

ARM = "ur_left"

CONFIG = PickPlaceConfig(
    transit_z=0.2,
    place_use_contact_descent=False,  # no force-seeking; no table contact                                              

    # Conservative defaults everywhere else. Speed things up in a later
    # pass once the motion is known-safe.
)


def plan_pick() -> "Grasp":  # type: ignore[name-defined]
    # NOTE We are using rim_grasp here instead of vertical grasp
    return plate_rim_grasp_edge(PLATE_PICK_POSE_TASK, angle_rad=GRASP_ANGLE_RAD)


def plan_place() -> Pose:
    """TCP pose at release. Uses the same rim-grasp factory at the place
    location so the gripper's orientation stays consistent between pick
    and place — easier on the wrist than re-solving a new orientation."""
    grasp_at_dest = plate_rim_grasp_edge(
        PLATE_PLACE_POSE_TASK, angle_rad=GRASP_ANGLE_RAD
    )
    return grasp_at_dest.grasp_pose


def run(dry: bool) -> None:
    grasp = plan_pick()
    place_pose = plan_place()

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

    if dry:
        print("[dry run] skipping RTDE connection. No motion commanded.")
        return


    with default_session(left=ARM == "ur_left", right=ARM == "ur_right") as session:
        arm = session.arms[ARM]

        # Optional: home before starting. Comment out if the arm is
        # already at a safe configuration.
        # session.move_to_home()

        # Pick.
        print(f"\n→ pick: {grasp.description}")
        pick_result = pick(arm, grasp, CONFIG)
        if not pick_result.success:
            print(f"  ✗ pick FAILED: {pick_result.reason}")
            return
        print("  ✓ pick succeeded.")

        # Place.
        print(f"\n→ place @ {PLATE_PLACE_POSE_TASK.translation}")
        place_result = place(arm, place_pose, CONFIG)
        if not place_result.success:
            print(f"  ✗ place FAILED: {place_result.reason}")
            return
        print("  ✓ place succeeded.")

        print("\nDone — arm retracted to transit altitude.")


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument(
        "--dry",
        action="store_true",
        help="Plan and print the grasp/place poses without connecting to RTDE.",
    )
    args = ap.parse_args()
    run(dry=args.dry)


if __name__ == "__main__":
    main()
