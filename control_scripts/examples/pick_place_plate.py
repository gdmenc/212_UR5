"""Round-trip pick-and-place of a plate using the right arm.

The first script to run at the lab once connections are known-good.
Demonstrates the full stack by moving the same plate through two pick/place
cycles:

    1. Pick the plate from ``PLATE_PICK_POSE_TASK``.
    2. Optionally carry it through ``PLATE_MIDPOINT_POSE_TASK``.
    3. Place it at ``PLATE_PLACE_POSE_TASK``.
    4. Optionally carry back through the midpoint.
    5. Pick the plate again from ``PLATE_PLACE_POSE_TASK``.
    6. Optionally carry through the midpoint again.
    7. Place it back at ``PLATE_PICK_POSE_TASK``.

Hardware assumptions
--------------------
  - ``ur_right`` at ``192.168.1.102`` with a Robotiq 2F-85 attached and
    pre-calibrated TCP offset (set in control_scripts/calibration.py).
  - An 8-inch plate sitting at task-frame ``PLATE_PICK_POSE_TASK``.
  - Task-frame ``PLATE_PLACE_POSE_TASK`` is the first destination and the
    second pick location.
  - ``USE_MIDPOINT`` controls whether the held plate travels through the
    intermediate task-frame waypoint before each long transit. The midpoint's
    Z value is not the travel height; ``PickPlaceConfig.transit_z`` sets the
    safe carry altitude.

Running
-------
    python3.11 -m control_scripts.examples.pick_place_plate

First-run checklist
-------------------
1. Verify ``PickPlaceConfig.transit_z`` is above every object and fixture in
   the workspace for both outbound and return motions.
2. Verify ``plate_rim_grasp_edge``'s ``PLATE_PREGRASP_OFFSET`` clears the
   plate and nearby fixtures at both pick/place locations.
3. Use ``--dry`` to print what the motion WOULD do without connecting
   to the robot — useful for a first sanity check at home.
"""

from __future__ import annotations

import argparse

import numpy as np

from ..config import PickPlaceConfig
from ..grasps.plate import plate_rim_grasp, plate_rim_grasp_edge
from ..moves import transit_xy
from ..pick import pick
from ..place import place
from ..session import default_session
from ..util.poses import Pose


# --- Tunables (edit to match your physical layout) ------------------------
# Pick, from pick_plate_1 with GRASP_ANGLE_RAD = 0
#0.025 is the height of the plate rim that is reasonable for pickup
PLATE_PICK_POSE_TASK = Pose(translation=[0.295521, -0.1, 0.01])
"""Plate center at PICK location, expressed in task frame."""

GRASP_ANGLE_RAD = 0
"""Which rim angle to grasp at. π/2 = approach the rim from the
microwave-facing side (y+ side of the plate)."""

# Place, from microwave_plate_2 with PLACE_ANGLE_RAD = -0.883826
#0.025 is the height of the plate rim that is reasonable for setting down
PLATE_PLACE_POSE_TASK = Pose(translation=[-0.215458, 0.568696, 0.065])
"""Plate center at PLACE location, task frame."""

PLACE_ANGLE_RAD = -0.88 # -50.64 deg roughly
"""Which rim angle to place at. π/2 = approach the rim from the
microwave-facing side (y+ side of the plate)."""

# Midpoint. NOTE that actual height does not matter here and comes from CONFIG.transit_z
PLATE_MIDPOINT_POSE_TASK = Pose(translation=[-0.1, 0.2, 0.0])
"""Intermediate plate-center waypoint at the task origin."""
USE_MIDPOINT = True
"""Whether to carry the plate through ``PLATE_MIDPOINT_POSE_TASK`` before placing."""

ARM = "ur_right"

CONFIG = PickPlaceConfig(
    transit_z=0.1325,
    place_use_contact_descent=False,  # no force-seeking; no table contact
    transit_speed=0.15,
    transit_accel=0.3,
    preplace_offset=0.05,

    # Staged release / pre-grasp aperture. ~25 mm clears the plate rim
    # but keeps the fingers narrow enough to stay inside the microwave
    # cavity. release_clearance lifts 2 cm between the partial open and
    # the full open so the fingers don't drag the released plate (and
    # symmetrically, lifts 2 cm after grasp before the larger retract).
    release_aperture_mm=30,
    # release_clearance=0.02,

    # Gripper speeds. Slower than the 100% defaults — controlled release
    # inside an enclosed cavity; gentle close on the plate rim.
    gripper_open_speed_pct=40,
    gripper_close_speed_pct=30,

    # Conservative defaults everywhere else. Speed things up in a later
    # pass once the motion is known-safe.
)


def plan_pick() -> "Grasp":  # type: ignore[name-defined]
    # NOTE We are using rim_grasp here instead of vertical grasp
    return plate_rim_grasp_edge(PLATE_PICK_POSE_TASK, angle_rad=GRASP_ANGLE_RAD)

def plan_pick_again() -> "Grasp":  # type: ignore[name-defined]
    return plate_rim_grasp_edge(PLATE_PLACE_POSE_TASK, angle_rad=PLACE_ANGLE_RAD)

def plan_place() -> Pose:
    """TCP pose at release. Uses the same rim-grasp factory at the place
    location so the gripper's orientation stays consistent between pick
    and place — easier on the wrist than re-solving a new orientation."""
    grasp_at_dest = plate_rim_grasp_edge(
        PLATE_PLACE_POSE_TASK, angle_rad=PLACE_ANGLE_RAD
    )
    return grasp_at_dest.grasp_pose

def plan_place_again() -> Pose:
    grasp_at_dest = plate_rim_grasp_edge(
        PLATE_PICK_POSE_TASK, angle_rad=GRASP_ANGLE_RAD
    )
    return grasp_at_dest.grasp_pose

def plan_midpoint() -> Pose:
    """TCP pose for carrying the plate through the task origin."""
    grasp_at_midpoint = plate_rim_grasp_edge(
        PLATE_MIDPOINT_POSE_TASK, angle_rad=PLACE_ANGLE_RAD
    )
    return grasp_at_midpoint.grasp_pose


def run(dry: bool) -> None:
    grasp = plan_pick()
    midpoint_pose = plan_midpoint() if USE_MIDPOINT else None
    place_pose = plan_place()

    grasp_again = plan_pick_again()
    place_pose_again = plan_place_again()

    print("=" * 60)
    print("  Pick location  :", PLATE_PICK_POSE_TASK.translation, "(task frame)")
    print("  Midpoint       :",
          PLATE_MIDPOINT_POSE_TASK.translation if USE_MIDPOINT else "disabled",
          "(task frame)" if USE_MIDPOINT else "")
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

        if USE_MIDPOINT:
            # Carry the held plate through the task origin before entering the
            # microwave-side place approach.
            print(f"\n→ midpoint @ {PLATE_MIDPOINT_POSE_TASK.translation}")
            transit_xy(
                arm,
                midpoint_pose,
                CONFIG.transit_z,
                CONFIG.transit_speed,
                CONFIG.transit_accel,
            )
            print("  ✓ midpoint reached.")

        # Place.
        print(f"\n→ place @ {PLATE_PLACE_POSE_TASK.translation}")
        place_result = place(arm, place_pose, CONFIG)
        if not place_result.success:
            print(f"  ✗ place FAILED: {place_result.reason}")
            return
        print("  ✓ place succeeded.")

        print("\nDone — arm retracted to transit altitude.")

        if USE_MIDPOINT:
            print(f"\n→ midpoint @ {PLATE_MIDPOINT_POSE_TASK.translation}")
            transit_xy(
                arm,
                midpoint_pose,
                CONFIG.transit_z,
                CONFIG.transit_speed,
                CONFIG.transit_accel,
            )
            print("  ✓ midpoint reached.")
        
        print(f"\n→ pick again: {grasp_again.description}")
        pick_result = pick(arm, grasp_again, CONFIG)
        if not pick_result.success:
            print(f"  ✗ pick again FAILED: {pick_result.reason}")
            return
        print("  ✓ pick again succeeded.")

        if USE_MIDPOINT:
            print(f"\n→ midpoint @ {PLATE_MIDPOINT_POSE_TASK.translation}")
            transit_xy(
                arm,
                midpoint_pose,
                CONFIG.transit_z,
                CONFIG.transit_speed,
                CONFIG.transit_accel,
            )
            print("  ✓ midpoint reached.")

        # Place.
        print(f"\n→ place again @ {PLATE_PICK_POSE_TASK.translation}")
        place_result = place(arm, place_pose_again, CONFIG)
        if not place_result.success:
            print(f"  ✗ place again FAILED: {place_result.reason}")
            return
        print("  ✓ place again succeeded.")



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
