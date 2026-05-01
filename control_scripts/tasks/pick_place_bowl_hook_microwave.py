"""End-to-end pick-and-place of a bowl using the HOOK arm, with the
pick or place pose optionally inside the microwave.

Same hook-on-rim grasp as ``pick_place_bowl_hook.py``. The microwave-
side half routes through ``pick_from_box`` / ``place_into_box`` so the
wrist stays below the cavity ceiling during transit inside.

Toggling sides
--------------
Edit ``PICK_FROM`` / ``PLACE_TO`` at the top of the file to choose
which side is inside the microwave. ``"outside"`` uses the standard
``pick`` / ``place`` flow with ``BOWL_PICK_POSE_TASK`` /
``BOWL_PLACE_POSE_TASK``. ``"microwave"`` overrides the location to
the microwave's interior center (from ``microwave.py``) at the glass-
tray height + 1 cm.

Approach angle
--------------
At ``GRASP_ANGLE_RAD = π`` the hook approaches from the bowl's −X
side, putting the wrist in −X and the forearm exiting back through
the −X microwave door. Don't change this for the microwave side.

Approach tilt
-------------
``APPROACH_TILT_RAD_OUTSIDE = +15°`` keeps the wrist clear of the
table for free-standing bowls. Inside the microwave the +15° tilt
would lift the wrist by ~2.7 cm into the 23 cm ceiling — so
``APPROACH_TILT_RAD_MICROWAVE = 0`` (pure vertical descent).

Running
-------
Standalone:
    python -m control_scripts.tasks.pick_place_bowl_hook_microwave [--dry]
"""

from __future__ import annotations

import argparse
from typing import Literal

import numpy as np

from ..arm import ArmHandle
from ..config import PickPlaceConfig
from ..grasps.bowl import bowl_hook_grasp
from ..microwave import (
    MICROWAVE_CENTER_XY_TASK,
    MICROWAVE_FLOOR_Z,
    entry_xy_for,
)
from ..moves import transit_xy
from ..pick import pick, pick_from_box
from ..place import place, place_into_box
from ..session import default_session
from ..util.poses import Pose


# --- Tunables --------------------------------------------------------------

PICK_FROM: Literal["outside", "microwave"] = "outside"
PLACE_TO: Literal["outside", "microwave"] = "microwave"

# Free-standing bowl poses (used when the corresponding side is "outside").
BOWL_PICK_POSE_TASK = Pose(translation=[0.1, 0.0, 0.01])
BOWL_PLACE_POSE_TASK = Pose(translation=[-0.1, 0.2, 0.01])

# Intermediate waypoint between pick and place. The transit_z is what
# actually sets the carry altitude; the Z below is ignored. Picked at a
# Cartesian location that breaks long pick→place swings into two
# shorter, predictable legs — moveL between far-apart poses can over-
# extend through awkward joint configurations even when the start and
# end are reachable. Move this until the path looks clean on the rig.
BOWL_MIDPOINT_POSE_TASK = Pose(translation=[0.0, 0.2, 0.0])
USE_MIDPOINT = True
"""Whether to carry the held bowl through ``BOWL_MIDPOINT_POSE_TASK``
between pick and place. Off-by-default would replicate the failure mode
the user just saw on the rig — leave True unless you've shortened the
pick↔place geometry."""

# Bowl-frame angle at which to engage the rim. π = approach from −X
# side; forearm exits back through the −X microwave door. Keep at π
# for any microwave-side leg.
GRASP_ANGLE_RAD = float(np.radians(180))
PLACE_ANGLE_RAD = float(np.radians(180))

# Tilt of the descent off pure-vertical (rotation around tool +Y).
# +15° lifts the wrist ~2.7 cm to clear the table for free-standing bowls.
# Inside the microwave that lift would scrape the 23 cm ceiling — so 0.
APPROACH_TILT_RAD_OUTSIDE = float(np.radians(15))
APPROACH_TILT_RAD_MICROWAVE = 0.0

# Constrained altitude inside the microwave cavity. Bowl rim sits at
# task z 8 cm (tray) + 7.2 cm (bowl height) = 15.2 cm. Hook TCP at the
# rim grasp is at 15.2 cm; the wrist sits at the same task z (offset is
# horizontal, see calibration.TCP_OFFSET_HOOK). 18 cm gives ~3 cm
# clearance over the rim during in-cavity transit and ~5 cm clearance
# below the 23 cm ceiling.
MICROWAVE_ENTRY_Z = 0.18

# Bowl center sits 1 cm above the glass tray to avoid scraping it on
# the descent — same convention as the free-standing pose.
MICROWAVE_BOWL_Z_OFFSET = 0.01

ARM = "ur_left"

CONFIG = PickPlaceConfig(
    transit_z=0.3,
    place_use_contact_descent=False,
    transit_speed=0.1,
    transit_accel=0.2,
    approach_speed=0.05,
    approach_accel=0.2,
    retract_speed=0.1,
    retract_accel=0.2,
    release_aperture_mm=None,  # hook has no continuous aperture
    gripper_open_speed_pct=40,
    gripper_close_speed_pct=30,
)


# --- Pose / grasp planning -------------------------------------------------

def _bowl_pose_for(side: str, default: Pose) -> Pose:
    if side == "microwave":
        return Pose(translation=[
            MICROWAVE_CENTER_XY_TASK[0],
            MICROWAVE_CENTER_XY_TASK[1],
            MICROWAVE_FLOOR_Z + MICROWAVE_BOWL_Z_OFFSET,
        ])
    return default


def _tilt_for(side: str) -> float:
    return (
        APPROACH_TILT_RAD_MICROWAVE
        if side == "microwave"
        else APPROACH_TILT_RAD_OUTSIDE
    )


def plan_pick():
    return bowl_hook_grasp(
        _bowl_pose_for(PICK_FROM, BOWL_PICK_POSE_TASK),
        angle_rad=GRASP_ANGLE_RAD,
        approach_tilt_rad=_tilt_for(PICK_FROM),
    )


def plan_place() -> Pose:
    grasp_at_dest = bowl_hook_grasp(
        _bowl_pose_for(PLACE_TO, BOWL_PLACE_POSE_TASK),
        angle_rad=PLACE_ANGLE_RAD,
        approach_tilt_rad=_tilt_for(PLACE_TO),
    )
    return grasp_at_dest.grasp_pose


def plan_midpoint() -> Pose:
    """TCP pose for carrying the held bowl through the midpoint XY at
    transit_z. Orientation matches the upcoming PLACE leg (PLACE_ANGLE_RAD
    + place-side tilt) so the wrist rotation happens during the safer
    pick→midpoint leg, not during the longer midpoint→place transit."""
    grasp_at_midpoint = bowl_hook_grasp(
        BOWL_MIDPOINT_POSE_TASK,
        angle_rad=PLACE_ANGLE_RAD,
        approach_tilt_rad=_tilt_for(PLACE_TO),
    )
    return grasp_at_midpoint.grasp_pose


def _print_plan(grasp, place_pose: Pose) -> None:
    print("=" * 60)
    print(f"  Arm           : {ARM} (hook gripper)")
    print(f"  Pick from     : {PICK_FROM}")
    print(f"  Place to      : {PLACE_TO}")
    print(f"  Grasp pose    : {grasp.grasp_pose.translation} (task)")
    print(f"  Place pose    : {place_pose.translation} (task)")
    print(f"  Grasp angle   : {np.degrees(GRASP_ANGLE_RAD):+.0f}°")
    print(f"  Place angle   : {np.degrees(PLACE_ANGLE_RAD):+.0f}°")
    print(f"  Tilt (pick)   : {np.degrees(_tilt_for(PICK_FROM)):+.1f}°")
    print(f"  Tilt (place)  : {np.degrees(_tilt_for(PLACE_TO)):+.1f}°")
    print(f"  Midpoint      : "
          f"{BOWL_MIDPOINT_POSE_TASK.translation if USE_MIDPOINT else 'disabled'}"
          f"{' (task)' if USE_MIDPOINT else ''}")
    print(f"  Transit Z     : {CONFIG.transit_z} m")
    if PICK_FROM == "microwave" or PLACE_TO == "microwave":
        print(f"  Microwave entry Z : {MICROWAVE_ENTRY_Z} m")
        if PICK_FROM == "microwave":
            xy = entry_xy_for(grasp.grasp_pose.translation[:2])
            print(f"  Entry XY (pick)   : {xy}")
        if PLACE_TO == "microwave":
            xy = entry_xy_for(place_pose.translation[:2])
            print(f"  Entry XY (place)  : {xy}")
    print("=" * 60)


# --- Execution -------------------------------------------------------------

def run_on_arm(
    arm: ArmHandle,
    grasp,
    place_pose: Pose,
    config: PickPlaceConfig = CONFIG,
) -> bool:
    print(f"\n→ pick: {grasp.description}  (from {PICK_FROM})")
    if PICK_FROM == "microwave":
        entry_xy = entry_xy_for(grasp.grasp_pose.translation[:2])
        pick_result = pick_from_box(
            arm, grasp, entry_xy, MICROWAVE_ENTRY_Z, config
        )
    else:
        pick_result = pick(arm, grasp, config)
    if not pick_result.success:
        print(f"  ✗ pick FAILED: {pick_result.reason}")
        return False
    print("  ✓ pick succeeded.")

    if USE_MIDPOINT:
        midpoint_pose = plan_midpoint()
        print(f"\n→ midpoint @ {BOWL_MIDPOINT_POSE_TASK.translation}")
        transit_xy(
            arm,
            midpoint_pose,
            config.transit_z,
            config.transit_speed,
            config.transit_accel,
        )
        print("  ✓ midpoint reached.")

    print(f"\n→ place @ {place_pose.translation}  (to {PLACE_TO})")
    if PLACE_TO == "microwave":
        entry_xy = entry_xy_for(place_pose.translation[:2])
        place_result = place_into_box(
            arm, place_pose, entry_xy, MICROWAVE_ENTRY_Z, config
        )
    else:
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
