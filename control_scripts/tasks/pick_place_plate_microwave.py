"""End-to-end pick-and-place of a plate (2F-85 arm), with the pick or
place pose optionally inside the microwave.

Same plate rim grasp as ``pick_place_plate.py``. The microwave-side
half routes through ``pick_from_box`` / ``place_into_box`` so the wrist
stays below the cavity ceiling during transit inside.

============================================================
WARNING — vertical 2F-85 plate place may NOT fit the cavity
============================================================
With ``TCP_OFFSET_ROBOTIQ_2F85`` ≈ 0.184 m, the wrist sits ~18.4 cm
above the TCP at a top-down rim grasp. With the plate sitting on the
glass tray:

    TCP at plate rim   ≈ task z 0.10 m   (8 cm tray + 2 cm plate rim)
    Wrist at descent   ≈ task z 0.284 m
    Microwave ceiling  = task z 0.230 m  (5.4 cm overshoot)

That means a vertical descent of the plate INTO the microwave will
hit the cavity ceiling with the wrist before the TCP reaches the tray.
``plate_rim_grasp_edge`` adds a small wrist tilt (rim slope, ~10°)
which buys back ~3 cm of vertical, but it's not enough to clear.

Options if you actually need plate-into-microwave on the rig:
  - Re-measure ``MICROWAVE_CEILING_Z`` — if the real cavity is taller
    than 23 cm (e.g. 28-30 cm), this script works as-is.
  - Mount the 2F-85 differently (shorter TCP offset).
  - Switch to a side-pinch grasp (would be a new ``grasps/plate.py``
    factory). Out of scope here.

This script is wired up for the geometrically-valid case (plate stays
outside the microwave, or the cavity is taller than the placeholder
spec). It will print a loud warning at plan time if the wrist will
exceed the cavity ceiling.

Toggling sides
--------------
``PICK_FROM`` and ``PLACE_TO`` flags at the top — same convention as
``pick_place_bowl_hook_microwave.py``. ``"outside"`` uses the standard
``pick`` / ``place`` flow; ``"microwave"`` overrides location to the
cavity center at tray-z + plate-rim-height and routes through the box
helpers.

Approach angle
--------------
The 2F-85 closes along tool +Y. ``GRASP_ANGLE_RAD = -π/2`` puts the
fingers' close axis along task +X / -X (radial across the rim). Pick
the angle that orients the wrist along -X so the forearm exits back
through the -X microwave door. (``plate_rim_grasp_edge`` rotates the
TCP yaw by ``angle_rad + π``, see ``grasps/plate.py``.)

Running
-------
Standalone:
    python -m control_scripts.tasks.pick_place_plate_microwave [--dry]
"""

from __future__ import annotations

import argparse
from typing import Literal

import numpy as np

from ..arm import ArmHandle
from ..calibration import TCP_OFFSET_ROBOTIQ_2F85
from ..config import PickPlaceConfig
from ..grasps.plate import PLATE_RIM_HEIGHT, plate_rim_grasp_edge
from ..microwave import (
    MICROWAVE_CEILING_Z,
    MICROWAVE_CENTER_XY_TASK,
    MICROWAVE_FLOOR_Z,
    entry_xy_for,
)
from ..pick import pick, pick_from_box
from ..place import place, place_into_box
from ..session import default_session
from ..util.poses import Pose


# --- Tunables --------------------------------------------------------------

PICK_FROM: Literal["outside", "microwave"] = "outside"
PLACE_TO: Literal["outside", "microwave"] = "microwave"

# Free-standing plate poses (used when the corresponding side is "outside").
# 0.025 m is the height of the plate rim that is reasonable for pickup /
# setdown — same convention as ``pick_place_plate.py``.
PLATE_PICK_POSE_TASK = Pose(translation=[0.295521, -0.1, 0.01])
PLATE_PLACE_POSE_TASK = Pose(translation=[-0.25, 0.5, 0.09])

# Plate-frame angle at which to engage the rim. Choose so the wrist
# exits back through the -X microwave door for any microwave-side leg.
# 2F-85 closes along tool +Y, so the rim-radial axis is tool +X. With
# ``plate_rim_grasp_edge`` the TCP yaw = angle_rad + π.
GRASP_ANGLE_RAD = 0.0
PLACE_ANGLE_RAD = -0.88  # ~-50.6° — same as the non-microwave plate task

# Constrained altitude inside the cavity. Plate on tray sits at task z
# ~10 cm (8 + 2 cm rim). Top of plate rim ~ 11 cm. 14 cm gives ~3 cm
# clearance over the rim during in-cavity transit. Wrist will be at
# 14 + 18.4 = 32.4 cm — ABOVE the 23 cm ceiling. See top-of-file
# warning; this constant is correct for an enlarged real cavity but
# does not magically fit a 23 cm one.
MICROWAVE_ENTRY_Z = 0.14

# Plate center at task z when sitting on tray = tray + plate rim height.
MICROWAVE_PLATE_Z = MICROWAVE_FLOOR_Z + PLATE_RIM_HEIGHT  # 0.10 m

ARM = "ur_right"

CONFIG = PickPlaceConfig(
    transit_z=0.3,
    place_use_contact_descent=False,
    transit_speed=0.15,
    transit_accel=0.3,

    # 25 mm release aperture — clears the plate rim but keeps fingers
    # narrow enough to stay inside the cavity. Same value as the
    # non-microwave plate task.
    release_aperture_mm=25,

    gripper_open_speed_pct=40,
    gripper_close_speed_pct=30,
)


# --- Pose / grasp planning -------------------------------------------------

def _plate_pose_for(side: str, default: Pose) -> Pose:
    if side == "microwave":
        return Pose(translation=[
            MICROWAVE_CENTER_XY_TASK[0],
            MICROWAVE_CENTER_XY_TASK[1],
            MICROWAVE_PLATE_Z,
        ])
    return default


def plan_pick():
    return plate_rim_grasp_edge(
        _plate_pose_for(PICK_FROM, PLATE_PICK_POSE_TASK),
        angle_rad=GRASP_ANGLE_RAD,
    )


def plan_place() -> Pose:
    grasp_at_dest = plate_rim_grasp_edge(
        _plate_pose_for(PLACE_TO, PLATE_PLACE_POSE_TASK),
        angle_rad=PLACE_ANGLE_RAD,
    )
    return grasp_at_dest.grasp_pose


def _check_wrist_clearance() -> None:
    """Loudly warn at plan time if the 2F-85 wrist will exceed the cavity
    ceiling on a microwave-side leg. Conservative: assumes pure-vertical
    descent (ignores the small rim-slope tilt)."""
    if PICK_FROM != "microwave" and PLACE_TO != "microwave":
        return
    tcp_offset_z = TCP_OFFSET_ROBOTIQ_2F85[2]
    wrist_at_descent_z = MICROWAVE_PLATE_Z + tcp_offset_z
    wrist_at_entry_z = MICROWAVE_ENTRY_Z + tcp_offset_z
    margin_descent = MICROWAVE_CEILING_Z - wrist_at_descent_z
    margin_entry = MICROWAVE_CEILING_Z - wrist_at_entry_z
    if margin_descent < 0 or margin_entry < 0:
        print("!" * 60)
        print(" GEOMETRY WARNING — 2F-85 wrist exceeds microwave ceiling")
        print(f"  ceiling z          = {MICROWAVE_CEILING_Z:.3f} m")
        print(f"  wrist z @ entry_z  = {wrist_at_entry_z:.3f} m  "
              f"(margin {margin_entry*100:+.1f} cm)")
        print(f"  wrist z @ plate    = {wrist_at_descent_z:.3f} m  "
              f"(margin {margin_descent*100:+.1f} cm)")
        print(" Re-measure ceiling, switch to a side grasp, or skip "
              "microwave leg.")
        print("!" * 60)


def _print_plan(grasp, place_pose: Pose) -> None:
    print("=" * 60)
    print(f"  Arm           : {ARM} (Robotiq 2F-85)")
    print(f"  Pick from     : {PICK_FROM}")
    print(f"  Place to      : {PLACE_TO}")
    print(f"  Grasp pose    : {grasp.grasp_pose.translation} (task)")
    print(f"  Place pose    : {place_pose.translation} (task)")
    print(f"  Grasp angle   : {np.degrees(GRASP_ANGLE_RAD):+.0f}°")
    print(f"  Place angle   : {np.degrees(PLACE_ANGLE_RAD):+.0f}°")
    print(f"  Transit Z     : {CONFIG.transit_z} m")
    print(f"  Release aper. : {CONFIG.release_aperture_mm} mm")
    if PICK_FROM == "microwave" or PLACE_TO == "microwave":
        print(f"  Microwave entry Z : {MICROWAVE_ENTRY_Z} m")
        if PICK_FROM == "microwave":
            xy = entry_xy_for(grasp.grasp_pose.translation[:2])
            print(f"  Entry XY (pick)   : {xy}")
        if PLACE_TO == "microwave":
            xy = entry_xy_for(place_pose.translation[:2])
            print(f"  Entry XY (place)  : {xy}")
    print("=" * 60)
    _check_wrist_clearance()


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
