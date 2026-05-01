"""Push the microwave door closed with the left arm (no grasp).

Uses the same geometry as open_microwave.py — same hinge position, same
arc parameters — but runs the arc in reverse.  The arm transits to the
fully-open handle position, then pushes through 8 moveL waypoints back
to door-closed.

Running
-------
    python3.11 -m control_scripts.examples.close_microwave
    python3.11 -m control_scripts.examples.close_microwave --dry

First-run checklist
-------------------
1. ``--dry`` first — confirm the arc reversal looks correct (Y values should
   increase from negative back toward +0.344, and final handle should match
   the recorded grasp waypoint position).
2. Confirm the door is actually fully open before running — if the door is
   only partially open the arm will approach from the wrong angle.
3. Keep approach_speed low (≤ 0.04 m/s) so the door doesn't slam and
   rebound into the arm.
4. If the door isn't fully closed at the end, increase arc_open_angle_rad
   slightly (e.g. 1.25 rad) so the final push goes a bit further.
"""

from __future__ import annotations

import argparse

import numpy as np

from ..config import PickPlaceConfig
from ..session import default_session
from ..tasks.close_microwave import (
    CloseMicrowaveDoorSpec,
    close_arc_waypoints,
    close_microwave_door,
    open_handle_pose,
)
from ..util.poses import Pose
from ..util.rotations import Rotation


# ---------------------------------------------------------------------------
#  Geometry — same constants as open_microwave.py
# ---------------------------------------------------------------------------

# TCP pose at the handle when the door is CLOSED.
# Source: logs/waypoints/ur_left_20260430_214535.json
#         snapshot "microwave initial open (grasp) 2"
HANDLE_CLOSED_POSE_TASK = Pose(
    translation=np.array([-0.07104107454852841, 0.34429568939693384, 0.15485172974632533]),
    rotation=Rotation.from_rotvec(
        [-1.5486042738909598, 0.04339334238190479, -0.09065433567416514]
    ),
)

DOOR_WIDTH_M = 0.44

# Hinge is offset from the handle in the [−1, +1] direction (left and
# deeper into the microwave) so that the opening arc tangent is [−1, −1].
_HINGE_DIR = np.array([-1.0, 1.0, 0.0]) / np.sqrt(2.0)
HINGE_POSITION_TASK = (
    HANDLE_CLOSED_POSE_TASK.translation + DOOR_WIDTH_M * _HINGE_DIR
)
# ≈ [−0.382, 0.655, 0.155] in task frame

ARM = "ur_left"

DOOR_SPEC = CloseMicrowaveDoorSpec(
    handle_closed_pose_task=HANDLE_CLOSED_POSE_TASK,
    hinge_position_task=HINGE_POSITION_TASK,
    pull_direction_task=np.array([-1.0, -1.0, 0.0]),
    arc_open_angle_rad=1.2,   # must match the value used to open
    n_arc_steps=8,
)

CONFIG = PickPlaceConfig(
    transit_z=0.25,         # same as open_microwave
    transit_speed=0.15,
    transit_accel=0.3,
    approach_speed=0.04,    # slow push so door doesn't slam
    approach_accel=0.1,
    retract_speed=0.10,
    retract_accel=0.2,
)


# ---------------------------------------------------------------------------
#  Run
# ---------------------------------------------------------------------------

def run(dry: bool) -> None:
    handle_open = open_handle_pose(DOOR_SPEC)
    waypoints = close_arc_waypoints(DOOR_SPEC)

    radius = float(np.linalg.norm(
        (HANDLE_CLOSED_POSE_TASK.translation - HINGE_POSITION_TASK)[:2]
    ))
    arc_length = radius * DOOR_SPEC.arc_open_angle_rad

    print("=" * 60)
    print("  Arm               :", ARM)
    print("  Approach pos(task):", np.round(handle_open.translation, 4),
          "  ← fully-open handle")
    print("  Hinge (task)      :", np.round(HINGE_POSITION_TASK, 4))
    print("  Arc radius        :", f"{radius:.3f} m")
    print("  Arc angle         :", f"{np.degrees(DOOR_SPEC.arc_open_angle_rad):.1f}°")
    print("  Arc length        :", f"{arc_length:.3f} m")
    print("  Arc steps         :", DOOR_SPEC.n_arc_steps)
    print("  Final handle pos  :", np.round(waypoints[-1].translation, 4),
          "  ← should match closed handle")
    print("  Transit Z         :", CONFIG.transit_z, "m (task frame)")
    print("-" * 60)
    print("  Close arc waypoints (task XY):")
    for i, wp in enumerate(waypoints):
        print(f"    [{i+1}] {np.round(wp.translation[:2], 4)}")
    print("=" * 60)

    if dry:
        print("[dry run] skipping RTDE connection. No motion commanded.")
        return

    with default_session(left=True, right=False) as session:
        arm = session.arms[ARM]

        print(f"\n→ close microwave door (reverse arc, {DOOR_SPEC.n_arc_steps} waypoints)")
        result = close_microwave_door(arm, DOOR_SPEC, CONFIG)

        if result.success:
            print("  ✓ door closed.")
        else:
            print(f"  ✗ FAILED: {result.reason}")


def main() -> None:
    ap = argparse.ArgumentParser(
        description="Push microwave door closed (reverse arc) with the left arm."
    )
    ap.add_argument(
        "--dry",
        action="store_true",
        help="Print planned arc waypoints without connecting to RTDE.",
    )
    args = ap.parse_args()
    run(dry=args.dry)


if __name__ == "__main__":
    main()
