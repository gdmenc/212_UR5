"""Push the microwave door closed with the left arm (no grasp).

Uses the hand-recorded task-frame waypoints from the provided
``saved_at=2026-05-05T01:14:19`` close snapshot.

The close motion is intentionally simple: move to ``preclosedoor``, then
travel directly to ``closedoor``.

Running
-------
    python3.11 -m control_scripts.tasks.close_microwave
    python3.11 -m control_scripts.tasks.close_microwave --dry

First-run checklist
-------------------
1. ``--dry`` first — confirm the two joint waypoints match the latest
   ``preclosedoor`` and ``closedoor`` snapshots.
2. Confirm the door is open enough for the preclose pose before running.
3. Keep joint speed modest so the final push does not slam the door.
"""

from __future__ import annotations

import argparse

import numpy as np

from ...config import PickPlaceConfig
from ...session import default_session
from .close_microwave import (
    CloseMicrowaveDoorSpec,
    close_microwave_door,
)
from ...util.poses import Pose
from ...util.rotations import Rotation


# ---------------------------------------------------------------------------
#  Waypoints — sourced from the provided 2026-05-05T01:14:19 close snapshot
# ---------------------------------------------------------------------------

# Source: snapshot "preclosedoor"
PRECLOSE_DOOR_POSE_TASK = Pose(
    translation=np.array([-0.6093772150461308, 0.27488770991396877, 0.41131672797896235]),
    rotation=Rotation.from_rotvec(
        [-1.0250496228801607, 1.3970250241161535, 0.8022438222398867]
    ),
)

# Source: snapshot "closedoor"
CLOSED_DOOR_POSE_TASK = Pose(
    translation=np.array([-0.27459578866963485, 0.25997937217088424, 0.41819012355144713]),
    rotation=Rotation.from_rotvec(
        [-0.5442464470297064, 1.4736026545243532, 0.3871995361438228]
    ),
)

PRECLOSE_DOOR_JOINTS_RAD = [
    1.5467852354049683,
    -1.1462249618819733,
    0.7166374365435999,
    -1.3493216794780274,
    -2.396539036427633,
    -2.1319807211505335,
]
CLOSED_DOOR_JOINTS_RAD = [
    1.897124171257019,
    -1.0298115772059937,
    1.0604260603534144,
    -1.3474183839610596,
    -2.403875176106588,
    -2.132052723561422,
]

ARM = "ur_left"

DOOR_SPEC = CloseMicrowaveDoorSpec(
    handle_closed_pose_task=CLOSED_DOOR_POSE_TASK,
    hinge_position_task=np.zeros(3),
    pull_direction_task=np.array([-1.0, -1.0, 0.0]),
    preclose_pose_task=PRECLOSE_DOOR_POSE_TASK,
    preclose_joints_rad=PRECLOSE_DOOR_JOINTS_RAD,
    closed_pose_task=CLOSED_DOOR_POSE_TASK,
    closed_joints_rad=CLOSED_DOOR_JOINTS_RAD,
    joint_speed=0.8,          # rad/s — matches open_microwave
    joint_accel=0.5,          # rad/s²
)

CONFIG = PickPlaceConfig(
    transit_z=0.25,         # same as open_microwave
    transit_speed=0.1,
    transit_accel=0.2,
    approach_speed=0.04,    # slow push so door doesn't slam
    approach_accel=0.1,
    retract_speed=0.10,
    retract_accel=0.2,
)


# ---------------------------------------------------------------------------
#  Run
# ---------------------------------------------------------------------------

def run(dry: bool) -> None:
    print("=" * 60)
    print("  Arm               :", ARM)
    print("  Preclose (task)   :", np.round(PRECLOSE_DOOR_POSE_TASK.translation, 4))
    print("  Closed (task)     :", np.round(CLOSED_DOOR_POSE_TASK.translation, 4))
    print("  Joint speed       :", DOOR_SPEC.joint_speed, "rad/s")
    print("-" * 60)
    print("  Close waypoints:")
    print("    [1] preclosedoor", np.round(PRECLOSE_DOOR_JOINTS_RAD, 4))
    print("    [2] closedoor   ", np.round(CLOSED_DOOR_JOINTS_RAD, 4))
    print("=" * 60)

    if dry:
        print("[dry run] skipping RTDE connection. No motion commanded.")
        return

    with default_session(left=True, right=False) as session:
        arm = session.arms[ARM]

        print("\n→ close microwave door (preclosedoor → closedoor)")
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
