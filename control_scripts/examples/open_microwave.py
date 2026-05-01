"""Open the microwave door with the left arm's hook gripper.

Uses hand-recorded task-frame waypoints from:
    logs/waypoints/ur_left_20260430_214535.json

    snapshot "microwave initial open (pre-grasp) 2"  → pre-engage pose
    snapshot "microwave initial open (grasp) 2"       → engage / handle pose

Phase 3 uses ARC MODE — a series of moveL waypoints along the computed
door arc — because the hinge location is now known from the door width
measurement (44 cm).  Arc mode is deterministic and needs no force sensing.

Arc geometry
------------
The door is 44 cm wide.  The door swings open in the −X and −Y direction
(diagonally toward the operator and to the left).  The hinge sits at a
45° diagonal offset from the handle:

    hinge_offset_direction = normalize([-1, +1, 0])   (left and deeper)
    hinge = handle + 0.44 * hinge_offset_direction
           ≈ [−0.382, 0.655, 0.155] in task frame

This places r_vec (hinge → handle) in the [+1, −1] direction, so the
clockwise arc tangent at the start is [−1, −1] (diagonal pull). The
arc rotation sign is determined automatically in ``_arc_waypoints``
from ``pull_direction_task``.

Running
-------
    python3.11 -m control_scripts.examples.open_microwave
    python3.11 -m control_scripts.examples.open_microwave --dry

First-run checklist
-------------------
1. ``--dry`` first — inspect the pre-engage, engage, and final arc poses.
2. Verify HINGE_SIDE prints the arc curving toward −Y (toward operator).
   If it curves away, flip HINGE_SIDE from −1 to +1.
3. Confirm CONFIG.transit_z clears the microwave housing (task-frame Z;
   handle is at ~0.158 m, so 0.25 m gives ~9 cm clearance).
4. Start with n_arc_steps=4 and a small arc_open_angle_rad (0.4 rad) to
   validate the arc shape before running to full open.
5. Tune arc_open_angle_rad until the door is visually fully open.
"""

from __future__ import annotations

import argparse

import numpy as np

from ..config import PickPlaceConfig
from ..session import default_session
from ..tasks.open_microwave import MicrowaveDoorSpec, _arc_waypoints, open_microwave_door
from ..util.poses import Pose
from ..util.rotations import Rotation


# ---------------------------------------------------------------------------
#  Waypoints — sourced from logs/waypoints/ur_left_20260430_214535.json
# ---------------------------------------------------------------------------

# TCP pose when the hook is seated under the door handle, ready to pull.
# Source: snapshot "microwave initial open (grasp) 2"
HANDLE_ENGAGE_POSE_TASK = Pose(
    translation=np.array([-0.07104107454852841, 0.34429568939693384, 0.15485172974632533]),
    rotation=Rotation.from_rotvec(
        [-1.5486042738909598, 0.04339334238190479, -0.09065433567416514]
    ),
)

# Pre-engage: hook tip clear of the handle before the slide that seats it.
# Uses the FULL recorded waypoint — both translation AND rotation — because
# the hook gripper's TCP offset (R_y π/2) means pre-grasp and engage have
# different orientations. Using only the engage rotation here caused the hook
# to approach at the wrong angle.
# Source: snapshot "microwave initial open (pre-grasp) 2"
PRE_ENGAGE_POSE_TASK = Pose(
    translation=np.array([-0.101657031669903, 0.3461991798550567, 0.16811394534347623]),
    rotation=Rotation.from_rotvec(
        [-1.5775781112086387, 0.018476229935449232, 0.0008169673631912397]
    ),
)

# Joint angles at the pre-grasp snapshot (radians).
# Used for moveJ approach so the arm reaches the pre-engage configuration
# directly in joint space — avoids the wrist over-rotation / singularity
# that occurs when moveL tries to simultaneously move XY and rotate the
# end effector from HOME to the pre-engage orientation.
# Source: snapshot "microwave initial open (pre-grasp) 2" → joints_rad
# PRE_ENGAGE_JOINTS_RAD = [
#     2.0675549507141113,
#     -0.7741321486285706,
#     0.8808053175555628,
#     -0.14715857923541265,
#     1.9965837001800537,
#     -0.750498119984762
# ]
PRE_ENGAGE_JOINTS_RAD = [
    2.0675549507141113,
    -0.7741321486285706,
    0.8808053175555628,
    -0.14715857923541265,
    1.9965837001800537,
    0.03490004341
]

# ---------------------------------------------------------------------------
#  Arc geometry  (door width = 44 cm)
# ---------------------------------------------------------------------------

DOOR_WIDTH_M = 0.38
"""Width of the microwave door panel (measured), used as the arc radius."""

# The door swings open in the −X, −Y direction (diagonally toward the operator and to the left).
# The hinge is directly left of the handle (pure -X offset) at the full door width.
_HINGE_DIR = np.array([-1.0, 0.0, 0.0]) / np.sqrt(2.0)
HINGE_POSITION_TASK = (
    HANDLE_ENGAGE_POSE_TASK.translation + DOOR_WIDTH_M * _HINGE_DIR
)
# ≈ [−0.382, 0.655, 0.155] in task frame


# ---------------------------------------------------------------------------
#  Door spec and motion config
# ---------------------------------------------------------------------------

ARM = "ur_left"

DOOR_SPEC = MicrowaveDoorSpec(
    handle_engage_pose_task=HANDLE_ENGAGE_POSE_TASK,
    pre_engage_joints_rad=PRE_ENGAGE_JOINTS_RAD,  # moveJ to here first — no wrist spin
    pre_engage_pose_task=PRE_ENGAGE_POSE_TASK,    # full Pose used for dry-run display

    # Arc mode — hinge is known.
    hinge_position_task=HINGE_POSITION_TASK,
    arc_open_angle_rad=1.6,   # ≈ 92° — tune until door is visually fully open
    n_arc_steps=15,            # moveJ waypoints along the arc

    # Slightly faster arc traversal for smoother inter-waypoint motion.
    joint_speed=0.8,           # rad/s (default 0.5)
    joint_accel=0.5,           # rad/s² (default 0.3)

    # pull_direction_task drives the arc rotation sign check.
    # −X, −Y = door swings diagonally toward operator and to the left.
    pull_direction_task=np.array([-1.0, -1.0, 0.0]),

    # Force-mode params below are only used if hinge_position_task were None.
    pull_force_n=20.0,
    pull_distance_task=0.25,
    pull_speed_limit=0.05,
    pull_timeout_s=8.0,
)

CONFIG = PickPlaceConfig(
    # transit_z is in TASK frame. Handle is at ~0.158 m; 0.25 m gives
    # ~9 cm clearance above it and the microwave housing.
    transit_z=0.25,
    transit_speed=0.1,
    transit_accel=0.2,
    approach_speed=0.04,    # slow final slide onto handle + arc steps
    approach_accel=0.1,
    retract_speed=0.10,
    retract_accel=0.2,
)


# ---------------------------------------------------------------------------
#  Run
# ---------------------------------------------------------------------------

def run(dry: bool) -> None:
    arc_waypoints = _arc_waypoints(DOOR_SPEC)
    final_arc_pos = arc_waypoints[-1].translation

    radius = float(np.linalg.norm(
        (HANDLE_ENGAGE_POSE_TASK.translation - HINGE_POSITION_TASK)[:2]
    ))
    arc_length = radius * DOOR_SPEC.arc_open_angle_rad

    print("=" * 60)
    print("  Arm               :", ARM)
    print("  Pre-engage (task) :", np.round(PRE_ENGAGE_POSE_TASK.translation, 4))
    print("  Engage pose(task) :", np.round(HANDLE_ENGAGE_POSE_TASK.translation, 4))
    print("  Hinge (task)      :", np.round(HINGE_POSITION_TASK, 4))
    print("  Arc radius        :", f"{radius:.3f} m")
    print("  Arc angle         :", f"{np.degrees(DOOR_SPEC.arc_open_angle_rad):.1f}°")
    print("  Arc length        :", f"{arc_length:.3f} m")
    print("  Arc steps         :", DOOR_SPEC.n_arc_steps)
    print("  Final handle pos  :", np.round(final_arc_pos, 4))
    print("  Transit Z         :", CONFIG.transit_z, "m (task frame)")
    print("-" * 60)
    print("  Arc waypoints (task XY):")
    for i, wp in enumerate(arc_waypoints):
        print(f"    [{i+1}] {np.round(wp.translation[:2], 4)}")
    print("=" * 60)

    if dry:
        print("[dry run] skipping RTDE connection. No motion commanded.")
        return

    with default_session(left=True, right=False) as session:
        arm = session.arms[ARM]

        print(f"\n→ open microwave door (arc mode, {DOOR_SPEC.n_arc_steps} waypoints)")
        result = open_microwave_door(arm, DOOR_SPEC, CONFIG)

        if result.success:
            print(f"  ✓ door opened — arc length {result.door_opened_distance:.3f} m")
        else:
            print(f"  ✗ FAILED: {result.reason}")
            print(f"    TCP moved {result.door_opened_distance:.3f} m before stopping")


def main() -> None:
    ap = argparse.ArgumentParser(
        description="Open microwave door (arc mode) with the left arm hook gripper."
    )
    ap.add_argument(
        "--dry",
        action="store_true",
        help="Print planned poses and arc waypoints without connecting to RTDE.",
    )
    args = ap.parse_args()
    run(dry=args.dry)


if __name__ == "__main__":
    main()
