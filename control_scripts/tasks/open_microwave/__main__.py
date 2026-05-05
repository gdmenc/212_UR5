"""Open the microwave door with the left arm's hook gripper.

Uses hand-recorded task-frame waypoints from:
    logs/waypoints/ur_left_20260505_011254.json

Phase 3 uses ARC MODE — a series of IK-guided moveJ waypoints along the
computed door arc between the recorded ``graspclose`` and ``opendoor``
poses. Arc mode is deterministic and needs no force sensing.

Arc geometry
------------
The hinge is chosen as the circle center, with radius ``DOOR_WIDTH_M``,
that connects the recorded ``graspclose`` and ``opendoor`` positions and
places the hinge left/deeper of the handle. Orientation is interpolated
between the recorded endpoint orientations.

Running
-------
    python3.11 -m control_scripts.tasks.open_microwave
    python3.11 -m control_scripts.tasks.open_microwave --dry

First-run checklist
-------------------
1. ``--dry`` first — inspect the pregrasp, grasp, arc, and slide-out poses.
2. Confirm the printed arc lands exactly on the recorded ``opendoor`` pose.
3. Keep ``n_arc_steps`` high enough that the handle contact stays smooth.
"""

from __future__ import annotations

import argparse

import numpy as np

from ...config import PickPlaceConfig
from ...session import default_session
from .open_microwave import (
    MicrowaveDoorSpec,
    _arc_waypoints_to_pose,
    open_microwave_door,
)
from ...util.poses import Pose
from ...util.rotations import Rotation


# ---------------------------------------------------------------------------
#  Waypoints — sourced from logs/waypoints/ur_left_20260505_011254.json
# ---------------------------------------------------------------------------

# Pregrasp before engaging the handle.
# Source: snapshot "pregrasp"
PRE_ENGAGE_POSE_TASK = Pose(
    translation=np.array([-0.07683383775230111, 0.34557897993134873, 0.22629524718131488]),
    rotation=Rotation.from_rotvec(
        [-1.2072070942247553, 1.3184590951899269, -1.1333357024991244]
    ),
)

# Hook seated at the handle while the hook throat is open.
# Source: snapshot "graspopen"
HANDLE_GRASP_OPEN_POSE_TASK = Pose(
    translation=np.array([-0.03679004487279813, 0.3464353723957377, 0.24758870542261513]),
    rotation=Rotation.from_rotvec(
        [-1.1592962255475672, 1.3337851202127953, -1.155115563527708]
    ),
)

# Hook latched on the handle, ready to pull.
# Source: snapshot "graspclose"
HANDLE_GRASP_CLOSED_POSE_TASK = Pose(
    translation=np.array([-0.03617530952655548, 0.3478974379645616, 0.24985614987175653]),
    rotation=Rotation.from_rotvec(
        [-1.1542880082655, 1.334799257776183, -1.15436939064674]
    ),
)

# Door open while still latched.
# Source: snapshot "opendoor"
OPEN_DOOR_POSE_TASK = Pose(
    translation=np.array([-0.4083709932963583, 0.06113811160757704, 0.263433329622522]),
    rotation=Rotation.from_rotvec(
        [0.06827493549060357, -2.131608284023123, 2.033609070101807]
    ),
)

PRE_ENGAGE_JOINTS_RAD = [
    2.1254944801330566,
    -0.6607252520373841,
    1.069571320210592,
    -3.4712687931456507,
    -2.1583827177630823,
    -3.8029139677630823,
]
HANDLE_GRASP_OPEN_JOINTS_RAD = [
    2.1378650665283203,
    -0.6095984739116211,
    1.0695202986346644,
    -3.478814264337057,
    -2.1583245436297815,
    -3.802957598363058,
]
HANDLE_GRASP_CLOSED_JOINTS_RAD = [
    2.1357154846191406,
    -0.6077410739711304,
    1.0694196859942835,
    -3.476312299767965,
    -2.1576154867755335,
    -3.802946154271261,
]
OPEN_DOOR_JOINTS_RAD = [
    1.3890619277954102,
    -1.607720514337057,
    1.2701624075519007,
    -0.8420926493457337,
    -0.6712873617755335,
    -5.0735958258258265,
]
RELEASE_OPEN_DOOR_JOINTS_RAD = [
    1.3733199834823608,
    -1.601081987420553,
    1.2711012999164026,
    -0.8427835267833252,
    -0.6785486380206507,
    -5.07365590730776,
]
SLIDE_OUT_HANDLE_JOINTS_RAD = [
    1.3787357807159424,
    -1.5832683048644007,
    1.2699788252459925,
    -0.9631260198405762,
    -0.6786406675921839,
    -5.073584024106161,
]

# ---------------------------------------------------------------------------
#  Arc geometry  (door width = 44 cm)
# ---------------------------------------------------------------------------

DOOR_WIDTH_M = 0.38
"""Width of the microwave door panel (measured), used as the arc radius."""


def _hinge_from_recorded_arc(start_xy: np.ndarray, end_xy: np.ndarray, radius: float) -> np.ndarray:
    """Choose the circle center that places the hinge left/deeper of the handle."""
    chord = end_xy - start_xy
    chord_len = float(np.linalg.norm(chord))
    if chord_len <= 0.0 or chord_len > 2.0 * radius:
        raise ValueError("recorded open arc is incompatible with the door radius")
    midpoint = 0.5 * (start_xy + end_xy)
    half_height = float(np.sqrt(radius * radius - (0.5 * chord_len) ** 2))
    perp = np.array([-chord[1], chord[0]]) / chord_len
    candidates = [midpoint + half_height * perp, midpoint - half_height * perp]
    hinge_xy = max(candidates, key=lambda p: (p[1], -p[0]))
    return np.array([hinge_xy[0], hinge_xy[1], HANDLE_GRASP_CLOSED_POSE_TASK.translation[2]])


HINGE_POSITION_TASK = _hinge_from_recorded_arc(
    HANDLE_GRASP_CLOSED_POSE_TASK.translation[:2],
    OPEN_DOOR_POSE_TASK.translation[:2],
    DOOR_WIDTH_M,
)

_START_R = HANDLE_GRASP_CLOSED_POSE_TASK.translation[:2] - HINGE_POSITION_TASK[:2]
_END_R = OPEN_DOOR_POSE_TASK.translation[:2] - HINGE_POSITION_TASK[:2]
ARC_OPEN_ANGLE_RAD = abs(float(np.arctan2(
    _START_R[0] * _END_R[1] - _START_R[1] * _END_R[0],
    np.dot(_START_R, _END_R),
)))


# ---------------------------------------------------------------------------
#  Door spec and motion config
# ---------------------------------------------------------------------------

ARM = "ur_left"

DOOR_SPEC = MicrowaveDoorSpec(
    handle_engage_pose_task=HANDLE_GRASP_CLOSED_POSE_TASK,
    pre_engage_joints_rad=PRE_ENGAGE_JOINTS_RAD,  # moveJ to here first — no wrist spin
    pre_engage_pose_task=PRE_ENGAGE_POSE_TASK,    # full Pose used for dry-run display
    handle_engage_joints_rad=HANDLE_GRASP_OPEN_JOINTS_RAD,
    handle_latched_joints_rad=HANDLE_GRASP_CLOSED_JOINTS_RAD,
    open_pose_task=OPEN_DOOR_POSE_TASK,
    open_joints_rad=OPEN_DOOR_JOINTS_RAD,
    release_joints_rad=RELEASE_OPEN_DOOR_JOINTS_RAD,
    slide_out_joints_rad=SLIDE_OUT_HANDLE_JOINTS_RAD,

    # Arc mode — hinge is known.
    hinge_position_task=HINGE_POSITION_TASK,
    arc_open_angle_rad=ARC_OPEN_ANGLE_RAD,
    n_arc_steps=24,            # denser arc helps keep IK on one branch
    arc_blend_radius_m=0.01,   # blended intermediate arc waypoints
    arc_max_joint_step_rad=1.3,
    arc_debug_joints=True,

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
    arc_waypoints = _arc_waypoints_to_pose(
        DOOR_SPEC,
        HANDLE_GRASP_CLOSED_POSE_TASK,
        OPEN_DOOR_POSE_TASK,
    )
    final_arc_pos = arc_waypoints[-1].translation

    radius = float(np.linalg.norm(
        (HANDLE_GRASP_CLOSED_POSE_TASK.translation - HINGE_POSITION_TASK)[:2]
    ))
    arc_length = radius * DOOR_SPEC.arc_open_angle_rad

    print("=" * 60)
    print("  Arm               :", ARM)
    print("  Pre-engage (task) :", np.round(PRE_ENGAGE_POSE_TASK.translation, 4))
    print("  Grasp-open (task) :", np.round(HANDLE_GRASP_OPEN_POSE_TASK.translation, 4))
    print("  Grasp-close(task) :", np.round(HANDLE_GRASP_CLOSED_POSE_TASK.translation, 4))
    print("  Hinge (task)      :", np.round(HINGE_POSITION_TASK, 4))
    print("  Arc radius        :", f"{radius:.3f} m")
    print("  Arc angle         :", f"{np.degrees(DOOR_SPEC.arc_open_angle_rad):.1f}°")
    print("  Arc length        :", f"{arc_length:.3f} m")
    print("  Arc steps         :", DOOR_SPEC.n_arc_steps)
    print("  Arc blend radius  :", f"{DOOR_SPEC.arc_blend_radius_m:.3f} m")
    print("  Max joint step    :", f"{DOOR_SPEC.arc_max_joint_step_rad:.3f} rad")
    print("  Open-door pos     :", np.round(final_arc_pos, 4))
    print("  Slide-out joints  :", np.round(SLIDE_OUT_HANDLE_JOINTS_RAD, 4))
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
