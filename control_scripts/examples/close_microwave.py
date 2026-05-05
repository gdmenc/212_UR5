"""Push the microwave door closed with the left arm (no grasp).

Uses the same geometry as open_microwave.py — same hinge position, same
arc angle — but runs the arc in reverse and at a slightly smaller radius
(``PUSH_RADIUS_M``) since closing doesn't need handle engagement; any
door-face contact works.  The arm transits to the fully-open push
position, then runs the reverse arc to door-closed.

Running
-------
    python3.11 -m control_scripts.examples.close_microwave
    python3.11 -m control_scripts.examples.close_microwave --dry
    python3.11 -m control_scripts.examples.close_microwave --mode path

``--mode {sequential,path}`` selects how Phase 2 executes:
    sequential  — N blocking moveJ calls (current default, decel to zero
                  at each waypoint).
    path        — single moveJ(path=[[q,v,a,blend], ...]) call with
                  corner blending controlled by ``arc_blend_radius_m``.

First-run checklist
-------------------
1. ``--dry`` first — confirm the arc reversal looks correct (Y values should
   increase from negative back toward +0.344, and the final position should
   be on the closed-door face at PUSH_RADIUS_M from the hinge).
2. Confirm the door is actually fully open before running — if the door is
   only partially open the arm will approach from the wrong angle.
3. Keep approach_speed low (≤ 0.04 m/s) so the door doesn't slam and
   rebound into the arm.
4. If the door isn't fully closed at the end, increase arc_open_angle_rad
   slightly (e.g. 1.7 rad) so the final push goes a bit further.
"""

from __future__ import annotations

import argparse

import numpy as np

from ..calibration import HOME_Q_RAD_LEFT
from ..config import PickPlaceConfig
from ..session import default_session
from ..tasks.close_microwave import (
    CloseMicrowaveDoorSpec,
    _close_arc_joint_path,
    close_arc_waypoints,
    close_microwave_door,
    open_handle_pose,
)
from ..util.fk_replay import (
    check_blend_radius,
    densify_joint_path,
    fk_replay,
    print_blend_radius_check,
    print_fk_replay_summary,
)
from ..util.poses import Pose
from ..util.rotations import Rotation
from ..util.rtde_convert import pose_to_rtde


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

DOOR_WIDTH_M = 0.36
"""Swinging door panel width (LEFT 36 cm of the 44 cm front face).
Must match ``open_microwave.py:DOOR_WIDTH_M`` and
``control_scripts/microwave.py:MICROWAVE_DOOR_WIDTH_X``."""

# Hinge is directly left of the handle (pure -X offset) at the full door width.
# Must match open_microwave.py exactly (no sqrt(2) divisor — that's a stale
# remnant from a 45° diagonal model and was already removed in open).
_HINGE_DIR = np.array([-1.0, 0.0, 0.0])
HINGE_POSITION_TASK = (
    HANDLE_CLOSED_POSE_TASK.translation + DOOR_WIDTH_M * _HINGE_DIR
)

# Push contact radius — smaller than the handle radius because closing
# doesn't need to hook the handle. Inboard contact shortens the arc and
# keeps the wrist away from the workspace edge. 0.32 m sits 6 cm inside
# the handle on a 38 cm-wide door.
PUSH_RADIUS_M = 0.32

ARM = "ur_left"

DOOR_SPEC = CloseMicrowaveDoorSpec(
    handle_closed_pose_task=HANDLE_CLOSED_POSE_TASK,
    hinge_position_task=HINGE_POSITION_TASK,
    push_radius_m=PUSH_RADIUS_M,
    pull_direction_task=np.array([-1.0, -1.0, 0.0]),
    arc_open_angle_rad=1.6,   # must match the value used to open
    n_arc_steps=14,
    arc_blend_radius_m=0.01,  # 1 cm TCP corner blend for --mode path
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

def _rotvec_deg(pose: Pose) -> np.ndarray:
    """Pose rotation as axis-angle vector in degrees, for dry-run printing."""
    return np.degrees(pose.rotation.as_rotvec())


def run(dry: bool, validate: bool, mode: str) -> None:
    handle_open = open_handle_pose(DOOR_SPEC)
    waypoints = close_arc_waypoints(DOOR_SPEC)

    handle_radius = float(np.linalg.norm(
        (HANDLE_CLOSED_POSE_TASK.translation - HINGE_POSITION_TASK)[:2]
    ))
    push_radius = (
        DOOR_SPEC.push_radius_m if DOOR_SPEC.push_radius_m is not None
        else handle_radius
    )
    arc_length = push_radius * DOOR_SPEC.arc_open_angle_rad

    print("=" * 60)
    print("  Arm               :", ARM)
    print("  Approach pos(task):", np.round(handle_open.translation, 4),
          "  ← fully-open push contact")
    print("              rotvec:", np.round(handle_open.rotation.as_rotvec(), 4),
          " (deg:", np.round(_rotvec_deg(handle_open), 1), ")")
    print("  Closed handle pos :", np.round(HANDLE_CLOSED_POSE_TASK.translation, 4))
    print("              rotvec:", np.round(HANDLE_CLOSED_POSE_TASK.rotation.as_rotvec(), 4),
          " (deg:", np.round(_rotvec_deg(HANDLE_CLOSED_POSE_TASK), 1), ")")
    print("  Hinge (task)      :", np.round(HINGE_POSITION_TASK, 4))
    print("  Handle radius     :", f"{handle_radius:.3f} m")
    print("  Push radius       :", f"{push_radius:.3f} m"
          f"{' (= handle)' if DOOR_SPEC.push_radius_m is None else ''}")
    print("  Arc angle         :", f"{np.degrees(DOOR_SPEC.arc_open_angle_rad):.1f}°")
    print("  Arc length        :", f"{arc_length:.3f} m")
    print("  Arc steps         :", DOOR_SPEC.n_arc_steps)
    print("  Blend radius      :", f"{DOOR_SPEC.arc_blend_radius_m:.3f} m (path mode only)")
    print("  Final pos (task)  :", np.round(waypoints[-1].translation, 4),
          "  ← should match closed-door face at push radius")
    print("              rotvec:", np.round(waypoints[-1].rotation.as_rotvec(), 4),
          " (deg:", np.round(_rotvec_deg(waypoints[-1]), 1), ")")
    print("  Transit Z         :", CONFIG.transit_z, "m (task frame)")
    print("-" * 60)
    print("  Close arc waypoints (task XYZ + rotvec deg):")
    for i, wp in enumerate(waypoints):
        print(f"    [{i+1:>2}] xyz={np.round(wp.translation, 4)}  "
              f"rv°={np.round(_rotvec_deg(wp), 1)}")
    print("=" * 60)

    if dry:
        print("[dry run] skipping RTDE connection. No motion commanded.")
        return

    if validate:
        _validate(mode)
        return

    with default_session(left=True, right=False) as session:
        arm = session.arms[ARM]

        print(
            f"\n→ close microwave door "
            f"(reverse arc, {mode} mode, {DOOR_SPEC.n_arc_steps} waypoints)"
        )
        result = close_microwave_door(arm, DOOR_SPEC, CONFIG, mode=mode)

        if result.success:
            print("  ✓ door closed.")
        else:
            print(f"  ✗ FAILED: {result.reason}")


def _validate(mode: str) -> None:
    """Open an RTDE session and FK-replay the planned close arc through the
    controller's kinematic model. Commands no motion.

    The seed for the close-arc IK chain is obtained by IKing the open-push
    approach pose from HOME_Q_RAD_LEFT — that mimics what the live Phase 1
    converges to before the arc starts. If the diagnosed wrist-branch issue
    in open_microwave.py shows up here too, the chained errors will be
    obvious; rerun once Phase 1's seed has been corrected."""
    arc_poses_task = close_arc_waypoints(DOOR_SPEC)
    print(
        f"\n→ FK-replay validation (mode={mode}, "
        f"n_waypoints={len(arc_poses_task)}, no motion commanded)"
    )

    with default_session(left=True, right=False) as session:
        arm = session.arms[ARM]

        # Resolve a starting joint config: IK the open-push approach pose
        # from HOME so the arc seed is whatever the controller's IK
        # actually converges to at the arc start. Avoids hardcoding a
        # branch-specific seed (which is exactly what trips up open's
        # validate today).
        approach_pose = open_handle_pose(DOOR_SPEC)
        approach_rtde = pose_to_rtde(arm.to_base(approach_pose))
        seed = arm.control.getInverseKinematics(
            approach_rtde, list(HOME_Q_RAD_LEFT), 0.001, 0.001,
        )
        if not seed or len(seed) != 6:
            print(
                "  ERROR: IK failed for the open-push approach pose; "
                "cannot validate. Verify HINGE_POSITION_TASK / PUSH_RADIUS_M "
                "and try again."
            )
            return

        result = fk_replay(
            arm,
            desired_poses_task=arc_poses_task,
            q_seed=list(seed),
        )
        ok = print_fk_replay_summary(result, label="close arc waypoints")

        # Inter-waypoint chord deviation: FK along joint-linear interp.
        # That's the path sequential moveJ traces; for path mode the
        # controller additionally smooths corners by arc_blend_radius_m
        # (cannot be modelled offline — see fk_replay.py docstring).
        joint_path = _close_arc_joint_path(arm, DOOR_SPEC, q_seed=list(seed))
        densify_joint_path(arm, joint_path, samples_per_segment=10)

        if mode == "path":
            print()
            blend_check = check_blend_radius(
                result.predicted_poses_task,
                blend_radius_m=DOOR_SPEC.arc_blend_radius_m,
                safety_factor=0.5,
            )
            blend_ok = print_blend_radius_check(blend_check, label="blend feasibility")
            print(
                f"\n  NOTE  the blend itself (corner smoothing within "
                f"{DOOR_SPEC.arc_blend_radius_m * 1000:.0f} mm of each waypoint) "
                f"cannot be modelled offline."
            )
            print(
                "        Run against URSim and log getActualTCPPose to observe "
                "the blended TCP path."
            )
            ok = ok and blend_ok

        if not ok:
            print("\n  validation FAILED — see warnings above.")


def main() -> None:
    ap = argparse.ArgumentParser(
        description="Push microwave door closed (reverse arc) with the left arm."
    )
    ap.add_argument(
        "--dry",
        action="store_true",
        help="Print planned arc waypoints without connecting to RTDE.",
    )
    ap.add_argument(
        "--validate",
        action="store_true",
        help=(
            "Connect via RTDE and FK-replay the planned close arc against "
            "the controller's kinematic model. Commands no motion. With "
            "--mode path, also runs the blend feasibility check."
        ),
    )
    ap.add_argument(
        "--mode",
        choices=("sequential", "path"),
        default="sequential",
        help=(
            "Phase-2 execution style. sequential: N blocking moveJ calls. "
            "path: single moveJ(path) with corner blending (uses "
            "arc_blend_radius_m)."
        ),
    )
    args = ap.parse_args()
    run(dry=args.dry, validate=args.validate, mode=args.mode)


if __name__ == "__main__":
    main()
