"""Interactive sandbox / shakedown script for real-robot bring-up.

How to use
----------
1. Edit the config block below to pick the arm and enable individual tests.
2. Start with ``--dry`` to see the planned actions printed without any motion:
       python -m control_scripts.examples.sandbox --dry
3. When the plan looks right and the workspace is clear, run for real:
       python -m control_scripts.examples.sandbox

Suggested order for first bring-up
----------------------------------
Leave the more dangerous tests (``RUN_MOVEJ``, ``RUN_MOVEL_DELTA``) off
for the first few runs. Start with read-only tests:

    1. RUN_CONNECTION          ← does RTDE reach this arm?
    2. RUN_READ_STATE          ← does the arm report sensible q and TCP?
    3. RUN_FRAME_CONVERSIONS   ← do our base↔task transforms round-trip?
    4. RUN_REACHABILITY_PROBE  ← does IK agree on reachable poses?
    5. RUN_GRIPPER             ← does open/close work?
    6. RUN_MOVEJ               ← motion test #1 (joint-space)
    7. RUN_MOVEL_DELTA         ← motion test #2 (Cartesian)

Safety
------
  - Clear the workspace before enabling any motion test.
  - Keep a finger on the teach-pendant E-stop the first time you run each
    motion test.
  - Speeds here are deliberately conservative. Raise only after the
    motion visibly matches the plan.
"""

from __future__ import annotations

import argparse
import time
from typing import Any, List, Optional

import numpy as np

from ..config import PickPlaceConfig
from ..util.rotations import Rotation
from ..grasps.plate import plate_rim_grasp
from ..reachability import (
    best_feasible_grasp,
    diagnose_reachability,
    is_task_pose_reachable,
)
from ..session import default_session
from ..util.poses import Pose
from ..util.rtde_convert import pose_to_rtde, rtde_to_pose


# =====================================================================
#  CONFIGURATION — edit these
# =====================================================================

# Which arm. Options: "ur_left", "ur_right".
ARM = "ur_right"

# --- Which tests to run. Start with False for motion-producing tests. ---
RUN_CONNECTION          = True
RUN_READ_STATE          = True
RUN_FRAME_CONVERSIONS   = True
RUN_REACHABILITY_PROBE  = True
RUN_GRIPPER             = True
RUN_MOVEJ               = False
RUN_MOVEL_DELTA         = True

# --- moveJ target (joint angles in DEGREES). Keep inside joint limits. ---
# Set to None to skip the actual moveJ and only print what would happen.
MOVEJ_TARGET_DEG: Optional[List[float]] = None  # e.g. [0, -90, 90, -90, -90, 0]
MOVEJ_SPEED_RAD_S  = 0.3
MOVEJ_ACCEL_RAD_S2 = 0.5

# --- moveL delta in TASK frame from the current TCP pose. ---
# Only translation for the first test; rotation left zero so it's predictable.
MOVEL_DELTA_TRANSLATION_TASK = [0.0, 0.0, 0.05]   # 5 cm UP in task frame
MOVEL_DELTA_ROTATION_DEG     = [0, 0, 0]          # no rotation change
MOVEL_SPEED_M_S  = 0.05       # slow — 5 cm/s
MOVEL_ACCEL_M_S2 = 0.10

# --- Gripper cycle parameters. ---
GRIPPER_PAUSE_BETWEEN_ACTIONS = 0.5
GRIPPER_OPEN_SPEED_PCT   = 100
GRIPPER_CLOSE_SPEED_PCT  = 30
GRIPPER_CLOSE_FORCE_PCT  = 20   # gentle — just closing on air here

# --- Reachability probe — a few task-frame poses to query. ---
# IMPORTANT: a Pose with no ``rotation=`` gets identity-in-task-frame,
# which in base frame puts the TCP at an orientation (tool Z tilted
# outward-downward) that IK rarely solves. Use a top-down tool
# orientation here — that's how pick/place actually runs.
_TOOL_DOWN = Rotation.from_rotvec([np.pi, 0.0, 0.0])

PROBE_POSES_TASK = [
    Pose(translation=[0.00, 0.00, 0.10], rotation=_TOOL_DOWN),
    Pose(translation=[-0.10, 0.20, 0.00], rotation=_TOOL_DOWN),
    Pose(translation=[0.10, 0.20, 0.00], rotation=_TOOL_DOWN),
    Pose(translation=[0.00, 0.40, 0.05], rotation=_TOOL_DOWN),
]

# =====================================================================
#  End of user config
# =====================================================================


def _banner(title: str) -> None:
    print()
    print("=" * 70)
    print(f" {title}")
    print("=" * 70)


def _step(msg: str) -> None:
    print(f"  → {msg}")


def _result(msg: str, ok: bool = True) -> None:
    mark = "✓" if ok else "✗"
    print(f"    {mark} {msg}")


def _fmt_pose(pose: Pose) -> str:
    t = pose.translation
    rv = pose.rotation.as_rotvec()
    return (f"xyz=[{t[0]:+.4f}, {t[1]:+.4f}, {t[2]:+.4f}]  "
            f"rotvec=[{rv[0]:+.4f}, {rv[1]:+.4f}, {rv[2]:+.4f}]")


def _fmt_q(q_rad) -> str:
    q_deg = np.degrees(q_rad).tolist()
    return "[" + ", ".join(f"{d:+7.2f}°" for d in q_deg) + "]"


# ---------------------------------------------------------------------
# Individual test sections. Each takes the ArmHandle and a ``dry`` flag
# (True = print what we WOULD do without commanding motion).
# ---------------------------------------------------------------------

def test_connection(arm) -> None:
    """Confirm RTDE round-trip — if we got this far via default_session,
    the connection is already up, but explicitly reading state is a
    final signal."""
    _banner("1. Connection")
    try:
        q = arm.receive.getActualQ()
        _result(f"RTDE receive responding — actualQ returned {len(q)} joint values")
    except Exception as e:
        _result(f"RTDE receive FAILED: {e}", ok=False)


def test_read_state(arm) -> None:
    """Read joint + TCP state from RTDE and print in readable form."""
    _banner("2. Read state")
    q = np.asarray(arm.receive.getActualQ(), dtype=float)
    _step("joint angles:")
    print(f"    {_fmt_q(q)}")

    tcp_rtde = arm.receive.getActualTCPPose()
    _step(f"TCP raw RTDE (base frame, axis-angle): {[round(v,4) for v in tcp_rtde]}")
    tcp_base = rtde_to_pose(tcp_rtde)
    _step(f"TCP as Pose (base frame):")
    print(f"    {_fmt_pose(tcp_base)}")

    tcp_task = arm.to_task(tcp_base)
    _step(f"TCP in TASK frame:")
    print(f"    {_fmt_pose(tcp_task)}")


def test_frame_conversions(arm) -> None:
    """Self-consistency: base → task → base should return the original.
    A failed round-trip indicates a bug in the calibration math."""
    _banner("3. Frame conversion round-trip")
    tcp_base = rtde_to_pose(arm.receive.getActualTCPPose())
    tcp_task = arm.to_task(tcp_base)
    tcp_base_again = arm.to_base(tcp_task)

    pos_err = np.linalg.norm(tcp_base_again.translation - tcp_base.translation)
    rot_err = np.linalg.norm(
        (tcp_base_again.rotation * tcp_base.rotation.inv()).as_rotvec()
    )
    _step(f"position error:  {pos_err:.3e} m")
    _step(f"rotation error:  {rot_err:.3e} rad")
    ok = pos_err < 1e-9 and rot_err < 1e-9
    _result("round-trip identity holds" if ok else "round-trip MISMATCH", ok=ok)


def test_reachability_probe(arm) -> None:
    """Run IK on each probe pose and report reachability + joint cost."""
    _banner("4. Reachability probe")
    current_q = np.asarray(arm.receive.getActualQ(), dtype=float)
    _step(f"current q: {_fmt_q(current_q)}")
    print()
    for i, X_task in enumerate(PROBE_POSES_TASK):
        print(f"  probe [{i}] at task {X_task.translation.tolist()}:")
        print(f"    {diagnose_reachability(arm, X_task)}")

    # Also test a plate grasp at one location.
    X_task_plate = Pose(translation=[-0.10, 0.20, 0.0])
    grasp = plate_rim_grasp(X_task_plate, angle_rad=np.pi / 2)
    print()
    _step(f"plate rim grasp @ {X_task_plate.translation.tolist()}, angle=90°:")
    print(f"    {diagnose_reachability(arm, grasp.grasp_pose)}")


def test_gripper(arm, dry: bool) -> None:
    """Open → close (gentle) → open cycle. No object expected in gripper."""
    _banner("5. Gripper open/close cycle")
    if arm.gripper is None:
        _result("no gripper attached — skipping", ok=False)
        return

    steps = [
        ("open (fast)",  lambda: (arm.gripper.set_speed_pct(GRIPPER_OPEN_SPEED_PCT),
                                  arm.gripper.open())),
        ("close (gentle)", lambda: (arm.gripper.set_speed_pct(GRIPPER_CLOSE_SPEED_PCT),
                                    arm.gripper.set_force_pct(GRIPPER_CLOSE_FORCE_PCT),
                                    arm.gripper.close())),
        ("open (fast)",  lambda: (arm.gripper.set_speed_pct(GRIPPER_OPEN_SPEED_PCT),
                                  arm.gripper.open())),
    ]
    for label, action in steps:
        _step(label)
        if dry:
            print(f"    [dry] skipping gripper RPC")
        else:
            action()
            time.sleep(GRIPPER_PAUSE_BETWEEN_ACTIONS)
    _result("gripper cycle complete")


def test_movej(arm, dry: bool) -> None:
    """Command a joint-space move to MOVEJ_TARGET_DEG. Verify arrival
    by reading actualQ afterward."""
    _banner("6. moveJ to target q")
    if MOVEJ_TARGET_DEG is None:
        _result("MOVEJ_TARGET_DEG is None — skipping actual motion", ok=False)
        return

    target_rad = np.radians(MOVEJ_TARGET_DEG)
    current = np.asarray(arm.receive.getActualQ(), dtype=float)
    delta = target_rad - current
    _step(f"current : {_fmt_q(current)}")
    _step(f"target  : {_fmt_q(target_rad)}")
    _step(f"delta   : {_fmt_q(delta)}")
    _step(f"speed={MOVEJ_SPEED_RAD_S} rad/s, accel={MOVEJ_ACCEL_RAD_S2} rad/s²")

    if dry:
        print(f"    [dry] skipping moveJ")
        return

    arm.control.moveJ(list(target_rad), MOVEJ_SPEED_RAD_S, MOVEJ_ACCEL_RAD_S2)
    after = np.asarray(arm.receive.getActualQ(), dtype=float)
    err = np.degrees(np.linalg.norm(after - target_rad))
    _result(f"arrived at {_fmt_q(after)}  (error {err:.3f}° total)",
            ok=err < 1.0)


def test_movel_delta(arm, dry: bool) -> None:
    """Cartesian delta move in TASK frame: read current TCP, apply
    offset, moveL to the resulting pose."""
    _banner("7. moveL delta in task frame")

    tcp_base  = rtde_to_pose(arm.receive.getActualTCPPose())
    tcp_task  = arm.to_task(tcp_base)
    delta = Pose(
        translation=np.asarray(MOVEL_DELTA_TRANSLATION_TASK, dtype=float),
        rotation=Rotation.from_rotvec(np.radians(MOVEL_DELTA_ROTATION_DEG)),
    )
    target_task = Pose(
        translation=tcp_task.translation + delta.translation,
        rotation=delta.rotation * tcp_task.rotation,
    )
    target_base = arm.to_base(target_task)

    _step(f"current task: {_fmt_pose(tcp_task)}")
    _step(f"target task : {_fmt_pose(target_task)}")
    _step(f"speed={MOVEL_SPEED_M_S} m/s, accel={MOVEL_ACCEL_M_S2} m/s²")

    # Sanity: kinematically reachable?
    q = is_task_pose_reachable(arm, target_task)
    if q is None:
        _result("IK rejected target — ABORTING moveL", ok=False)
        return
    _step(f"IK solution: {_fmt_q(q)}")

    if dry:
        print(f"    [dry] skipping moveL")
        return

    arm.control.moveL(pose_to_rtde(target_base), MOVEL_SPEED_M_S, MOVEL_ACCEL_M_S2)

    # Verify we got there.
    tcp_task_after = arm.to_task(rtde_to_pose(arm.receive.getActualTCPPose()))
    err = np.linalg.norm(tcp_task_after.translation - target_task.translation)
    _result(f"arrived at task xyz={tcp_task_after.translation}  (error {err*1000:.2f} mm)",
            ok=err < 0.005)


# ---------------------------------------------------------------------
#  Orchestration
# ---------------------------------------------------------------------

def run(dry: bool) -> None:
    _banner(f"Sandbox for {ARM}  (dry={dry})")
    print(f"  RUN_CONNECTION         = {RUN_CONNECTION}")
    print(f"  RUN_READ_STATE         = {RUN_READ_STATE}")
    print(f"  RUN_FRAME_CONVERSIONS  = {RUN_FRAME_CONVERSIONS}")
    print(f"  RUN_REACHABILITY_PROBE = {RUN_REACHABILITY_PROBE}")
    print(f"  RUN_GRIPPER            = {RUN_GRIPPER}")
    print(f"  RUN_MOVEJ              = {RUN_MOVEJ}")
    print(f"  RUN_MOVEL_DELTA        = {RUN_MOVEL_DELTA}")

    if dry:
        # Dry-run — don't open RTDE. Still run the offline-only tests
        # (reachability probe doesn't need a connection if we fake q).
        _banner("DRY RUN — no RTDE connection will be opened")
        if RUN_REACHABILITY_PROBE:
            print("\n[dry] reachability probe requires a live arm for current_q; "
                  "skipped. Use the main reachability smoke test for offline checks.")
        return

    left  = (ARM == "ur_left")
    right = (ARM == "ur_right")
    with default_session(left=left, right=right) as session:
        arm = session.arms[ARM]

        if RUN_CONNECTION:
            test_connection(arm)
        if RUN_READ_STATE:
            test_read_state(arm)
        if RUN_FRAME_CONVERSIONS:
            test_frame_conversions(arm)
        if RUN_REACHABILITY_PROBE:
            test_reachability_probe(arm)
        if RUN_GRIPPER:
            test_gripper(arm, dry=False)
        if RUN_MOVEJ:
            test_movej(arm, dry=False)
        if RUN_MOVEL_DELTA:
            test_movel_delta(arm, dry=False)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--dry", action="store_true",
                    help="Print the test plan without connecting to the arm.")
    args = ap.parse_args()
    run(dry=args.dry)


if __name__ == "__main__":
    main()
