"""End-to-end pick-and-place of a cup on the right arm.

Mirrors ``pick_place_plate.py``: pick the cup with a top-down rim pinch,
transit, and place it. The grasp is a vertical (top-down) rim grasp —
fingers straddle the rim and close radially across the ~2 mm wall.

Hardware assumptions
--------------------
  - The right arm has a Robotiq 2F-85 attached and a pre-calibrated TCP
    offset (set in ``control_scripts/calibration.py``).
  - A cup sitting at task-frame ``CUP_PICK_POSE_TASK``. The cup's task-z
    is the resting surface — the rim ends up at task-z + CUP_HEIGHT_M.
  - ``CUP_PLACE_POSE_TASK`` is the intended cup base pose after placement.

Running
-------
Standalone:
    python -m control_scripts.tasks.pick_place_cup [--dry]

Via the unified entrypoint:
    python -m control_scripts.run task pick_place_cup [--dry]
"""

from __future__ import annotations

import argparse

import numpy as np

from ..arm import ArmHandle
from ..config import PickPlaceConfig
from ..grasps.cup import cup_rim_grasp
from ..moves import transit_xy
from ..pick import pick
from ..place import place
from ..session import default_session
from ..util.poses import Pose


# --- Tunables (edit to match your physical layout) ------------------------
CUP_PICK_POSE_TASK = Pose(translation=[-0.08, -0.125, 0.0])
# CUP_PICK_POSE_TASK = Pose(translation=[0.3, 0.075, 0.0])
"""Cup base at PICK location, expressed in task frame. Edit to match the
table layout. Z is the resting surface; the rim is at z + CUP_HEIGHT_M."""

CUP_PLACE_POSE_TASK = Pose(translation=[0, 0, 0.0])
# CUP_PLACE_POSE_TASK = Pose(translation=[0.33, 0.075, 0.0])
"""Cup base at PLACE location, task frame. Defaults to the same as the
pick pose (set it back down) — set to a different translation to relocate."""

FINAL_XY = np.array([0.35, 0.0])
"""Task-frame XY to move the arm to after placing, at ``CONFIG.transit_z``."""

GRASP_ANGLE_RAD = 0.0
"""Which rim angle to grasp at, in cup frame."""

PLACE_ANGLE_RAD = 0.0
"""Which rim angle to release at. Keeping it equal to GRASP_ANGLE_RAD
means the gripper's orientation is unchanged through the whole motion —
easier on the wrist than re-solving a new orientation at the destination."""

ARM = "ur_right"

CONFIG = PickPlaceConfig(
    # Transit clearance above all workspace objects. Cup is 15.4 cm tall;
    # 30 cm leaves ~15 cm overhead for the held cup during transit.
    transit_z=0.30,
    place_use_contact_descent=False,  # no force-seeking; no table contact
    transit_speed=0.15,
    transit_accel=0.3,

    # Release aperture. Rim wall is ~2 mm; opening to 30 mm puts each
    # finger ~14 mm from TCP center, well clear of the rim wall (and the
    # cup's interior at the rim height ≈ 4.2 cm radius).
    release_aperture_mm=30,

    # Gripper speeds. Slower than 100% — gentle close on the thin rim,
    # controlled release so the cup doesn't get knocked when fingers part.
    gripper_open_speed_pct=40,
    gripper_close_speed_pct=30,
)


def plan_pick():
    return cup_rim_grasp(CUP_PICK_POSE_TASK, angle_rad=GRASP_ANGLE_RAD)


def plan_place() -> Pose:
    """TCP pose at release. Uses the same rim-grasp factory at the place
    location so the gripper's orientation stays consistent between pick
    and place."""
    grasp_at_dest = cup_rim_grasp(
        CUP_PLACE_POSE_TASK, angle_rad=PLACE_ANGLE_RAD
    )
    return grasp_at_dest.grasp_pose


def plan_final_pose(reference_pose: Pose) -> Pose:
    """Pose used only for final XY transit after the cup is released."""
    return Pose(
        translation=[FINAL_XY[0], FINAL_XY[1], 0.0],
        rotation=reference_pose.rotation,
    )


def _print_plan(grasp, place_pose: Pose) -> None:
    print("=" * 60)
    print("  Pick location  :", CUP_PICK_POSE_TASK.translation, "(task frame)")
    print("  Place location :", CUP_PLACE_POSE_TASK.translation, "(task frame)")
    print("  Grasp angle    :", f"{np.degrees(GRASP_ANGLE_RAD):+.0f}°")
    print("  Place angle    :", f"{np.degrees(PLACE_ANGLE_RAD):+.0f}°")
    print("  Grasp pose     :", grasp.grasp_pose.translation, "(task frame)")
    print("  Place pose     :", place_pose.translation, "(task frame)")
    print("  Final XY       :", FINAL_XY, f"(task frame @ z={CONFIG.transit_z})")
    print("  Transit Z      :", CONFIG.transit_z, "m")
    print("  Release aperture:", CONFIG.release_aperture_mm, "mm")
    print("  Gripper speeds :",
          f"open={CONFIG.gripper_open_speed_pct}%,",
          f"close={CONFIG.gripper_close_speed_pct}%")
    print("  Grasp force    :", grasp.grasp_force, "N")
    print("=" * 60)


def run_on_arm(
    arm: ArmHandle,
    grasp,
    place_pose: Pose,
    config: PickPlaceConfig = CONFIG,
) -> bool:
    """Execute the pick + place on a live ArmHandle. Returns True on success.

    Use this from a routine that owns the Session — it does not connect
    or disconnect on its own."""
    print(f"\n→ pick: {grasp.description}")
    pick_result = pick(arm, grasp, config)
    if not pick_result.success:
        print(f"  ✗ pick FAILED: {pick_result.reason}")
        return False
    print("  ✓ pick succeeded.")

    print(f"\n→ place @ {CUP_PLACE_POSE_TASK.translation}")
    place_result = place(arm, place_pose, config)
    if not place_result.success:
        print(f"  ✗ place FAILED: {place_result.reason}")
        return False
    print("  ✓ place succeeded.")

    final_pose = plan_final_pose(place_pose)
    print(f"\n→ final XY @ {FINAL_XY} (transit_z={config.transit_z})")
    transit_xy(
        arm,
        final_pose,
        config.transit_z,
        config.transit_speed,
        config.transit_accel,
    )
    print("  ✓ final XY reached.")

    print("\nDone — arm retracted to transit altitude.")
    return True


def main(dry: bool = False) -> int:
    """CLI / registry entry point. Returns 0 on success, 1 on task
    failure, leaves exceptions to the caller."""
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