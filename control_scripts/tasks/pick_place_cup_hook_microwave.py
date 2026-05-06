"""Pick the cup-with-stick with the LEFT (hook) arm and place it on
top of the microwave.

Variant of ``pick_place_cup_microwave`` for the hook gripper: same
pick / place locations, but ur_left engages the cup's rim with the
hook on a side-of-the-cup approach instead of ur_right's top-down
pinch. Trades the Robotiq's tight Vention-stand clearance against the
hook's different reach geometry — the left arm picks naturally from
the −y side of the cup, with the forearm exiting back through +y/+x
toward the arm base, so it doesn't sweep through the Vention column
the way the right-arm 2F-85 does.

Geometry assumptions
--------------------
  - Cup-with-stick is rotationally symmetric about its +z axis (rim
    radius 4.4 cm, height 15.4 cm). Stick is half-submerged, projects
    23.1 cm above the cup base — the hook approaches from radially
    outside, so the stick stays inside the cup envelope and clear of
    the wrist (~10 cm xy clearance throughout the descent at
    GRASP_ANGLE_RAD = 135°).
  - Microwave-top place pose ``CUP_MICROWAVE_TOP_XYZ_TASK`` already
    has the cup body 3 cm clear of each microwave outer-top edge
    (wall-clearance reading; see lab_landmarks.py).

Tabletop layout: all default static objects are present (plate, plain
cup, bowl, bottle, tray). The cup-with-stick itself is welded into
the gripper for the carry rather than sitting statically in the scene.

Running
-------
    python3.11 -m control_scripts.tasks.pick_place_cup_hook_microwave [--dry]
"""

from __future__ import annotations

import numpy as np

from ..config import PickPlaceConfig
from ..grasps.base import Grasp
from ..grasps.cup import CUP_HEIGHT_M, cup_hook_grasp
from ..lab_landmarks import CUP_MICROWAVE_TOP_XYZ_TASK
from ..planning.scene.objects import CUP_WITH_STICK_DEFAULT_TASK_XYZ
from ..util.poses import Pose
from ..world import World
from ._pick_place_cup_core import CupTaskCfg, parse_args, run_main


# --- Tunables -------------------------------------------------------------
ARM = "ur_left"

CUP_PICK_POSE_TASK = Pose(translation=CUP_WITH_STICK_DEFAULT_TASK_XYZ)
"""Cup-with-stick base at PICK location. Sourced from
``CUP_WITH_STICK_DEFAULT_TASK_XYZ`` (planning/scene/objects.py)."""

CUP_PLACE_POSE_TASK = Pose(translation=list(CUP_MICROWAVE_TOP_XYZ_TASK))
"""Cup-with-stick base at PLACE location: 3 cm of wall clearance from
the right-front microwave-top corner. Sourced from
``CUP_MICROWAVE_TOP_XYZ_TASK`` (lab_landmarks.py)."""

FINAL_XY = (-0.30, 0.0)
"""Task-frame XY to retract to after placing, at ``CONFIG.transit_z``.
Pulled toward −x to keep the post-place return on the left-arm side
of the workspace (the right arm is the partner here)."""

# 180° − 45° = 135° → hook engages on the (−x, +y) side of the cup.
# Forearm extends from the engagement point toward the left-arm base
# at (−x, +y), passing wholly outside the cup's xy footprint and
# avoiding the Vention column that limits the right-arm pick.
GRASP_ANGLE_RAD = float(np.radians(120))

# 180° + 45° = 225° → hook engages on the (−x, −y) side of the cup at
# the microwave-top place pose. Forearm extends back toward the left-
# arm base in the +x/+y direction; passes y ≈ 0.39, well above the
# open-microwave door (door free-end y ≈ 0.01 at door angle 1.8 rad)
# and clear of the bottle stash at (−0.30, +0.42).
PLACE_ANGLE_RAD = float(np.radians(180 + 45))

# Lift the TCP this much (m) above the cup rim's nominal grasp z, so
# the hook's inside finger has clear vertical space above the rim
# wall during engagement instead of being flush with it. 0.5 cm gives
# the rim ~5 mm of "play" inside the hook's throat — light contact
# with the rim wall as the cup is lifted, no hard interference if
# the cup pose drifts ±5 mm. Set to 0 to revert to a flush rim grasp.
TCP_RIM_LEEWAY_M = 0.005


def _cup_hook_grasp_with_leeway(cup_pose: Pose, angle_rad: float = 0.0) -> Grasp:
    """Wrap ``cup_hook_grasp`` with a small rim-z lift so the inside
    finger sits ``TCP_RIM_LEEWAY_M`` above the rim wall instead of
    flush with it. ``rim_z_offset_m`` overrides the factory's default
    rim z (= ``CUP_HEIGHT_M``)."""
    return cup_hook_grasp(
        cup_pose,
        angle_rad=angle_rad,
        rim_z_offset_m=CUP_HEIGHT_M + TCP_RIM_LEEWAY_M,
    )

WORLD = World(
    include_microwave=True,
    include_objects=True,
    # cup-with-stick welded via in_hand for the carry leg; drop the
    # static copy so it's not double-loaded. Plate, plain cup, bowl,
    # bottle, and tray remain at default tabletop poses for collision
    # avoidance.
    skip_static_objects=("cup_with_stick",),
    robotiq_mode="closed",
    microwave_door_open_rad=0.0,
)


CONFIG = PickPlaceConfig(
    # Carry crosses the microwave (top z=0.28) with the cup hanging
    # below the hook TCP. With the hook the cup base hangs ≈ 15.4 cm
    # below the TCP, so transit_z = 0.55 puts the cup base at 0.396
    # — same ~12 cm clearance over the microwave top as the 2F-85
    # variant.
    transit_z=0.55,
    place_use_contact_descent=False,
    transit_speed=0.15,
    transit_accel=0.3,
    # Hook approach / retract speeds match pour_bottle_hook —
    # slower than free-air transit so the rim engagement is gentle.
    approach_speed=0.05,
    approach_accel=0.2,
    retract_speed=0.1,
    retract_accel=0.2,
    # Hook has no continuous aperture — None disables the
    # release-aperture-mm command in place().
    release_aperture_mm=None,
    gripper_open_speed_pct=40,
    gripper_close_speed_pct=30,
)


CFG = CupTaskCfg(
    arm=ARM,
    pick_pose_task=CUP_PICK_POSE_TASK,
    place_pose_task=CUP_PLACE_POSE_TASK,
    final_xy=FINAL_XY,
    grasp_angle_rad=GRASP_ANGLE_RAD,
    place_angle_rad=PLACE_ANGLE_RAD,
    in_hand_kind="cup_with_stick",
    world=WORLD,
    config=CONFIG,
    grasp_fn=_cup_hook_grasp_with_leeway,
)


def main(
    dry: bool = False,
    mode: str = "real",
    motion_planning: bool = True,
) -> int:
    return run_main(
        CFG, dry=dry, mode=mode, motion_planning=motion_planning,
    )


if __name__ == "__main__":
    raise SystemExit(main(**parse_args(description=__doc__.splitlines()[0])))
