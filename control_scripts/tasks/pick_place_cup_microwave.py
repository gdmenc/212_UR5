"""Pick the cup-with-stick and place it on top of the microwave.

Variant of ``pick_place_cup`` for the dowel-in-cup composite. Picks
from the cup-with-stick default tabletop position and places at
``CUP_MICROWAVE_TOP_XYZ_TASK`` (3 cm wall-clearance from the
right-front outer-top corner of the microwave; see lab_landmarks.py).

Tabletop layout: all default static objects are present (plate, plain
cup, bowl, bottle, tray). The cup-with-stick itself is welded into the
gripper for the carry rather than sitting statically in the scene.

Running
-------
    python -m control_scripts.tasks.pick_place_cup_microwave [--dry]
    python -m control_scripts.run task pick_place_cup_microwave [--dry]
"""

from __future__ import annotations

from ..config import PickPlaceConfig
from ..lab_landmarks import CUP_MICROWAVE_TOP_XYZ_TASK
from ..planning.scene.objects import CUP_WITH_STICK_DEFAULT_TASK_XYZ
from ..util.poses import Pose
from ..world import World
from ._pick_place_cup_core import CupTaskCfg, parse_args, run_main

import numpy as np


# --- Tunables -------------------------------------------------------------
ARM = "ur_right"

CUP_PICK_POSE_TASK = Pose(translation=CUP_WITH_STICK_DEFAULT_TASK_XYZ)
"""Cup-with-stick base at PICK location. Sourced from
``CUP_WITH_STICK_DEFAULT_TASK_XYZ`` (planning/scene/objects.py); update
that constant when the lab measurement is finalised."""

CUP_PLACE_POSE_TASK = Pose(translation=list(CUP_MICROWAVE_TOP_XYZ_TASK))
"""Cup-with-stick base at PLACE location: 3 cm of wall clearance from
each microwave-top edge (right-front corner). Sourced from
``CUP_MICROWAVE_TOP_XYZ_TASK`` (lab_landmarks.py)."""

FINAL_XY = (0.4, 0.0)
"""Task-frame XY to retract to after placing, at ``CONFIG.transit_z``."""

GRASP_ANGLE_RAD = np.radians(15)

PLACE_ANGLE_RAD = 0.0
"""Cup-axis rotation at release. Default 0; tune in ``--mode sim`` if
the wrist lands on an awkward IK branch reaching across the microwave's
front-right corner."""

WORLD = World(
    include_microwave=True,
    include_objects=True,
    # The cup-with-stick is welded via in_hand during the carry leg, so
    # drop its static copy to avoid double-loading it. Plate, plain cup,
    # bowl, bottle, and tray remain at their default tabletop poses for
    # collision avoidance.
    skip_static_objects=("cup_with_stick",),
    robotiq_mode="closed",
    microwave_door_open_rad=0.0,
)


CONFIG = PickPlaceConfig(
    # Carry crosses over the microwave (top z=0.28) and starts next to the
    # bottle (top z=0.165). With the cup welded at the rim, cup base hangs
    # 15.4 cm below the TCP — transit_z = 0.55 puts the cup base at 0.396 m,
    # ~12 cm above the microwave top and ~23 cm above the bottle. Tighten
    # to ~0.50 once reach is confirmed in sim.
    transit_z=0.55,
    place_use_contact_descent=False,
    transit_speed=0.15,
    transit_accel=0.3,
    release_aperture_mm=30,
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
