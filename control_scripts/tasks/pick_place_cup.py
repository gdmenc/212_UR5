"""End-to-end pick-and-place of a cup on the right arm.

Picks the cup from its default tabletop location and places it at the
pour station (``CUP_POUR_STATION_XY_TASK`` in ``lab_landmarks.py``) so
``pour_bottle_hook`` can pour into it. Sibling variants live in
``pick_place_cup_tray`` (cup → tray) and ``pick_place_cup_microwave``
(cup-with-stick → microwave top); shared helpers live in
``_pick_place_cup_core``.

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

from ..config import PickPlaceConfig
from ..lab_landmarks import CUP_POUR_STATION_XY_TASK
from ..planning.scene.objects import CUP_DEFAULT_TASK_XYZ
from ..util.poses import Pose
from ..world import World
from ._pick_place_cup_core import CupTaskCfg, parse_args, run_main


# --- Tunables (edit to match your physical layout) ------------------------
ARM = "ur_right"

CUP_PICK_POSE_TASK = Pose(translation=CUP_DEFAULT_TASK_XYZ)
"""Cup base at PICK location, expressed in task frame. Sourced from
``CUP_DEFAULT_TASK_XYZ`` (planning/scene/objects.py) — the canonical
lab cup position. Z is the resting surface; the rim is at
z + CUP_HEIGHT_M."""

CUP_PLACE_POSE_TASK = Pose(translation=[
    CUP_POUR_STATION_XY_TASK[0],
    CUP_POUR_STATION_XY_TASK[1],
    0.0,
])
"""Cup base at PLACE location, task frame. Sourced from
``CUP_POUR_STATION_XY_TASK`` (lab_landmarks.py) so the pour task and any
future cup-pickup task land on the same xy. Z is the resting-surface z."""

FINAL_XY = (0.4, 0.0)
"""Task-frame XY to move the arm to after placing, at ``CONFIG.transit_z``."""

GRASP_ANGLE_RAD = 0.0
PLACE_ANGLE_RAD = 0.0

WORLD = World(
    include_microwave=True,
    include_objects=False,        # cup is welded via in_hand for the carry leg
    robotiq_mode="closed",
    microwave_door_open_rad=0.0,  # microwave is in the rig but not interacted with here
)
"""Single source of env truth for this task's planning + sim scenes."""

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


CFG = CupTaskCfg(
    arm=ARM,
    pick_pose_task=CUP_PICK_POSE_TASK,
    place_pose_task=CUP_PLACE_POSE_TASK,
    final_xy=FINAL_XY,
    grasp_angle_rad=GRASP_ANGLE_RAD,
    place_angle_rad=PLACE_ANGLE_RAD,
    in_hand_kind="cup",
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
