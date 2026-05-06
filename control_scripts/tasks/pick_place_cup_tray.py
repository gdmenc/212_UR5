"""Pick the (poured) cup from the pour station and place it on the tray.

Variant of ``pick_place_cup``: the cup starts at the pour station
(``CUP_POUR_STATION_XY_TASK``) where ``pour_bottle_hook`` left it, and
gets carried to the tray's "cup" slot (``util.tray_layout``).

Tabletop layout for this task: the plate and bowl are also on the table
(at their default poses). The bottle and the cup-with-stick are not in
the scene.

Running
-------
    python -m control_scripts.tasks.pick_place_cup_tray [--dry]
    python -m control_scripts.run task pick_place_cup_tray [--dry]
"""

from __future__ import annotations

from ..config import PickPlaceConfig
from ..lab_landmarks import CUP_POUR_STATION_XY_TASK
from ..util.poses import Pose
from ..util.tray_layout import place_pose_on_tray
from ..world import World
from ._pick_place_cup_core import CupTaskCfg, parse_args, run_main


# --- Tunables -------------------------------------------------------------
ARM = "ur_right"

CUP_PICK_POSE_TASK = Pose(translation=[
    CUP_POUR_STATION_XY_TASK[0],
    CUP_POUR_STATION_XY_TASK[1],
    0.0,
])
"""Cup base at PICK location: the pour station, projected to the table z=0.
Sourced from ``CUP_POUR_STATION_XY_TASK`` (lab_landmarks.py)."""

CUP_PLACE_POSE_TASK = place_pose_on_tray("cup")
"""Cup base at PLACE location: the tray's "cup" slot. Sourced from
``util/tray_layout.py`` so the on-tray slot table stays the single
source of truth for tray-aware tasks."""

FINAL_XY = (0.4, 0.0)
"""Task-frame XY to retract to after placing, at ``CONFIG.transit_z``."""

GRASP_ANGLE_RAD = 0.0
PLACE_ANGLE_RAD = 0.0

WORLD = World(
    include_microwave=True,
    include_objects=True,
    # The cup we're carrying is welded via in_hand during the carry leg, so
    # drop the static "cup" to avoid double-loading it. Bottle and
    # cup-with-stick are out of scope for this task per the lab layout —
    # plate, bowl, and the tray remain at their default poses.
    skip_static_objects=("cup", "cup_with_stick", "bottle"),
    robotiq_mode="closed",
    microwave_door_open_rad=0.0,
)


CONFIG = PickPlaceConfig(
    # Carry stays well south of the microwave (y < 0.22), so 30 cm transit
    # altitude leaves ~15 cm of overhead for the held cup — same budget as
    # ``pick_place_cup``.
    transit_z=0.30,
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
