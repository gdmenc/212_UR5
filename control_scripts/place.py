"""Place: 7-segment RTDE place primitive (task-frame).

Mirror of pick. Input is ``place_pose`` (TCP pose in task frame at which
the fingers should open to release the object). Assumes the arm is already
holding the object — only pick() sets that up.

    1. lift_to_transit          current XY held, Z -> transit_z
    2. transit_xy               target XY, orientation, Z held
    3. approach_to   preplace   back off along gripper -Z by preplace_offset
    4. final descent            depends on config.place_use_contact_descent:
        - True (default): move_until_contact — probes in -task-z until a
                          force spike. Robust to surface-height error.
        - False          : approach_to(place_pose) — deterministic descent
                          to the supplied place pose. No force mode RPCs.
    5. gripper.open()
    6. retract_to    preplace   back along gripper +Z to preplace
    7. retract_to    transit    back up to task-frame transit altitude
"""

from dataclasses import dataclass
from typing import Optional, Sequence

import numpy as np

from .arm import ArmHandle
from .config import DEFAULT, PickPlaceConfig
from .moves import (
    approach_to,
    lift_to_transit,
    move_until_contact,
    retract_to,
    transit_xy,
)
from .util.poses import Pose, offset_along_tool_z, pose_at_altitude


# Downward task-frame velocity used for the contact probe — a property of
# how we place, not a user-tunable.
_CONTACT_PROBE_V_TASK = [0.0, 0.0, -0.02, 0.0, 0.0, 0.0]
_CONTACT_PROBE_ACCEL = 0.25


@dataclass
class PlaceResult:
    success: bool
    reason: Optional[str] = None


def place(
    arm: ArmHandle,
    place_pose: Pose,
    config: PickPlaceConfig = DEFAULT,
) -> PlaceResult:
    if config.transit_z is None:
        raise ValueError(
            "config.transit_z is unset — set it from measurements before calling place()."
        )
    if arm.gripper is None:
        raise ValueError(f"arm {arm.name!r} has no gripper attached.")

    preplace = offset_along_tool_z(place_pose, config.preplace_offset)
    transit_over_target = pose_at_altitude(preplace, config.transit_z)

    # 1. Lift from wherever pick() ended (should already be at transit, but
    # re-enforce so place() is safe to call standalone).
    lift_to_transit(arm, config.transit_z, config.transit_speed, config.transit_accel)

    # 2. Transit at altitude, rotating to target orientation.
    transit_xy(arm, place_pose, config.transit_z,
               config.transit_speed, config.transit_accel)

    # 3. Approach preplace along gripper -Z.
    approach_to(arm, preplace, config.approach_speed, config.approach_accel)

    # 4. Final descent — either force-seeking (robust to surface-height
    # error) or deterministic approach_to the supplied place pose.
    if config.place_use_contact_descent:
        move_until_contact(
            arm,
            _CONTACT_PROBE_V_TASK,
            _CONTACT_PROBE_ACCEL,
            config.place_contact_threshold,
        )
    else:
        approach_to(arm, place_pose, config.approach_speed, config.approach_accel)

    # 5. Release. Apply open-speed once; if a partial release aperture is
    # configured (and the gripper supports move_mm), do a staged release:
    # partial-open → small clearance hop → STAY at the capped aperture.
    # The fingers are already clear of the released object via the
    # clearance hop, so there's no upside to swinging out to ~85 mm —
    # and the cap means the next pick's pre-grasp preset already matches,
    # avoiding a wasted full-open / re-close cycle inside enclosed cavities.
    # Without release_aperture_mm we keep the legacy full open.
    arm.gripper.set_speed_pct(config.gripper_open_speed_pct)
    if config.release_aperture_mm is not None and hasattr(arm.gripper, "move_mm"):
        arm.gripper.move_mm(config.release_aperture_mm)
        if config.release_clearance > 0.0:
            clearance_pose = offset_along_tool_z(place_pose, config.release_clearance)
            retract_to(arm, clearance_pose, config.retract_speed, config.retract_accel)
    else:
        arm.gripper.open()

    # 6. Retract to preplace along gripper +Z.
    retract_to(arm, preplace, config.retract_speed, config.retract_accel)

    # 7. Retract to transit altitude.
    retract_to(arm, transit_over_target,
               config.retract_speed, config.retract_accel)

    return PlaceResult(success=True)


def place_into_box(
    arm: ArmHandle,
    place_pose: Pose,
    entry_xy: Sequence[float],
    entry_z: float,
    config: PickPlaceConfig = DEFAULT,
) -> PlaceResult:
    """Place where the release pose lives inside a ceiling-constrained
    cavity. Mirrors ``pick_from_box`` — the standard ``place`` flow
    retracts up along tool +Z and would smash the wrist into the cavity
    ceiling. This routes:

        transit_z → (entry_xy, transit_z) → (entry_xy, entry_z)
                  → preplace → place_pose
                  → release → preplace → (entry_xy, entry_z)
                  → (entry_xy, transit_z)

    The in-cavity waypoint is ``preplace = offset_along_tool_z
    (place_pose, config.preplace_offset)``, so the final descent to
    place_pose is a pure tool +Z move — the same final approach as the
    standard ``place``. Final descent is always deterministic
    (``approach_to(place_pose)``); the ``place_use_contact_descent``
    config flag is ignored — running a force-mode probe inside a
    ceiling-constrained cavity is too risky on a first cut.

    Same caller responsibilities as ``pick_from_box``: pick a place
    angle that puts the forearm pointing back through the door, and
    set ``entry_z`` (the OUTSIDE-the-door descent altitude) above the
    cavity floor (with object clearance) and below the cavity ceiling
    (with wrist clearance)."""
    if config.transit_z is None:
        raise ValueError(
            "config.transit_z is unset — set it from measurements before calling place_into_box()."
        )
    if arm.gripper is None:
        raise ValueError(f"arm {arm.name!r} has no gripper attached.")

    entry_xy = np.asarray(entry_xy, dtype=float).reshape(2)

    entry_pose = Pose(
        translation=np.array([entry_xy[0], entry_xy[1], entry_z]),
        rotation=place_pose.rotation,
    )
    preplace = offset_along_tool_z(place_pose, config.preplace_offset)
    transit_over_entry = pose_at_altitude(entry_pose, config.transit_z)

    # 1. Lift to transit altitude.
    lift_to_transit(arm, config.transit_z, config.transit_speed, config.transit_accel)

    # 2. Transit at altitude to ABOVE the entry XY.
    transit_xy(arm, entry_pose, config.transit_z,
               config.transit_speed, config.transit_accel)

    # 3. Descend OUTSIDE the box from transit_z to entry_z.
    approach_to(arm, entry_pose, config.approach_speed, config.approach_accel)

    # 4. Move from the OUTSIDE-the-door entry pose to preplace (along
    # tool -Z from place_pose). One moveL — combines lateral motion
    # through the door with any vertical adjustment from entry_z to
    # preplace_z. Orientation held constant.
    approach_to(arm, preplace, config.approach_speed, config.approach_accel)

    # 5. Final descent to the place pose. With step 4 landing at
    # preplace, this is a pure tool +Z move — same final approach as
    # the standard place().
    approach_to(arm, place_pose, config.approach_speed, config.approach_accel)

    # 6. Release. If a release aperture is set and the gripper supports
    # move_mm, just narrow to that aperture (no clearance hop — we manage
    # vertical clearance via the explicit ascend in step 7). Otherwise
    # full open().
    arm.gripper.set_speed_pct(config.gripper_open_speed_pct)
    if config.release_aperture_mm is not None and hasattr(arm.gripper, "move_mm"):
        arm.gripper.move_mm(config.release_aperture_mm)
    else:
        arm.gripper.open()

    # 7. Lift along tool -Z back to preplace.
    retract_to(arm, preplace, config.retract_speed, config.retract_accel)

    # 8. Move from preplace back out to the entry pose (lateral exit
    # plus any Z adjustment back to entry_z).
    retract_to(arm, entry_pose, config.retract_speed, config.retract_accel)

    # 9. Ascend to transit_z above the entry XY.
    retract_to(arm, transit_over_entry,
               config.retract_speed, config.retract_accel)

    return PlaceResult(success=True)
