"""Place: 7-segment RTDE place primitive (task-frame).

Mirror of pick. Input is ``place_pose`` (TCP pose in task frame at which
the fingers should open to release the object). Assumes the arm is already
holding the object — only pick() sets that up.

    1. lift_to_transit          current XY held, Z -> transit_z
    2. transit_xy               target XY, orientation, Z held
    3. approach_to   preplace   back off along gripper -Z by preplace_offset
    4. move_until_contact       probe in -task-z until surface force spike
    5. gripper.open()
    6. retract_to    preplace   back along gripper +Z to preplace
    7. retract_to    transit    back up to task-frame transit altitude

Step 4 replaces a hand-measured final Z with a force-triggered stop —
robust to differences between table, tray, and microwave-shelf heights.
"""

from dataclasses import dataclass
from typing import Optional

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

    # Apply gripper speed / baseline force from config. No-op on grippers
    # that don't support these (e.g. HookGripper).
    arm.gripper.set_speed_pct(config.gripper_speed_pct)
    arm.gripper.set_force_pct(config.gripper_force_pct)

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

    # 4. Probe down in task frame until surface contact.
    move_until_contact(
        arm,
        _CONTACT_PROBE_V_TASK,
        _CONTACT_PROBE_ACCEL,
        config.place_contact_threshold,
    )

    # 5. Release.
    arm.gripper.open()

    # 6. Retract to preplace along gripper +Z.
    retract_to(arm, preplace, config.retract_speed, config.retract_accel)

    # 7. Retract to transit altitude.
    retract_to(arm, transit_over_target,
               config.retract_speed, config.retract_accel)

    return PlaceResult(success=True)
