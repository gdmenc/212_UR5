"""Pick: 7-segment RTDE pick primitive (task-frame).

Takes a ``Grasp`` — built by a per-object factory in ``grasps/`` — which
carries the task-frame grasp pose together with the per-object pregrasp
offset and grasp force. The primitive sequence:

    1. lift_to_transit          current XY held, Z -> transit_z (task frame)
    2. transit_xy               target XY, target orientation, Z held
    3. approach_to   pregrasp   back off along gripper -Z by grasp.pregrasp_offset
    4. approach_to   grasp      final approach to the grasp pose
    5. gripper.grasp(force)
    6. retract_to    pregrasp   back along gripper +Z to pregrasp
    7. retract_to    transit    back up to task-frame transit altitude

Object-specific magnitudes (pregrasp standoff, close force) come from the
Grasp, not from config. Only workspace-level tunables (transit_z, speeds,
accels) are read from PickPlaceConfig — those are the same for every
object at a given rig.
"""

from dataclasses import dataclass
from typing import Optional

from .arm import ArmHandle
from .config import DEFAULT, PickPlaceConfig
from .grasps import Grasp
from .moves import approach_to, lift_to_transit, retract_to, transit_xy
from .util.poses import Pose, offset_along_tool_z, pose_at_altitude


@dataclass
class PickResult:
    success: bool
    reason: Optional[str] = None
    grasp: Optional[Grasp] = None


def pick(
    arm: ArmHandle,
    grasp: Grasp,
    config: PickPlaceConfig = DEFAULT,
) -> PickResult:
    if config.transit_z is None:
        raise ValueError(
            "config.transit_z is unset — set it from measurements before calling pick()."
        )
    if arm.gripper is None:
        raise ValueError(f"arm {arm.name!r} has no gripper attached.")

    grasp_pose = grasp.grasp_pose
    pregrasp = offset_along_tool_z(grasp_pose, grasp.pregrasp_offset)
    transit_over_target = pose_at_altitude(pregrasp, config.transit_z)

    # 1. Lift straight up in task frame to transit altitude.
    lift_to_transit(arm, config.transit_z, config.transit_speed, config.transit_accel)

    # 2. Transit across at altitude, rotating to grasp orientation en route.
    transit_xy(arm, grasp_pose, config.transit_z,
               config.transit_speed, config.transit_accel)

    # 3. Approach pregrasp (along gripper -Z from transit altitude).
    approach_to(arm, pregrasp, config.approach_speed, config.approach_accel)

    # 3b. If a release/pre-grasp aperture is configured, preset the gripper
    # before the final approach so the fingers enter the object envelope
    # at the right width rather than swinging in from fully open.
    if config.release_aperture_mm is not None and hasattr(arm.gripper, "move_mm"):
        arm.gripper.set_speed_pct(config.gripper_open_speed_pct)
        arm.gripper.move_mm(config.release_aperture_mm)

    # 4. Approach grasp (final offset along gripper -Z).
    approach_to(arm, grasp_pose, config.approach_speed, config.approach_accel)

    # 5. Close gripper. Apply close-speed from config just before grasp;
    # grasp() handles per-object force internally from grasp.grasp_force.
    arm.gripper.set_speed_pct(config.gripper_close_speed_pct)
    held = arm.gripper.grasp(force=grasp.grasp_force)
    if not held:
        retract_to(arm, pregrasp, config.retract_speed, config.retract_accel)
        retract_to(arm, transit_over_target,
                   config.retract_speed, config.retract_accel)
        return PickResult(success=False,
                          reason=f"grasp did not detect object ({grasp.description})",
                          grasp=grasp)

    # 5b. Small clearance lift along tool +Z before the larger retract —
    # breaks static friction with the surface gently before the full
    # retract to pregrasp.
    if config.release_clearance > 0.0:
        clearance_pose = offset_along_tool_z(grasp_pose, config.release_clearance)
        retract_to(arm, clearance_pose, config.retract_speed, config.retract_accel)

    # 6. Retract to pregrasp along gripper +Z.
    retract_to(arm, pregrasp, config.retract_speed, config.retract_accel)

    # 7. Retract to transit altitude (task-frame Z, XY/orientation held).
    retract_to(arm, transit_over_target,
               config.retract_speed, config.retract_accel)

    return PickResult(success=True, grasp=grasp)
