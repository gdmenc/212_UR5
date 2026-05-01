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
from typing import Optional, Sequence

import numpy as np

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

    # 3b. Preset the gripper for the final descent. Per-gripper semantics:
    #   - 2F-85: narrow to release_aperture_mm if given; else leave alone.
    #   - Hook:  open the throat (regardless of the aperture argument).
    arm.gripper.set_speed_pct(config.gripper_open_speed_pct)
    arm.gripper.prepare_for_grasp(target_aperture_mm=config.release_aperture_mm)

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


def pick_from_box(
    arm: ArmHandle,
    grasp: Grasp,
    entry_xy: Sequence[float],
    entry_z: float,
    config: PickPlaceConfig = DEFAULT,
) -> PickResult:
    """Pick where the grasp pose lives inside a ceiling-constrained cavity
    (microwave, shelf, ...). The standard ``pick`` flow retracts upward
    along tool +Z after grasping — that smashes the wrist into the ceiling
    when grasp_pose is inside a box. Instead this routes:

        transit_z → (entry_xy, transit_z) → (entry_xy, entry_z)
                  → pregrasp → grasp_pose
                  → close → pregrasp → (entry_xy, entry_z)
                  → (entry_xy, transit_z)

    so the gripper stays below the ceiling while inside the box, then
    ascends only after exiting.

    ``entry_xy`` is the task-frame XY OUTSIDE the box where the descent
    from transit_z to entry_z happens — typically a few cm beyond the
    door plane on the same axis the cavity opens along (e.g. for a
    -Y-facing microwave door: same X as the target, Y a few cm below
    the door plane). ``entry_z`` is the OUTSIDE-the-door descent
    altitude only — it is NOT the in-cavity transit altitude. Once
    inside the cavity the gripper goes to ``pregrasp`` (which is
    ``offset_along_tool_z(grasp_pose, grasp.pregrasp_offset)``), so the
    final descent to grasp_pose is a pure tool +Z move — the same
    final approach as the standard ``pick``.

    The approach orientation is taken from ``grasp.grasp_pose`` and is
    held constant from the entry pose all the way through the box, so
    the gripper does not rotate inside the cavity.

    Caller is responsible for choosing a grasp angle that puts the
    forearm pointing back through the door."""
    if config.transit_z is None:
        raise ValueError(
            "config.transit_z is unset — set it from measurements before calling pick_from_box()."
        )
    if arm.gripper is None:
        raise ValueError(f"arm {arm.name!r} has no gripper attached.")

    grasp_pose = grasp.grasp_pose
    entry_xy = np.asarray(entry_xy, dtype=float).reshape(2)

    entry_pose = Pose(
        translation=np.array([entry_xy[0], entry_xy[1], entry_z]),
        rotation=grasp_pose.rotation,
    )
    pregrasp = offset_along_tool_z(grasp_pose, grasp.pregrasp_offset)
    transit_over_entry = pose_at_altitude(entry_pose, config.transit_z)

    # 1. Lift to transit altitude (task frame).
    lift_to_transit(arm, config.transit_z, config.transit_speed, config.transit_accel)

    # 2. Transit at altitude to ABOVE the entry XY, rotating to grasp orientation.
    transit_xy(arm, entry_pose, config.transit_z,
               config.transit_speed, config.transit_accel)

    # 3. Descend OUTSIDE the box from transit_z to entry_z.
    approach_to(arm, entry_pose, config.approach_speed, config.approach_accel)

    # 3b. Preset gripper aperture before going inside the cavity. Per-gripper
    # semantics match pick(): 2F-85 narrows to release_aperture_mm if given,
    # hook opens its throat regardless. Doing it BEFORE the lateral entry
    # so the fingers are at their narrow profile inside the box.
    arm.gripper.set_speed_pct(config.gripper_open_speed_pct)
    arm.gripper.prepare_for_grasp(target_aperture_mm=config.release_aperture_mm)

    # 4. Move from the OUTSIDE-the-door entry pose to the pregrasp
    # (along tool -Z from grasp_pose). One moveL — combines lateral
    # motion through the door with any vertical adjustment from
    # entry_z to pregrasp_z. Orientation held constant.
    approach_to(arm, pregrasp, config.approach_speed, config.approach_accel)

    # 5. Final descent to grasp pose. With step 4 landing at pregrasp,
    # this is a pure tool +Z move — the same final approach as the
    # standard pick().
    approach_to(arm, grasp_pose, config.approach_speed, config.approach_accel)

    # 6. Close gripper.
    arm.gripper.set_speed_pct(config.gripper_close_speed_pct)
    held = arm.gripper.grasp(force=grasp.grasp_force)
    if not held:
        # Failure path: retrace the entry sequence.
        retract_to(arm, pregrasp, config.retract_speed, config.retract_accel)
        retract_to(arm, entry_pose, config.retract_speed, config.retract_accel)
        retract_to(arm, transit_over_entry,
                   config.retract_speed, config.retract_accel)
        return PickResult(success=False,
                          reason=f"grasp did not detect object ({grasp.description})",
                          grasp=grasp)

    # 7. Lift along tool -Z back to pregrasp.
    retract_to(arm, pregrasp, config.retract_speed, config.retract_accel)

    # 8. Move from pregrasp back out to the entry pose (lateral exit
    # plus any Z adjustment back to entry_z).
    retract_to(arm, entry_pose, config.retract_speed, config.retract_accel)

    # 9. Ascend to transit_z (above the entry XY, outside the box).
    retract_to(arm, transit_over_entry,
               config.retract_speed, config.retract_accel)

    return PickResult(success=True, grasp=grasp)
