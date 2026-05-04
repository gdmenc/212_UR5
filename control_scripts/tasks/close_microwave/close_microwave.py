"""Push the microwave door closed along the reverse arc (left arm, no grasp).

Three-phase strategy:

    1. Approach   (moveL)      Transit to the fully-open handle position.
                               The arm adopts the door-open TCP orientation
                               during transit so the descent is clean.

    2. Push close (moveL arc)  Reverse of the opening arc: moveL waypoints
                               from the fully-open angle back to 0 (closed).
                               Both TCP position and orientation rotate around
                               the hinge axis (task Z) in the closing direction.

    3. Retract    (moveL)      Lift straight up to transit altitude. Safe
                               because the hook is not latched (no grasp was
                               performed), so the door face doesn't catch it.

No force mode, no gripper operation. The arm simply follows the computed arc;
the door geometry provides the contact. Keep push speed low so the door
doesn't slam and rebound.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import List, Optional

import numpy as np

from ...arm import ArmHandle
from ...config import DEFAULT, PickPlaceConfig
from ...moves import approach_to, lift_to_transit, transit_xy
from ...tasks.open_microwave import MicrowaveDoorSpec, _arc_waypoints
from ...util.poses import Pose
from ...util.rotations import Rotation
from ...util.rtde_convert import pose_to_rtde, rtde_to_pose


# -----------------------------------------------------------------------
#  Spec
# -----------------------------------------------------------------------

@dataclass
class CloseMicrowaveDoorSpec:
    """Geometry for pushing the microwave door closed.

    All poses and positions in task frame.  The parameters mirror those
    in ``MicrowaveDoorSpec`` so the same measured values can be reused."""

    handle_closed_pose_task: Pose
    """TCP pose at the door handle when the door is CLOSED.
    This is identical to ``MicrowaveDoorSpec.handle_engage_pose_task`` —
    reuse the same recorded waypoint."""

    hinge_position_task: np.ndarray
    """Door hinge position in task frame.  Same value as used to open."""

    pull_direction_task: np.ndarray = field(
        default_factory=lambda: np.array([0.0, -1.0, 0.0])
    )
    """Same as ``MicrowaveDoorSpec.pull_direction_task``. Used only to
    determine the arc rotation sign so the close arc is consistent with
    the open arc."""

    arc_open_angle_rad: float = 1.2
    """The angle (rad) at which the door is fully open — same value used
    when opening. The close arc runs from this angle back to 0."""

    n_arc_steps: int = 8
    """Number of moveJ waypoints for the closing push. Keep ≥ 6 so the
    arc is smooth enough that the hook face stays flush with the door."""

    joint_speed: float = 0.5
    """Joint speed (rad/s) for the ``moveJ`` arc steps."""

    joint_accel: float = 0.3
    """Joint acceleration (rad/s²) for the ``moveJ`` arc steps."""


@dataclass
class CloseMicrowaveResult:
    success: bool
    reason: Optional[str] = None


# -----------------------------------------------------------------------
#  Arc helpers
# -----------------------------------------------------------------------

# During opening the arc orientation tracks the pull (opening) tangent direction.
# For closing we travel the arc in reverse, so without correction the tool would
# face backward relative to the direction of travel — "gliding" rather than
# "facing" the arc.  A 180° flip around task Z (the hinge axis) corrects this so
# the tool always faces forward in the closing direction.
_FLIP_Z = Rotation.from_rotvec(np.array([0.0, 0.0, np.pi]))


def _as_open_spec(door: CloseMicrowaveDoorSpec) -> MicrowaveDoorSpec:
    """Build a MicrowaveDoorSpec from the close spec so we can reuse
    ``_arc_waypoints`` from the open task."""
    return MicrowaveDoorSpec(
        handle_engage_pose_task=door.handle_closed_pose_task,
        hinge_position_task=door.hinge_position_task,
        pull_direction_task=door.pull_direction_task,
        arc_open_angle_rad=door.arc_open_angle_rad,
        n_arc_steps=door.n_arc_steps,
    )


def open_handle_pose(door: CloseMicrowaveDoorSpec) -> Pose:
    """Compute the handle TCP pose when the door is fully open.

    The orientation is flipped 180° around task Z relative to the open-arc
    endpoint so the approach already has the closing-direction tool face.
    """
    open_wps = _arc_waypoints(_as_open_spec(door))
    last = open_wps[-1]
    return Pose(translation=last.translation, rotation=_FLIP_Z * last.rotation)


def close_arc_waypoints(door: CloseMicrowaveDoorSpec) -> List[Pose]:
    """Generate task-frame TCP poses from fully-open back to closed.

    Reuses ``_arc_waypoints`` from the open task, reverses the list, and
    applies a 180° task-Z flip to every orientation so the tool faces the
    closing direction of travel rather than the opening direction.
    """
    open_wps = _arc_waypoints(_as_open_spec(door))
    return [
        Pose(translation=wp.translation, rotation=_FLIP_Z * wp.rotation)
        for wp in reversed(open_wps)
    ]


# -----------------------------------------------------------------------
#  Primitive
# -----------------------------------------------------------------------

def close_microwave_door(
    arm: ArmHandle,
    door: CloseMicrowaveDoorSpec,
    config: PickPlaceConfig = DEFAULT,
) -> CloseMicrowaveResult:
    """Push the microwave door closed. No gripping required.

    Assumes the door is already at ``arc_open_angle_rad`` when called.
    The arm transits to the fully-open handle position, pushes along the
    reverse arc to closed, then retracts.
    """
    if config.transit_z is None:
        raise ValueError("config.transit_z is unset.")

    handle_open = open_handle_pose(door)
    waypoints = close_arc_waypoints(door)

    # --- Phase 1: approach the fully-open handle position ---
    # Use moveJ with IK hint (same as the arc) to avoid the wrist
    # singularity that moveL hits, which was arriving with ~90° wrong
    # orientation.
    #
    # Step 1a: lift to transit Z (still use moveL — vertical only, no
    # orientation change, so no singularity risk).
    lift_to_transit(arm, config.transit_z, config.transit_speed, config.transit_accel)

    # Step 1b: move to the open-handle XY at transit Z, then descend,
    # all via moveJ with IK to keep the wrist on the correct branch.
    from ...util.poses import pose_at_altitude
    transit_pose = pose_at_altitude(handle_open, config.transit_z)

    prev_joints = list(arm.receive.getActualQ())
    for target_pose in [transit_pose, handle_open]:
        rp = pose_to_rtde(arm.to_base(target_pose))
        target_joints = arm.control.getInverseKinematics(
            rp, prev_joints, 0.001, 0.001,
        )
        arm.control.moveJ(
            target_joints,
            door.joint_speed,
            door.joint_accel,
        )
        prev_joints = list(arm.receive.getActualQ())

    # --- Phase 2: push closed along the reverse arc ---
    # Use moveJ with IK solved via q_near to avoid wrist singularities,
    # same approach as the open_microwave arc.
    rtde_poses = [pose_to_rtde(arm.to_base(wp)) for wp in waypoints]
    prev_joints = list(arm.receive.getActualQ())

    for rp in rtde_poses:
        target_joints = arm.control.getInverseKinematics(
            rp, prev_joints, 0.001, 0.001,
        )
        arm.control.moveJ(
            target_joints,
            door.joint_speed,
            door.joint_accel,
        )
        prev_joints = list(arm.receive.getActualQ())

    # --- Phase 3: retract (lift straight up, hook not latched) ---
    lift_to_transit(arm, config.transit_z, config.retract_speed, config.retract_accel)

    return CloseMicrowaveResult(success=True)
