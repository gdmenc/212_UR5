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

from ..arm import ArmHandle
from ..config import DEFAULT, PickPlaceConfig
from ..moves import approach_to, lift_to_transit, transit_xy
from ..util.poses import Pose
from ..util.rotations import Rotation
from ..util.rtde_convert import pose_to_rtde


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
    """Number of moveL waypoints for the closing push. Keep ≥ 6 so the
    arc is smooth enough that the hook face stays flush with the door."""


@dataclass
class CloseMicrowaveResult:
    success: bool
    reason: Optional[str] = None


# -----------------------------------------------------------------------
#  Arc helpers
# -----------------------------------------------------------------------

def _arc_angle_sign(door: CloseMicrowaveDoorSpec) -> float:
    """Return +1 (CCW) or -1 (CW) for the opening rotation direction."""
    r_vec = door.handle_closed_pose_task.translation - door.hinge_position_task
    ccw_tangent_2d = np.array([-r_vec[1], r_vec[0]])
    pull_2d = door.pull_direction_task[:2]
    return +1.0 if np.dot(ccw_tangent_2d, pull_2d) > 0.0 else -1.0


def open_handle_pose(door: CloseMicrowaveDoorSpec) -> Pose:
    """Compute the handle TCP pose when the door is fully open.

    This is the position the arm needs to reach before starting the
    push — the door should already be at this angle when close_microwave_door
    is called.
    """
    engage = door.handle_closed_pose_task
    hinge = door.hinge_position_task
    r_vec = engage.translation - hinge

    total_angle = _arc_angle_sign(door) * door.arc_open_angle_rad
    c, s = np.cos(total_angle), np.sin(total_angle)
    r_open = np.array([c * r_vec[0] - s * r_vec[1],
                       s * r_vec[0] + c * r_vec[1],
                       r_vec[2]])
    dR = Rotation.from_rotvec(np.array([0.0, 0.0, total_angle]))
    return Pose(
        translation=hinge + r_open,
        rotation=dR * engage.rotation,
    )


def close_arc_waypoints(door: CloseMicrowaveDoorSpec) -> List[Pose]:
    """Generate task-frame TCP poses from fully-open back to closed.

    The reverse of ``_arc_waypoints`` in open_microwave — same geometry,
    angle sequence runs from ``total_angle`` back to 0. The final waypoint
    is at angle=0, i.e., door fully closed.
    """
    engage = door.handle_closed_pose_task
    hinge = door.hinge_position_task
    r_vec = engage.translation - hinge

    total_angle = _arc_angle_sign(door) * door.arc_open_angle_rad
    # Exclude total_angle (already there), include 0 (fully closed).
    angles = np.linspace(total_angle, 0.0, door.n_arc_steps + 1)[1:]

    waypoints: List[Pose] = []
    for theta in angles:
        c, s = np.cos(theta), np.sin(theta)
        new_r = np.array([c * r_vec[0] - s * r_vec[1],
                          s * r_vec[0] + c * r_vec[1],
                          r_vec[2]])
        dR = Rotation.from_rotvec(np.array([0.0, 0.0, theta]))
        waypoints.append(Pose(
            translation=hinge + new_r,
            rotation=dR * engage.rotation,
        ))
    return waypoints


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
    lift_to_transit(arm, config.transit_z, config.transit_speed, config.transit_accel)
    transit_xy(arm, handle_open, config.transit_z,
               config.transit_speed, config.transit_accel)
    approach_to(arm, handle_open, config.approach_speed, config.approach_accel)

    # --- Phase 2: push closed along the reverse arc ---
    for wp in waypoints:
        arm.control.moveL(
            pose_to_rtde(arm.to_base(wp)),
            config.approach_speed,
            config.approach_accel,
        )

    # --- Phase 3: retract (lift straight up, hook not latched) ---
    lift_to_transit(arm, config.transit_z, config.retract_speed, config.retract_accel)

    return CloseMicrowaveResult(success=True)
