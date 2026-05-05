"""Push the microwave door closed along the reverse arc (left arm, no grasp).

Three-phase strategy:

    1. Approach   (moveJ)      Transit to the fully-open push contact
                               position. The arm adopts the closing-direction
                               TCP orientation during transit so the descent
                               is clean.

    2. Push close (arc)        Reverse of the opening arc: waypoints from the
                               fully-open angle back to 0 (closed). Both TCP
                               position and orientation rotate around the
                               hinge axis (task Z) in the closing direction.

                               ``mode='sequential'`` runs N blocking ``moveJ``
                               calls (decel to zero at every waypoint).
                               ``mode='path'`` runs a single ``moveJ(path)``
                               with corner blending controlled by
                               ``arc_blend_radius_m`` — visibly smoother.

    3. Retract    (moveL)      Lift straight up to transit altitude. Safe
                               because the hook is not latched (no grasp was
                               performed), so the door face doesn't catch it.

No force mode, no gripper operation. The arm simply follows the computed arc;
the door geometry provides the contact. Keep push speed low so the door
doesn't slam and rebound.

Push radius
-----------
Closing doesn't need handle engagement — any contact on the door face
works. ``push_radius_m`` lets the contact point sit inboard of the
handle (i.e. closer to the hinge), shortening the arc and keeping the
wrist away from the workspace edge. When unset, push happens at the
handle radius (matching the open arc).
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import List, Optional

import numpy as np

from ..arm import ArmHandle
from ..config import DEFAULT, PickPlaceConfig
from ..moves import approach_to, lift_to_transit, transit_xy
from ..tasks.open_microwave import MicrowaveDoorSpec, _arc_waypoints
from ..util.poses import Pose
from ..util.rotations import Rotation
from ..util.rtde_convert import pose_to_rtde, rtde_to_pose


SUPPORTS_SIM = False
"""This task uses hand-coded ``moveJ`` arc waypoints; no segments go through
the planner, so there is nothing for ``--mode sim`` to visualize. The CLI
entry in ``examples/close_microwave.py`` already uses ``--mode {sequential,
path}`` for Phase-2 execution style — that flag is preserved for byte-
identity. There is no ``--mode sim`` for this task; the constant is here
as a passive marker for any future tooling."""


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

    push_radius_m: Optional[float] = None
    """Radius (m) at which the tool pushes the door, measured from the hinge.
    When None, push happens at the handle radius (matches the open arc).
    Closing doesn't need handle engagement, so a slightly smaller value
    (e.g. 0.32 m for a 0.38 m door) puts contact inboard of the handle,
    shortens the arc, and keeps the wrist away from the workspace edge.
    The push start is computed by scaling the hinge→handle ray (XY only) to
    this radius and keeping the handle's Z and rotation."""

    arc_blend_radius_m: float = 0.0
    """TCP-distance blend radius (m) for ``mode='path'`` execution.

    Only consulted by ``_phase2_arc_blended``; the sequential phase 2
    ignores this. Must be smaller than half the chord length between
    consecutive arc waypoints, otherwise the controller rejects the path
    with C400 'blend radius too large'. 0.0 = no blending."""

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


def _push_start_pose(door: CloseMicrowaveDoorSpec) -> Pose:
    """Resolve the push contact pose at the door-closed angle.

    When ``push_radius_m`` is None or matches the handle radius, returns the
    recorded handle pose unchanged. Otherwise scales the hinge→handle ray
    (XY only) to ``push_radius_m`` and keeps the handle's Z and rotation —
    so the contact point sits inboard of the handle on the closed-door
    face."""
    handle = door.handle_closed_pose_task
    if door.push_radius_m is None:
        return handle
    hinge = door.hinge_position_task
    r_xy = handle.translation[:2] - hinge[:2]
    current_radius = float(np.linalg.norm(r_xy))
    if current_radius < 1e-6:
        return handle
    scaled_xy = r_xy * (door.push_radius_m / current_radius)
    new_translation = np.array([
        hinge[0] + scaled_xy[0],
        hinge[1] + scaled_xy[1],
        handle.translation[2],
    ])
    return Pose(translation=new_translation, rotation=handle.rotation)


def _as_open_spec(door: CloseMicrowaveDoorSpec) -> MicrowaveDoorSpec:
    """Build a MicrowaveDoorSpec from the close spec so we can reuse
    ``_arc_waypoints`` from the open task. Substitutes the push start
    pose for the handle pose so the resulting arc sits at
    ``push_radius_m`` instead of the handle radius."""
    return MicrowaveDoorSpec(
        handle_engage_pose_task=_push_start_pose(door),
        hinge_position_task=door.hinge_position_task,
        pull_direction_task=door.pull_direction_task,
        arc_open_angle_rad=door.arc_open_angle_rad,
        n_arc_steps=door.n_arc_steps,
    )


def open_handle_pose(door: CloseMicrowaveDoorSpec) -> Pose:
    """Compute the push contact TCP pose when the door is fully open.

    Sits at ``push_radius_m`` on the door face (or the handle radius when
    ``push_radius_m`` is None) at the fully-open arc angle. Orientation is
    flipped 180° around task Z relative to the open-arc endpoint so the
    approach already has the closing-direction tool face.
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
    *,
    mode: str = "sequential",
) -> CloseMicrowaveResult:
    """Push the microwave door closed. No gripping required.

    Assumes the door is already at ``arc_open_angle_rad`` when called.
    The arm transits to the fully-open push position, pushes along the
    reverse arc to closed, then retracts.

    ``mode`` selects how the arc is executed:
        ``"sequential"``  — N blocking ``moveJ`` calls (decel to zero at
                            every waypoint).
        ``"path"``        — single ``moveJ(path=...)`` with corner blending
                            controlled by ``door.arc_blend_radius_m``.
                            Visibly smoother; requires the blend radius to
                            be below half the shortest chord.
    """
    if mode not in ("sequential", "path"):
        raise ValueError(f"mode must be 'sequential' or 'path', got {mode!r}")
    if config.transit_z is None:
        raise ValueError("config.transit_z is unset.")

    handle_open = open_handle_pose(door)

    # --- Phase 1: approach the fully-open push position ---
    # Use moveJ with IK hint (same as the arc) to avoid the wrist
    # singularity that moveL hits, which was arriving with ~90° wrong
    # orientation.
    #
    # Step 1a: lift to transit Z (still use moveL — vertical only, no
    # orientation change, so no singularity risk).
    lift_to_transit(arm, config.transit_z, config.transit_speed, config.transit_accel)

    # Step 1b: move to the open-push XY at transit Z, then descend,
    # all via moveJ with IK to keep the wrist on the correct branch.
    from ..util.poses import pose_at_altitude
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
    if mode == "path":
        _phase2_arc_blended(arm, door)
    else:
        _phase2_arc_sequential(arm, door)

    # --- Phase 3: retract (lift straight up, hook not latched) ---
    lift_to_transit(arm, config.transit_z, config.retract_speed, config.retract_accel)

    return CloseMicrowaveResult(success=True)


# -----------------------------------------------------------------------
#  Phase 2 implementations
# -----------------------------------------------------------------------

def _close_arc_joint_path(
    arm: ArmHandle,
    door: CloseMicrowaveDoorSpec,
    q_seed: List[float],
    ik_pos_eps: float = 0.001,
    ik_rot_eps: float = 0.001,
) -> List[List[float]]:
    """IK chain for the close-arc waypoints with deterministic q-near seeding.

    Mirrors ``_arc_joint_path`` from the open task: each IK call seeds with
    the previous IK *result* so the joint path is purely a function of
    geometric inputs."""
    waypoints = close_arc_waypoints(door)
    joint_path: List[List[float]] = []
    prev_q = list(q_seed)
    for wp in waypoints:
        rtde_target = pose_to_rtde(arm.to_base(wp))
        q = arm.control.getInverseKinematics(
            rtde_target, prev_q, ik_pos_eps, ik_rot_eps,
        )
        if not q or len(q) != 6:
            raise RuntimeError(
                f"_close_arc_joint_path: IK returned no solution at waypoint "
                f"{len(joint_path) + 1}/{len(waypoints)} "
                f"(target task pos {np.round(wp.translation, 4)})."
            )
        joint_path.append(list(q))
        prev_q = list(q)
    return joint_path


def _phase2_arc_sequential(
    arm: ArmHandle,
    door: CloseMicrowaveDoorSpec,
) -> None:
    """Phase 2 (sequential): blocking ``moveJ`` per waypoint."""
    waypoints = close_arc_waypoints(door)
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


def _phase2_arc_blended(
    arm: ArmHandle,
    door: CloseMicrowaveDoorSpec,
) -> None:
    """Phase 2 (blended): single ``moveJ(path=...)`` with corner blending.

    Same joint waypoints as ``_phase2_arc_sequential`` (computed via
    ``_close_arc_joint_path``) but executed as one ``moveJ(path=[...])``
    call with corner blending. The controller blends consecutive segments
    inside ``door.arc_blend_radius_m``, eliminating the decel-to-zero
    between each step. The terminal waypoint is forced to ``blend_r = 0``
    because the controller requires a zero-blend final segment (otherwise
    the motion never converges)."""
    q_seed = list(arm.receive.getActualQ())
    joint_path = _close_arc_joint_path(arm, door, q_seed)

    blend_r = float(door.arc_blend_radius_m)
    path = [
        list(q) + [door.joint_speed, door.joint_accel, blend_r]
        for q in joint_path
    ]
    path[-1][-1] = 0.0
    arm.control.moveJ(path)
