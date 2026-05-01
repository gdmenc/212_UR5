"""Open the microwave door with the hook gripper (left arm).

Four-phase strategy:

    1. Approach   (moveL)      Hook tip to pre-engagement pose below handle.
    2. Engage     (moveL + gripper)
                               Short slide to seat hook under the handle,
                               then extend hook (close).
    3. Pull open               Two modes, selected by whether
                               ``MicrowaveDoorSpec.hinge_position_task`` is set:

                               ARC MODE (preferred when hinge is measured):
                               A series of ``moveL`` waypoints along the
                               computed arc. The TCP position and orientation
                               both rotate around the hinge axis (task Z) so
                               the hook tracks the handle throughout the sweep.
                               Deterministic and doesn't require force sensing.

                               FORCE MODE (fallback, no hinge needed):
                               ``forceMode`` with a commanded pull force along
                               the door's initial tangent direction. The arm is
                               compliant in both X and Y of the pull frame so
                               it can follow the arc passively. Exits when TCP
                               has moved past a position threshold OR time cap.

    4. Release    (moveL + gripper)
                               Open gripper (retract hook), lift to transit Z.

forceMode notes (phase 3 fallback):
  - ``rtde_c.forceModeStop()`` MUST fire before returning, even on exception —
    dangling force mode leaves the next RTDE call in an unpredictable state.
    This module wraps phase 3 in try/finally.
  - selection_vector [1, 1, 0, 0, 0, 0]: X and Y are force-controlled (Y force
    = 0 N, so the arm drifts freely along the arc), Z/Rx/Ry/Rz are
    position-controlled. This allows arc following without needing hinge geometry.
  - Force magnitude kept low (~20 N) so a hook slip doesn't pull hard.
    Tune based on the door's resistance.
  - Position threshold + time cap catch both "door reached stop" and
    "hook slipped / door got stuck."
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import List, Optional

import numpy as np
from scipy.spatial.transform import Rotation as ScipyRotation

from ..arm import ArmHandle
from ..config import DEFAULT, PickPlaceConfig
from ..moves import approach_to, lift_to_transit, retract_to, transit_xy
from ..reachability import is_task_pose_reachable
from ..util.poses import Pose, pose_at_altitude
from ..util.rotations import Rotation
from ..util.rtde_convert import pose_to_rtde, rtde_to_pose


@dataclass
class MicrowaveDoorSpec:
    """Hand-measured door / handle geometry.

    All poses and positions in task frame."""

    handle_engage_pose_task: Pose
    """Task-frame pose the TCP adopts when the hook is engaged under the
    door handle, before pulling. Hook pointing into the handle from
    below; tool orientation chosen so the hook latches when extended."""

    pre_engage_joints_rad: Optional[List[float]] = None
    """Recorded joint angles (6-element list, radians) at the pre-grasp
    waypoint. When set, phase 1 uses ``moveJ`` to this configuration
    instead of the ``lift → transit_xy → approach`` Cartesian chain.

    This avoids the wrist over-rotation / singularity that happens when
    ``moveL`` has to simultaneously move XY and rotate the end effector
    from its starting orientation (e.g. HOME) to the pre-engage
    orientation — a large orientation change in a single Cartesian move
    spins the wrist excessively. ``moveJ`` goes directly to the correct
    arm configuration in joint space with no orientation interpolation.

    Set from the ``joints_rad`` field of the recorded waypoint snapshot."""

    joint_speed: float = 0.5
    """Joint speed (rad/s) for the ``moveJ`` approach. Conservative default;
    speed up once the joint path is verified collision-free."""

    joint_accel: float = 0.3
    """Joint acceleration (rad/s²) for the ``moveJ`` approach."""

    pre_engage_pose_task: Optional[Pose] = None
    """Full task-frame Pose for the pre-engagement waypoint — both
    translation AND rotation from the recorded pre-grasp snapshot.
    Used only in the Cartesian fallback (when ``pre_engage_joints_rad``
    is None). If also None, falls back to engage pose + offset."""

    pre_engage_offset: np.ndarray = field(
        default_factory=lambda: np.array([0.0, 0.0, -0.03])
    )
    """Last-resort fallback: task-frame translation from engage to
    pre-engage, reusing the engage rotation. Prefer
    ``pre_engage_joints_rad`` + ``pre_engage_pose_task`` instead."""

    pull_direction_task: np.ndarray = field(
        default_factory=lambda: np.array([-1.0, -1.0, 0.0])
    )
    """Unit vector in task frame pointing along the INITIAL door-pull
    direction (tangent to the door's arc at the start of the swing).
    Default: -y (away from microwave, toward operator)."""

    # --- Arc mode (preferred) ---

    hinge_position_task: Optional[np.ndarray] = None
    """Position of the door hinge (rotation axis) in task frame.
    When set, phase 3 uses a series of moveL arc waypoints instead of
    force mode. Compute from handle position + door width:
        hinge = handle_translation + np.array([-door_width, 0, 0])
    (adjust sign based on which side the hinge is on)."""

    arc_open_angle_rad: float = 1.2
    """Total door opening angle (rad) for arc mode. 1.2 rad ≈ 69°.
    Adjust until the door is visually fully open."""

    n_arc_steps: int = 10
    """Number of intermediate moveL waypoints along the arc. More steps =
    smoother motion but more RTDE round-trips. 8 is a good start."""

    # --- Force mode (fallback when hinge_position_task is None) ---

    pull_force_n: float = 20.0
    """Force magnitude (N) commanded during force-mode pull. Tune so the
    door opens without slipping the hook."""

    pull_distance_task: float = 0.25
    """Target TCP displacement (m) along ``pull_direction_task`` before
    we consider the door 'open' in force mode."""

    pull_speed_limit: float = 0.05
    """Max TCP speed (m/s) during force mode. Low to avoid yanking."""

    pull_timeout_s: float = 5.0
    """Hard cap on force-mode phase 3. Stops even if the position
    threshold wasn't met (e.g., hook slipped, door stuck)."""


@dataclass
class OpenMicrowaveResult:
    success: bool
    reason: Optional[str] = None
    door_opened_distance: float = 0.0
    """How far (m) the TCP moved — arc length in arc mode, projected
    displacement in force mode."""


# -----------------------------------------------------------------------
#  Arc-mode helpers
# -----------------------------------------------------------------------

def _arc_waypoints(door: MicrowaveDoorSpec) -> List[Pose]:
    """Generate task-frame TCP poses along the door's opening arc.

    The door rotates about the hinge axis (task Z). Each waypoint:
      - Rotates the handle position around the hinge by a small angle increment.
      - Rotates the TCP orientation by the same angle so the hook stays
        aligned with the handle throughout the sweep.

    Rotation sign is chosen so the initial arc tangent aligns with
    ``pull_direction_task``.
    """
    engage = door.handle_engage_pose_task
    hinge = door.hinge_position_task

    r_vec = engage.translation - hinge  # hinge → handle, task frame

    # Determine whether the door swings CCW (+) or CW (-) around task Z.
    # CCW tangent at θ=0: [-r_y, r_x, 0].  Pick the sign that aligns
    # this tangent with the pull direction.
    ccw_tangent_2d = np.array([r_vec[1], -r_vec[0]])
    pull_2d = door.pull_direction_task[:2]
    # Flip sign logic: if the CCW tangent aligns with the pull direction, we need a CW (negative) rotation.
    angle_sign = -1.0 if np.dot(ccw_tangent_2d, pull_2d) > 0.0 else +1.0

    total_angle = angle_sign * door.arc_open_angle_rad
    angles = np.linspace(0.0, total_angle, door.n_arc_steps + 1)[1:]

    waypoints: List[Pose] = []
    for theta in angles:
        c, s = np.cos(theta), np.sin(theta)
        # Rotate r_vec around task Z by theta.
        new_r = np.array([
            c * r_vec[0] - s * r_vec[1],
            s * r_vec[0] + c * r_vec[1],
            r_vec[2],
        ])
        new_translation = hinge + new_r

        # Rotate TCP orientation by the same angle around task Z.
        dR = Rotation.from_rotvec(np.array([0.0, 0.0, theta]))
        new_rotation = dR * engage.rotation

        waypoints.append(Pose(translation=new_translation, rotation=new_rotation))

    return waypoints


# -----------------------------------------------------------------------
#  Force-mode helpers (used only when hinge_position_task is None)
# -----------------------------------------------------------------------

def _build_task_frame_for_pull(
    engage_pose_task: Pose,
    pull_direction_task: np.ndarray,
    arm: ArmHandle,
) -> List[float]:
    """Build the rtde_c.forceMode ``task_frame`` argument.

    The task frame is a 6-vector [x,y,z, rx,ry,rz] in base frame coords
    (axis-angle rotation). We want its +x axis aligned with the desired
    pull direction, origin at the current TCP. Position doesn't matter
    mechanically (force mode uses the frame's orientation) but we set
    it to the TCP position for clarity in logs.
    """
    pull_dir_base = arm.X_base_task.rotation.apply(pull_direction_task)
    pull_dir_base = pull_dir_base / np.linalg.norm(pull_dir_base)

    world_up_base = np.array([0.0, 0.0, 1.0])
    if abs(np.dot(pull_dir_base, world_up_base)) > 0.95:
        world_up_base = np.array([0.0, 1.0, 0.0])
    z_axis = world_up_base - pull_dir_base * np.dot(pull_dir_base, world_up_base)
    z_axis = z_axis / np.linalg.norm(z_axis)
    y_axis = np.cross(z_axis, pull_dir_base)

    R = np.column_stack([pull_dir_base, y_axis, z_axis])
    rotvec = ScipyRotation.from_matrix(R).as_rotvec()

    tcp_base = rtde_to_pose(arm.receive.getActualTCPPose())

    return [
        tcp_base.translation[0],
        tcp_base.translation[1],
        tcp_base.translation[2],
        float(rotvec[0]),
        float(rotvec[1]),
        float(rotvec[2]),
    ]


# -----------------------------------------------------------------------
#  Primitive
# -----------------------------------------------------------------------

def open_microwave_door(
    arm: ArmHandle,
    door: MicrowaveDoorSpec,
    config: PickPlaceConfig = DEFAULT,
) -> OpenMicrowaveResult:
    """Open the microwave door end-to-end.

    Phase 3 automatically uses arc-waypoint mode when
    ``door.hinge_position_task`` is set, falling back to force mode
    otherwise. See module docstring for the full four-phase breakdown.

    Assumes ``arm`` has a ``HookGripper`` attached.
    """
    if config.transit_z is None:
        raise ValueError("config.transit_z is unset.")
    if arm.gripper is None:
        raise ValueError(f"arm {arm.name!r} has no gripper attached.")

    engage_pose = door.handle_engage_pose_task

    # Use the full recorded pre-grasp Pose when available — this preserves
    # the exact hook orientation from the recording, which differs from the
    # engage orientation due to the hook gripper's R_y(π/2) TCP offset.
    if door.pre_engage_pose_task is not None:
        pre_engage_pose = door.pre_engage_pose_task
    else:
        pre_engage_pose = Pose(
            translation=engage_pose.translation + door.pre_engage_offset,
            rotation=engage_pose.rotation,
        )

    # Pre-flight reachability (only for the Cartesian fallback path).
    if door.pre_engage_joints_rad is None:
        for label, pose in [("pre_engage", pre_engage_pose), ("engage", engage_pose)]:
            if is_task_pose_reachable(arm, pose) is None:
                return OpenMicrowaveResult(
                    success=False,
                    reason=f"waypoint '{label}' is kinematically infeasible.",
                )

    # --- Phase 1: approach ---
    if door.pre_engage_joints_rad is not None:
        # Joint-space approach: go directly to the recorded pre-grasp joint
        # configuration. Avoids the wrist over-rotation that happens when a
        # single moveL tries to simultaneously move XY and rotate the end
        # effector from its starting orientation to the pre-engage orientation.
        arm.control.moveJ(
            list(door.pre_engage_joints_rad),
            door.joint_speed,
            door.joint_accel,
        )
    else:
        # Cartesian fallback: lift → transit at safe altitude → descend.
        # May cause wrist spin if the starting orientation is far from the
        # pre-engage orientation. Prefer providing pre_engage_joints_rad.
        lift_to_transit(arm, config.transit_z, config.transit_speed, config.transit_accel)
        transit_xy(arm, pre_engage_pose, config.transit_z,
                   config.transit_speed, config.transit_accel)
        approach_to(arm, pre_engage_pose, config.approach_speed, config.approach_accel)

    # --- Phase 2: engage (short moveL slide to seat hook, then latch) ---
    approach_to(arm, engage_pose, config.approach_speed, config.approach_accel)
    arm.gripper.close()

    # --- Phase 3: pull open ---
    if door.hinge_position_task is not None:
        distance_moved = _phase3_arc(arm, door, config)
        final_pose_task = _arc_waypoints(door)[-1]
    else:
        distance_moved = _phase3_force(arm, door, engage_pose)
        final_pose_task = None

    # --- Phase 4: release and retract ---
    arm.gripper.open()

    if final_pose_task is not None:
        # Arc mode: lift straight up from wherever the door ended.
        lift_to_transit(
            arm, config.transit_z, config.retract_speed, config.retract_accel
        )
    else:
        # Force mode: back to pre-engage, then up.
        retract_to(arm, pre_engage_pose, config.retract_speed, config.retract_accel)
        retract_to(
            arm,
            pose_at_altitude(pre_engage_pose, config.transit_z),
            config.retract_speed, config.retract_accel,
        )

    if door.hinge_position_task is not None:
        # Arc: distance is the arc length.
        r_vec = engage_pose.translation - door.hinge_position_task
        radius = float(np.linalg.norm(r_vec[:2]))
        arc_length = radius * door.arc_open_angle_rad
        opened_ok = True
        return OpenMicrowaveResult(
            success=opened_ok,
            door_opened_distance=arc_length,
        )
    else:
        opened_ok = distance_moved >= 0.8 * door.pull_distance_task
        return OpenMicrowaveResult(
            success=opened_ok,
            reason=None if opened_ok else "door did not reach target pull distance",
            door_opened_distance=distance_moved,
        )


# -----------------------------------------------------------------------
#  Phase 3 implementations (split out to keep open_microwave_door readable)
# -----------------------------------------------------------------------

def _phase3_arc(
    arm: ArmHandle,
    door: MicrowaveDoorSpec,
    config: PickPlaceConfig,
) -> float:
    """Phase 3 (arc mode): moveL through computed arc waypoints.

    Returns the arc length traveled (m).
    """
    waypoints = _arc_waypoints(door)
    for wp in waypoints:
        arm.control.moveL(
            pose_to_rtde(arm.to_base(wp)),
            config.approach_speed,
            config.approach_accel,
        )
    r_vec = door.handle_engage_pose_task.translation - door.hinge_position_task
    return float(np.linalg.norm(r_vec[:2])) * door.arc_open_angle_rad


def _phase3_force(
    arm: ArmHandle,
    door: MicrowaveDoorSpec,
    engage_pose_task: Pose,
) -> float:
    """Phase 3 (force mode): compliant pull along pull_direction_task.

    Compliant in both X and Y of the pull frame (selection_vector
    [1, 1, 0, 0, 0, 0]) so the arm can passively follow the door's arc
    without fighting the perpendicular motion.

    Returns the projected distance traveled along pull_direction_task (m).
    """
    start_tcp_base = rtde_to_pose(arm.receive.getActualTCPPose())
    task_frame = _build_task_frame_for_pull(engage_pose_task, door.pull_direction_task, arm)

    # Compliant in X (pull direction) and Y (arc drift), position-controlled
    # in Z and all rotations.
    selection_vector = [1, 1, 0, 0, 0, 0]
    wrench = [door.pull_force_n, 0.0, 0.0, 0.0, 0.0, 0.0]
    limits = [door.pull_speed_limit] * 3 + [0.5] * 3
    FORCE_MODE_TYPE = 2

    distance_moved = 0.0
    try:
        arm.control.forceMode(task_frame, selection_vector, wrench,
                              FORCE_MODE_TYPE, limits)
        t_start = time.time()
        while (time.time() - t_start) < door.pull_timeout_s:
            now_tcp = rtde_to_pose(arm.receive.getActualTCPPose())
            delta_base = now_tcp.translation - start_tcp_base.translation
            pull_base = arm.X_base_task.rotation.apply(door.pull_direction_task)
            pull_base = pull_base / np.linalg.norm(pull_base)
            distance_moved = float(np.dot(delta_base, pull_base))
            if distance_moved >= door.pull_distance_task:
                break
            time.sleep(0.01)
    finally:
        arm.control.forceModeStop()

    return distance_moved
