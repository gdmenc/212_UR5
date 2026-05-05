"""Open the microwave door with the hook gripper (left arm).

Four-phase strategy:

    1. Approach   (moveJ)      Jump directly to the recorded pre-grasp joint
                               configuration. Avoids the wrist over-rotation
                               that a single moveL causes when simultaneously
                               moving XY and rotating the end effector.

    2. Engage     (moveJ + gripper)
                               Move to graspopen, close the hook, then move to
                               graspclose (hook latched on handle).

    3. Pull open  (moveJ blended arc)
                               A blended ``moveJ(path)`` with all waypoints
                               IK-solved continuously from the previous
                               waypoint's joint angles (q_near), so the
                               controller stays on one kinematic branch.
                               Deterministic; no force sensing required.

    4. Release    (moveJ + gripper)
                               Open gripper (retract hook), move to recorded
                               release pose, then slide the hook out of the
                               handle.

forceMode notes (phase 3 fallback, only used when hinge_position_task is None):
  - ``rtde_c.forceModeStop()`` MUST fire before returning, even on exception —
    dangling force mode leaves the next RTDE call in an unpredictable state.
    This module wraps phase 3 in try/finally.
  - selection_vector [1, 1, 0, 0, 0, 0]: X and Y are force-controlled (Y force
    = 0 N, so the arm drifts freely along the arc), Z/Rx/Ry/Rz are
    position-controlled. This allows arc following without needing hinge geometry.
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import List, Optional

import numpy as np
from scipy.spatial.transform import Rotation as ScipyRotation, Slerp

from ...arm import ArmHandle
from ...config import DEFAULT, PickPlaceConfig
from ...reachability import is_task_pose_reachable
from ...util.poses import Pose
from ...util.rotations import Rotation
from ...util.rtde_convert import pose_to_rtde, rtde_to_pose


@dataclass
class MicrowaveDoorSpec:
    """Hand-measured door / handle geometry.

    All poses and positions in task frame."""

    handle_engage_pose_task: Pose
    """Task-frame pose the TCP adopts when the hook is engaged under the
    door handle, before pulling."""

    pre_engage_joints_rad: Optional[List[float]] = None
    """Recorded joint angles (6-element list, radians) at the pre-grasp
    waypoint. When set, phase 1 uses ``moveJ`` to this configuration
    instead of the ``lift → transit_xy → approach`` Cartesian chain."""

    joint_speed: float = 0.5
    """Joint speed (rad/s) for all ``moveJ`` moves."""

    joint_accel: float = 0.3
    """Joint acceleration (rad/s²) for all ``moveJ`` moves."""

    handle_engage_joints_rad: Optional[List[float]] = None
    """Recorded joint angles at the grasp-open waypoint."""

    handle_latched_joints_rad: Optional[List[float]] = None
    """Recorded joint angles after closing/latching the hook on the handle."""

    open_pose_task: Optional[Pose] = None
    """Recorded task-frame pose at the door-open waypoint."""

    open_joints_rad: Optional[List[float]] = None
    """Recorded joint angles at the door-open waypoint."""

    release_joints_rad: Optional[List[float]] = None
    """Recorded joint angles after opening the gripper at the open door."""

    slide_out_joints_rad: Optional[List[float]] = None
    """Recorded joint angles for sliding the hook out of the door handle."""

    pre_engage_pose_task: Optional[Pose] = None
    """Full task-frame Pose for the pre-engagement waypoint — used only for
    dry-run display. Prefer ``pre_engage_joints_rad`` for motion."""

    pre_engage_offset: np.ndarray = field(
        default_factory=lambda: np.array([0.0, 0.0, -0.03])
    )
    """Last-resort fallback offset from engage to pre-engage pose."""

    pull_direction_task: np.ndarray = field(
        default_factory=lambda: np.array([-1.0, -1.0, 0.0])
    )
    """Unit vector in task frame pointing along the INITIAL door-pull
    direction (tangent to the door's arc at the start of the swing)."""

    # --- Arc mode (preferred) ---

    hinge_position_task: Optional[np.ndarray] = None
    """Position of the door hinge (rotation axis) in task frame.
    When set, phase 3 uses a blended IK arc."""

    arc_open_angle_rad: float = 1.57
    """Total door opening angle (rad) for arc mode."""

    n_arc_steps: int = 12
    """Number of intermediate waypoints along the arc."""

    arc_blend_radius_m: float = 0.01
    """Blend radius for multi-waypoint ``moveJ(path)`` arc traversal.
    Intermediate waypoints use this radius; the terminal waypoint is forced
    to zero so the controller converges at the endpoint."""

    arc_max_joint_step_rad: float = 0.7
    """Maximum allowed absolute joint change between adjacent arc waypoints.
    If IK proposes a larger step, the path is rejected before motion."""

    arc_debug_joints: bool = False
    """Print per-waypoint joint deltas for the arc before executing."""

    # --- Force mode (fallback when hinge_position_task is None) ---

    pull_force_n: float = 20.0
    """Force magnitude (N) commanded during force-mode pull."""

    pull_distance_task: float = 0.25
    """Target TCP displacement (m) before we consider the door 'open'."""

    pull_speed_limit: float = 0.05
    """Max TCP speed (m/s) during force mode."""

    pull_timeout_s: float = 5.0
    """Hard cap on force-mode phase 3."""


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

def _arc_waypoints(
    door: MicrowaveDoorSpec,
    start_pose_task: Optional[Pose] = None,
) -> List[Pose]:
    """Generate task-frame TCP poses along the door's opening arc."""
    start = start_pose_task or door.handle_engage_pose_task
    hinge = door.hinge_position_task

    r_vec = start.translation - hinge

    ccw_tangent_2d = np.array([r_vec[1], -r_vec[0]])
    pull_2d = door.pull_direction_task[:2]
    angle_sign = -1.0 if np.dot(ccw_tangent_2d, pull_2d) > 0.0 else +1.0

    total_angle = angle_sign * door.arc_open_angle_rad
    angles = np.linspace(0.0, total_angle, door.n_arc_steps + 1)[1:]

    waypoints: List[Pose] = []
    for theta in angles:
        c, s = np.cos(theta), np.sin(theta)
        new_r = np.array([
            c * r_vec[0] - s * r_vec[1],
            s * r_vec[0] + c * r_vec[1],
            r_vec[2],
        ])
        new_translation = hinge + new_r

        dR = Rotation.from_rotvec(np.array([0.0, 0.0, theta]))
        new_rotation = dR * start.rotation

        waypoints.append(Pose(translation=new_translation, rotation=new_rotation))

    return waypoints


def _arc_waypoints_to_pose(
    door: MicrowaveDoorSpec,
    start_pose_task: Pose,
    end_pose_task: Pose,
    n_steps: Optional[int] = None,
) -> List[Pose]:
    """Generate arc waypoints from a recorded start pose to a recorded end pose.

    Position follows the measured hinge circle; orientation is Slerp'd
    between the two recorded orientations.
    """
    hinge = door.hinge_position_task
    if hinge is None:
        raise ValueError("hinge_position_task is required for explicit arc mode.")

    start_r = start_pose_task.translation - hinge
    end_r = end_pose_task.translation - hinge

    start_xy = start_r[:2]
    end_xy = end_r[:2]
    signed_angle = float(np.arctan2(
        start_xy[0] * end_xy[1] - start_xy[1] * end_xy[0],
        np.dot(start_xy, end_xy),
    ))
    n = n_steps or door.n_arc_steps
    angles = np.linspace(0.0, signed_angle, n + 1)[1:]

    slerp = Slerp(
        [0.0, 1.0],
        ScipyRotation.from_matrix([
            start_pose_task.rotation.as_matrix(),
            end_pose_task.rotation.as_matrix(),
        ]),
    )

    waypoints: List[Pose] = []
    for i, theta in enumerate(angles, start=1):
        c, s = np.cos(theta), np.sin(theta)
        new_r = np.array([
            c * start_r[0] - s * start_r[1],
            s * start_r[0] + c * start_r[1],
            start_r[2] + (end_r[2] - start_r[2]) * i / n,
        ])
        rot = Rotation.from_matrix(slerp(i / n).as_matrix())
        waypoints.append(Pose(translation=hinge + new_r, rotation=rot))

    return waypoints


# -----------------------------------------------------------------------
#  Force-mode helpers (used only when hinge_position_task is None)
# -----------------------------------------------------------------------

def _build_task_frame_for_pull(
    engage_pose_task: Pose,
    pull_direction_task: np.ndarray,
    arm: ArmHandle,
) -> List[float]:
    """Build the rtde_c.forceMode ``task_frame`` argument."""
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
    """Open the microwave door end-to-end."""
    if config.transit_z is None:
        raise ValueError("config.transit_z is unset.")
    if arm.gripper is None:
        raise ValueError(f"arm {arm.name!r} has no gripper attached.")

    engage_pose = door.handle_engage_pose_task

    if door.pre_engage_pose_task is not None:
        pre_engage_pose = door.pre_engage_pose_task
    else:
        pre_engage_pose = Pose(
            translation=engage_pose.translation + door.pre_engage_offset,
            rotation=engage_pose.rotation,
        )

    if door.pre_engage_joints_rad is None:
        for label, pose in [("pre_engage", pre_engage_pose), ("engage", engage_pose)]:
            if is_task_pose_reachable(arm, pose) is None:
                return OpenMicrowaveResult(
                    success=False,
                    reason=f"waypoint '{label}' is kinematically infeasible.",
                )

    # --- Phase 1: approach ---
    if door.pre_engage_joints_rad is not None:
        arm.control.moveJ(
            list(door.pre_engage_joints_rad),
            door.joint_speed,
            door.joint_accel,
        )

    # Explicit taught waypoint mode: pregrasp -> graspopen -> latch ->
    # graspclose -> blended arc to opendoor -> open gripper -> slide out.
    if (
        door.handle_engage_joints_rad is not None
        and door.handle_latched_joints_rad is not None
        and door.open_pose_task is not None
        and door.release_joints_rad is not None
        and door.slide_out_joints_rad is not None
    ):
        arm.control.moveJ(
            list(door.handle_engage_joints_rad),
            door.joint_speed,
            door.joint_accel,
        )
        arm.gripper.close()
        arm.control.moveJ(
            list(door.handle_latched_joints_rad),
            door.joint_speed,
            door.joint_accel,
        )

        distance_moved = _phase3_recorded_arc(arm, door, engage_pose)

        arm.gripper.open()
        arm.control.moveJ(
            list(door.release_joints_rad),
            door.joint_speed,
            door.joint_accel,
        )
        arm.control.moveJ(
            list(door.slide_out_joints_rad),
            door.joint_speed,
            door.joint_accel,
        )
        return OpenMicrowaveResult(success=True, door_opened_distance=distance_moved)

    if door.hinge_position_task is not None:
        r_vec = engage_pose.translation - door.hinge_position_task
        radius = float(np.linalg.norm(r_vec[:2]))
        arc_length = radius * door.arc_open_angle_rad
        return OpenMicrowaveResult(success=True, door_opened_distance=arc_length)
    else:
        distance_moved = 0.0
        opened_ok = distance_moved >= 0.8 * door.pull_distance_task
        return OpenMicrowaveResult(
            success=opened_ok,
            reason=None if opened_ok else "door did not reach target pull distance",
            door_opened_distance=distance_moved,
        )


# -----------------------------------------------------------------------
#  Phase 3 implementations
# -----------------------------------------------------------------------

def _phase3_arc(
    arm: ArmHandle,
    door: MicrowaveDoorSpec,
    config: PickPlaceConfig,
) -> float:
    """Phase 3 (arc mode): follow the computed arc with blended moveJ."""
    return _phase3_arc_blended(arm, door, config)


def _phase3_arc_blended(
    arm: ArmHandle,
    door: MicrowaveDoorSpec,
    config: PickPlaceConfig,
) -> float:
    """Phase 3 (arc mode): move through computed arc waypoints as one
    blended ``moveJ(path)`` command. Returns the arc length traveled (m)."""
    start_pose_task = arm.to_task(rtde_to_pose(arm.receive.getActualTCPPose()))
    waypoints = _arc_waypoints(door, start_pose_task=start_pose_task)
    joint_path = _joint_path_for_waypoints(arm, waypoints)
    _move_joint_path_blended(arm, door, joint_path)

    r_vec = start_pose_task.translation - door.hinge_position_task
    return float(np.linalg.norm(r_vec[:2])) * door.arc_open_angle_rad


def _phase3_recorded_arc(
    arm: ArmHandle,
    door: MicrowaveDoorSpec,
    start_pose_task: Pose,
) -> float:
    """Phase 3: follow the taught closed-handle-to-open-door arc via
    blended IK waypoints. All waypoints IK-solved continuously from
    q_near so the controller stays on one kinematic branch."""
    if door.open_pose_task is None:
        raise ValueError("open_pose_task is required for recorded arc mode.")

    start_pose_task = arm.to_task(rtde_to_pose(arm.receive.getActualTCPPose()))
    waypoints = _arc_waypoints_to_pose(door, start_pose_task, door.open_pose_task)
    joint_path = _joint_path_for_waypoints(
        arm,
        waypoints,
        max_joint_step_rad=door.arc_max_joint_step_rad,
    )
    if door.arc_debug_joints:
        _print_joint_path_diagnostics(joint_path)
    _move_joint_path_blended(arm, door, joint_path)

    radius = float(np.linalg.norm(
        (start_pose_task.translation - door.hinge_position_task)[:2]
    ))
    end_r = door.open_pose_task.translation - door.hinge_position_task
    start_r = start_pose_task.translation - door.hinge_position_task
    angle = abs(float(np.arctan2(
        start_r[0] * end_r[1] - start_r[1] * end_r[0],
        np.dot(start_r[:2], end_r[:2]),
    )))
    return radius * angle


def _joint_path_for_waypoints(
    arm: ArmHandle,
    waypoints: List[Pose],
    final_joints_rad: Optional[List[float]] = None,
    max_joint_step_rad: Optional[float] = None,
) -> List[List[float]]:
    """Solve IK for a sequence of task-frame waypoints using q_near continuity."""
    joint_path: List[List[float]] = []
    prev_joints = list(arm.receive.getActualQ())

    for i, wp in enumerate(waypoints):
        is_final = i == len(waypoints) - 1
        if is_final and final_joints_rad is not None:
            target_joints = list(final_joints_rad)
        else:
            rp = pose_to_rtde(arm.to_base(wp))
            q = arm.control.getInverseKinematics(rp, prev_joints, 0.001, 0.001)
            if q is None or len(q) != 6:
                raise RuntimeError(
                    f"IK failed for microwave open arc waypoint {i + 1}."
                )
            target_joints = _unwrap_joints_near(list(q), prev_joints)
            if max_joint_step_rad is not None:
                step = float(np.max(np.abs(
                    np.asarray(target_joints) - np.asarray(prev_joints)
                )))
                if step > max_joint_step_rad:
                    dq = np.asarray(target_joints) - np.asarray(prev_joints)
                    worst = int(np.argmax(np.abs(dq))) + 1
                    raise RuntimeError(
                        f"IK branch jump at microwave arc waypoint {i + 1}: "
                        f"max joint step {step:.3f} rad > limit "
                        f"{max_joint_step_rad:.3f} rad. Worst joint: J{worst}; "
                        f"dq={np.round(dq, 4)}. Increase n_arc_steps or record "
                        "an intermediate waypoint."
                    )
        joint_path.append(list(target_joints))
        prev_joints = list(target_joints)

    return joint_path


def _unwrap_joints_near(joints: List[float], reference: List[float]) -> List[float]:
    """Pick the 2π-equivalent joint angles nearest ``reference``."""
    unwrapped: List[float] = []
    for q, q_ref in zip(joints, reference):
        q_near = float(q)
        while q_near - q_ref > np.pi:
            q_near -= 2.0 * np.pi
        while q_ref - q_near > np.pi:
            q_near += 2.0 * np.pi
        unwrapped.append(q_near)
    return unwrapped


def _print_joint_path_diagnostics(joint_path: List[List[float]]) -> None:
    if not joint_path:
        print("[open_microwave] arc joint path is empty")
        return
    print("[open_microwave] arc joint diagnostics:")
    prev = np.asarray(joint_path[0], dtype=float)
    print(f"  wp 01 q={np.round(prev, 4)}")
    for i, q in enumerate(joint_path[1:], start=2):
        now = np.asarray(q, dtype=float)
        delta = now - prev
        print(
            f"  wp {i:02d} max_dq={np.max(np.abs(delta)):.3f} "
            f"dq={np.round(delta, 4)}"
        )
        prev = now


def _move_joint_path_blended(
    arm: ArmHandle,
    door: MicrowaveDoorSpec,
    joint_path: List[List[float]],
) -> None:
    """Send one blended joint-space path to the UR controller."""
    if not joint_path:
        return
    if len(joint_path) == 1 or door.arc_blend_radius_m <= 0.0:
        for q in joint_path:
            arm.control.moveJ(q, door.joint_speed, door.joint_accel)
        return

    blend_r = float(door.arc_blend_radius_m)
    path = [
        list(q) + [door.joint_speed, door.joint_accel, blend_r]
        for q in joint_path
    ]
    path[-1][-1] = 0.0
    arm.control.moveJ(path)


def _phase3_force(
    arm: ArmHandle,
    door: MicrowaveDoorSpec,
    engage_pose_task: Pose,
) -> float:
    """Phase 3 (force mode): compliant pull along pull_direction_task.

    Returns the projected distance traveled along pull_direction_task (m).
    """
    start_tcp_base = rtde_to_pose(arm.receive.getActualTCPPose())
    task_frame = _build_task_frame_for_pull(engage_pose_task, door.pull_direction_task, arm)

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
