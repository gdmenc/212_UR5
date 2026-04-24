"""UR5e reachability: given a task-frame pose, can the arm reach it?

What this module does
---------------------
  - Forward kinematics using the UR5e standard DH parameters from the
    Universal Robots data sheet. Hand-coded so we don't need a URDF
    parser or ``pydrake`` (neither installs cleanly on Intel macOS).
  - Numerical inverse kinematics via ``scipy.optimize.least_squares``.
    Finds ONE solution near a seed — fast (~10-50 ms), zero extra deps,
    and good enough to answer 'is this grasp feasible'.
  - A grasp-selector helper that picks, from a list of candidate
    grasps, the one whose IK solution is nearest (in joint space) to
    the current arm configuration — i.e., the 'most natural' grasp
    for where the arm is right now.

What this module does NOT do
-----------------------------
  - Scene collision checking (arm hitting microwave, table, other arm,
    held object). That's a separate module — would need a scene graph
    + FCL/Drake. Not in scope here.
  - Path feasibility. We check endpoints (grasp + pregrasp), not
    every intermediate pose along a moveL. A pose pair can be
    individually reachable but pass through a singularity during
    interpolation. Catch this in testing; add path sampling later if
    it becomes an issue.
  - Multiple-seed IK. Numerical IK finds the solution nearest the
    seed. For a UR5e, there are up to 8 analytical solutions; we only
    find one. Usually fine, occasionally misses feasible alternatives.
    Mitigation: if an 'unreachable' result feels wrong, call
    ``inverse_kinematics`` with different seeds (e.g. elbow-up vs
    elbow-down) and union the results.

Frame convention
----------------
  - FK/IK work in the UR controller's base frame (the frame RTDE speaks).
    That's the same frame ``X_base_task`` is expressed in.
  - Task-frame poses go through ``arm.X_base_task`` before reaching
    IK; users pass task-frame poses to the public helpers.

TCP offset
----------
  - We bake the TCP offset into forward kinematics as a pure translation
    along the flange's Z axis. This matches what ``rtde_c.setTcp([0, 0,
    0.174, 0, 0, 0])`` does for the Robotiq 2F-85. If the tool offset
    ever includes a rotation or non-Z translation, generalise the
    ``tcp_offset_z`` argument into a full transform.

DH sources
----------
  UR5e standard DH from UR's published kinematics:
    https://www.universal-robots.com/articles/ur/application-installation/
    dh-parameters-for-calculations-of-kinematics-and-dynamics/
"""

from __future__ import annotations

from typing import List, Optional, Tuple

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

from .arm import ArmHandle
from .grasps.base import Grasp
from .util.poses import Pose, offset_along_tool_z


# =========================================================================
#  UR5e kinematic model
# =========================================================================

UR5E_DH: np.ndarray = np.array([
    # [a (m),    d (m),    alpha (rad),   theta_offset (rad)]
    [0.0,        0.1625,   np.pi / 2,     0.0],   # J1 shoulder_pan
    [-0.425,     0.0,      0.0,           0.0],   # J2 shoulder_lift
    [-0.3922,    0.0,      0.0,           0.0],   # J3 elbow
    [0.0,        0.1333,   np.pi / 2,     0.0],   # J4 wrist_1
    [0.0,        0.0997,  -np.pi / 2,     0.0],   # J5 wrist_2
    [0.0,        0.0996,   0.0,           0.0],   # J6 wrist_3
])

UR5E_JOINT_LIMITS_RAD: np.ndarray = np.array([
    [-2 * np.pi, 2 * np.pi],   # J1 (UR5e allows full rotation)
    [-np.pi,      np.pi],      # J2 shoulder_lift — limited to ±180°
    [-np.pi,      np.pi],      # J3 elbow — mechanically ±180°
    [-2 * np.pi, 2 * np.pi],   # J4 wrist_1
    [-2 * np.pi, 2 * np.pi],   # J5 wrist_2
    [-2 * np.pi, 2 * np.pi],   # J6 wrist_3
])
"""Nominal joint limits (radians). UR5e actually allows more range on most
joints; these are conservative values used in UR's default safety
configuration. If a grasp feels like it should be reachable but IK
rejects it, widening J2/J3 is the first thing to try."""

UR5E_MAX_REACH = 0.85
"""Maximum distance (m) from the shoulder joint to the TCP. Datasheet
value; used as a quick pre-IK filter."""

UR5E_MIN_REACH = 0.12
"""Approximate minimum reach from the shoulder — at smaller distances
the arm folds into itself. Used as a pre-IK filter; actual minimum
depends on tool length and is enforced implicitly by joint limits."""

UR5E_SHOULDER_OFFSET_FROM_BASE = np.array([0.0, 0.0, 0.1625])
"""Position of the shoulder joint (J2 pivot) in base frame. Used as the
reference point for the reach-sphere pre-filter."""

UR5E_TCP_OFFSET_ROBOTIQ_M = 0.174
"""Default TCP Z offset — matches ``calibration.TCP_OFFSET_ROBOTIQ_2F85``."""

UR5E_READY_Q = np.array([0.0, -np.pi / 2, np.pi / 2, -np.pi / 2, -np.pi / 2, 0.0])
"""A generic 'ready' configuration used as a fallback IK seed when the
caller doesn't have the actual current joint config."""


def dh_matrix(a: float, d: float, alpha: float, theta: float) -> np.ndarray:
    """Standard (distal) DH homogeneous transform from frame i-1 to i.

    T = Rz(theta) · Tz(d) · Tx(a) · Rx(alpha)
    """
    ct, st = np.cos(theta), np.sin(theta)
    ca, sa = np.cos(alpha), np.sin(alpha)
    return np.array([
        [ct, -st * ca,  st * sa, a * ct],
        [st,  ct * ca, -ct * sa, a * st],
        [0.0, sa,       ca,      d],
        [0.0, 0.0,      0.0,     1.0],
    ])


def forward_kinematics(
    q: np.ndarray,
    tcp_offset_z: float = UR5E_TCP_OFFSET_ROBOTIQ_M,
) -> np.ndarray:
    """UR5e forward kinematics from base to TCP.

    q: length-6 array of joint angles (rad). Order:
        [shoulder_pan, shoulder_lift, elbow, wrist_1, wrist_2, wrist_3]
    tcp_offset_z: TCP offset along the flange Z axis (m). Default matches
        the Robotiq 2F-85 setTcp in this project.

    Returns 4x4 homogeneous transform of the TCP in base frame.
    """
    q = np.asarray(q, dtype=float).reshape(6)
    T = np.eye(4)
    for i in range(6):
        a, d, alpha, theta_off = UR5E_DH[i]
        T = T @ dh_matrix(a, d, alpha, q[i] + theta_off)
    if tcp_offset_z != 0.0:
        tool = np.eye(4)
        tool[2, 3] = tcp_offset_z
        T = T @ tool
    return T


def inverse_kinematics(
    target_T: np.ndarray,
    q_seed: np.ndarray,
    tcp_offset_z: float = UR5E_TCP_OFFSET_ROBOTIQ_M,
    tolerance: float = 1e-3,
    max_nfev: int = 200,
) -> Optional[np.ndarray]:
    """Numerical IK for UR5e. Returns joint angles (rad) or None if
    unreachable / IK failed to converge.

    target_T: 4x4 homogeneous transform of the DESIRED TCP pose in base frame.
    q_seed: 6-vector starting guess. Pass the current q for 'nearest-
        to-current' solutions; pass UR5E_READY_Q as a neutral fallback.
    tolerance: max allowed residual norm (position err in m + rotvec err
        in rad) for the solution to count as converged.
    max_nfev: hard cap on function evaluations — stops scipy from spending
        forever on infeasible targets.
    """
    q_seed = np.asarray(q_seed, dtype=float).reshape(6)
    target_T = np.asarray(target_T, dtype=float)
    target_R_T = target_T[:3, :3].T
    target_p = target_T[:3, 3]

    def residual(q: np.ndarray) -> np.ndarray:
        T = forward_kinematics(q, tcp_offset_z)
        pos_err = T[:3, 3] - target_p
        R_err = T[:3, :3] @ target_R_T
        rot_err = Rotation.from_matrix(R_err).as_rotvec()
        return np.concatenate([pos_err, rot_err])

    try:
        result = least_squares(
            residual,
            q_seed,
            bounds=(UR5E_JOINT_LIMITS_RAD[:, 0], UR5E_JOINT_LIMITS_RAD[:, 1]),
            max_nfev=max_nfev,
        )
    except Exception:
        return None

    if np.linalg.norm(result.fun) > tolerance:
        return None
    return result.x


# =========================================================================
#  Task-frame public API
# =========================================================================

def _pose_to_matrix(pose: Pose) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3] = pose.rotation.as_matrix()
    T[:3, 3] = pose.translation
    return T


def _tcp_offset_z_from_arm(arm: ArmHandle) -> float:
    if arm.tcp_offset is None:
        return UR5E_TCP_OFFSET_ROBOTIQ_M
    return float(arm.tcp_offset[2])


def is_task_pose_reachable(
    arm: ArmHandle,
    X_task_target: Pose,
    q_seed: Optional[np.ndarray] = None,
) -> Optional[np.ndarray]:
    """Returns the joint configuration to reach ``X_task_target`` with
    the given arm, or None if the pose is infeasible.

    q_seed: if None, uses the arm's current configuration (from
        ``rtde_r.getActualQ``) if available, else UR5E_READY_Q.
    """
    X_base_target = arm.X_base_task @ X_task_target
    target_T = _pose_to_matrix(X_base_target)
    tcp_z = _tcp_offset_z_from_arm(arm)

    if q_seed is None:
        q_seed = _current_q_or_default(arm)

    return inverse_kinematics(target_T, q_seed, tcp_offset_z=tcp_z)


def _current_q_or_default(arm: ArmHandle) -> np.ndarray:
    """Read current joint config via RTDE if possible, else ready pose."""
    try:
        return np.asarray(arm.receive.getActualQ(), dtype=float).reshape(6)
    except Exception:
        return UR5E_READY_Q.copy()


def diagnose_reachability(
    arm: ArmHandle,
    X_task_target: Pose,
    q_seed: Optional[np.ndarray] = None,
) -> str:
    """Human-readable diagnosis of why a task-frame target is or isn't
    reachable. Useful in print-debugging first-time grasp failures."""
    X_base_target = arm.X_base_task @ X_task_target
    pos_base = X_base_target.translation

    dist = float(np.linalg.norm(pos_base - UR5E_SHOULDER_OFFSET_FROM_BASE))
    if dist > UR5E_MAX_REACH:
        return (f"unreachable: TCP target is {dist:.3f} m from shoulder; "
                f"UR5e max reach is {UR5E_MAX_REACH} m. "
                f"Move the object closer or grasp at a smaller rim radius.")
    if dist < UR5E_MIN_REACH:
        return (f"unreachable: TCP target is only {dist:.3f} m from "
                f"shoulder — too close to base ({UR5E_MIN_REACH} m min). "
                f"Arm would have to fold into itself.")

    q = is_task_pose_reachable(arm, X_task_target, q_seed=q_seed)
    if q is not None:
        return (f"reachable at q = "
                f"{np.round(np.degrees(q), 1).tolist()}°  "
                f"(dist from shoulder = {dist:.3f} m)")
    return (f"infeasible orientation / joint-limit: TCP position is in "
            f"range ({dist:.3f} m from shoulder) but the required "
            f"orientation violates joint limits or IK did not converge "
            f"from the seed. Try a different grasp angle.")


# =========================================================================
#  Grasp selection
# =========================================================================

def _joint_distance(q1: np.ndarray, q2: np.ndarray,
                    weights: Optional[np.ndarray] = None) -> float:
    """Weighted L2 distance between joint configs. Default weights
    emphasise proximal joints — shoulder rotation 'costs' more than wrist
    twist because it swings the whole arm through the workspace."""
    if weights is None:
        weights = np.array([3.0, 3.0, 2.0, 1.0, 1.0, 0.5])
    return float(np.sqrt(np.sum(weights * (q1 - q2) ** 2)))


def best_feasible_grasp(
    arm: ArmHandle,
    grasps: List[Grasp],
    current_q: Optional[np.ndarray] = None,
    check_pregrasp: bool = True,
) -> Optional[Tuple[Grasp, np.ndarray]]:
    """Select the 'most natural' grasp from a list of candidates.

    For each candidate, solves IK at the grasp pose (seeded from current
    q). Optionally also solves IK at the pregrasp pose (pass/fail — if
    pregrasp is infeasible, the whole candidate is dropped). Returns the
    feasible candidate with minimum joint-space distance from current q,
    together with the IK solution.

    Returns ``None`` if NO candidate is feasible. Callers should then
    either widen the candidate set (more rim angles, different grasp
    radius) or reposition the object.

    Heuristic: 'minimum joint distance' is a reasonable proxy for
    'natural' because it penalises shoulder rotation and elbow flips
    more than wrist twist. Not guaranteed to pick the absolutely
    optimal grasp — just avoids obviously contorted ones.
    """
    if current_q is None:
        current_q = _current_q_or_default(arm)

    tcp_z = _tcp_offset_z_from_arm(arm)

    best: Optional[Tuple[Grasp, np.ndarray]] = None
    best_cost = np.inf

    for grasp in grasps:
        X_base_grasp = arm.X_base_task @ grasp.grasp_pose
        q_grasp = inverse_kinematics(
            _pose_to_matrix(X_base_grasp),
            q_seed=current_q,
            tcp_offset_z=tcp_z,
        )
        if q_grasp is None:
            continue

        if check_pregrasp:
            pregrasp_pose = offset_along_tool_z(
                grasp.grasp_pose, grasp.pregrasp_offset
            )
            X_base_pregrasp = arm.X_base_task @ pregrasp_pose
            q_pregrasp = inverse_kinematics(
                _pose_to_matrix(X_base_pregrasp),
                q_seed=q_grasp,   # seed from the grasp solution — close by construction
                tcp_offset_z=tcp_z,
            )
            if q_pregrasp is None:
                continue

        cost = _joint_distance(q_grasp, current_q)
        if cost < best_cost:
            best = (grasp, q_grasp)
            best_cost = cost

    return best


def filter_feasible_grasps(
    arm: ArmHandle,
    grasps: List[Grasp],
    current_q: Optional[np.ndarray] = None,
    check_pregrasp: bool = True,
) -> List[Grasp]:
    """All candidates whose grasp (and optionally pregrasp) poses are
    kinematically reachable. Use when you want to hand multiple options
    to the next stage (e.g. a collision-aware picker) rather than
    committing to the lowest-cost one."""
    if current_q is None:
        current_q = _current_q_or_default(arm)

    tcp_z = _tcp_offset_z_from_arm(arm)
    out: List[Grasp] = []

    for grasp in grasps:
        X_base_grasp = arm.X_base_task @ grasp.grasp_pose
        q_grasp = inverse_kinematics(
            _pose_to_matrix(X_base_grasp),
            q_seed=current_q,
            tcp_offset_z=tcp_z,
        )
        if q_grasp is None:
            continue

        if check_pregrasp:
            pregrasp_pose = offset_along_tool_z(
                grasp.grasp_pose, grasp.pregrasp_offset
            )
            X_base_pregrasp = arm.X_base_task @ pregrasp_pose
            if inverse_kinematics(
                _pose_to_matrix(X_base_pregrasp),
                q_seed=q_grasp,
                tcp_offset_z=tcp_z,
            ) is None:
                continue

        out.append(grasp)

    return out
