"""Execute a planned ``TransitPlan`` on a UR via RTDE.

Two execution methods, same ``TransitPlan`` input:

  * ``moveJ_path`` (default) — sparse-sample the trajectory, send as one
    blended ``moveJ(path=[[q, v, a, blend], ...])`` call. One blocking
    RPC, no Python control loop, controller-side blending. Best for
    free-space transits with safety margins.

  * ``servoJ`` — dense-sample the trajectory at fixed dt (8 ms = 125 Hz)
    and stream via ``servoJ``. Faithfully tracks the planned q(t)
    velocity profile. Best for tight-tolerance constraints or when
    you want KTO's smoothness preserved on the wire.

Both methods extract the planning arm's 6 joints from the (full plant)
trajectory before sending — KTO outputs a 12-DOF trajectory with the
partner arm pinned, but the controller only wants this arm's q.
"""

from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Optional

import numpy as np

from ..arm import ArmHandle
from ..session import Session
from ..util.fk_replay import check_blend_radius, print_blend_radius_check
from .transit import TransitPlan


# Defaults for moveJ_path. Lift these into PickPlaceConfig later if useful.
_DEFAULT_SPEED = 1.5     # rad/s — joint speed cap
_DEFAULT_ACCEL = 2.0     # rad/s² — joint acceleration cap


@dataclass
class ExecutionResult:
    """What happened when a plan was executed."""

    success: bool
    method: str
    duration_s: float
    reason: Optional[str] = None


def execute_plan(
    plan: TransitPlan,
    session: Session,
    *,
    method: str = "moveJ_path",
    n_waypoints: int = 20,
    dt: float = 0.008,
    blend_r_m: float = 0.02,
    speed: float = _DEFAULT_SPEED,
    accel: float = _DEFAULT_ACCEL,
    servo_time_scale: float = 1.0,
    blend_check_safety_factor: float = 0.5,
    abort_on_blend_overrun: bool = True,
) -> ExecutionResult:
    """Run a planned trajectory on the chosen arm.

    Parameters
    ----------
    method : 'moveJ_path' | 'servoJ'
        Execution style. See module docstring.
    n_waypoints : int
        Used only for ``moveJ_path``. Number of samples along the
        trajectory; controller blends between them. 20 is a good
        default for short transits; bump to 40+ for tight curves.
    dt : float
        Used only for ``servoJ``. Loop period in seconds (default 8 ms).
    blend_r_m : float
        Used only for ``moveJ_path``. TCP-distance corner blend.
    """
    if plan.arm not in session.arms:
        raise KeyError(f"plan arm {plan.arm!r} not in session.arms")
    arm_handle = session.arms[plan.arm]

    if method == "moveJ_path":
        return _execute_moveJ_path(
            plan, arm_handle, n_waypoints, blend_r_m,
            speed, accel,
            blend_check_safety_factor=blend_check_safety_factor,
            abort_on_overrun=abort_on_blend_overrun,
        )
    elif method == "servoJ":
        return _execute_servoJ(plan, arm_handle, dt, servo_time_scale)
    else:
        raise ValueError(
            f"unknown execution method {method!r}; "
            "use 'moveJ_path' or 'servoJ'."
        )


# ---------------------------------------------------------------------------
#  Helpers: extract per-arm joints from the (full-plant) trajectory
# ---------------------------------------------------------------------------

def _arm_q_at(trajectory, t: float, arm_idx: np.ndarray) -> list:
    """Sample the trajectory at time ``t`` and return the planning arm's
    6-vector as a Python list (RTDE wants a list)."""
    q_full = np.asarray(trajectory.value(t)).flatten()
    return q_full[arm_idx].tolist()


def _planning_arm_indices(plan: TransitPlan) -> np.ndarray:
    """Indices of the planning arm's joints in the full-plant q vector.

    Recovered from ``plan.waypoints_q[0]`` length: if it's 6, the
    trajectory is per-arm (single arm plant). If 12, KTO produced a
    full-plant trajectory and we need to slice — but ``waypoints_q``
    was already sliced to the planning arm by the planner, so the
    trajectory itself is the only place full-plant q lives.
    """
    # Defer to metadata if the planner stored arm_idx; otherwise fall
    # back to assuming the trajectory is full-plant 12-dof and the
    # planning arm sits in the standard slot.
    if "arm_idx" in plan.metadata:
        return np.asarray(plan.metadata["arm_idx"], dtype=int)
    # Heuristic from arm name. Matches add_left_arm() / add_right_arm()
    # call order in build_scene.py.
    if plan.arm == "ur_left":
        return np.arange(0, 6)
    elif plan.arm == "ur_right":
        return np.arange(6, 12)
    raise ValueError(f"can't infer joint indices for arm {plan.arm!r}")


# ---------------------------------------------------------------------------
#  moveJ(path) executor
# ---------------------------------------------------------------------------

def _execute_moveJ_path(
    plan: TransitPlan,
    arm: ArmHandle,
    n_waypoints: int,
    blend_r: float,
    speed: float,
    accel: float,
    *,
    blend_check_safety_factor: float,
    abort_on_overrun: bool,
) -> ExecutionResult:
    """Sparse-sample the trajectory and send as one blended moveJ(path)."""
    t0, t1 = plan.trajectory.start_time(), plan.trajectory.end_time()
    arm_idx = _planning_arm_indices(plan)
    sample_times = np.linspace(t0, t1, n_waypoints)
    waypoints_q = [
        np.asarray(plan.trajectory.value(t)).flatten()[arm_idx]
        for t in sample_times
    ]

    # Blend-radius safety: reuse the FK-replay-side check we already
    # built. Predicted poses come from the controller's FK so this
    # matches what the real arm will see.
    predicted_poses = []
    try:
        from ..util.rtde_convert import rtde_to_pose
        for q in waypoints_q:
            X_base = rtde_to_pose(arm.control.getForwardKinematics(q.tolist()))
            predicted_poses.append(arm.to_task(X_base))
        check = check_blend_radius(
            predicted_poses,
            blend_radius_m=blend_r,
            safety_factor=blend_check_safety_factor,
        )
        ok = print_blend_radius_check(check, label="moveJ_path blend")
        if not ok and abort_on_overrun:
            return ExecutionResult(
                success=False, method="moveJ_path",
                duration_s=plan.duration_s,
                reason=(
                    f"blend_r {blend_r * 1000:.1f} mm exceeds chord limit "
                    f"({check.max_blend_allowed_m * 1000:.1f} mm). "
                    "Reduce blend_r_m or bump n_waypoints."
                ),
            )
    except Exception as exc:
        # Don't block execution on the safety check itself failing —
        # log and continue. The controller will still reject an
        # over-large blend at command time.
        print(f"[execute] blend-radius check skipped: {exc}")

    path = [list(q) + [speed, accel, blend_r] for q in waypoints_q]
    path[-1][-1] = 0.0   # terminal blend = 0 (controller requires this)

    t_start = time.time()
    arm.control.moveJ(path)
    elapsed = time.time() - t_start

    return ExecutionResult(
        success=True, method="moveJ_path", duration_s=elapsed,
    )


# ---------------------------------------------------------------------------
#  servoJ streaming executor
# ---------------------------------------------------------------------------

def _execute_servoJ(
    plan: TransitPlan,
    arm: ArmHandle,
    dt: float,
    time_scale: float,
) -> ExecutionResult:
    """Dense-sample the trajectory at fixed dt, stream via servoJ.

    Uses ``initPeriod`` / ``waitPeriod`` for cycle-locked timing so the
    Python loop period matches the ``dt`` the controller is expecting.
    Drift here shows up as setpoint stutter, not as final-pose error.
    """
    if time_scale <= 0.0:
        raise ValueError(f"servo time_scale must be positive, got {time_scale}")
    arm_idx = _planning_arm_indices(plan)
    t0_real = time.time()
    t0_traj = plan.trajectory.start_time()
    t1_traj = plan.trajectory.end_time()

    try:
        while True:
            t_traj = t0_traj + (time.time() - t0_real) / time_scale
            if t_traj >= t1_traj:
                break

            q = _arm_q_at(plan.trajectory, t_traj, arm_idx)
            t_cycle = arm.control.initPeriod()
            arm.control.servoJ(
                q, 0.0, 0.0, dt,
                0.1,    # lookahead time (s)
                300,    # gain
            )
            arm.control.waitPeriod(t_cycle)
    finally:
        arm.control.servoStop()

    # Settle final pose explicitly (servoJ doesn't guarantee exact
    # final-state convergence on its own).
    final_q = _arm_q_at(plan.trajectory, t1_traj, arm_idx)
    arm.control.moveJ(
        final_q,
        _DEFAULT_SPEED * 0.3 / time_scale,
        _DEFAULT_ACCEL * 0.3 / time_scale,
    )

    return ExecutionResult(
        success=True, method="servoJ", duration_s=plan.duration_s * time_scale,
    )
