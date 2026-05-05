"""Offline FK-replay validation for joint trajectories.

Predicts the TCP poses the controller will visit when given a planned
joint trajectory, by routing the same IK + FK computations through the
UR controller's own kinematic model:

    arm.control.getInverseKinematics(rtde_pose, q_near, ...)
    arm.control.getForwardKinematics(q)

Both calls are pure kinematic queries — the controller does the math
locally, no motion is commanded and the arm does not need to be powered
on with brakes released. Works equally against URSim and the real
controller; just point ``default_session`` at whichever host you want.

Why this exists
---------------
Replaces "send the trajectory and hope" with a measurable error budget
*before* the trajectory leaves Python. Catches:
  - IK failures or branch jumps (q5 sign flips, wrist over-rotation)
  - Frame / TCP-offset mismatches between planning and execution
  - Cartesian deviation between the joint-linear path the arm will
    actually trace and the geometric arc you intended

What it cannot catch
--------------------
  - moveJ(path) blending behaviour — the controller's blend algorithm
    is not exposed offline. Use URSim to observe the blended path.
  - Collision with anything outside the arm itself (microwave, table,
    Vention stand). Bring Drake in for that.
  - X_base_task calibration error — FK uses whatever calibration is
    loaded in arm.X_base_task. Run examples/verify_calibration.py
    to bound that separately.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import List, Optional, Sequence

import numpy as np

from ..arm import ArmHandle
from ..util.poses import Pose
from ..util.rtde_convert import pose_to_rtde, rtde_to_pose


@dataclass
class FKReplayResult:
    """Per-waypoint outcome of an IK -> FK round-trip against a planner."""

    joint_path: List[List[float]]
    """The joint configurations the IK chain produced (one per desired pose)."""

    predicted_poses_task: List[Pose]
    """FK of each joint config, expressed in task frame."""

    desired_poses_task: List[Pose]
    """The arc poses we asked the IK to reach, in task frame."""

    pos_errors_m: List[float]
    """Euclidean distance (m) between predicted and desired translation."""

    rot_errors_rad: List[float]
    """Magnitude of the relative rotation between predicted and desired
    orientation (radians, always non-negative)."""

    @property
    def max_pos_error_m(self) -> float:
        return max(self.pos_errors_m) if self.pos_errors_m else 0.0

    @property
    def max_rot_error_rad(self) -> float:
        return max(self.rot_errors_rad) if self.rot_errors_rad else 0.0


def fk_replay(
    arm: ArmHandle,
    desired_poses_task: Sequence[Pose],
    q_seed: Sequence[float],
    ik_pos_eps: float = 1e-3,
    ik_rot_eps: float = 1e-3,
) -> FKReplayResult:
    """IK each desired pose with chained seeding, FK back, compare.

    The first waypoint uses ``q_seed`` as the IK seed; each subsequent
    waypoint seeds with the previous IK result. This matches the
    branch-stable seeding the live arc loop uses, but no motion is
    commanded — the controller answers via its kinematic model alone.

    Returns position and rotation deviation per waypoint plus the
    resulting joint path. ``FKReplayResult.max_pos_error_m`` is the
    headline metric.
    """
    joint_path: List[List[float]] = []
    predicted_poses_task: List[Pose] = []
    pos_errors: List[float] = []
    rot_errors: List[float] = []

    prev_q = list(q_seed)
    for desired_task in desired_poses_task:
        rtde_target = pose_to_rtde(arm.to_base(desired_task))
        q = arm.control.getInverseKinematics(
            rtde_target, prev_q, ik_pos_eps, ik_rot_eps
        )
        if not q or len(q) != 6:
            raise RuntimeError(
                f"getInverseKinematics returned no solution for desired pose "
                f"{np.round(desired_task.translation, 4)} (task frame). "
                f"Pose may be unreachable from seed {np.round(prev_q, 3)}."
            )

        rtde_predicted = arm.control.getForwardKinematics(list(q))
        predicted_base = rtde_to_pose(rtde_predicted)
        predicted_task = arm.to_task(predicted_base)

        pos_err = float(np.linalg.norm(
            predicted_task.translation - desired_task.translation
        ))
        rel_rot = predicted_task.rotation.inv() * desired_task.rotation
        rot_err = float(np.linalg.norm(rel_rot.as_rotvec()))

        joint_path.append(list(q))
        predicted_poses_task.append(predicted_task)
        pos_errors.append(pos_err)
        rot_errors.append(rot_err)

        prev_q = list(q)

    return FKReplayResult(
        joint_path=joint_path,
        predicted_poses_task=predicted_poses_task,
        desired_poses_task=list(desired_poses_task),
        pos_errors_m=pos_errors,
        rot_errors_rad=rot_errors,
    )


def densify_joint_path(
    arm: ArmHandle,
    joint_path: Sequence[Sequence[float]],
    samples_per_segment: int = 10,
) -> List[Pose]:
    """FK along the joint-linear interpolation between consecutive waypoints.

    Returns predicted TCP poses (task frame) at ``samples_per_segment``
    interior points of every segment, plus the final endpoint. This is
    the path that **sequential ``moveJ``** will trace between waypoints
    (the controller interpolates linearly in joint space). For
    ``moveJ(path)`` with corner blending the actual path is similar but
    smoothed near each waypoint — this offline replay does NOT model
    the blend.

    Use this to quantify chord deviation between waypoints — i.e. how
    far the joint-linear path drifts off the desired Cartesian arc.
    Combine with the desired arc parameterisation to compute a worst-
    case Cartesian error per segment.
    """
    if len(joint_path) < 2:
        return []

    predicted: List[Pose] = []
    for i in range(len(joint_path) - 1):
        q_start = np.asarray(joint_path[i], dtype=float)
        q_end = np.asarray(joint_path[i + 1], dtype=float)
        for s in np.linspace(0.0, 1.0, samples_per_segment, endpoint=False):
            q_interp = ((1.0 - s) * q_start + s * q_end).tolist()
            X_base = rtde_to_pose(arm.control.getForwardKinematics(q_interp))
            predicted.append(arm.to_task(X_base))

    q_final = list(joint_path[-1])
    X_base = rtde_to_pose(arm.control.getForwardKinematics(q_final))
    predicted.append(arm.to_task(X_base))
    return predicted


@dataclass
class BlendRadiusCheck:
    """Result of checking that a configured blend radius is feasible
    for the chord lengths between consecutive waypoints."""

    chord_lengths_m: List[float]
    """TCP-distance chord between each consecutive pair of waypoints (m)."""

    blend_radius_m: float
    """The blend radius being checked (m)."""

    safety_factor: float
    """Fraction of the shortest chord that the blend radius must stay
    under.  0.5 matches the UR controller's hard constraint that two
    blend spheres on a single segment may not overlap; lower values give
    margin for IK perturbation."""

    @property
    def min_chord_m(self) -> float:
        return min(self.chord_lengths_m) if self.chord_lengths_m else float("inf")

    @property
    def max_blend_allowed_m(self) -> float:
        return self.safety_factor * self.min_chord_m

    @property
    def ok(self) -> bool:
        return self.blend_radius_m <= self.max_blend_allowed_m


def check_blend_radius(
    predicted_poses_task: Sequence[Pose],
    blend_radius_m: float,
    safety_factor: float = 0.5,
) -> BlendRadiusCheck:
    """Validate that ``blend_radius_m`` is feasible for the given waypoints.

    UR's ``moveJ(path)`` blend radius is interpreted as TCP distance.
    The controller rejects a path whenever a configured blend radius
    is too large for the segment lengths around it (rule of thumb:
    must stay under half the chord to either neighbour).  This check
    computes the chord between consecutive predicted TCP positions and
    flags configurations that would be rejected.

    Pass the ``predicted_poses_task`` field of an ``FKReplayResult`` —
    that uses the FK of the IK'd joint configurations, so the chord
    lengths reflect what the controller will actually see, not the
    theoretical desired arc.
    """
    chords: List[float] = []
    for i in range(len(predicted_poses_task) - 1):
        d = float(np.linalg.norm(
            predicted_poses_task[i + 1].translation
            - predicted_poses_task[i].translation
        ))
        chords.append(d)
    return BlendRadiusCheck(
        chord_lengths_m=chords,
        blend_radius_m=float(blend_radius_m),
        safety_factor=float(safety_factor),
    )


def print_blend_radius_check(
    check: BlendRadiusCheck,
    label: str = "",
) -> bool:
    """Pretty-print a blend-radius check.  Returns ``check.ok``."""
    prefix = f"  [{label}] " if label else "  "
    if not check.chord_lengths_m:
        print(f"{prefix}blend check skipped — fewer than 2 waypoints")
        return True

    print(f"{prefix}blend radius       : {check.blend_radius_m * 1000:7.3f} mm")
    print(f"{prefix}min chord (TCP)    : {check.min_chord_m * 1000:7.3f} mm")
    print(f"{prefix}max allowed blend  : "
          f"{check.max_blend_allowed_m * 1000:7.3f} mm  "
          f"({check.safety_factor:.2f} × min chord)")
    if check.ok:
        print(f"{prefix}OK    blend radius is below the chord limit.")
    else:
        excess_mm = (check.blend_radius_m - check.max_blend_allowed_m) * 1000
        print(
            f"{prefix}FAIL  blend radius exceeds chord limit by "
            f"{excess_mm:.2f} mm — controller will reject moveJ(path).\n"
            f"{prefix}      Either reduce arc_blend_radius_m below "
            f"{check.max_blend_allowed_m * 1000:.2f} mm, or increase "
            f"n_arc_steps to lengthen each chord."
        )
    return check.ok


def chord_deviation_vs_arc(
    predicted_dense: Sequence[Pose],
    desired_dense: Sequence[Pose],
) -> List[float]:
    """Per-sample translation deviation (m) between two equal-length pose lists.

    Use after ``densify_joint_path`` to compare against analytically-
    sampled desired arc poses at the same parameterisation.
    """
    if len(predicted_dense) != len(desired_dense):
        raise ValueError(
            f"length mismatch: predicted {len(predicted_dense)} vs "
            f"desired {len(desired_dense)}"
        )
    return [
        float(np.linalg.norm(p.translation - d.translation))
        for p, d in zip(predicted_dense, desired_dense)
    ]


def print_fk_replay_summary(
    result: FKReplayResult,
    label: str = "",
    per_waypoint: bool = True,
    pos_threshold_mm: Optional[float] = 2.0,
    rot_threshold_deg: Optional[float] = 1.0,
) -> bool:
    """Pretty-print a replay result. Returns True if within thresholds."""
    prefix = f"  [{label}] " if label else "  "
    n = len(result.joint_path)
    print(f"{prefix}waypoints checked  : {n}")
    print(f"{prefix}max pos deviation  : "
          f"{result.max_pos_error_m * 1000:7.3f} mm")
    print(f"{prefix}max rot deviation  : "
          f"{np.degrees(result.max_rot_error_rad):7.3f}°")

    if per_waypoint:
        for i, (p, r) in enumerate(zip(result.pos_errors_m, result.rot_errors_rad)):
            print(
                f"{prefix}  [{i + 1:2d}] pos {p * 1000:6.3f} mm   "
                f"rot {np.degrees(r):6.3f}°"
            )

    ok = True
    if pos_threshold_mm is not None and result.max_pos_error_m * 1000 > pos_threshold_mm:
        print(
            f"{prefix}WARN  pos deviation exceeds {pos_threshold_mm:.2f} mm threshold"
        )
        ok = False
    if rot_threshold_deg is not None and np.degrees(result.max_rot_error_rad) > rot_threshold_deg:
        print(
            f"{prefix}WARN  rot deviation exceeds {rot_threshold_deg:.2f}° threshold"
        )
        ok = False
    return ok
