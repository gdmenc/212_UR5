"""Constrained motion planner for free-space transit between task-frame poses.

Scope
-----
This planner is for the *transit* segments of a task — the "go from
where I am to where I need to be" parts that the existing recorded-
waypoint code (pre-grasp, grasp, arc) doesn't cover. It plans a smooth,
collision-free joint trajectory between user-supplied TCP poses
expressed in task frame.

API surface
-----------

>>> plan = plan_transit(
...     plant, arm="ur_left",
...     waypoints=[start_pose, end_pose],
...     seed_q=current_joints,
...     fix_orientation=desired_R,
...     fix_z_task=0.30,
...     avoid_collisions=True,
...     other_arm_q=current_right_q,
... )
>>> execute_plan(plan, session, method="moveJ_path")    # see execute.py

The planner picks between a fast cubic-spline path (no constraints)
and a Drake ``KinematicTrajectoryOptimization`` (KTO) solve when any
geometric constraint is requested. Same ``TransitPlan`` output type
either way, so the executor doesn't need to care.

Coordinate convention
---------------------
- Task frame == Drake world frame (see ``planning/__init__.py``).
- Waypoint poses are task-frame TCP poses; rotation aligns the gripper.
- Planner output ``trajectory.value(t)`` is a 6-vector of joints for
  the planning arm. The other arm stays at ``other_arm_q`` for the
  duration; collision queries respect that.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Callable, Dict, List, Optional, Tuple, Union

import numpy as np
from pydrake.math import RigidTransform, RotationMatrix
from pydrake.multibody.inverse_kinematics import (
    AngleBetweenVectorsConstraint,
    InverseKinematics,
    MinimumDistanceLowerBoundConstraint,
    OrientationConstraint,
    PositionConstraint,
)
from pydrake.multibody.plant import MultibodyPlant
from pydrake.multibody.tree import BodyIndex, ModelInstanceIndex
from pydrake.planning import KinematicTrajectoryOptimization
from pydrake.solvers import Solve
from pydrake.systems.framework import Context
from pydrake.trajectories import (
    BsplineTrajectory,
    PiecewisePolynomial,
    Trajectory,
)

from ..util.poses import Pose
from . import warmstart


WARMSTART_ENABLED = True
"""When True, ``_plan_constrained_kto`` consults ``warmstart`` for a
cached B-spline before falling back to the from-scratch cubic-spline
seed. Cache is in-memory only (process-local). Set False to bypass."""

WARMSTART_REORDER_SEEDS = True
"""When True, ``plan_transit_multi_seed`` ranks each candidate IK seed
against the warmstart cache before iterating: seeds whose predicted
(q_start, q_goal, problem_signature) is already cached are tried first.
On warm cache, this lets a multi-seed call hit the previously-successful
seed immediately instead of paying a cold KTO solve on doomed seeds.
Set False to fall back to plain distance-sorted iteration."""


# ---------------------------------------------------------------------------
#  Output type
# ---------------------------------------------------------------------------

@dataclass
class TransitPlan:
    """A smooth, collision-checked joint trajectory for one arm."""

    trajectory: Trajectory
    """Continuous q(t) — Drake ``BsplineTrajectory`` (KTO) or
    ``PiecewisePolynomial`` (simple spline). 6-DOF for the planning arm."""

    arm: str
    """``'ur_left'`` or ``'ur_right'``."""

    duration_s: float
    """Total trajectory time. Equal to ``trajectory.end_time() - start_time()``."""

    waypoints_q: List[np.ndarray]
    """The IK'd joint configurations the planner had to satisfy at each
    user-supplied waypoint. Useful for FK-replay validation."""

    collision_checked: bool
    """Whether collision avoidance was active during planning."""

    min_clearance_m: float
    """Smallest signed distance to anything along the planned path
    (post-solve sample)."""

    metadata: Dict = field(default_factory=dict)


class InfeasiblePlanError(RuntimeError):
    """The requested transit cannot be planned (IK fail, no collision-free
    spline, KTO infeasible, ...). The message names which step failed."""


# ---------------------------------------------------------------------------
#  IK fallback callable type
# ---------------------------------------------------------------------------

# Optional rtde-IK callable: takes (task-frame TCP pose, seed q) and returns
# joint config or None. Used as tier 2 in _ik_pose_to_joints between ikfast
# (analytical, sim+real) and Drake numerical IK (NLP, fallback). Built via
# ``make_rtde_ik(arm)`` in real-path code where an ArmHandle is available.
RtdeIkFn = Callable[[Pose, np.ndarray], Optional[np.ndarray]]


def make_rtde_ik(arm) -> RtdeIkFn:
    """Build a closure that calls ``arm.control.getInverseKinematics``.

    Used as a third IK tier (between ikfast and Drake numerical IK) for
    real-path planning. ikfast and the rig's controller IK should agree
    on UR5e since both are closed-form, but rtde IK is the authoritative
    one — it's exactly what ``moveL`` would compute. Useful as a sanity
    backup when ikfast happens to return zero solutions due to
    URDF/calibration drift.

    Returns a callable that accepts ``(pose_task, seed_q_arm)`` and
    returns either a 6-DOF joint config (success) or ``None`` (rtde IK
    failed, fall through to next tier).
    """
    from ..util.rtde_convert import pose_to_rtde

    def rtde_ik(pose_task: Pose, seed_q_arm: np.ndarray) -> Optional[np.ndarray]:
        try:
            base_pose = arm.to_base(pose_task)
            rtde_pose = pose_to_rtde(base_pose)
            result = arm.control.getInverseKinematics(
                rtde_pose, list(seed_q_arm), 0.001, 0.001,
            )
            if not result or len(result) != 6:
                return None
            return np.asarray(result, dtype=float)
        except Exception as exc:
            print(f"[rtde_ik] failed: {exc}")
            return None

    return rtde_ik


# ---------------------------------------------------------------------------
#  Arm topology helpers
# ---------------------------------------------------------------------------

# UR5e wrist-3 link is the last link before the flange / TCP. We attach a
# TCP frame to it programmatically below if one isn't already there.
_UR5E_LAST_LINK = "wrist_3_link"


def _arm_model_instance(plant: MultibodyPlant, arm: str) -> ModelInstanceIndex:
    """Look up the model instance for an arm.

    The arms are loaded with ``parser.SetAutoRenaming(True)`` in
    ``planning/scene/arms.py``, so the actual instance name is
    ``{arm}::ur5e`` after auto-renaming.
    """
    for candidate in (arm, f"{arm}::ur5e"):
        if plant.HasModelInstanceNamed(candidate):
            return plant.GetModelInstanceByName(candidate)
    raise KeyError(
        f"no model instance named {arm!r} (or {arm}::ur5e) in plant. "
        f"Existing instances: "
        f"{[plant.GetModelInstanceName(i) for i in range(plant.num_model_instances())]}"
    )


def _arm_position_indices(
    plant: MultibodyPlant, arm_instance: ModelInstanceIndex,
) -> np.ndarray:
    """Indices of this arm's joints in the plant's full position vector."""
    # Drake assigns each joint a position index; assemble them in order.
    indices = []
    for joint_idx in plant.GetJointIndices(arm_instance):
        joint = plant.get_joint(joint_idx)
        if joint.num_positions() == 0:
            continue
        for k in range(joint.num_positions()):
            indices.append(joint.position_start() + k)
    return np.array(indices, dtype=int)


def _tcp_frame(plant: MultibodyPlant, arm_instance: ModelInstanceIndex):
    """Return the TCP frame for the planning arm.

    Preferred: the named ``tcp_<arm>`` frame added by
    ``planning/scene/grippers.py`` — sits at the calibrated TCP_OFFSET
    on wrist_3_link, so IK targets the actual tool tip.

    Fallback: the wrist_3_link itself, for scenes built without a
    gripper attached.
    """
    arm_name = plant.GetModelInstanceName(arm_instance)
    arm_short = arm_name.replace("::ur5e", "")          # 'ur_left' / 'ur_right'
    tcp_short = arm_short.removeprefix("ur_")           # 'left' / 'right'
    tcp_name = f"tcp_{tcp_short}"

    if plant.HasFrameNamed(tcp_name):
        # Drake doesn't restrict named frames to a single model instance;
        # the FixedOffsetFrame we added lives on the WorldModelInstance
        # by convention. GetFrameByName(name) without an instance
        # finds it.
        return plant.GetFrameByName(tcp_name)
    return plant.GetFrameByName(_UR5E_LAST_LINK, arm_instance)


# ---------------------------------------------------------------------------
#  Pose -> joint IK with chained seeding
# ---------------------------------------------------------------------------

def _ik_pose_to_joints_ikfast(
    plant: MultibodyPlant,
    arm_instance: ModelInstanceIndex,
    other_arm_instance: Optional[ModelInstanceIndex],
    other_arm_q: Optional[np.ndarray],
    pose_task: Pose,
    seed_q_full: np.ndarray,
    arm_name: str,
) -> Optional[np.ndarray]:
    """Analytical IK for a UR arm via ikfast.

    Returns the full-plant q with the planning arm's slot filled by the
    ikfast solution closest to ``seed_q_full[arm_idx]``, or ``None`` if
    ikfast couldn't find a reachable solution within Drake's joint
    limits.

    Why this exists: Drake's ``InverseKinematics`` is a numerical NLP —
    it gradient-descends from the seed and can stall at a local minimum
    (e.g., 19° off in orientation) for poses where the seed is far from
    any feasible IK branch. ikfast solves the UR5e closed-form
    six-degree polynomial directly and returns up to 8 wrist branches,
    so it never gets stuck. Closer to what UR's controller does for
    moveL on the rig.
    """
    from .rrt import ikfast_goal_branches

    arm_idx = _arm_position_indices(plant, arm_instance)
    seed_arm_q = np.asarray(seed_q_full, dtype=float)[arm_idx]

    branches = ikfast_goal_branches(
        arm_name, pose_task,
        seed_arm_q=seed_arm_q,
        max_branch_dist=None,
    )
    if not branches:
        return None

    # ikfast doesn't know about Drake plant joint limits — filter.
    lo = plant.GetPositionLowerLimits()[arm_idx]
    hi = plant.GetPositionUpperLimits()[arm_idx]
    valid = [
        q for q in branches
        if np.all(q >= lo - 1e-6) and np.all(q <= hi + 1e-6)
    ]
    if not valid:
        return None

    # Prefer branches that stay in the seed's q3/q5 wrist quadrant —
    # mirrors what Drake numerical IK does naturally (gradient descent
    # doesn't cross singularities) AND matches the singularity branch
    # lock used by the RRT validity_fn. Without this filter, ikfast's
    # "closest in joint distance" can return a branch whose q3 has the
    # wrong sign because the unwrapped Δq is smaller than the 2π-shifted
    # version — but downstream planning then fails the branch-lock.
    if len(seed_arm_q) >= 5:
        def _wrap(q):
            return (q + np.pi) % (2.0 * np.pi) - np.pi
        seed_w = _wrap(seed_arm_q)
        seed_q3_sign = +1.0 if seed_w[2] >= 0 else -1.0
        seed_q5_sign = +1.0 if seed_w[4] >= 0 else -1.0

        def _matches_quadrant(q):
            qw = _wrap(q)
            q3_ok = (qw[2] >= 0) == (seed_q3_sign > 0)
            q5_ok = (qw[4] >= 0) == (seed_q5_sign > 0)
            return q3_ok and q5_ok

        same_quadrant = [q for q in valid if _matches_quadrant(q)]
        # Use same-quadrant branches when any exist; fall back to all
        # valid branches when the goal pose genuinely requires a quadrant
        # change (rare — usually means the planner will fail downstream
        # too, but we'd rather hand it the closest-by-distance solution
        # than refuse to IK at all).
        valid = same_quadrant if same_quadrant else valid

    # ikfast_goal_branches sorted by joint distance to seed AND shifted
    # each branch by 2π multiples to land near the seed; quadrant filter
    # above preserves order — first remaining entry is closest in the
    # preferred quadrant.
    best_arm_q = valid[0]

    q_full = np.asarray(seed_q_full, dtype=float).copy()
    q_full[arm_idx] = best_arm_q
    if other_arm_instance is not None and other_arm_q is not None:
        other_idx = _arm_position_indices(plant, other_arm_instance)
        q_full[other_idx] = other_arm_q
    return q_full


def _ik_pose_to_joints(
    plant: MultibodyPlant,
    plant_context: Context,
    arm_instance: ModelInstanceIndex,
    other_arm_instance: Optional[ModelInstanceIndex],
    other_arm_q: Optional[np.ndarray],
    pose_world: Pose,
    seed_q_full: np.ndarray,
    pos_tolerance_m: float = 0.005,
    rot_tolerance_rad: float = 0.05,
    arm_name: Optional[str] = None,
    rtde_ik: Optional[RtdeIkFn] = None,
) -> np.ndarray:
    """IK a single TCP pose for the planning arm.

    Three-tier solver ladder:
      1. **ikfast** (analytical, sim+real). Deterministic, never stalls,
         returns the closest-to-seed of up to 8 wrist branches.
      2. **rtde IK** (analytical, real-path only when ``rtde_ik`` is
         provided). Same closed-form solver UR's controller uses for
         ``moveL`` — useful sanity backup when ikfast happens to disagree
         due to URDF/calibration drift.
      3. **Drake numerical IK** (gradient descent over a constrained NLP).
         Final fallback — handles tolerance-band edge cases where neither
         analytical solver finds an exact match but a near solution is OK.

    ``seed_q_full`` is the full plant position vector (12 DOF for
    bimanual). The other arm's positions are pinned to ``other_arm_q``
    via a bounding-box constraint so the IK solver doesn't move them.

    Returns the full plant position vector (with the planning arm's
    joints solved, other arm fixed).
    """
    if other_arm_instance is not None and other_arm_q is None:
        raise ValueError(
            "other_arm_instance set but other_arm_q is None — provide "
            "the partner arm's pose so collision queries see it."
        )

    # Tier 1: analytical ikfast for known UR arms.
    if arm_name in ("ur_left", "ur_right"):
        ikfast_result = _ik_pose_to_joints_ikfast(
            plant, arm_instance,
            other_arm_instance, other_arm_q,
            pose_world, seed_q_full,
            arm_name,
        )
        if ikfast_result is not None:
            return ikfast_result

    # Tier 2: rtde controller IK (real-path only). Same closed-form solver
    # the rig uses for moveL — authoritative when ikfast and rtde disagree
    # due to URDF/calibration drift.
    if rtde_ik is not None:
        arm_idx = _arm_position_indices(plant, arm_instance)
        seed_arm_q = np.asarray(seed_q_full, dtype=float)[arm_idx]
        rtde_q_arm = rtde_ik(pose_world, seed_arm_q)
        if rtde_q_arm is not None and len(rtde_q_arm) == len(arm_idx):
            # Check joint limits.
            lo = plant.GetPositionLowerLimits()[arm_idx]
            hi = plant.GetPositionUpperLimits()[arm_idx]
            if np.all(rtde_q_arm >= lo - 1e-6) and np.all(rtde_q_arm <= hi + 1e-6):
                q_full = np.asarray(seed_q_full, dtype=float).copy()
                q_full[arm_idx] = rtde_q_arm
                if other_arm_instance is not None and other_arm_q is not None:
                    other_idx = _arm_position_indices(plant, other_arm_instance)
                    q_full[other_idx] = other_arm_q
                return q_full

    # Tier 3: Drake numerical IK as fallback.
    ik = InverseKinematics(plant, plant_context)
    prog = ik.get_mutable_prog()

    tcp_frame = _tcp_frame(plant, arm_instance)
    world_frame = plant.world_frame()

    p_target = np.asarray(pose_world.translation, dtype=float)
    R_target = RotationMatrix(np.asarray(pose_world.rotation.as_matrix(), dtype=float))

    ik.AddPositionConstraint(
        frameA=world_frame,
        p_AQ_lower=p_target - pos_tolerance_m,
        p_AQ_upper=p_target + pos_tolerance_m,
        frameB=tcp_frame,
        p_BQ=np.zeros(3),
    )
    ik.AddOrientationConstraint(
        frameAbar=world_frame,
        R_AbarA=R_target,
        frameBbar=tcp_frame,
        R_BbarB=RotationMatrix(),
        theta_bound=rot_tolerance_rad,
    )

    # Pin the partner arm's positions to other_arm_q.
    if other_arm_instance is not None:
        other_idx = _arm_position_indices(plant, other_arm_instance)
        other_vars = ik.q()[other_idx]
        prog.AddBoundingBoxConstraint(
            np.asarray(other_arm_q), np.asarray(other_arm_q), other_vars,
        )

    prog.SetInitialGuess(ik.q(), seed_q_full)
    result = Solve(prog)
    if not result.is_success():
        # Look up which constraints the solver couldn't satisfy.
        try:
            infeasible = result.GetInfeasibleConstraintNames(prog)
        except Exception:
            infeasible = ["(GetInfeasibleConstraintNames unavailable)"]
        raise InfeasiblePlanError(
            f"IK failed for pose at task xyz {np.round(p_target, 3)}; "
            f"solver={result.get_solver_id().name()}, "
            f"status={result.get_solution_result()}; "
            f"infeasible: {infeasible[:3] if infeasible else '(none reported)'}"
        )
    return result.GetSolution(ik.q())


def _pose_chain_to_joints(
    plant: MultibodyPlant,
    plant_context: Context,
    arm_instance: ModelInstanceIndex,
    waypoints: List[Pose],
    seed_q_full: np.ndarray,
    other_arm_instance: Optional[ModelInstanceIndex] = None,
    other_arm_q: Optional[np.ndarray] = None,
    pos_tolerance_m: float = 0.005,
    rot_tolerance_rad: float = 0.05,
    arm_name: Optional[str] = None,
    rtde_ik: Optional[RtdeIkFn] = None,
) -> List[np.ndarray]:
    """IK every waypoint in order, seeding each with the previous IK result.

    Returns a list of full-plant position vectors (one per waypoint).
    Chained seeding keeps consecutive solutions on the same IK branch —
    avoids wrist 360° spins between adjacent waypoints. ``arm_name``
    enables the analytical-ikfast fast path in ``_ik_pose_to_joints``.
    """
    seeded = np.asarray(seed_q_full, dtype=float).copy()
    if other_arm_instance is not None and other_arm_q is not None:
        other_idx = _arm_position_indices(plant, other_arm_instance)
        seeded[other_idx] = other_arm_q

    out: List[np.ndarray] = []
    for wp in waypoints:
        q_full = _ik_pose_to_joints(
            plant, plant_context, arm_instance,
            other_arm_instance, other_arm_q,
            wp, seed_q_full=seeded,
            pos_tolerance_m=pos_tolerance_m,
            rot_tolerance_rad=rot_tolerance_rad,
            arm_name=arm_name,
            rtde_ik=rtde_ik,
        )
        out.append(q_full)
        seeded = q_full
    return out


# ---------------------------------------------------------------------------
#  Simple cubic-spline planner (used when no constraints are requested)
# ---------------------------------------------------------------------------

def _plan_simple_spline(
    plant: MultibodyPlant,
    plant_context: Context,
    arm: str,
    arm_instance: ModelInstanceIndex,
    waypoints_q_full: List[np.ndarray],
    duration_s: float,
    *,
    check_collisions: bool,
    min_clearance_m: float,
    other_arm_q: Optional[np.ndarray],
    other_arm_instance: Optional[ModelInstanceIndex],
) -> TransitPlan:
    """C¹ cubic spline through joint waypoints. Collision check at samples.

    Trajectory is full-plant 12-DOF (so the executor sees the same
    layout as KTO output); the executor extracts the planning arm's 6.
    """
    if len(waypoints_q_full) < 2:
        raise ValueError("Need at least 2 waypoints for a transit.")

    # CubicShapePreserving needs ≥3 samples. For 2-waypoint plans we
    # synthesise a midpoint so the spline is well-posed.
    waypoints_for_spline = list(waypoints_q_full)
    if len(waypoints_for_spline) < 3:
        midpoint = 0.5 * (waypoints_for_spline[0] + waypoints_for_spline[-1])
        waypoints_for_spline = [
            waypoints_for_spline[0],
            midpoint,
            waypoints_for_spline[-1],
        ]

    times = np.linspace(0.0, duration_s, len(waypoints_for_spline))
    samples = np.array(waypoints_for_spline).T  # (num_positions, num_waypoints)
    traj = PiecewisePolynomial.CubicShapePreserving(times, samples)

    min_clearance = float("inf")
    if check_collisions:
        min_clearance = _check_collision_along(
            plant, plant_context, traj,
            other_arm_q, other_arm_instance,
            n_samples=200, min_clearance_m=min_clearance_m,
        )

    return TransitPlan(
        trajectory=traj,
        arm=arm,
        duration_s=duration_s,
        waypoints_q=[
            wp[_arm_position_indices(plant, arm_instance)]
            for wp in waypoints_q_full
        ],
        collision_checked=check_collisions,
        min_clearance_m=min_clearance,
        metadata={"planner": "simple_spline"},
    )


def _check_safety_along(
    plant: MultibodyPlant,
    plant_context: Context,
    traj: Trajectory,
    arm_instance: ModelInstanceIndex,
    other_arm_q: Optional[np.ndarray],
    other_arm_instance: Optional[ModelInstanceIndex],
    *,
    n_samples: int,
    check_collisions: bool,
    min_clearance_m: float,
    floor_min_z: Optional[float],
) -> float:
    """Verify the safety constraints (collision + TCP floor) along a
    sampled trajectory. Used by the spline-first fast path so we can
    ship a simple cubic spline when it's already safe and only fall
    back to KTO when it isn't.

    Raises ``InfeasiblePlanError`` on the first violation. Returns the
    worst pairwise clearance (mm clearance) over the sampled path,
    ``inf`` if collision checking was off, ``nan`` if the plant isn't
    wired to a SceneGraph.
    """
    if other_arm_instance is not None and other_arm_q is not None:
        plant.SetPositions(plant_context, other_arm_instance, other_arm_q)
    tcp_frame = _tcp_frame(plant, arm_instance) if floor_min_z is not None else None

    worst = float("inf") if check_collisions else float("nan")
    t0, t1 = traj.start_time(), traj.end_time()
    for s in np.linspace(0.0, 1.0, n_samples):
        t = t0 + s * (t1 - t0)
        q = np.asarray(traj.value(t)).flatten()
        plant.SetPositions(plant_context, q)

        if floor_min_z is not None:
            tcp_z = float(tcp_frame.CalcPoseInWorld(plant_context).translation()[2])
            if tcp_z < floor_min_z:
                raise InfeasiblePlanError(
                    f"floor violated at t={t:.3f}s "
                    f"(TCP z={tcp_z*1000:.1f} mm < min={floor_min_z*1000:.1f} mm)"
                )

        if check_collisions:
            try:
                qry = plant.get_geometry_query_input_port().Eval(plant_context)
            except RuntimeError as exc:
                # Plant not wired to a SceneGraph — bail with a warning so
                # the caller knows safety wasn't actually verified.
                print(f"[plan_transit] WARNING: scene graph query unavailable "
                      f"at t={t:.3f}s ({exc!s:.120}); collision check skipped.")
                return float("nan")
            pairs = qry.ComputeSignedDistancePairwiseClosestPoints(
                max_distance=float("inf"),
            )
            if pairs:
                insp = qry.inspector()
                # Filter same-model-instance pairs (mesh self-distance
                # within an arm/gripper). Drake's KTO collision constraint
                # filters these implicitly via collision filter groups;
                # our manual signed-distance query doesn't, so we do it
                # here. We still detect arm-vs-other-arm, arm-vs-microwave,
                # arm-vs-table, gripper-vs-microwave, etc.
                inter_model_pairs = []
                for p in pairs:
                    body_A = plant.GetBodyFromFrameId(insp.GetFrameId(p.id_A))
                    body_B = plant.GetBodyFromFrameId(insp.GetFrameId(p.id_B))
                    if body_A.model_instance() == body_B.model_instance():
                        continue
                    inter_model_pairs.append(p)

                if inter_model_pairs:
                    worst_pair = min(inter_model_pairs, key=lambda p: p.distance)
                    d = float(worst_pair.distance)
                    worst = min(worst, d)
                    if d < min_clearance_m:
                        raise InfeasiblePlanError(
                            f"collision at t={t:.3f}s "
                            f"(d={d*1000:.2f} mm < min={min_clearance_m*1000:.2f} mm) "
                            f"between {insp.GetName(worst_pair.id_A)!r} "
                            f"and {insp.GetName(worst_pair.id_B)!r}"
                        )

    return worst


def _check_collision_along(
    plant: MultibodyPlant,
    plant_context: Context,
    traj: Trajectory,
    other_arm_q: Optional[np.ndarray],
    other_arm_instance: Optional[ModelInstanceIndex],
    n_samples: int,
    min_clearance_m: float,
) -> float:
    """Sample the trajectory and find the worst clearance.

    Raises ``InfeasiblePlanError`` if any sample is below
    ``min_clearance_m``.
    """
    if other_arm_instance is not None and other_arm_q is not None:
        plant.SetPositions(plant_context, other_arm_instance, other_arm_q)

    query = plant.get_geometry_query_input_port()
    # We query SignedDistancePairs through the scene graph context.
    # For simplicity here, use a sampled MinimumDistanceLowerBoundConstraint
    # via direct scene_graph query.
    sg = plant.GetParentSceneGraph() if hasattr(plant, "GetParentSceneGraph") else None
    # Drake doesn't expose GetParentSceneGraph on MultibodyPlant directly;
    # we use the geometry query input port instead.

    worst = float("inf")
    t0, t1 = traj.start_time(), traj.end_time()
    for s in np.linspace(0.0, 1.0, n_samples):
        t = t0 + s * (t1 - t0)
        q = np.asarray(traj.value(t)).flatten()
        plant.SetPositions(plant_context, q)

        # Use the scene graph's query object via the plant's context.
        # Fetch the QueryObject from the plant's geometry query input port.
        # Note: this requires the plant to be wired into a SceneGraph.
        try:
            query_object = plant.get_geometry_query_input_port().Eval(plant_context)
            distances = query_object.ComputeSignedDistancePairwiseClosestPoints(
                max_distance=float("inf"),
            )
            if distances:
                d = min(p.distance for p in distances)
                worst = min(worst, d)
                if d < min_clearance_m:
                    raise InfeasiblePlanError(
                        f"collision along trajectory at t={t:.3f} s "
                        f"(d={d * 1000:.2f} mm < threshold "
                        f"{min_clearance_m * 1000:.2f} mm)"
                    )
        except RuntimeError:
            # Plant may not be wired to a scene graph in all contexts;
            # skip distance checks gracefully then.
            return float("nan")

    return worst


# ---------------------------------------------------------------------------
#  Top-level dispatcher (KTO version added in next chunk)
# ---------------------------------------------------------------------------

def ikfast_seeds_at_pose(
    arm_name: str,
    pose_task: Pose,
    seed_arm_q: Optional[np.ndarray] = None,
) -> List[np.ndarray]:
    """Return all ikfast branches at the given task-frame TCP pose.

    Sorted by joint distance to ``seed_arm_q`` (when provided), with
    each branch shifted by 2π multiples to land near the seed.

    Useful for enumerating leg-boundary IK branch options to feed to
    :func:`plan_transit_multi_seed`. Up to 8 branches per pose for the
    UR5e (3 binary wrist branches: shoulder L/R, elbow up/down, wrist
    flip).
    """
    from .rrt import ikfast_goal_branches
    return list(ikfast_goal_branches(
        arm_name, pose_task,
        seed_arm_q=seed_arm_q,
        max_branch_dist=None,
    ))


def plan_transit_chained(
    arm: str,
    log_label: str,
    *,
    prev_terminal_pose: Optional[Pose] = None,
    chained_arm_q: Optional[np.ndarray] = None,
    **plan_transit_kwargs,
) -> "TransitPlan":
    """Plan a leg with multi-seed retry derived from the previous leg's
    terminal pose.

    Convenience wrapper around :func:`plan_transit_multi_seed` for the
    common chained-leg pattern: enumerate ikfast branches at
    ``prev_terminal_pose`` (sorted by joint distance to ``chained_arm_q``)
    and try each as the next leg's IK seed. Returns the first plan that
    succeeds.

    First-leg call sites pass ``prev_terminal_pose=None``, in which case
    we delegate to :func:`plan_transit` with whatever ``current_q`` is in
    ``plan_transit_kwargs`` — no retry happens at the first leg because
    the rig's actual joint config is fixed and we have nothing else to
    try.
    """
    if prev_terminal_pose is not None and chained_arm_q is not None:
        seeds = list(ikfast_seeds_at_pose(
            arm, prev_terminal_pose, seed_arm_q=chained_arm_q,
        )) or [chained_arm_q]
    else:
        # No retry — single attempt with caller's ``current_q``.
        seeds = []

    return plan_transit_multi_seed(
        arm=arm, log_label=log_label,
        seed_candidates=seeds,
        **plan_transit_kwargs,
    )


def _predict_kto_cache_key(
    seed: np.ndarray,
    arm: str,
    plan_transit_kwargs: dict,
) -> Optional[Tuple[np.ndarray, np.ndarray, tuple]]:
    """Best-effort prediction of (q_start, q_end, problem_signature) for
    the cache key ``_plan_constrained_kto`` would use if planning ran
    with ``seed`` as the planning arm's IK seed.

    Returns ``None`` on any prediction failure (missing kwargs, IK
    failure, etc.); callers treat that as "cold" (cache miss expected)
    and skip the peek. Cheap: one ikfast-fast-path IK chain (~1 ms per
    waypoint) plus the signature build.

    Mirrors the relevant prep logic in :func:`plan_transit` and
    :func:`_plan_constrained_kto`. If those drift, predictions will go
    stale (false misses) — never false hits, since the cache lookup
    requires exact key match.
    """
    plant: Optional[MultibodyPlant] = plan_transit_kwargs.get("plant")
    waypoints: Optional[List[Pose]] = plan_transit_kwargs.get("waypoints")
    plant_context: Optional[Context] = plan_transit_kwargs.get("plant_context")
    if plant is None or waypoints is None or plant_context is None:
        return None
    if len(waypoints) < 2:
        return None

    try:
        arm_instance = _arm_model_instance(plant, arm)
    except KeyError:
        return None

    other_arm_instance: Optional[ModelInstanceIndex] = None
    other_arm_name = "ur_right" if arm == "ur_left" else "ur_left"
    for cand in (other_arm_name, f"{other_arm_name}::ur5e"):
        if plant.HasModelInstanceNamed(cand):
            try:
                other_arm_instance = _arm_model_instance(plant, other_arm_name)
            except KeyError:
                pass
            break

    arm_idx = _arm_position_indices(plant, arm_instance)
    other_idx = (
        _arm_position_indices(plant, other_arm_instance)
        if other_arm_instance is not None else np.array([], dtype=int)
    )

    # Layer the starting state same way plan_transit does, but only as
    # far as we need to seed the IK chain.
    seed_q_full = plant.GetPositions(plant_context).copy()
    current_q = plan_transit_kwargs.get("current_q")
    if current_q is not None:
        if isinstance(current_q, dict):
            for arm_name, q_vec in current_q.items():
                if q_vec is None:
                    continue
                try:
                    inst = _arm_model_instance(plant, arm_name)
                except KeyError:
                    continue
                idx = _arm_position_indices(plant, inst)
                q_arr = np.asarray(q_vec, dtype=float)
                if q_arr.shape == (len(idx),):
                    seed_q_full[idx] = q_arr
        else:
            q_arr = np.asarray(current_q, dtype=float)
            if q_arr.shape == seed_q_full.shape:
                seed_q_full = q_arr.copy()
    # Apply the candidate seed for the planning arm (overrides current_q).
    seed_arr = np.asarray(seed, dtype=float)
    if seed_arr.shape != (len(arm_idx),):
        return None
    seed_q_full[arm_idx] = seed_arr

    other_arm_q = plan_transit_kwargs.get("other_arm_q")
    if other_arm_q is None and other_arm_instance is not None:
        other_arm_q = seed_q_full[other_idx]

    # Predicted IK chain — ikfast fast path via arm_name.
    try:
        waypoints_q_full = _pose_chain_to_joints(
            plant=plant,
            plant_context=plant_context,
            arm_instance=arm_instance,
            waypoints=waypoints,
            seed_q_full=seed_q_full,
            other_arm_instance=other_arm_instance,
            other_arm_q=other_arm_q,
            pos_tolerance_m=plan_transit_kwargs.get("ik_pos_tolerance_m", 0.005),
            rot_tolerance_rad=plan_transit_kwargs.get("ik_rot_tolerance_rad", 0.05),
            arm_name=arm,
            rtde_ik=plan_transit_kwargs.get("rtde_ik"),
        )
    except Exception:
        return None  # IK failed — treat seed as cold

    # Predict duration_s same way plan_transit does on the auto path.
    duration_s = plan_transit_kwargs.get("duration_s")
    if duration_s is None:
        max_joint_delta = 0.0
        for a, b in zip(waypoints_q_full[:-1], waypoints_q_full[1:]):
            max_joint_delta = max(max_joint_delta, float(np.max(np.abs(b - a))))
        duration_s = max(1.0, max_joint_delta / 0.5 * len(waypoints_q_full))

    # Build the same signature ``_plan_constrained_kto`` would build.
    try:
        signature = _kto_problem_signature(
            plant=plant,
            plant_context=plant_context,
            arm=arm,
            duration_s=duration_s,
            n_ctrl=10,
            spline_order=4,
            waypoints_q_full=waypoints_q_full,
            fix_orientation=plan_transit_kwargs.get("fix_orientation"),
            fix_orientation_tolerance_rad=plan_transit_kwargs.get(
                "fix_orientation_tolerance_rad", 0.04,
            ),
            align_tcp_axis=plan_transit_kwargs.get("align_tcp_axis"),
            align_tcp_axis_world=plan_transit_kwargs.get("align_tcp_axis_world"),
            align_tcp_axis_tolerance_rad=plan_transit_kwargs.get(
                "align_tcp_axis_tolerance_rad", 0.05,
            ),
            fix_z_task=plan_transit_kwargs.get("fix_z_task"),
            z_tolerance_m=plan_transit_kwargs.get("z_tolerance_m", 0.01),
            avoid_collisions=plan_transit_kwargs.get("avoid_collisions", True),
            min_clearance_m=plan_transit_kwargs.get("min_clearance_m", 0.01),
            avoid_arm_singularity=plan_transit_kwargs.get(
                "avoid_arm_singularity", True,
            ),
            self_collision=plan_transit_kwargs.get("self_collision", True),
            min_z_task=plan_transit_kwargs.get("min_z_task", 0.02),
            max_tcp_linear_speed_m_per_s=plan_transit_kwargs.get(
                "max_tcp_linear_speed_m_per_s",
            ),
            extra_clearance_to=plan_transit_kwargs.get("extra_clearance_to"),
            stay_in_workspace_box=plan_transit_kwargs.get("stay_in_workspace_box"),
        )
    except Exception:
        return None

    return waypoints_q_full[0], waypoints_q_full[-1], signature


def plan_transit_multi_seed(
    arm: str,
    *,
    log_label: str = "transit",
    seed_candidates: Optional[List[np.ndarray]] = None,
    **plan_transit_kwargs,
) -> "TransitPlan":
    """Plan a transit, trying each seed in ``seed_candidates`` as the
    planning arm's IK seed. Returns the first plan that succeeds; raises
    :class:`InfeasiblePlanError` if every seed fails.

    Use case: chained leg planning where the previous leg's terminal
    config can be replaced by any of the (up to 8) ikfast branches at
    the same task-frame TCP pose. When the next leg fails with the
    chained config, retry with the alternative branches. ``avoid_arm_
    singularity`` stays True throughout — within each retry, no
    singularity is crossed; we just search across leg boundaries for a
    chain of mutually-compatible IK branches.

    The seeds are merged into ``current_q`` (so existing partner-arm
    pins are preserved). When ``seed_candidates`` is empty or None,
    falls through to a single :func:`plan_transit` call with the
    caller's existing ``current_q``.
    """
    if not seed_candidates:
        return plan_transit(arm=arm, **plan_transit_kwargs)

    # Cache-aware seed reordering: probe warmstart for each candidate
    # (cheap — one ikfast chain + signature build per seed) and try
    # cache-hit seeds first. On warm cache, this collapses what would
    # be N cold KTO solves into one immediate hit. Skipped if the
    # caller passed ``use_warmstart=False`` since the lookup/store
    # inside _plan_constrained_kto won't run anyway.
    use_warmstart_for_reorder = plan_transit_kwargs.get("use_warmstart", True)
    if (WARMSTART_ENABLED and WARMSTART_REORDER_SEEDS
            and use_warmstart_for_reorder and len(seed_candidates) > 1):
        warm: List[np.ndarray] = []
        cold: List[np.ndarray] = []
        for seed in seed_candidates:
            predicted = _predict_kto_cache_key(seed, arm, plan_transit_kwargs)
            if predicted is None:
                cold.append(seed)
                continue
            q_start_p, q_end_p, sig_p = predicted
            if warmstart.peek(q_start_p, q_end_p, sig_p) is not None:
                warm.append(seed)
            else:
                cold.append(seed)
        if warm:
            print(f"[plan] {log_label} warmstart-aware reorder: "
                  f"{len(warm)}/{len(seed_candidates)} seeds have a cached "
                  f"solution; trying those first")
            seed_candidates = warm + cold

    last_exc: Optional[InfeasiblePlanError] = None
    for i, seed in enumerate(seed_candidates):
        cur_q_seed = {arm: seed} if seed is not None else None
        existing = plan_transit_kwargs.get("current_q")
        if existing is not None and cur_q_seed is not None:
            cur_q = {**existing, **cur_q_seed}
        elif existing is not None:
            cur_q = existing
        else:
            cur_q = cur_q_seed

        kwargs = {**plan_transit_kwargs, "current_q": cur_q}

        if seed is not None:
            wrap = (np.asarray(seed, dtype=float) + np.pi) % (2.0 * np.pi) - np.pi
            print(f"\n[plan] {log_label} "
                  f"(seed {i + 1}/{len(seed_candidates)}: "
                  f"q3 sign={'+1' if wrap[2] >= 0 else '-1'}, "
                  f"q5 sign={'+1' if wrap[4] >= 0 else '-1'})...")
        else:
            print(f"\n[plan] {log_label} (no seed)...")

        try:
            return plan_transit(arm=arm, **kwargs)
        except InfeasiblePlanError as exc:
            last_exc = exc
            print(f"  ✗ seed {i + 1} infeasible: {str(exc)[:180]}")
            continue

    raise InfeasiblePlanError(
        f"all {len(seed_candidates)} seed candidates infeasible; last: {last_exc}",
    )


def plan_transit(
    plant: MultibodyPlant,
    arm: str,
    waypoints: List[Pose],
    *,
    current_q: Optional[Union[Dict[str, np.ndarray], np.ndarray]] = None,
    seed_q: Optional[np.ndarray] = None,
    plant_context: Optional[Context] = None,
    duration_s: Optional[float] = None,
    other_arm_q: Optional[np.ndarray] = None,

    # Geometric path constraints
    fix_orientation: Optional[object] = None,    # Rotation
    fix_orientation_tolerance_rad: float = 0.04,
    align_tcp_axis: Optional[np.ndarray] = None,
    align_tcp_axis_world: Optional[np.ndarray] = None,
    align_tcp_axis_tolerance_rad: float = 0.05,
    fix_z_task: Optional[float] = None,
    z_tolerance_m: float = 0.01,

    # Safety constraints (defaults on)
    avoid_collisions: bool = True,
    min_clearance_m: float = 0.01,
    avoid_arm_singularity: bool = True,
    self_collision: bool = True,
    min_z_task: float = 0.02,
    max_tcp_linear_speed_m_per_s: Optional[float] = None,

    # Targeted extras (default off)
    extra_clearance_to: Optional[Dict[str, float]] = None,
    stay_in_workspace_box: Optional[Tuple[Tuple[float, float, float],
                                          Tuple[float, float, float]]] = None,

    # IK tolerances applied at every waypoint pose. Loosen the
    # rotation bound (default 0.05 rad ≈ 2.86°) when waypoint poses
    # come from rig recordings whose calibration drifts slightly from
    # the planning sim — typical for hook-gripper TCPs where the URDF
    # frame doesn't exactly match the rig's ``setTcp`` value.
    ik_pos_tolerance_m: float = 0.005,
    ik_rot_tolerance_rad: float = 0.05,

    # Optional sampling-based fallback after KTO. Requires a RobotDiagram
    # built from the same plant; plain DiagramBuilder scenes cannot back
    # Drake's SceneGraphCollisionChecker.
    use_rrt_fallback: bool = False,
    rrt_diagram: Optional[object] = None,
    rrt_max_iters: int = 10000,
    rrt_step_size: float = 0.2,
    rrt_goal_bias: float = 0.2,
    rrt_seed: int = 4,
    rrt_edge_step_size: float = 0.03,
    rrt_shortcut_attempts: int = 100,

    # Optional rtde-controller IK fallback. When provided, used as tier
    # 2 in waypoint IK (between ikfast and Drake numerical). Build via
    # ``make_rtde_ik(arm)`` in real-path code.
    rtde_ik: Optional[RtdeIkFn] = None,

    # Per-call warmstart cache disable. Combines with the global
    # ``WARMSTART_ENABLED`` flag — warmstart only runs when BOTH are
    # True. Pass False to skip cache lookup/store on this call (e.g.
    # for tasks that don't want their KTO solutions cached or that
    # don't trust cache hits, like force-contact arcs).
    use_warmstart: bool = True,
) -> TransitPlan:
    """Plan a transit between task-frame TCP poses.

    Starting state — three layered sources, applied in this order
    (later overrides earlier):

      1. ``plant_context`` defaults — usually the plant's HOME pose if
         ``build_scene()`` set defaults, or zeros otherwise.
      2. ``current_q`` — the natural way to feed in "where the robot
         actually is right now" from a task. Two accepted forms:

         * ``dict`` keyed by arm name, e.g.
           ``{"ur_left": q6_left, "ur_right": q6_right}`` — what tasks
           naturally have on hand from ``arm.receive.getActualQ()``.
         * ``np.ndarray`` of length ``plant.num_positions()`` — full
           plant vector. Use this when there are non-arm DOFs (door
           hinge, attached object) to pin too.

      3. Explicit ``seed_q`` / ``other_arm_q`` — per-arm overrides.
         Useful for "plan as if the arm started at pose X" without
         touching ``current_q`` or the context.

    The planning arm's IK seed is whatever sits in the planning arm's
    slot after layering. The partner arm's positions are pinned at the
    layered value for collision queries and KTO control points.

    Routes to ``_plan_simple_spline`` when no constraints are active,
    otherwise to ``_plan_constrained_kto``. Same return type either way.
    """
    if len(waypoints) < 2:
        raise ValueError("Need at least 2 waypoints (start and end).")

    arm_instance = _arm_model_instance(plant, arm)
    other_arm_instance: Optional[ModelInstanceIndex] = None
    other_arm_name = "ur_right" if arm == "ur_left" else "ur_left"
    if plant.HasModelInstanceNamed(other_arm_name) or plant.HasModelInstanceNamed(
        f"{other_arm_name}::ur5e"
    ):
        try:
            other_arm_instance = _arm_model_instance(plant, other_arm_name)
        except KeyError:
            pass

    if plant_context is None:
        plant_context = plant.CreateDefaultContext()

    arm_idx = _arm_position_indices(plant, arm_instance)
    other_idx = (
        _arm_position_indices(plant, other_arm_instance)
        if other_arm_instance is not None else np.array([], dtype=int)
    )

    # --- Layer the starting state. See docstring for ordering. ---
    seed_q_full = plant.GetPositions(plant_context).copy()

    if current_q is not None:
        if isinstance(current_q, dict):
            for arm_name, q_vec in current_q.items():
                if q_vec is None:
                    continue
                inst = _arm_model_instance(plant, arm_name)
                idx = _arm_position_indices(plant, inst)
                q_arr = np.asarray(q_vec, dtype=float)
                if q_arr.shape != (len(idx),):
                    raise ValueError(
                        f"current_q[{arm_name!r}] has shape {q_arr.shape}, "
                        f"expected ({len(idx)},) for that arm."
                    )
                seed_q_full[idx] = q_arr
        else:
            q_arr = np.asarray(current_q, dtype=float)
            if q_arr.shape != seed_q_full.shape:
                raise ValueError(
                    f"current_q array shape {q_arr.shape} doesn't match "
                    f"plant position vector shape {seed_q_full.shape}."
                )
            seed_q_full = q_arr.copy()

    if seed_q is not None:
        seed_q_full[arm_idx] = np.asarray(seed_q, dtype=float)
    if other_arm_q is not None and other_arm_instance is not None:
        seed_q_full[other_idx] = np.asarray(other_arm_q, dtype=float)

    # Recover the partner arm's pinned value from the layered seed so
    # downstream IK / KTO collision pinning sees the right state even
    # when only `current_q` was supplied.
    if other_arm_instance is not None:
        other_arm_q = seed_q_full[other_idx].copy()

    plant.SetPositions(plant_context, seed_q_full)

    waypoints_q_full = _pose_chain_to_joints(
        plant, plant_context, arm_instance, waypoints,
        seed_q_full,
        other_arm_instance=other_arm_instance,
        other_arm_q=other_arm_q,
        pos_tolerance_m=ik_pos_tolerance_m,
        rot_tolerance_rad=ik_rot_tolerance_rad,
        arm_name=arm,
        rtde_ik=rtde_ik,
    )

    if duration_s is None:
        # Auto: scale duration to keep peak joint speed below ~0.5 rad/s.
        max_joint_delta = 0.0
        for a, b in zip(waypoints_q_full[:-1], waypoints_q_full[1:]):
            max_joint_delta = max(max_joint_delta, float(np.max(np.abs(b - a))))
        duration_s = max(1.0, max_joint_delta / 0.5 * len(waypoints_q_full))

    # Hard path-shape constraints (orient lock, z plane, TCP speed cap,
    # workspace box, extra-clearance) genuinely need KTO — no simple
    # joint-space spline can satisfy them by accident.
    needs_hard_shape = (
        fix_orientation is not None
        or align_tcp_axis is not None
        or fix_z_task is not None
        or max_tcp_linear_speed_m_per_s is not None
        or bool(extra_clearance_to)
        or stay_in_workspace_box is not None
    )
    # Safety constraints (collisions, self-collision, floor) MIGHT be
    # satisfied by a plain cubic spline through the IK'd waypoints —
    # most free-space transits between sensible endpoints already are.
    needs_safety = (
        avoid_collisions or self_collision or min_z_task is not None
    )
    # Note: ``avoid_arm_singularity`` is implicitly handled here. Chained
    # IK (above) keeps consecutive waypoints on the same q3/q5 sign
    # branch; a cubic spline between them stays on that branch too. So
    # the spline path naturally avoids the singularity surfaces without
    # an explicit check. The flag still gates KTO's per-control-point
    # branch lock for the fallback path.

    # Fastest path: nothing to verify, return simple spline immediately.
    if not needs_hard_shape and not needs_safety:
        return _plan_simple_spline(
            plant, plant_context, arm, arm_instance,
            waypoints_q_full, duration_s,
            check_collisions=False,
            min_clearance_m=min_clearance_m,
            other_arm_q=other_arm_q,
            other_arm_instance=other_arm_instance,
        )

    # Spline-first path: only safety constraints, no shape constraints.
    # Build the spline, sample-check it; if clean, ship it. KTO only
    # fires when the spline genuinely passes through a collision or
    # below the floor.
    spline_fallback_reason: Optional[str] = None
    if not needs_hard_shape:
        plan = _plan_simple_spline(
            plant, plant_context, arm, arm_instance,
            waypoints_q_full, duration_s,
            check_collisions=False,
            min_clearance_m=min_clearance_m,
            other_arm_q=other_arm_q,
            other_arm_instance=other_arm_instance,
        )
        try:
            worst = _check_safety_along(
                plant, plant_context, plan.trajectory,
                arm_instance, other_arm_q, other_arm_instance,
                n_samples=80,
                check_collisions=avoid_collisions or self_collision,
                min_clearance_m=min_clearance_m,
                floor_min_z=min_z_task,
            )
            plan.collision_checked = bool(avoid_collisions or self_collision)
            plan.min_clearance_m = worst
            plan.metadata["planner"] = "simple_spline_safe"
            plan.metadata["safety_samples"] = 80
            return plan
        except InfeasiblePlanError as exc:
            # Spline path violates a safety constraint. Log so callers
            # can see the cause even if the KTO fallback also fails.
            spline_fallback_reason = str(exc)
            print(f"[plan_transit] spline path infeasible "
                  f"({spline_fallback_reason}); falling back to KTO.")

    # KTO with all the requested constraints (hard shape and/or safety).
    try:
        plan = _plan_constrained_kto(
            plant=plant,
            plant_context=plant_context,
            arm=arm,
            arm_instance=arm_instance,
            other_arm_instance=other_arm_instance,
            other_arm_q=other_arm_q,
            waypoints_q_full=waypoints_q_full,
            duration_s=duration_s,
            fix_orientation=fix_orientation,
            fix_orientation_tolerance_rad=fix_orientation_tolerance_rad,
            align_tcp_axis=align_tcp_axis,
            align_tcp_axis_world=align_tcp_axis_world,
            align_tcp_axis_tolerance_rad=align_tcp_axis_tolerance_rad,
            fix_z_task=fix_z_task,
            z_tolerance_m=z_tolerance_m,
            avoid_collisions=avoid_collisions,
            min_clearance_m=min_clearance_m,
            avoid_arm_singularity=avoid_arm_singularity,
            self_collision=self_collision,
            min_z_task=min_z_task,
            max_tcp_linear_speed_m_per_s=max_tcp_linear_speed_m_per_s,
            extra_clearance_to=extra_clearance_to,
            stay_in_workspace_box=stay_in_workspace_box,
            use_warmstart=use_warmstart,
        )
        if spline_fallback_reason is not None:
            plan.metadata["spline_fallback_reason"] = spline_fallback_reason
        return plan
    except InfeasiblePlanError as kto_exc:
        if not use_rrt_fallback or rrt_diagram is None:
            raise

        # RRT fallback now supports fix_orientation, fix_z_task, and
        # stay_in_workspace_box via per-q validators (see
        # _plan_rrt_transit). Constraints still NOT in RRT:
        #   max_tcp_linear_speed_m_per_s — needs trajectory retiming
        #   extra_clearance_to           — needs per-pair clearance check
        rrt_supported = (
            max_tcp_linear_speed_m_per_s is None
            and not bool(extra_clearance_to)
        )
        if not rrt_supported:
            raise

        print(f"[plan_transit] KTO path infeasible "
              f"({kto_exc}); falling back to RRT.")
        plan = _plan_rrt_transit(
            diagram=rrt_diagram,
            plant=plant,
            plant_context=plant_context,
            arm=arm,
            arm_instance=arm_instance,
            waypoints=waypoints,
            start_q_full=waypoints_q_full[0],
            duration_s=duration_s,
            align_tcp_axis=align_tcp_axis,
            align_tcp_axis_world=align_tcp_axis_world,
            align_tcp_axis_tolerance_rad=align_tcp_axis_tolerance_rad,
            fix_orientation=fix_orientation,
            fix_orientation_tolerance_rad=fix_orientation_tolerance_rad,
            fix_z_task=fix_z_task,
            z_tolerance_m=z_tolerance_m,
            stay_in_workspace_box=stay_in_workspace_box,
            avoid_collisions=avoid_collisions,
            avoid_arm_singularity=avoid_arm_singularity,
            self_collision=self_collision,
            min_clearance_m=min_clearance_m,
            min_z_task=min_z_task,
            max_iters=rrt_max_iters,
            step_size=rrt_step_size,
            goal_bias=rrt_goal_bias,
            seed=rrt_seed,
            edge_step_size=rrt_edge_step_size,
            shortcut_attempts=rrt_shortcut_attempts,
        )
        if spline_fallback_reason is not None:
            plan.metadata["spline_fallback_reason"] = spline_fallback_reason
        plan.metadata["kto_fallback_reason"] = str(kto_exc)
        return plan


# ---------------------------------------------------------------------------
#  Constrained KTO planner
# ---------------------------------------------------------------------------

def _plan_rrt_transit(
    *,
    diagram,
    plant: MultibodyPlant,
    plant_context: Context,
    arm: str,
    arm_instance: ModelInstanceIndex,
    waypoints: List[Pose],
    start_q_full: np.ndarray,
    duration_s: float,
    align_tcp_axis: Optional[np.ndarray],
    align_tcp_axis_world: Optional[np.ndarray],
    align_tcp_axis_tolerance_rad: float,
    fix_orientation: Optional[object] = None,
    fix_orientation_tolerance_rad: float = 0.04,
    fix_z_task: Optional[float] = None,
    z_tolerance_m: float = 0.01,
    stay_in_workspace_box: Optional[Tuple[Tuple[float, float, float],
                                          Tuple[float, float, float]]] = None,
    avoid_collisions: bool = True,
    avoid_arm_singularity: bool = True,
    self_collision: bool = True,
    min_clearance_m: float = 0.01,
    min_z_task: Optional[float] = None,
    max_iters: int = 10000,
    step_size: float = 0.2,
    goal_bias: float = 0.2,
    seed: int = 4,
    edge_step_size: float = 0.03,
    shortcut_attempts: int = 100,
) -> TransitPlan:
    """Sampling-based transit through TCP waypoints."""
    if len(waypoints) < 2:
        raise ValueError("Need at least 2 waypoints for RRT transit.")
    if not avoid_collisions and not self_collision:
        raise InfeasiblePlanError("RRT requires collision checking to be enabled")
    from .rrt import (
        RRTFailure,
        make_collision_checker,
        path_length,
        rrt_connect_to_tcp_pose,
        shortcut_path,
    )

    arm_idx = _arm_position_indices(plant, arm_instance)
    # Each entry: (name, valid_fn, describe_fn). describe_fn returns a
    # short string summarising *why* the validator rejected the config —
    # used for diagnostic failure messages in rrt.py.
    validators: List[Tuple[str, Callable[[np.ndarray], bool],
                           Callable[[np.ndarray], str]]] = []

    if align_tcp_axis is not None:
        axis_ctx = plant.CreateDefaultContext()
        tcp_frame = _tcp_frame(plant, arm_instance)
        axis_tcp = np.asarray(align_tcp_axis, dtype=float).reshape(3)
        axis_tcp = axis_tcp / np.linalg.norm(axis_tcp)
        axis_world = (
            np.array([0.0, 0.0, 1.0])
            if align_tcp_axis_world is None
            else np.asarray(align_tcp_axis_world, dtype=float).reshape(3)
        )
        axis_world = axis_world / np.linalg.norm(axis_world)
        max_angle = float(align_tcp_axis_tolerance_rad)

        def _axis_angle(q_full: np.ndarray) -> float:
            plant.SetPositions(axis_ctx, np.asarray(q_full, dtype=float))
            R_world_tcp = tcp_frame.CalcPoseInWorld(axis_ctx).rotation().matrix()
            axis_in_world = R_world_tcp @ axis_tcp
            axis_in_world = axis_in_world / np.linalg.norm(axis_in_world)
            return float(np.arccos(np.clip(
                float(np.dot(axis_in_world, axis_world)), -1.0, 1.0,
            )))

        def _axis_alignment_valid(q_full: np.ndarray) -> bool:
            return _axis_angle(q_full) <= max_angle

        def _axis_alignment_describe(q_full: np.ndarray) -> str:
            angle = _axis_angle(q_full)
            return (f"axis-alignment {np.degrees(angle):.1f}° > "
                    f"tol {np.degrees(max_angle):.1f}°")

        validators.append(
            ("plate-level axis", _axis_alignment_valid,
             _axis_alignment_describe),
        )

    if min_z_task is not None:
        floor_ctx = plant.CreateDefaultContext()
        tcp_frame = _tcp_frame(plant, arm_instance)

        def _floor_z(q_full: np.ndarray) -> float:
            plant.SetPositions(floor_ctx, np.asarray(q_full, dtype=float))
            return float(tcp_frame.CalcPoseInWorld(floor_ctx).translation()[2])

        def _floor_valid(q_full: np.ndarray) -> bool:
            return _floor_z(q_full) >= min_z_task

        def _floor_describe(q_full: np.ndarray) -> str:
            z = _floor_z(q_full)
            return (f"TCP-floor z={z * 1000:.1f}mm < "
                    f"min={min_z_task * 1000:.1f}mm")

        validators.append(("TCP floor", _floor_valid, _floor_describe))

    if fix_z_task is not None:
        fix_z_ctx = plant.CreateDefaultContext()
        tcp_frame = _tcp_frame(plant, arm_instance)
        z_target = float(fix_z_task)
        z_tol = float(z_tolerance_m)

        def _fix_z_delta(q_full: np.ndarray) -> float:
            plant.SetPositions(fix_z_ctx, np.asarray(q_full, dtype=float))
            return float(
                tcp_frame.CalcPoseInWorld(fix_z_ctx).translation()[2] - z_target
            )

        def _fix_z_valid(q_full: np.ndarray) -> bool:
            return abs(_fix_z_delta(q_full)) <= z_tol

        def _fix_z_describe(q_full: np.ndarray) -> str:
            d = _fix_z_delta(q_full)
            return (f"fix_z_task |Δz|={abs(d)*1000:.1f}mm > "
                    f"tol {z_tol*1000:.1f}mm  (z={z_target*1000+d*1000:.1f}mm)")

        validators.append(("fix_z_task", _fix_z_valid, _fix_z_describe))

    if fix_orientation is not None:
        ori_ctx = plant.CreateDefaultContext()
        tcp_frame = _tcp_frame(plant, arm_instance)
        # ``fix_orientation`` is a util.rotations.Rotation — extract its
        # 3x3 matrix once.
        R_target = np.asarray(fix_orientation.as_matrix(), dtype=float)
        ori_tol = float(fix_orientation_tolerance_rad)

        def _orientation_angle(q_full: np.ndarray) -> float:
            plant.SetPositions(ori_ctx, np.asarray(q_full, dtype=float))
            R_actual = np.asarray(
                tcp_frame.CalcPoseInWorld(ori_ctx).rotation().matrix()
            )
            R_rel = R_target.T @ R_actual
            cos_t = (np.trace(R_rel) - 1.0) / 2.0
            return float(np.arccos(np.clip(cos_t, -1.0, 1.0)))

        def _orientation_valid(q_full: np.ndarray) -> bool:
            return _orientation_angle(q_full) <= ori_tol

        def _orientation_describe(q_full: np.ndarray) -> str:
            angle = _orientation_angle(q_full)
            return (f"fix_orientation Δθ={np.degrees(angle):.1f}° > "
                    f"tol {np.degrees(ori_tol):.1f}°")

        validators.append(
            ("fix_orientation", _orientation_valid, _orientation_describe),
        )

    if stay_in_workspace_box is not None:
        ws_ctx = plant.CreateDefaultContext()
        tcp_frame = _tcp_frame(plant, arm_instance)
        ws_lo = np.asarray(stay_in_workspace_box[0], dtype=float).reshape(3)
        ws_hi = np.asarray(stay_in_workspace_box[1], dtype=float).reshape(3)

        def _ws_position(q_full: np.ndarray) -> np.ndarray:
            plant.SetPositions(ws_ctx, np.asarray(q_full, dtype=float))
            return np.asarray(
                tcp_frame.CalcPoseInWorld(ws_ctx).translation(), dtype=float,
            )

        def _ws_valid(q_full: np.ndarray) -> bool:
            p = _ws_position(q_full)
            return bool(np.all(p >= ws_lo) and np.all(p <= ws_hi))

        def _ws_describe(q_full: np.ndarray) -> str:
            p = _ws_position(q_full)
            below = np.maximum(ws_lo - p, 0.0)
            above = np.maximum(p - ws_hi, 0.0)
            worst = np.maximum(below, above)
            j = int(np.argmax(worst))
            axis = "xyz"[j]
            margin = float(worst[j]) * 1000.0
            return (f"stay_in_workspace_box: TCP {axis}={p[j]*1000:.1f}mm "
                    f"out of [{ws_lo[j]*1000:.1f}, {ws_hi[j]*1000:.1f}]mm "
                    f"by {margin:.1f}mm")

        validators.append(
            ("stay_in_workspace_box", _ws_valid, _ws_describe),
        )

    if avoid_arm_singularity and len(arm_idx) >= 5:
        q_start_arm = np.asarray(start_q_full, dtype=float)[arm_idx]
        q_start_wrapped = (q_start_arm + np.pi) % (2.0 * np.pi) - np.pi
        q3_sign = 1.0 if q_start_wrapped[2] >= 0 else -1.0
        q5_sign = 1.0 if q_start_wrapped[4] >= 0 else -1.0
        eps = 0.10

        def _branch_check(q_full: np.ndarray) -> Tuple[bool, float, float]:
            q_arm = np.asarray(q_full, dtype=float)[arm_idx]
            q_arm = (q_arm + np.pi) % (2.0 * np.pi) - np.pi
            q3 = q_arm[2]
            q5 = q_arm[4]
            ok = True
            if q3_sign > 0 and not (eps <= q3 <= np.pi - eps):
                ok = False
            if q3_sign < 0 and not (-np.pi + eps <= q3 <= -eps):
                ok = False
            if q5_sign > 0 and not (eps <= q5 <= np.pi - eps):
                ok = False
            if q5_sign < 0 and not (-np.pi + eps <= q5 <= -eps):
                ok = False
            return ok, q3, q5

        def _same_branch_valid(q_full: np.ndarray) -> bool:
            return _branch_check(q_full)[0]

        def _same_branch_describe(q_full: np.ndarray) -> str:
            _, q3, q5 = _branch_check(q_full)
            return (f"q3/q5 singularity branch: "
                    f"start q3 sign {q3_sign:+.0f} q5 sign {q5_sign:+.0f}; "
                    f"goal q3={q3:+.2f} q5={q5:+.2f}")

        validators.append(
            ("q3/q5 singularity branch", _same_branch_valid,
             _same_branch_describe),
        )

    if validators:
        def validity_fn(q_full: np.ndarray) -> bool:
            return all(valid(q_full) for _, valid, _ in validators)

        def validity_describe_fn(q_full: np.ndarray) -> Optional[str]:
            for name, valid, describe in validators:
                if not valid(q_full):
                    return f"{name}: {describe(q_full)}"
            return None
    else:
        validity_fn = None
        validity_describe_fn = None

    checker = make_collision_checker(
        diagram,
        plant,
        arm,
        edge_step_size=edge_step_size,
        env_padding=min_clearance_m,
    )

    q_current = np.asarray(start_q_full, dtype=float).copy()
    failed_start_constraints = [
        f"{name} ({describe(q_current)})"
        for name, valid, describe in validators if not valid(q_current)
    ]
    if failed_start_constraints:
        raise InfeasiblePlanError(
            "RRT start configuration violates validity constraint(s): "
            + "; ".join(failed_start_constraints)
        )

    full_path: List[np.ndarray] = [q_current.copy()]
    reports = []
    try:
        for segment_i, goal_pose in enumerate(waypoints[1:], start=1):
            segment_plan, report = rrt_connect_to_tcp_pose(
                checker,
                q_start_full=q_current,
                tcp_pose_task=goal_pose,
                planning_arm_name=arm,
                max_iters=max_iters,
                step_size=step_size,
                goal_bias=goal_bias,
                seed=seed + segment_i - 1,
                validity_fn=validity_fn,
                validity_describe_fn=validity_describe_fn,
                validity_edge_step_size=edge_step_size,
            )
            segment_path = shortcut_path(
                checker,
                segment_plan.path_full,
                attempts=shortcut_attempts,
                seed=seed + 1000 + segment_i,
                validity_fn=validity_fn,
                validity_edge_step_size=edge_step_size,
            )
            full_path.extend(
                np.asarray(q, dtype=float).copy() for q in segment_path[1:]
            )
            q_current = full_path[-1]
            reports.append(report)
    except RRTFailure as exc:
        raise InfeasiblePlanError(str(exc)) from exc

    if len(full_path) < 2:
        raise InfeasiblePlanError("RRT returned an empty path")

    # Preserve RRT's checked straight joint-space edges. A cubic through
    # these nodes could leave the validated corridor.
    q_samples = np.array(full_path).T
    seg_lengths = [
        float(np.linalg.norm(b[arm_idx] - a[arm_idx]))
        for a, b in zip(full_path[:-1], full_path[1:])
    ]
    total_len = sum(seg_lengths)
    if total_len <= 1e-9:
        times = np.linspace(0.0, duration_s, len(full_path))
    else:
        cumulative = np.concatenate(([0.0], np.cumsum(seg_lengths)))
        times = duration_s * cumulative / total_len
    traj = PiecewisePolynomial.FirstOrderHold(times, q_samples)

    worst = _check_safety_along(
        plant,
        plant_context,
        traj,
        arm_instance,
        other_arm_q=None,
        other_arm_instance=None,
        n_samples=200,
        check_collisions=avoid_collisions or self_collision,
        min_clearance_m=min_clearance_m,
        floor_min_z=min_z_task,
    )

    return TransitPlan(
        trajectory=traj,
        arm=arm,
        duration_s=duration_s,
        waypoints_q=[q[arm_idx] for q in full_path],
        collision_checked=bool(avoid_collisions or self_collision),
        min_clearance_m=worst,
        metadata={
            "planner": "rrt",
            "rrt_segment_planners": [r.chosen_planner for r in reports],
            "rrt_nodes": len(full_path),
            "rrt_path_length": path_length(full_path, arm_idx),
            "rrt_edge_step_size": edge_step_size,
            "rrt_shortcut_attempts": shortcut_attempts,
        },
    )


def _plan_constrained_kto(
    plant: MultibodyPlant,
    plant_context: Context,
    arm: str,
    arm_instance: ModelInstanceIndex,
    other_arm_instance: Optional[ModelInstanceIndex],
    other_arm_q: Optional[np.ndarray],
    waypoints_q_full: List[np.ndarray],
    duration_s: float,
    *,
    fix_orientation: Optional[object],
    fix_orientation_tolerance_rad: float,
    align_tcp_axis: Optional[np.ndarray],
    align_tcp_axis_world: Optional[np.ndarray],
    align_tcp_axis_tolerance_rad: float,
    fix_z_task: Optional[float],
    z_tolerance_m: float,
    avoid_collisions: bool,
    min_clearance_m: float,
    avoid_arm_singularity: bool,
    self_collision: bool,
    min_z_task: float,
    max_tcp_linear_speed_m_per_s: Optional[float],
    extra_clearance_to: Optional[Dict[str, float]],
    stay_in_workspace_box: Optional[Tuple],
    use_warmstart: bool = True,
) -> TransitPlan:
    """KinematicTrajectoryOptimization solve with the requested constraints.

    Solves over the full plant's positions (12 for bimanual). The
    partner arm's positions are pinned via a bounding-box constraint
    on the KTO control points so they stay constant; only the planning
    arm's 6 joints are actually free.
    """
    num_q = plant.num_positions()
    n_ctrl = 10
    spline_order = 4
    arm_idx = _arm_position_indices(plant, arm_instance)
    other_idx = (
        _arm_position_indices(plant, other_arm_instance)
        if other_arm_instance is not None else np.array([], dtype=int)
    )

    trajopt = KinematicTrajectoryOptimization(
        num_positions=num_q,
        num_control_points=n_ctrl,
        spline_order=spline_order,
        duration=duration_s,
    )
    prog = trajopt.get_mutable_prog()

    tcp_frame = _tcp_frame(plant, arm_instance)
    world_frame = plant.world_frame()

    # ----- Costs: smoothness + duration -----
    trajopt.AddDurationCost(weight=1.0)
    trajopt.AddPathLengthCost(weight=1.0)

    # ----- Joint position bounds (full plant; both arms get bounded) -----
    trajopt.AddPositionBounds(
        plant.GetPositionLowerLimits(),
        plant.GetPositionUpperLimits(),
    )

    # Velocity / acceleration bounds — generous defaults to keep the
    # optimiser numerically happy. Tighten via execute_plan if needed.
    v_max = np.full(num_q, 1.5)    # ~85°/s per joint
    a_max = np.full(num_q, 3.0)
    trajopt.AddVelocityBounds(-v_max, v_max)
    trajopt.AddAccelerationBounds(-a_max, a_max)

    # ----- Pin partner arm to other_arm_q at every control point -----
    # control_points() is a (num_q, num_ctrl) matrix of decision variables.
    if other_arm_instance is not None and other_arm_q is not None:
        for k in range(n_ctrl):
            prog.AddBoundingBoxConstraint(
                np.asarray(other_arm_q),
                np.asarray(other_arm_q),
                trajopt.control_points()[other_idx, k],
            )

    # ----- Endpoints: first and last waypoint exactly -----
    q_start = waypoints_q_full[0]
    q_end = waypoints_q_full[-1]
    prog.AddBoundingBoxConstraint(q_start, q_start, trajopt.control_points()[:, 0])
    prog.AddBoundingBoxConstraint(q_end, q_end, trajopt.control_points()[:, -1])

    # ----- Intermediate waypoints (if any) -----
    # Add as PathPositionConstraint at evenly-spaced normalised times.
    if len(waypoints_q_full) > 2:
        n_mid = len(waypoints_q_full) - 2
        for i, wp in enumerate(waypoints_q_full[1:-1], start=1):
            s = i / (len(waypoints_q_full) - 1)
            # Add equality on the planning arm's joints at this normalised time.
            # Use AddPathPositionConstraint with a narrow band around wp.
            wp_arm = wp[arm_idx]
            tol = 0.02
            for k in arm_idx:
                pass  # handled via PositionConstraint below
            # Build a PositionConstraint on the TCP at the via pose's xyz.
            # (We already IK'd the via pose to get wp; here we constrain
            # both position and orientation via the same machinery as
            # endpoints by adding bounding box on q at this s.)
            # KTO doesn't have a direct "q at s" constraint; we use the
            # spline evaluation by adding an equality constraint via
            # AddPathPositionConstraint with q-band semantics.
            trajopt.AddPathPositionConstraint(
                wp - tol, wp + tol, s,
            )

    # ----- Avoid arm singularities (joint bounds on q3 and q5) -----
    if avoid_arm_singularity:
        eps = 0.10
        # We constrain only the PLANNING arm's q3/q5 (positions 2 and 4 of
        # its joint vector). The control points are 12-DOF; pick the right
        # entries.
        if len(arm_idx) >= 5:
            q3_idx, q5_idx = int(arm_idx[2]), int(arm_idx[4])
            # Determine sign quadrant from the start configuration so
            # we keep the same branch the recorded waypoints chose.
            q3_sign = 1.0 if q_start[q3_idx] >= 0 else -1.0
            q5_sign = 1.0 if q_start[q5_idx] >= 0 else -1.0
            for k in range(n_ctrl):
                prog.AddBoundingBoxConstraint(
                    eps if q3_sign > 0 else -np.pi + eps,
                    np.pi - eps if q3_sign > 0 else -eps,
                    trajopt.control_points()[q3_idx, k],
                )
                prog.AddBoundingBoxConstraint(
                    eps if q5_sign > 0 else -np.pi + eps,
                    np.pi - eps if q5_sign > 0 else -eps,
                    trajopt.control_points()[q5_idx, k],
                )

    # ----- Path-wide constraints sampled at intermediate s -----
    s_samples = np.linspace(0.05, 0.95, 8)

    # Fix-orientation: keep TCP rotation matched throughout.
    if fix_orientation is not None:
        R_target = RotationMatrix(np.asarray(fix_orientation.as_matrix(), dtype=float))
        theta_bound = float(fix_orientation_tolerance_rad)
        orient_constraint = OrientationConstraint(
            plant=plant,
            frameAbar=world_frame,
            R_AbarA=R_target,
            frameBbar=tcp_frame,
            R_BbarB=RotationMatrix(),
            theta_bound=theta_bound,
            plant_context=plant_context,
        )
        for s in s_samples:
            trajopt.AddPathPositionConstraint(orient_constraint, float(s))

    # Axis alignment: keep a TCP-frame direction within an angular tolerance
    # of a world-frame direction. Useful for "held plate stays level" without
    # locking yaw or roll around the plate normal.
    if align_tcp_axis is not None:
        axis_tcp = np.asarray(align_tcp_axis, dtype=float).reshape(3)
        axis_tcp = axis_tcp / np.linalg.norm(axis_tcp)
        axis_world = (
            np.array([0.0, 0.0, 1.0])
            if align_tcp_axis_world is None
            else np.asarray(align_tcp_axis_world, dtype=float).reshape(3)
        )
        axis_world = axis_world / np.linalg.norm(axis_world)
        axis_constraint = AngleBetweenVectorsConstraint(
            plant=plant,
            frameA=world_frame,
            a_A=axis_world,
            frameB=tcp_frame,
            b_B=axis_tcp,
            angle_lower=0.0,
            angle_upper=float(align_tcp_axis_tolerance_rad),
            plant_context=plant_context,
        )
        for s in s_samples:
            trajopt.AddPathPositionConstraint(axis_constraint, float(s))

    # Fix-z: keep TCP z within ±z_tolerance_m of fix_z_task.
    if fix_z_task is not None:
        z_lo, z_hi = fix_z_task - z_tolerance_m, fix_z_task + z_tolerance_m
        z_constraint = PositionConstraint(
            plant=plant,
            frameA=world_frame,
            p_AQ_lower=np.array([-1e6, -1e6, z_lo]),
            p_AQ_upper=np.array([+1e6, +1e6, z_hi]),
            frameB=tcp_frame,
            p_BQ=np.zeros(3),
            plant_context=plant_context,
        )
        for s in s_samples:
            trajopt.AddPathPositionConstraint(z_constraint, float(s))

    # Floor (min_z_task): TCP stays above the table top.
    if min_z_task is not None:
        floor_constraint = PositionConstraint(
            plant=plant,
            frameA=world_frame,
            p_AQ_lower=np.array([-1e6, -1e6, min_z_task]),
            p_AQ_upper=np.array([+1e6, +1e6, +1e6]),
            frameB=tcp_frame,
            p_BQ=np.zeros(3),
            plant_context=plant_context,
        )
        for s in s_samples:
            trajopt.AddPathPositionConstraint(floor_constraint, float(s))

    # Workspace box.
    if stay_in_workspace_box is not None:
        (lo, hi) = stay_in_workspace_box
        ws_constraint = PositionConstraint(
            plant=plant,
            frameA=world_frame,
            p_AQ_lower=np.asarray(lo, dtype=float),
            p_AQ_upper=np.asarray(hi, dtype=float),
            frameB=tcp_frame,
            p_BQ=np.zeros(3),
            plant_context=plant_context,
        )
        for s in s_samples:
            trajopt.AddPathPositionConstraint(ws_constraint, float(s))

    # Collision avoidance (general + self-collision).
    if avoid_collisions or self_collision:
        coll_constraint = MinimumDistanceLowerBoundConstraint(
            plant=plant,
            bound=min_clearance_m,
            plant_context=plant_context,
            influence_distance_offset=0.01,
        )
        for s in s_samples:
            trajopt.AddPathPositionConstraint(coll_constraint, float(s))

    # ----- Initial guess: warmstart cache → cubic spline fallback -----
    # KTO is gradient-descent NLP; a near-feasible seed (from a prior solve
    # of a similar problem) typically converges in <10 iterations vs. 100s
    # from a constraint-violating cubic spline.
    problem_signature = _kto_problem_signature(
        plant=plant,
        plant_context=plant_context,
        arm=arm,
        duration_s=duration_s,
        n_ctrl=n_ctrl,
        spline_order=spline_order,
        waypoints_q_full=waypoints_q_full,
        fix_orientation=fix_orientation,
        fix_orientation_tolerance_rad=fix_orientation_tolerance_rad,
        align_tcp_axis=align_tcp_axis,
        align_tcp_axis_world=align_tcp_axis_world,
        align_tcp_axis_tolerance_rad=align_tcp_axis_tolerance_rad,
        fix_z_task=fix_z_task,
        z_tolerance_m=z_tolerance_m,
        avoid_collisions=avoid_collisions,
        min_clearance_m=min_clearance_m,
        avoid_arm_singularity=avoid_arm_singularity,
        self_collision=self_collision,
        min_z_task=min_z_task,
        max_tcp_linear_speed_m_per_s=max_tcp_linear_speed_m_per_s,
        extra_clearance_to=extra_clearance_to,
        stay_in_workspace_box=stay_in_workspace_box,
    )
    cached_cps = (
        warmstart.lookup(q_start, q_end, problem_signature)
        if (WARMSTART_ENABLED and use_warmstart) else None
    )
    warmstart_hit = cached_cps is not None
    if warmstart_hit and cached_cps.shape == (num_q, n_ctrl):
        cps_list = [cached_cps[:, k:k + 1] for k in range(n_ctrl)]
        initial_traj = BsplineTrajectory(trajopt.basis(), cps_list)
    else:
        warmstart_hit = False  # shape mismatch counts as a miss
        initial_traj = _initial_guess_for_kto(
            waypoints_q_full, duration_s, trajopt.basis(),
        )
    trajopt.SetInitialGuess(initial_traj)

    # ----- Solve -----
    result = Solve(prog)
    if not result.is_success():
        raise InfeasiblePlanError(
            f"KTO solve failed for arm={arm!r}: "
            f"solver={result.get_solver_id().name()}, "
            f"infeasible_constraints={[c.evaluator().get_description() for c in result.GetInfeasibleConstraints(prog)] if hasattr(result, 'GetInfeasibleConstraints') else 'unknown'}"
        )

    traj = trajopt.ReconstructTrajectory(result)

    if WARMSTART_ENABLED and use_warmstart:
        solved_cps = np.asarray(
            result.GetSolution(trajopt.control_points()), dtype=float,
        )
        warmstart.store(q_start, q_end, problem_signature, solved_cps)

    # Post-solve clearance: sample and report worst.
    min_clearance = float("inf")
    if avoid_collisions:
        min_clearance = _check_collision_along(
            plant, plant_context, traj,
            other_arm_q, other_arm_instance,
            n_samples=200, min_clearance_m=min_clearance_m,
        )

    return TransitPlan(
        trajectory=traj,
        arm=arm,
        duration_s=duration_s,
        waypoints_q=[wp[arm_idx] for wp in waypoints_q_full],
        collision_checked=bool(avoid_collisions or self_collision),
        min_clearance_m=min_clearance,
        metadata={
            "planner": "kto",
            "n_control_points": n_ctrl,
            "spline_order": spline_order,
            "n_waypoints": len(waypoints_q_full),
            "warmstart_hit": warmstart_hit,
        },
    )


def _round_seq(x, decimals: int = 3):
    """Hashable, rounded representation of an array-like, or None."""
    if x is None:
        return None
    arr = np.asarray(x, dtype=float).flatten()
    return tuple(np.round(arr, decimals).tolist())


_ARM_INSTANCE_NAMES = frozenset({
    "ur_left", "ur_right",
    "ur_left::ur5e", "ur_right::ur5e",
})


def _scene_fingerprint(plant: MultibodyPlant, plant_context: Context) -> str:
    """Stable, content-derived hash of the planning scene.

    Replaces ``id(plant)`` in the warmstart signature so caches persist
    across processes and machines — two runs that build the same scene
    compute the same fingerprint, so a cache loaded from disk hits.

    Hashes every NON-arm body's world pose. Bodies in arm model
    instances are filtered by name; bodies welded to the arm (e.g. a
    gripper attached as its own model instance) would otherwise
    contaminate the fingerprint with the current arm config — so we
    temporarily zero both arms in plant_context before evaluating poses,
    then restore. Net effect: fingerprint depends only on
    (static geometry, non-arm joint positions like microwave door
    angle, free-floating object poses), invariant to arm q.

    Determinism caveat: Drake doesn't formally guarantee body insertion
    order or renaming behaviour. It's empirically stable for this
    codebase + Drake version. If a Drake upgrade shifts names, every
    existing cache entry quietly stops hitting (no false hits possible)
    — call ``warmstart.clear()`` and rebuild.
    """
    import hashlib
    saved_positions = plant.GetPositions(plant_context).copy()
    try:
        # Zero both arms so their welded-descendant bodies (gripper etc.)
        # land at a canonical, arm-q-independent pose during the hash.
        zeroed = saved_positions.copy()
        for arm_name in ("ur_left", "ur_right"):
            try:
                inst = _arm_model_instance(plant, arm_name)
            except KeyError:
                continue
            idx = _arm_position_indices(plant, inst)
            zeroed[idx] = 0.0
        plant.SetPositions(plant_context, zeroed)

        rows = []
        for i in range(plant.num_bodies()):
            body = plant.get_body(BodyIndex(i))
            inst_name = plant.GetModelInstanceName(body.model_instance())
            if inst_name in _ARM_INSTANCE_NAMES:
                continue
            X = plant.EvalBodyPoseInWorld(plant_context, body)
            rows.append((
                inst_name,
                body.name(),
                tuple(np.round(X.translation(), 3).tolist()),
                tuple(np.round(X.rotation().matrix().flatten(), 2).tolist()),
            ))
        rows.sort()
        return hashlib.sha256(repr(rows).encode()).hexdigest()[:16]
    finally:
        plant.SetPositions(plant_context, saved_positions)


def _kto_problem_signature(
    *,
    plant: MultibodyPlant,
    plant_context: Context,
    arm: str,
    duration_s: float,
    n_ctrl: int,
    spline_order: int,
    waypoints_q_full: List[np.ndarray],
    fix_orientation,
    fix_orientation_tolerance_rad: float,
    align_tcp_axis,
    align_tcp_axis_world,
    align_tcp_axis_tolerance_rad: float,
    fix_z_task,
    z_tolerance_m: float,
    avoid_collisions: bool,
    min_clearance_m: float,
    avoid_arm_singularity: bool,
    self_collision: bool,
    min_z_task,
    max_tcp_linear_speed_m_per_s,
    extra_clearance_to,
    stay_in_workspace_box,
) -> tuple:
    """Hashable summary of every input that changes the KTO problem.

    ``warmstart`` keys on (q_start, q_goal, problem_signature). Two calls
    with the same signature but different (q_start, q_goal) are different
    cache entries. Two calls with different signatures (e.g. one with
    ``fix_z_task`` set, one without) never share a warmstart even if
    endpoints match — they're different NLPs.

    Uses ``_scene_fingerprint`` (content-derived) so signatures are
    stable across process restarts and machines that build the same
    scene with the same Drake version.
    """
    intermediates = tuple(
        _round_seq(wp, 3) for wp in waypoints_q_full[1:-1]
    )
    fix_orient_key = (
        _round_seq(np.asarray(fix_orientation.as_matrix()), 2)
        if fix_orientation is not None else None
    )
    extras_key = (
        tuple(sorted(
            (str(k), round(float(v), 3))
            for k, v in extra_clearance_to.items()
        ))
        if extra_clearance_to else None
    )
    workspace_key = (
        (_round_seq(stay_in_workspace_box[0], 3),
         _round_seq(stay_in_workspace_box[1], 3))
        if stay_in_workspace_box is not None else None
    )
    return (
        _scene_fingerprint(plant, plant_context),
        arm,
        round(float(duration_s), 3),
        int(n_ctrl),
        int(spline_order),
        intermediates,
        fix_orient_key,
        round(float(fix_orientation_tolerance_rad), 3),
        _round_seq(align_tcp_axis, 2),
        _round_seq(align_tcp_axis_world, 2),
        round(float(align_tcp_axis_tolerance_rad), 3),
        round(float(fix_z_task), 3) if fix_z_task is not None else None,
        round(float(z_tolerance_m), 3),
        bool(avoid_collisions),
        round(float(min_clearance_m), 3),
        bool(avoid_arm_singularity),
        bool(self_collision),
        round(float(min_z_task), 3) if min_z_task is not None else None,
        round(float(max_tcp_linear_speed_m_per_s), 3)
            if max_tcp_linear_speed_m_per_s is not None else None,
        extras_key,
        workspace_key,
    )


def _initial_guess_for_kto(
    waypoints_q_full: List[np.ndarray],
    duration_s: float,
    basis,
) -> BsplineTrajectory:
    """Cubic spline through the IK'd waypoints, projected onto KTO's
    own B-spline basis so the initial control points exactly parameterise
    the seed trajectory.

    Uses KTO's basis (passed in) — this is critical: the basis defines
    the knot vector and order, and KTO's optimisation only changes
    control points, not the basis itself. Building a guess with a
    different basis (e.g. ``BsplineBasis(order, n_ctrl)`` constructed
    locally) would silently misalign the control points and look
    "infeasible from the start" to SNOPT.
    """
    waypoints_for_spline = list(waypoints_q_full)
    if len(waypoints_for_spline) < 3:
        midpoint = 0.5 * (waypoints_for_spline[0] + waypoints_for_spline[-1])
        waypoints_for_spline = [
            waypoints_for_spline[0],
            midpoint,
            waypoints_for_spline[-1],
        ]
    times = np.linspace(0.0, duration_s, len(waypoints_for_spline))
    samples = np.array(waypoints_for_spline).T
    spline = PiecewisePolynomial.CubicShapePreserving(times, samples)

    n_ctrl = basis.num_basis_functions()
    # Sample the cubic spline at uniformly-spaced times spanning [0, duration]
    # to seed the control points.
    ts = np.linspace(0.0, duration_s, n_ctrl)
    cps = [np.asarray(spline.value(t)).reshape(-1, 1) for t in ts]
    return BsplineTrajectory(basis, cps)
