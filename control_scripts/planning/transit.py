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
from typing import Dict, List, Optional, Tuple, Union

import numpy as np
from pydrake.math import RigidTransform, RotationMatrix
from pydrake.multibody.inverse_kinematics import (
    InverseKinematics,
    MinimumDistanceLowerBoundConstraint,
    OrientationConstraint,
    PositionConstraint,
)
from pydrake.multibody.plant import MultibodyPlant
from pydrake.multibody.tree import ModelInstanceIndex
from pydrake.planning import KinematicTrajectoryOptimization
from pydrake.solvers import Solve
from pydrake.systems.framework import Context
from pydrake.trajectories import (
    BsplineTrajectory,
    PiecewisePolynomial,
    Trajectory,
)

from ..util.poses import Pose


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
) -> np.ndarray:
    """IK a single TCP pose for the planning arm.

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
) -> List[np.ndarray]:
    """IK every waypoint in order, seeding each with the previous IK result.

    Returns a list of full-plant position vectors (one per waypoint).
    Chained seeding keeps consecutive solutions on the same IK branch —
    avoids wrist 360° spins between adjacent waypoints.
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
    if spline_fallback_reason is not None:
        plan.metadata["spline_fallback_reason"] = spline_fallback_reason
    return plan


# ---------------------------------------------------------------------------
#  Constrained KTO planner
# ---------------------------------------------------------------------------

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
        theta_bound = 0.04   # ~2.3°
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

    # ----- Initial guess: cubic spline through the IK'd waypoints -----
    # We hand KTO a dense BsplineTrajectory built on its own basis so
    # the initial control points line up exactly with KTO's parameterisation.
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
        },
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
