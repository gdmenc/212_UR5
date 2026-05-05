"""RRT-Connect motion planning over Drake's SceneGraphCollisionChecker.

pydrake bundles the collision-check infrastructure but does NOT bind a
sampling-based planner to Python (see drake/issues/14431). This module
implements the standard RRT-Connect (bidirectional RRT) algorithm on top
of ``SceneGraphCollisionChecker`` so we get Drake's collision-filter
groups, padding, edge subdivision, and parallel-context machinery for
free, while owning only the planner logic itself.

Architectural choices
---------------------

- **Pin partner arm as static environment.** The collision checker is
  constructed with ``robot_model_instances=[planning_arm, planning_gripper]``;
  everything else (other arm, microwave, table, vention) is treated as
  environment, frozen at whatever values are in the plant's default
  context. To pin the partner arm at a particular q, set it via
  ``checker.UpdatePositions(...)`` once before calling ``rrt_connect``.

- **Plan in full-plant q space, sample in planning-arm subspace.**
  ``CheckEdgeCollisionFree`` wants a full-plant position vector; we
  build one for each candidate by overlaying the planning arm's 6-DOF
  sample onto the static base vector.

- **Edge collision uses Drake's adaptive subdivision.** ``edge_step_size``
  (rad in C-space) controls how finely an edge is subdivided into
  intermediate configurations for collision checking — Drake handles
  this internally.

- **Branch consistency** (e.g., q3/q5 sign) is the caller's job. RRT
  samples globally so it can hop branches mid-path; if you want a
  single-branch path, restrict the sampling bounds before calling.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Callable, Dict, List, Optional, Sequence, Tuple

import numpy as np
from pydrake.math import RigidTransform, RotationMatrix
from pydrake.multibody.plant import MultibodyPlant
from pydrake.multibody.tree import ModelInstanceIndex
from pydrake.planning import (
    RobotDiagram,
    RobotDiagramBuilder,
    SceneGraphCollisionChecker,
)

from . import default_home_q
from ..calibration import (
    TCP_OFFSET_HOOK,
    TCP_OFFSET_ROBOTIQ_2F85,
    X_LEFT_BASE_TASK,
    X_RIGHT_BASE_TASK,
)
from ..util.poses import Pose
from .build_scene import _compose_scene_fragments
from .scene.grippers import _tcp_offset_to_rigid_transform


ConfigValidityFn = Callable[[np.ndarray], bool]


# ---------------------------------------------------------------------------
#  Scene wrapper for SceneGraphCollisionChecker
# ---------------------------------------------------------------------------


def build_planning_scene(
    *,
    include_microwave: bool = True,
    include_objects: bool = True,
    robotiq_mode: str = "closed",
    microwave_door_open_angle_rad: float = 0.0,
    skip_static_objects: tuple = (),
    attached_objects: tuple = (),
) -> Tuple[RobotDiagram, MultibodyPlant, dict, dict]:
    """Build a ``RobotDiagram`` + plant + arm/gripper handles dict.

    Mirrors ``build_scene.build_scene()`` but uses ``RobotDiagramBuilder``
    so the resulting diagram is the right wrapper type for
    ``SceneGraphCollisionChecker``. Composition is shared via
    ``_compose_scene_fragments`` so this can't drift from the
    visualizer-side build.

    ``microwave_door_open_angle_rad`` welds the microwave door at the
    given angle (0 = closed; ~π/2 to ~7π/12 for typical open). Use to
    plan into the open cavity vs. avoid a closed door.

    ``skip_static_objects`` + ``attached_objects`` configure in-hand
    carrying — see ``build_scene.build_scene`` and
    ``scene.objects.attach_object_to_gripper``.
    """
    rdb = RobotDiagramBuilder(time_step=0.0)
    plant = rdb.plant()

    fragments = _compose_scene_fragments(
        plant,
        include_microwave=include_microwave,
        include_grippers=True,
        include_objects=include_objects,
        robotiq_mode=robotiq_mode,
        microwave_door_open_angle_rad=microwave_door_open_angle_rad,
        skip_static_objects=skip_static_objects,
        attached_objects=attached_objects,
    )

    plant.Finalize()
    plant.SetDefaultPositions(default_home_q(plant))

    diagram = rdb.Build()
    return diagram, plant, fragments.arms, fragments.grippers


def _gripper_instance_for(plant: MultibodyPlant, arm_name: str
                          ) -> Optional[ModelInstanceIndex]:
    """Find the gripper model instance welded to the given arm.

    Gripper instances get auto-renamed to ``hook_gripper_<arm>::hook_gripper``
    or ``robotiq_2f_85_<arm>::robotiq_2f_85`` by the parser; match by prefix.
    """
    for prefix in (f"hook_gripper_{arm_name}", f"robotiq_2f_85_{arm_name}"):
        for i in range(plant.num_model_instances()):
            mi = ModelInstanceIndex(i)
            if plant.GetModelInstanceName(mi).startswith(prefix):
                return mi
    return None


def _attached_object_instances_for(
    plant: MultibodyPlant,
    arm_name: str,
) -> List[ModelInstanceIndex]:
    """Find object models welded to the given arm's gripper.

    Attached objects are part of the moving collision body for RRT; if
    omitted from ``robot_model_instances``, SceneGraphCollisionChecker
    treats them as frozen environment and can accept paths that later
    collide when replayed with the object moving.
    """
    out: List[ModelInstanceIndex] = []
    suffix = f"_in_hand_{arm_name}"
    for i in range(plant.num_model_instances()):
        mi = ModelInstanceIndex(i)
        if plant.GetModelInstanceName(mi).endswith(suffix):
            out.append(mi)
    return out


def make_collision_checker(
    diagram: RobotDiagram,
    plant: MultibodyPlant,
    planning_arm_name: str,
    *,
    edge_step_size: float = 0.05,
    env_padding: float = 0.005,
) -> SceneGraphCollisionChecker:
    """Build a checker treating the named arm + its gripper as the
    robot, everything else (other arm, microwave, table, vention) as
    static environment.

    Per Drake convention: model instances NOT listed in
    ``robot_model_instances`` are treated as environment with their
    joint values frozen at whatever's in the plant context. Pin the
    partner arm via ``checker.UpdatePositions(q_full)`` before planning.
    """
    from .transit import _arm_model_instance

    arm_inst = _arm_model_instance(plant, planning_arm_name)
    robot_models: List[ModelInstanceIndex] = [arm_inst]
    grip_inst = _gripper_instance_for(plant, planning_arm_name)
    if grip_inst is not None:
        robot_models.append(grip_inst)
    robot_models.extend(_attached_object_instances_for(plant, planning_arm_name))

    return SceneGraphCollisionChecker(
        model=diagram,
        robot_model_instances=robot_models,
        edge_step_size=edge_step_size,
        env_collision_padding=env_padding,
    )


def make_tcp_axis_alignment_validator(
    plant: MultibodyPlant,
    arm_name: str,
    *,
    axis_tcp: np.ndarray,
    axis_world: Optional[np.ndarray] = None,
    tolerance_rad: float,
) -> ConfigValidityFn:
    """Return a q-full predicate for yaw-free axis alignment.

    This is the RRT-side counterpart to ``plan_transit``'s
    ``align_tcp_axis`` constraint. It checks that a vector fixed in the
    TCP frame, such as the held plate's normal expressed in the TCP frame
    at grasp time, stays within ``tolerance_rad`` of a world-frame vector.

    RRT calls this on configurations and edge samples; yaw around the
    constrained axis remains free.
    """
    from .transit import _arm_model_instance, _tcp_frame

    ctx = plant.CreateDefaultContext()
    tcp_frame = _tcp_frame(plant, _arm_model_instance(plant, arm_name))
    axis_tcp = np.asarray(axis_tcp, dtype=float).reshape(3)
    axis_tcp = axis_tcp / np.linalg.norm(axis_tcp)
    if axis_world is None:
        axis_world = np.array([0.0, 0.0, 1.0])
    axis_world = np.asarray(axis_world, dtype=float).reshape(3)
    axis_world = axis_world / np.linalg.norm(axis_world)
    min_dot = float(np.cos(tolerance_rad))

    def _valid(q_full: np.ndarray) -> bool:
        plant.SetPositions(ctx, np.asarray(q_full, dtype=float))
        R_world_tcp = tcp_frame.CalcPoseInWorld(ctx).rotation().matrix()
        axis_in_world = R_world_tcp @ axis_tcp
        axis_in_world = axis_in_world / np.linalg.norm(axis_in_world)
        return float(np.dot(axis_in_world, axis_world)) >= min_dot

    return _valid


# ---------------------------------------------------------------------------
#  RRT-Connect (BiRRT)
# ---------------------------------------------------------------------------


@dataclass
class RRTPlan:
    """Result of an RRT-Connect run."""

    path_full: List[np.ndarray]
    """Sequence of full-plant q vectors, start..goal, collision-free
    pairwise. Includes the planning arm's interpolated waypoints with
    every other DOF held at the base value."""

    arm_indices: np.ndarray
    """Indices into the full-plant q vector for the planning arm."""

    iterations: int
    """How many extend cycles RRT-Connect actually used."""

    tree_sizes: Tuple[int, int]
    """Final node counts (start tree, goal tree). Useful for diagnosing
    whether the planner explored vs. found a direct connection."""

    metadata: Dict = field(default_factory=dict)


class RRTFailure(RuntimeError):
    """Raised when RRT-Connect can't find a path within the iteration budget."""


def _arm_position_indices(plant: MultibodyPlant,
                          arm_instance: ModelInstanceIndex) -> np.ndarray:
    """Same logic as transit._arm_position_indices, duplicated here to
    avoid a circular import (transit imports nothing from rrt, rrt
    imports nothing from transit)."""
    indices = []
    for joint_idx in plant.GetJointIndices(arm_instance):
        joint = plant.get_joint(joint_idx)
        if joint.num_positions() == 0:
            continue
        for k in range(joint.num_positions()):
            indices.append(joint.position_start() + k)
    return np.array(indices, dtype=int)


def rrt_connect(
    checker: SceneGraphCollisionChecker,
    q_start_full: np.ndarray,
    q_goal_full: np.ndarray,
    planning_arm_name: str,
    *,
    max_iters: int = 2000,
    step_size: float = 0.3,
    goal_bias: float = 0.1,
    sample_lower: Optional[np.ndarray] = None,
    sample_upper: Optional[np.ndarray] = None,
    seed: int = 0,
    validity_fn: Optional[ConfigValidityFn] = None,
    validity_edge_step_size: float = 0.05,
) -> RRTPlan:
    """Bidirectional RRT-Connect over the planning arm's DOF.

    Standard algorithm: maintain two trees rooted at start and goal;
    each iteration sample a random q (with ``goal_bias`` probability,
    sample the *other* tree's root instead), EXTEND the active tree by
    one step toward the sample, then CONNECT the other tree to the new
    node. Trees swap roles every iteration. When CONNECT reaches the
    new node, splice the two paths.

    Parameters
    ----------
    q_start_full, q_goal_full : full-plant position vectors. The
        non-planning-arm slots define the static environment pin.
    step_size : extend distance per RRT step (radians in joint space).
    goal_bias : probability of sampling the goal arm-q instead of a
        uniform random one. 0.1 is a good default.
    sample_lower, sample_upper : per-arm joint bounds for sampling.
        Default uses ``plant.GetPosition*Limits`` clamped to ±π.

    Raises ``RRTFailure`` if no path is found in ``max_iters``.
    """
    plant = checker.plant()
    from .transit import _arm_model_instance
    arm_inst = _arm_model_instance(plant, planning_arm_name)
    arm_idx = _arm_position_indices(plant, arm_inst)

    q_start_full = np.asarray(q_start_full, dtype=float).copy()
    q_goal_full = np.asarray(q_goal_full, dtype=float).copy()
    q_start_arm = q_start_full[arm_idx]
    q_goal_arm = q_goal_full[arm_idx]

    # Pin the static environment to whatever's in q_start_full's
    # non-arm slots. UpdatePositions touches the checker's implicit
    # context — single-threaded, which is what we want here.
    checker.UpdatePositions(q_start_full)

    if sample_lower is None or sample_upper is None:
        # UR5e joint limits are ±2π in the URDF; clamp to ±π for sane sampling.
        lo = np.maximum(plant.GetPositionLowerLimits()[arm_idx], -np.pi)
        hi = np.minimum(plant.GetPositionUpperLimits()[arm_idx], +np.pi)
        sample_lower = lo if sample_lower is None else np.asarray(sample_lower)
        sample_upper = hi if sample_upper is None else np.asarray(sample_upper)

    rng = np.random.default_rng(seed)
    base_full = q_start_full.copy()

    def _full(arm_q: np.ndarray) -> np.ndarray:
        out = base_full.copy()
        out[arm_idx] = arm_q
        return out

    def _free_edge(q_from_arm: np.ndarray, q_to_arm: np.ndarray) -> bool:
        q_from_full = _full(q_from_arm)
        q_to_full = _full(q_to_arm)
        if not checker.CheckEdgeCollisionFree(q_from_full, q_to_full):
            return False
        if validity_fn is None:
            return True
        dist = float(np.linalg.norm(q_to_arm - q_from_arm))
        n = max(2, int(np.ceil(dist / validity_edge_step_size)) + 1)
        for s in np.linspace(0.0, 1.0, n):
            q_full = (1.0 - s) * q_from_full + s * q_to_full
            if not validity_fn(q_full):
                return False
        return True

    def _free_config(q_arm: np.ndarray) -> bool:
        q_full = _full(q_arm)
        return (
            checker.CheckConfigCollisionFree(q_full)
            and (validity_fn is None or validity_fn(q_full))
        )

    if not _free_config(q_start_arm):
        raise RRTFailure("start configuration is in collision")
    if not _free_config(q_goal_arm):
        raise RRTFailure("goal configuration is in collision")
    # Direct shot first — no need for an actual tree if the straight
    # line is already free.
    if _free_edge(q_start_arm, q_goal_arm):
        return RRTPlan(
            path_full=[_full(q_start_arm), _full(q_goal_arm)],
            arm_indices=arm_idx,
            iterations=0,
            tree_sizes=(1, 1),
            metadata={"planner": "direct_connect", "seed": seed},
        )

    def _sample() -> np.ndarray:
        return rng.uniform(sample_lower, sample_upper)

    def _nearest(tree: List[np.ndarray], q: np.ndarray) -> int:
        diffs = np.array(tree) - q
        return int(np.argmin(np.linalg.norm(diffs, axis=1)))

    def _steer(q_from: np.ndarray, q_to: np.ndarray) -> np.ndarray:
        d = float(np.linalg.norm(q_to - q_from))
        if d <= step_size:
            return q_to.copy()
        return q_from + (q_to - q_from) * (step_size / d)

    def _extend(tree: List[np.ndarray], parent: Dict[int, Optional[int]],
                q_target: np.ndarray) -> Tuple[Optional[int], Optional[np.ndarray]]:
        """One-step extend toward q_target. Returns (new_idx, new_q) on
        success, (None, None) on collision."""
        i_near = _nearest(tree, q_target)
        q_new = _steer(tree[i_near], q_target)
        if not _free_edge(tree[i_near], q_new):
            return None, None
        new_idx = len(tree)
        tree.append(q_new)
        parent[new_idx] = i_near
        return new_idx, q_new

    def _connect(tree: List[np.ndarray], parent: Dict[int, Optional[int]],
                 q_target: np.ndarray) -> Tuple[int, bool]:
        """Repeated extend until we reach q_target (within step_size)
        or hit a collision. Returns (last_idx, reached_target)."""
        last_idx = _nearest(tree, q_target)
        last_q = tree[last_idx]
        while True:
            new_idx, new_q = _extend(tree, parent, q_target)
            if new_idx is None:
                return last_idx, False
            last_idx, last_q = new_idx, new_q
            # Reached target if we landed within one step (== q_target).
            if np.allclose(new_q, q_target, atol=1e-6):
                return new_idx, True

    # Two trees. Ta is always rooted at start; Tb at goal. We track
    # which one is "active" (extended toward random) via `start_active`.
    Ta: List[np.ndarray] = [q_start_arm]
    Tb: List[np.ndarray] = [q_goal_arm]
    pa: Dict[int, Optional[int]] = {0: None}
    pb: Dict[int, Optional[int]] = {0: None}
    start_active = True

    for it in range(max_iters):
        if start_active:
            T_grow, p_grow = Ta, pa
            T_target, p_target = Tb, pb
            other_root = q_goal_arm
        else:
            T_grow, p_grow = Tb, pb
            T_target, p_target = Ta, pa
            other_root = q_start_arm

        q_rand = other_root if rng.random() < goal_bias else _sample()

        new_grow, q_new = _extend(T_grow, p_grow, q_rand)
        if new_grow is None:
            start_active = not start_active
            continue

        new_target, reached = _connect(T_target, p_target, q_new)
        if reached:
            # Walk start-tree from start to its meeting node; walk
            # goal-tree from its meeting node to goal.
            if start_active:
                # T_grow == Ta: meeting node in Ta is new_grow,
                # meeting node in Tb is new_target.
                meet_in_Ta, meet_in_Tb = new_grow, new_target
            else:
                # T_grow == Tb: meeting node in Tb is new_grow,
                # in Ta is new_target.
                meet_in_Ta, meet_in_Tb = new_target, new_grow

            start_to_meet: List[np.ndarray] = []
            i: Optional[int] = meet_in_Ta
            while i is not None:
                start_to_meet.append(Ta[i])
                i = pa[i]
            start_to_meet.reverse()

            meet_to_goal: List[np.ndarray] = []
            i = meet_in_Tb
            while i is not None:
                meet_to_goal.append(Tb[i])
                i = pb[i]

            arm_path = start_to_meet + meet_to_goal
            return RRTPlan(
                path_full=[_full(q) for q in arm_path],
                arm_indices=arm_idx,
                iterations=it + 1,
                tree_sizes=(len(Ta), len(Tb)),
                metadata={"planner": "rrt_connect", "seed": seed},
            )

        start_active = not start_active

    raise RRTFailure(
        f"RRT-Connect failed to find a path in {max_iters} iterations "
        f"(tree sizes: Ta={len(Ta)}, Tb={len(Tb)})"
    )


# ---------------------------------------------------------------------------
#  IKFast goal-branch enumeration
# ---------------------------------------------------------------------------


def _pose_to_rigid_transform(pose: Pose) -> RigidTransform:
    """Convert ``util.poses.Pose`` to ``pydrake.math.RigidTransform``."""
    return RigidTransform(
        RotationMatrix(np.asarray(pose.rotation.as_matrix())),
        np.asarray(pose.translation, dtype=float),
    )


def _arm_calibration(arm_name: str) -> Tuple[RigidTransform, RigidTransform]:
    """Return ``(X_base_task, X_wrist3_tcp)`` for the named arm.

    ``X_base_task``    — translates a task-frame point to the arm's
                         controller base frame.
    ``X_wrist3_tcp``   — wrist_3 → TCP weld used by ``setTcp`` on
                         the real controller and as the FixedOffsetFrame
                         in the Drake plant. ``solve_ik`` wants
                         wrist_3, so we'll invert this.
    """
    if arm_name == "ur_left":
        return (
            _pose_to_rigid_transform(X_LEFT_BASE_TASK),
            _tcp_offset_to_rigid_transform(TCP_OFFSET_HOOK),
        )
    if arm_name == "ur_right":
        return (
            _pose_to_rigid_transform(X_RIGHT_BASE_TASK),
            _tcp_offset_to_rigid_transform(TCP_OFFSET_ROBOTIQ_2F85),
        )
    raise KeyError(f"unknown arm {arm_name!r} (expected ur_left or ur_right)")


def _orthonormalize(R: np.ndarray) -> np.ndarray:
    """Project ``R`` to the nearest rotation matrix via polar decomposition.

    Compose a chain of rotations through Drake's ``RigidTransform @``
    operator and the result is *almost* orthogonal but drifts at the
    1e-4 level. ikfast's analytic UR5e cpp returns 0 solutions for any
    rotation whose ``det`` deviates from 1 noticeably, so we project
    here as a defensive cleanup right before handing it off."""
    U, _, Vt = np.linalg.svd(R)
    R_proj = U @ Vt
    if np.linalg.det(R_proj) < 0:
        # Flip the smallest singular value to keep det = +1.
        U[:, -1] *= -1
        R_proj = U @ Vt
    return R_proj


def ikfast_goal_branches(
    arm_name: str,
    tcp_pose_task: Pose,
    *,
    seed_arm_q: Optional[np.ndarray] = None,
    max_branch_dist: Optional[float] = None,
) -> List[np.ndarray]:
    """Enumerate IKFast solutions for the named arm at a task-frame TCP.

    Parameters
    ----------
    arm_name : ``"ur_left"`` or ``"ur_right"``.
    tcp_pose_task : task-frame pose of the gripper's TCP point. The
        per-arm TCP_OFFSET is applied internally so the IK target is
        wrist_3 (which is what ikfast actually solves for).
    seed_arm_q : optional 6-vector. When provided, branches are
        returned sorted by joint-distance to this seed AND each
        branch is shifted by 2π multiples to land near the seed
        (so a "near branch" stays near, not at the equivalent angle
        on the wrong side of ±π).
    max_branch_dist : drop branches whose joint-distance from
        ``seed_arm_q`` exceeds this (radians). Ignored when
        ``seed_arm_q`` is None.

    Returns up to 8 candidate goal arm-q's. Empty list if the pose
    is unreachable (e.g., outside workspace).
    """
    from .ikfast import solve_ik

    X_base_task, X_wrist3_tcp = _arm_calibration(arm_name)
    X_task_tcp = _pose_to_rigid_transform(tcp_pose_task)
    # ikfast actually solves wrist_3 ↔ joints; convert TCP target to
    # wrist_3 by removing the TCP offset (right-multiply by inverse).
    X_base_tcp = X_base_task @ X_task_tcp
    X_base_wrist3 = X_base_tcp @ X_wrist3_tcp.inverse()

    # Project the chain-composed rotation back onto SO(3) — float drift
    # otherwise sinks ikfast's solution count to 0.
    X_base_wrist3_clean = RigidTransform(
        RotationMatrix(_orthonormalize(np.asarray(X_base_wrist3.rotation().matrix()))),
        np.asarray(X_base_wrist3.translation()),
    )

    return solve_ik(
        X_base_wrist3_clean,
        seed_q=seed_arm_q,
        max_branch_dist=max_branch_dist,
    )


# ---------------------------------------------------------------------------
#  Multi-goal RRT: try a SET of candidate goal arm-q's
# ---------------------------------------------------------------------------


@dataclass
class GoalSetReport:
    """Diagnostics from rrt_connect_to_goal_set: which goals were
    discarded and why, which one finally won, etc."""

    n_input: int
    """How many candidate goals the caller passed in."""

    n_in_collision: int
    """Goals discarded because the goal config itself is in collision."""

    n_direct_connect: int
    """Goals reachable via straight C-space line from start."""

    chosen_goal_index: int
    """Position in the (collision-filtered) candidate list of the goal
    that ended up producing the returned plan."""

    chosen_planner: str
    """``direct_connect`` or ``rrt_connect`` — which method produced
    the returned plan."""


def rrt_connect_to_goal_set(
    checker: SceneGraphCollisionChecker,
    q_start_full: np.ndarray,
    goal_arm_qs: Sequence[np.ndarray],
    planning_arm_name: str,
    *,
    max_iters: int = 2000,
    step_size: float = 0.3,
    goal_bias: float = 0.1,
    sample_lower: Optional[np.ndarray] = None,
    sample_upper: Optional[np.ndarray] = None,
    seed: int = 0,
    pick: str = "first",
    validity_fn: Optional[ConfigValidityFn] = None,
    validity_edge_step_size: float = 0.05,
) -> Tuple[RRTPlan, GoalSetReport]:
    """Plan to whichever of several candidate goal configs is reachable.

    Standard pattern when the goal is a TCP pose, not a single q:
    enumerate IK branches (``ikfast.solve_ik`` returns up to 8), throw
    out the ones whose goal config is itself in collision, then ask the
    planner to find a path to one of the survivors.

    Goals are tried in the order given. ``pick="first"`` returns as soon
    as ANY plan succeeds (faster). ``pick="shortest"`` plans to every
    surviving goal and returns the shortest path (slower but better).

    Per-goal strategy:
      1. Skip if goal full-q is in self/env collision.
      2. Try the trivial straight-line edge from start (``CheckEdgeCollisionFree``).
         If free, that's a 2-node "direct connect" plan — cheapest possible.
      3. Otherwise, run ``rrt_connect`` to that goal.

    Raises ``RRTFailure`` if every candidate is unreachable.
    """
    if len(goal_arm_qs) == 0:
        raise RRTFailure("goal set is empty (no candidate goals supplied)")
    if pick not in ("first", "shortest"):
        raise ValueError(f"pick must be 'first' or 'shortest', got {pick!r}")

    plant = checker.plant()
    from .transit import _arm_model_instance
    arm_inst = _arm_model_instance(plant, planning_arm_name)
    arm_idx = _arm_position_indices(plant, arm_inst)

    q_start_full = np.asarray(q_start_full, dtype=float).copy()
    q_start_arm = q_start_full[arm_idx]

    # Pin partner arm + scene to whatever was in q_start_full so the
    # collision-checker context matches what rrt_connect would use.
    checker.UpdatePositions(q_start_full)

    if not checker.CheckConfigCollisionFree(q_start_full):
        raise RRTFailure("start configuration is in collision")
    if validity_fn is not None and not validity_fn(q_start_full):
        raise RRTFailure("start configuration violates validity constraint")

    # Step 1: filter goals by config-collision-free + validity.
    # Track per-branch failure reasons so the failure message is actually
    # diagnostic rather than just "all in collision".
    survivors: List[Tuple[int, np.ndarray]] = []  # (orig_index, arm_q)
    n_pure_collision = 0
    n_validity_only = 0
    fail_reasons: List[str] = []
    for orig_i, arm_q in enumerate(goal_arm_qs):
        arm_q = np.asarray(arm_q, dtype=float)
        full = q_start_full.copy()
        full[arm_idx] = arm_q
        coll_free = checker.CheckConfigCollisionFree(full)
        validity_ok = validity_fn is None or validity_fn(full)
        if coll_free and validity_ok:
            survivors.append((orig_i, arm_q))
            continue
        # Categorise the failure for the error message.
        if not coll_free:
            n_pure_collision += 1
            fail_reasons.append(f"#{orig_i}: env/self collision")
        else:
            n_validity_only += 1
            fail_reasons.append(f"#{orig_i}: validity_fn rejected")

    n_collision = len(goal_arm_qs) - len(survivors)
    if not survivors:
        breakdown = (
            f"{n_pure_collision} env/self-collision, "
            f"{n_validity_only} validity-rejected"
        )
        # Print per-branch detail at error time so the caller can see
        # whether the singularity-branch / floor / axis validator is
        # the offender (validity_fn lumps them; this raw breakdown
        # makes it obvious when *every* branch is validity-only).
        details = "; ".join(fail_reasons[:8])
        raise RRTFailure(
            f"all {len(goal_arm_qs)} candidate goal configs were rejected "
            f"({breakdown}); detail: {details}"
        )

    # Step 2: try direct-connect to each survivor first — virtually free.
    direct_plans: List[Tuple[int, RRTPlan]] = []
    for surv_i, (_orig_i, arm_q) in enumerate(survivors):
        full = q_start_full.copy()
        full[arm_idx] = arm_q
        edge_valid = checker.CheckEdgeCollisionFree(q_start_full, full)
        if edge_valid and validity_fn is not None:
            dist = float(np.linalg.norm(full[arm_idx] - q_start_arm))
            n = max(2, int(np.ceil(dist / validity_edge_step_size)) + 1)
            for s in np.linspace(0.0, 1.0, n):
                q = (1.0 - s) * q_start_full + s * full
                if not validity_fn(q):
                    edge_valid = False
                    break
        if edge_valid:
            plan = RRTPlan(
                path_full=[q_start_full.copy(), full],
                arm_indices=arm_idx,
                iterations=0,
                tree_sizes=(1, 1),
                metadata={
                    "planner": "direct_connect",
                    "goal_index": survivors[surv_i][0],
                    "seed": seed,
                },
            )
            direct_plans.append((surv_i, plan))
            if pick == "first":
                report = GoalSetReport(
                    n_input=len(goal_arm_qs),
                    n_in_collision=n_collision,
                    n_direct_connect=1,
                    chosen_goal_index=survivors[surv_i][0],
                    chosen_planner="direct_connect",
                )
                return plan, report

    if direct_plans and pick == "shortest":
        # All direct-connects are 2-node lines; "shortest" picks the
        # one with smallest joint-space step from start.
        best = min(direct_plans, key=lambda sp: float(np.linalg.norm(
            sp[1].path_full[1][arm_idx] - q_start_arm,
        )))
        report = GoalSetReport(
            n_input=len(goal_arm_qs),
            n_in_collision=n_collision,
            n_direct_connect=len(direct_plans),
            chosen_goal_index=survivors[best[0]][0],
            chosen_planner="direct_connect",
        )
        return best[1], report

    # Step 3: real RRT for each survivor not already covered by a
    # direct_connect. Try in input order; first success wins (or, if
    # pick == "shortest", plan for everyone and pick smallest path
    # length).
    rrt_plans: List[Tuple[int, RRTPlan]] = []
    for surv_i, (_orig_i, arm_q) in enumerate(survivors):
        full = q_start_full.copy()
        full[arm_idx] = arm_q
        try:
            plan = rrt_connect(
                checker,
                q_start_full=q_start_full,
                q_goal_full=full,
                planning_arm_name=planning_arm_name,
                max_iters=max_iters,
                step_size=step_size,
                goal_bias=goal_bias,
                sample_lower=sample_lower,
                sample_upper=sample_upper,
                seed=seed,
                validity_fn=validity_fn,
                validity_edge_step_size=validity_edge_step_size,
            )
        except RRTFailure:
            continue
        plan.metadata.setdefault("goal_index", survivors[surv_i][0])
        rrt_plans.append((surv_i, plan))
        if pick == "first":
            report = GoalSetReport(
                n_input=len(goal_arm_qs),
                n_in_collision=n_collision,
                n_direct_connect=0,
                chosen_goal_index=survivors[surv_i][0],
                chosen_planner="rrt_connect",
            )
            return plan, report

    if rrt_plans:
        # pick == "shortest" path
        best = min(rrt_plans, key=lambda sp: path_length(
            sp[1].path_full, sp[1].arm_indices,
        ))
        report = GoalSetReport(
            n_input=len(goal_arm_qs),
            n_in_collision=n_collision,
            n_direct_connect=0,
            chosen_goal_index=survivors[best[0]][0],
            chosen_planner="rrt_connect",
        )
        return best[1], report

    raise RRTFailure(
        f"none of the {len(survivors)} collision-free candidate goals "
        f"were reachable in {max_iters} iterations each"
    )


# ---------------------------------------------------------------------------
#  Top-level convenience: TCP-pose goal → joint path
# ---------------------------------------------------------------------------


def rrt_connect_to_tcp_pose(
    checker: SceneGraphCollisionChecker,
    q_start_full: np.ndarray,
    tcp_pose_task: Pose,
    planning_arm_name: str,
    *,
    max_branch_dist: Optional[float] = None,
    max_iters: int = 2000,
    step_size: float = 0.3,
    goal_bias: float = 0.1,
    sample_lower: Optional[np.ndarray] = None,
    sample_upper: Optional[np.ndarray] = None,
    seed: int = 0,
    pick: str = "first",
    validity_fn: Optional[ConfigValidityFn] = None,
    validity_edge_step_size: float = 0.05,
) -> Tuple[RRTPlan, GoalSetReport]:
    """Plan from ``q_start_full`` to a task-frame TCP pose.

    Composes :func:`ikfast_goal_branches` (enumerate up to 8 IK roots)
    and :func:`rrt_connect_to_goal_set` (filter colliding goals,
    direct-connect each, fall back to RRT). The seed q for ikfast is
    pulled from the planning arm's slice of ``q_start_full`` so
    branches come back ordered by joint-distance from the start.

    See :func:`rrt_connect_to_goal_set` for the ``pick`` semantics.
    Raises ``RRTFailure`` if the TCP pose is unreachable, every
    branch is in collision, or no branch can be RRT-connected within
    ``max_iters``.
    """
    plant = checker.plant()
    from .transit import _arm_model_instance
    arm_inst = _arm_model_instance(plant, planning_arm_name)
    arm_idx = _arm_position_indices(plant, arm_inst)
    seed_arm_q = np.asarray(q_start_full, dtype=float)[arm_idx]

    branches = ikfast_goal_branches(
        planning_arm_name, tcp_pose_task,
        seed_arm_q=seed_arm_q, max_branch_dist=max_branch_dist,
    )
    if not branches:
        raise RRTFailure(
            f"ikfast returned 0 solutions for the requested TCP pose "
            f"(arm {planning_arm_name}, translation "
            f"{np.round(tcp_pose_task.translation, 3)}) — "
            f"likely out of reach"
        )

    return rrt_connect_to_goal_set(
        checker,
        q_start_full=q_start_full,
        goal_arm_qs=branches,
        planning_arm_name=planning_arm_name,
        max_iters=max_iters,
        step_size=step_size,
        goal_bias=goal_bias,
        sample_lower=sample_lower,
        sample_upper=sample_upper,
        seed=seed,
        pick=pick,
        validity_fn=validity_fn,
        validity_edge_step_size=validity_edge_step_size,
    )


# ---------------------------------------------------------------------------
#  Shortcut smoothing
# ---------------------------------------------------------------------------


def shortcut_path(
    checker: SceneGraphCollisionChecker,
    path_full: Sequence[np.ndarray],
    *,
    attempts: int = 200,
    seed: int = 0,
    validity_fn: Optional[ConfigValidityFn] = None,
    validity_edge_step_size: float = 0.05,
) -> List[np.ndarray]:
    """Greedy random shortcut smoother.

    Standard post-process for sampling-based planners: repeatedly pick
    two random waypoints (i, j) with j > i+1, try to replace the
    segment between them with a straight edge in joint space; if the
    new edge is collision-free, drop the intervening waypoints.

    This is the same algorithm Russ's manipulation textbook recommends
    (Ch 12). Halves to thirds of the original path length is typical
    after ~200 attempts on a 50-100 node path.
    """
    rng = np.random.default_rng(seed)
    path = [np.asarray(q, dtype=float) for q in path_full]

    for _ in range(attempts):
        if len(path) < 3:
            break
        i = int(rng.integers(0, len(path) - 2))
        j = int(rng.integers(i + 2, len(path)))
        edge_valid = checker.CheckEdgeCollisionFree(path[i], path[j])
        if edge_valid and validity_fn is not None:
            dist = float(np.linalg.norm(path[j] - path[i]))
            n = max(2, int(np.ceil(dist / validity_edge_step_size)) + 1)
            for s in np.linspace(0.0, 1.0, n):
                q = (1.0 - s) * path[i] + s * path[j]
                if not validity_fn(q):
                    edge_valid = False
                    break
        if edge_valid:
            path = path[:i + 1] + path[j:]

    return path


def path_length(path_full: Sequence[np.ndarray],
                arm_indices: Optional[np.ndarray] = None) -> float:
    """Total joint-space path length (radians, summed across DOF)."""
    arr = np.array([np.asarray(q) for q in path_full])
    if arm_indices is not None:
        arr = arr[:, np.asarray(arm_indices)]
    diffs = arr[1:] - arr[:-1]
    return float(np.sum(np.linalg.norm(diffs, axis=1)))
