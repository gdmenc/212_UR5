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
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np
from pydrake.multibody.plant import MultibodyPlant
from pydrake.multibody.tree import ModelInstanceIndex
from pydrake.planning import (
    RobotDiagram,
    RobotDiagramBuilder,
    SceneGraphCollisionChecker,
)

from . import default_home_q
from .scene.arms import add_both_arms
from .scene.grippers import add_grippers
from .scene.microwave import add_microwave
from .scene.tables import add_workspace_table
from .scene.vention import add_vention_stand


# ---------------------------------------------------------------------------
#  Scene wrapper for SceneGraphCollisionChecker
# ---------------------------------------------------------------------------


def build_planning_scene(
    *,
    include_microwave: bool = True,
    robotiq_mode: str = "closed",
) -> Tuple[RobotDiagram, MultibodyPlant, dict, dict]:
    """Build a ``RobotDiagram`` + plant + arm/gripper handles dict.

    Mirrors ``build_scene.build_scene()`` but uses ``RobotDiagramBuilder``
    so the resulting diagram is the right wrapper type for
    ``SceneGraphCollisionChecker``. Also calls ``SetDefaultPositions``
    so a fresh context lands at the sim HOME pose.
    """
    rdb = RobotDiagramBuilder(time_step=0.0)
    plant = rdb.plant()

    add_workspace_table(plant)
    add_vention_stand(plant)
    arms = add_both_arms(plant)
    grippers = add_grippers(plant, arms, robotiq_mode=robotiq_mode)
    if include_microwave:
        add_microwave(plant)

    plant.Finalize()
    plant.SetDefaultPositions(default_home_q(plant))

    diagram = rdb.Build()
    return diagram, plant, arms, grippers


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

    return SceneGraphCollisionChecker(
        model=diagram,
        robot_model_instances=robot_models,
        edge_step_size=edge_step_size,
        env_collision_padding=env_padding,
    )


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
        return checker.CheckEdgeCollisionFree(_full(q_from_arm), _full(q_to_arm))

    def _free_config(q_arm: np.ndarray) -> bool:
        return checker.CheckConfigCollisionFree(_full(q_arm))

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
#  Shortcut smoothing
# ---------------------------------------------------------------------------


def shortcut_path(
    checker: SceneGraphCollisionChecker,
    path_full: Sequence[np.ndarray],
    *,
    attempts: int = 200,
    seed: int = 0,
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
        if checker.CheckEdgeCollisionFree(path[i], path[j]):
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
