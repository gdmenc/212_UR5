"""RRT-Connect sampling-based planner (fallback).

Use this when trajopt fails — typically at narrow passages where continuous
optimization gets stuck, e.g. reaching the gripper through the microwave
door opening without hitting the frame.

RRT-Connect is probabilistically complete (it will find a path in finite
time if one exists) but the raw output is jagged and slow to execute
directly. Standard post-processing:
  1. Shortcut smoothing: repeatedly try to replace random (q_i, q_j) pairs
     in the waypoint list with a straight segment; accept if collision-free.
  2. Refit to a B-spline (same type trajopt returns) so downstream code sees
     a uniform trajectory representation.

Only reach for this when trajopt fails — the RRT + smooth + refit pipeline
is ~5-10x slower than a warm-started trajopt solve.
"""

from typing import Optional

import numpy as np
from pydrake.multibody.plant import MultibodyPlant
from pydrake.trajectories import BsplineTrajectory


def plan_rrt_connect(
    plant: MultibodyPlant,
    q_start: np.ndarray,
    q_goal: np.ndarray,
    duration: Optional[float] = None,
    max_iters: int = 5000,
    step_size: float = 0.1,
) -> Optional[BsplineTrajectory]:
    """Build a tree from q_start and another from q_goal; connect when they
    meet; shortcut; refit as B-spline.

    Parameters
    ----------
    plant : for collision queries.
    q_start, q_goal : joint configs.
    duration : target total duration for the refit spline. If None, use a
        speed-limited default based on path length.
    max_iters : total samples across both trees before giving up.
    step_size : radians per extend step.

    Returns
    -------
    BsplineTrajectory on success, None if no path found within max_iters.
    """
    # TODO: implement.
    # Consider using pydrake.planning's CollisionChecker for fast queries,
    # or roll a thin KD-tree + plant.SetPositions+collision-check loop.
    raise NotImplementedError
