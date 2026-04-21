"""Kinematic trajectory optimization (primary planner).

Drake provides ``KinematicTrajectoryOptimization``: it represents a joint
trajectory as a B-spline with N control points and solves a nonlinear
program that minimizes duration (and/or effort) subject to collision-free,
joint-limit, velocity, and acceleration constraints.

Use this as the default planner for free-space transits. It produces smooth
paths and is fast when a reasonable initial guess is available (see
``warmstart.py``). Fails gracefully into ``None`` on infeasibility — caller
is expected to fall back to RRT-Connect for narrow passages like reaching
inside the microwave.

Reference:
https://drake.mit.edu/doxygen_cxx/classdrake_1_1planning_1_1trajectory__optimization_1_1_kinematic_trajectory_optimization.html
"""

from typing import Optional

import numpy as np
from pydrake.multibody.plant import MultibodyPlant
from pydrake.trajectories import BsplineTrajectory


def plan_trajopt(
    plant: MultibodyPlant,
    q_start: np.ndarray,
    q_goal: np.ndarray,
    duration: Optional[float] = None,
    num_control_points: int = 10,
    warmstart_cps: Optional[np.ndarray] = None,
) -> Optional[BsplineTrajectory]:
    """Solve a B-spline joint trajectory from q_start to q_goal.

    Parameters
    ----------
    plant : the MultibodyPlant used for collision and kinematic queries.
        Must have collision geometry registered for the arm and the scene.
    q_start, q_goal : joint configurations (shape (num_positions,)).
    duration : if given, fix total time; otherwise let the solver minimize.
    num_control_points : B-spline control-point count. More points = more
        flexibility, slower solve.
    warmstart_cps : optional (num_control_points, num_positions) initial
        guess. Typically pulled from ``warmstart.lookup(...)``.

    Returns
    -------
    BsplineTrajectory on success, None on solver failure.
    """
    # TODO: implement.
    # Outline:
    #   1. trajopt = KinematicTrajectoryOptimization(
    #        plant.num_positions(), num_control_points, spline_order=4)
    #   2. prog = trajopt.get_mutable_prog()
    #   3. Add boundary constraints (start/end positions, zero start/end vel).
    #   4. Add collision-avoidance constraint via MinimumDistanceConstraint.
    #   5. Add position/velocity/acceleration bounds from plant joint limits.
    #   6. If warmstart_cps: trajopt.SetInitialGuess(warmstart_cps).
    #   7. result = Solve(prog); return trajopt.ReconstructTrajectory(result)
    #      or None on failure.
    raise NotImplementedError
