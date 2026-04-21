"""Inverse kinematics: world-frame target pose -> joint configuration.

Called by primitives before path planning: given a desired gripper pose
(e.g. the pre-grasp pose for a plate), return a ``q`` that reaches it.
``plan_trajopt`` / ``plan_rrt_connect`` then plans the path from the current
joint state to this ``q``.

Multiple IK solutions typically exist. ``q_seed`` biases the solver toward
the current configuration (so the arm does not flip from elbow-up to
elbow-down just to reach the same Cartesian point). For cluttered scenes,
``collision_avoid=True`` adds a minimum-distance constraint so the solver
rejects configurations that would collide — significantly slower but
essential when the initial q_seed is near obstacles.
"""

from typing import Optional

import numpy as np
from pydrake.math import RigidTransform
from pydrake.multibody.plant import MultibodyPlant


def solve_ik(
    plant: MultibodyPlant,
    X_WGoal: RigidTransform,
    q_seed: Optional[np.ndarray] = None,
    arm_name: str = "ur_left",
    tcp_body_name: Optional[str] = None,
    collision_avoid: bool = True,
    position_tol: float = 1e-3,   # meters
    orientation_tol: float = 1e-2,  # radians
) -> Optional[np.ndarray]:
    """Return a joint configuration that places the TCP at X_WGoal.

    Parameters
    ----------
    plant : the MultibodyPlant; used for FK and collision queries.
    X_WGoal : target pose in the world frame.
    q_seed : initial guess. Defaults to the plant's current context positions.
    arm_name : model instance name; used to select which arm's TCP to
        constrain (this matters in a bimanual setup).
    tcp_body_name : the body name whose origin is the TCP. Defaults to the
        welded gripper body for the given arm.
    collision_avoid : add a MinimumDistanceConstraint to reject colliding
        solutions. Slower; essential in clutter.
    position_tol, orientation_tol : tolerance for the pose constraint.

    Returns
    -------
    Joint config on success, None on infeasibility.
    """
    # TODO: implement using pydrake.multibody.inverse_kinematics.InverseKinematics
    #   1. ik = InverseKinematics(plant)
    #   2. Add PositionConstraint + OrientationConstraint on the TCP frame.
    #   3. If collision_avoid: add MinimumDistanceConstraint(1e-3).
    #   4. Set initial guess to q_seed (or plant context positions).
    #   5. result = Solve(ik.prog()); return result.GetSolution(ik.q()) or None.
    raise NotImplementedError
