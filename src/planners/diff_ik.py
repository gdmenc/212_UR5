"""Differential inverse kinematics (Cartesian velocity streaming).

Used when we want the TCP to follow a Cartesian velocity profile in real
time — pre-grasp approach, pour-arc tracking, Cartesian push — and we need
joint commands sent at the controller's update rate.

Drake's ``DifferentialInverseKinematicsIntegrator`` does the heavy lifting:
given a spatial velocity ``V_WTcp`` and a time step ``dt``, it produces the
next joint configuration while respecting joint position, velocity, and
acceleration limits (and nullspace preferences if configured).

This wrapper gives callers a simple ``step(V_WTcp, dt) -> q_next`` method,
hiding the LeafSystem / Context plumbing.
"""

from typing import Optional

import numpy as np
from pydrake.multibody.plant import MultibodyPlant


class DiffIK:
    """Cartesian-velocity -> joint-position integrator for one arm."""

    def __init__(
        self,
        plant: MultibodyPlant,
        arm_name: str,
        tcp_body_name: Optional[str] = None,
        time_step: float = 0.008,
    ) -> None:
        self._plant = plant
        self._arm = arm_name
        self._tcp = tcp_body_name
        self._dt = time_step
        # TODO: build DifferentialInverseKinematicsIntegrator,
        # allocate a context, configure parameters (velocity bounds,
        # nominal joint position, end-effector frame).
        raise NotImplementedError

    def step(self, V_WTcp: np.ndarray, dt: Optional[float] = None) -> np.ndarray:
        """Integrate one step and return the commanded joint configuration.

        V_WTcp : shape (6,), [omega_x, omega_y, omega_z, v_x, v_y, v_z]
            spatial velocity in the world frame.
        dt : step size; defaults to the integrator's configured time_step.
        """
        raise NotImplementedError

    def reset(self, q: np.ndarray) -> None:
        """Reset the integrator's internal joint state (e.g. after a MoveJ
        that was executed by a different controller)."""
        raise NotImplementedError
