"""Stir primitive: sweep the TCP along a horizontal circle inside the cup.

Assumes the arm is already holding the stirrer (after a pick). We
parameterize the TCP position on a circle of radius ``radius`` centered on
``cup_origin`` and sweep through ``num_rotations`` full turns. Orientation
stays fixed — the stirrer rod points straight down throughout, which
keeps the rod's lower end submerged.

Implementation is the same shape as ``pour``: compute a Cartesian
trajectory at control rate, use ``DiffIK`` to produce joint velocities,
stream as a ``ServoStream``. Using ``MoveL`` waypoints would cause a
hiccup at each segment boundary — we want a continuous sweep.

High-level flow:

    diff_ik = DiffIK(plant, arm)
    diff_ik.reset(scene.q(arm))
    thetas = linspace(0, 2*pi*num_rotations, N)
    q_traj = []
    for theta in thetas:
        p = cup_origin + radius * [cos(theta), sin(theta), 0]
        # hold orientation fixed; only linear velocity changes each step
        V_WTcp = spatial_velocity(p_prev, p_curr, dt)
        q_traj.append(diff_ik.step(V_WTcp, dt))
    backend.execute(arm, ServoStream(q_traj, dt=dt))
"""

from typing import Optional

import numpy as np

from .base import PrimitiveResult


def stir(
    backend,
    planner,
    scene,
    arm: str,
    cup_origin: np.ndarray,    # shape (3,), world-frame cup center (inside bottom)
    cup_axis: np.ndarray,      # shape (3,), unit axis (usually [0,0,1]); stirring plane is perpendicular
    radius: float = 0.015,     # m; rod tip radius from cup center
    num_rotations: float = 3.0,
    revs_per_second: float = 1.0,
) -> PrimitiveResult:
    """Sweep the held stirrer in a circle inside the cup."""
    # TODO
    raise NotImplementedError
