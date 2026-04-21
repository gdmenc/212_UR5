"""Pour primitive: rotate TCP about a fixed axis through a fixed origin.

Assumes the arm is already holding the bottle/container (after a pick). We
parameterize the TCP pose as a rotation about ``pour_axis`` through
``pour_origin`` (which is approximately the bottle's lip, so liquid comes
out where we want it) and sweep from 0 to ``pour_angle``.

Implementation choice: compute the Cartesian path at control rate, use
``DiffIK`` to map each Cartesian velocity sample to joint velocities, and
stream the resulting joint waypoints through ``ServoStream``. This is
preferable to expressing the arc as a sequence of ``MoveL`` segments
because MoveL replans internally each time and the arm would hiccup
between segments.

High-level flow:

    diff_ik = DiffIK(plant, arm)
    diff_ik.reset(scene.q(arm))
    q_traj = []
    for theta in linspace(0, pour_angle, N):
        V_WTcp = axis_angle_velocity(pour_axis, pour_origin, dtheta)
        q_next = diff_ik.step(V_WTcp, dt)
        q_traj.append(q_next)
    backend.execute(arm, ServoStream(q_traj))
    Wait(hold_time)                 # let the liquid exit
    reverse same trajectory         # return to upright
"""

from typing import Optional

import numpy as np

from .base import PrimitiveResult


def pour(
    backend,
    planner,
    scene,
    arm: str,
    pour_origin: np.ndarray,   # shape (3,), world-frame point the lip rotates about
    pour_axis: np.ndarray,     # shape (3,), unit axis of rotation (world frame)
    pour_angle: float,         # radians; ~1.5 for full pour
    hold_time: float = 2.0,    # seconds at pour_angle before returning
    num_samples: int = 50,
) -> PrimitiveResult:
    """Pour by rotating the held container about pour_axis through pour_origin."""
    # TODO
    raise NotImplementedError
