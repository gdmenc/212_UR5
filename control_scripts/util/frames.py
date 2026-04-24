"""Task frame conversion.

Task frame is a shared, human-friendly frame that both arms and all cameras
refer to. By convention for the 212 rig:
    - origin at top center of the table
    - x axis points toward the right arm
    - y axis points toward the microwave
    - z axis points up

Each arm's base frame is related to the task frame by a per-arm calibration
pose ``X_base_task`` — the pose of the task origin expressed in the arm's
base frame. Primitives reason in task frame; the conversion to base frame
happens just before the RTDE call.

``Pose.__matmul__`` already composes correctly (same convention as
pydrake.RigidTransform). These wrappers exist so frame intent is loud at
the call site.
"""

from __future__ import annotations

import numpy as np
from scipy.spatial.transform import Rotation

from .poses import Pose


def task_to_base(X_base_task: Pose, p_task: Pose) -> Pose:
    """Express ``p_task`` (given in task frame) in the arm's base frame."""
    return X_base_task @ p_task


def base_to_task(X_base_task: Pose, p_base: Pose) -> Pose:
    """Express ``p_base`` (given in base frame) in task frame."""
    return X_base_task.inverse() @ p_base


def velocity_task_to_base(X_base_task: Pose, v_task) -> list:
    """Rotate a Cartesian twist (linear + angular, length 6) from task frame
    to base frame. Translation part of ``X_base_task`` is irrelevant for a
    velocity — only the rotation rotates the vectors.

    Used by move_until_contact so callers can say 'probe downward in task
    frame' without having to know each arm's base-frame orientation.
    """
    v = np.asarray(v_task, dtype=float).reshape(6)
    R = X_base_task.rotation
    linear_base = R.apply(v[:3])
    angular_base = R.apply(v[3:])
    return [*linear_base.tolist(), *angular_base.tolist()]
