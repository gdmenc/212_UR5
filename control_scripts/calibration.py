"""Per-arm rig calibration: X_base_task and TCP offset.

Values reflect hand-measured geometry of the 212 lab bimanual rig (vention-
mounted bimanual UR5e, left arm IP .101, right arm IP .102). Numbers are
copied from ``ur_2026/object_grasp_example.py`` so this module is the
single source of truth going forward — update once here when the rig
changes and every call site picks it up.

Task frame convention (see ``util/frames.py``):
    - origin at top center of the table
    - x axis toward the right arm
    - y axis toward the microwave
    - z axis up

TODO: the origin choice is arbitrary and was never written down in prose.
The strongest evidence for what it actually is: the top-down camera's
task-frame x-offset is ``dx_task_to_TC = 0.0125`` m (~1 cm), i.e. the
camera sits almost exactly over task x = 0. This is very unlikely to be
coincidence — the origin was probably picked to line up with the top-down
camera in x so perception outputs translate cleanly into task-frame xy.
Origin y is also close to under the camera but with a ~16 cm offset
(``dy_task_to_TC ≈ 0.157``), so it's not centered under the camera in y.
Best current interpretation: **origin sits on the tabletop directly below
the top-down camera in x, and at the geometric center of the table's
working depth in y**.

Confirm on day one at the lab: with both arms home, touch each arm's TCP
to the point directly below the camera at table height and compare the
xy task-frame reading against 0. If off by more than ~2 cm, update these
numbers before running any perception-driven motion.

What the numbers mean
---------------------
For each arm we measure the vector from the arm's base origin to the task
origin, **expressed in task-frame coordinates** (dx_t, dy_t, dz_t). Then a
rotation matrix R whose columns are the task-frame basis vectors expressed
in the arm's base frame — i.e. R takes a task-frame vector and returns the
same vector in base frame. ``_build_X_base_task`` combines these into the
``Pose`` of the task origin as seen from the arm's base.

Note on the rotation: the columns of R being sin/cos of 45° tells you the
arms are mounted at 45° pointing down at the table — consistent with the
vention rig in ``scenario_files/bimanual.yaml`` welded at rpy [90, 0, 180].
"""

from __future__ import annotations

import numpy as np
from scipy.spatial.transform import Rotation

from .util.poses import Pose


def _build_X_base_task(
    dx_t: float,
    dy_t: float,
    dz_t: float,
    R_task_to_base: np.ndarray,
) -> Pose:
    """dx_t, dy_t, dz_t: vector from base origin to task origin, in TASK frame.
    R_task_to_base: 3x3, columns are task basis vectors in base frame.
    Returns X_base_task: pose of the task origin in base frame."""
    v_task = np.array([dx_t, dy_t, dz_t], dtype=float)
    v_base = R_task_to_base @ v_task
    return Pose(
        translation=v_base,
        rotation=Rotation.from_matrix(R_task_to_base),
    )


# ---------------------------------------------------------------------------
# Left arm (ur_left, 192.168.1.101, hook gripper)
# ---------------------------------------------------------------------------
_LEFT_DX_T = 0.090 / 2 + 0.010 + 0.110  # half vertical beam + plate + base-to-beam
_LEFT_DY_T = 0.225 / 2 + 0.540 / 2      # half mounting plate + half table depth
_LEFT_DZ_T = -0.753                     # task origin is 0.753 m BELOW base along task z

_LEFT_R = np.array([
    [ 0.707, 0, -0.707],
    [ 0,    -1,  0    ],
    [-0.707, 0, -0.707],
])

X_LEFT_BASE_TASK: Pose = _build_X_base_task(
    _LEFT_DX_T, _LEFT_DY_T, _LEFT_DZ_T, _LEFT_R
)


# ---------------------------------------------------------------------------
# Right arm (ur_right, 192.168.1.102, Robotiq 2F-85)
# ---------------------------------------------------------------------------
_RIGHT_DX_T = -(0.090 / 2 + 0.010 + 0.110)  # sign flip — right arm is on the +x side
_RIGHT_DY_T = 0.225 / 2 + 0.540 / 2
_RIGHT_DZ_T = -0.753

_RIGHT_R = np.array([
    [ 0.707, 0, 0.707],
    [ 0,    -1, 0    ],
    [ 0.707, 0, -0.707],
])

X_RIGHT_BASE_TASK: Pose = _build_X_base_task(
    _RIGHT_DX_T, _RIGHT_DY_T, _RIGHT_DZ_T, _RIGHT_R
)


# ---------------------------------------------------------------------------
# TCP offsets — 6-vector [x, y, z, rx, ry, rz] for rtde_c.setTcp().
# Rotation zero because standard tools don't re-orient the tool axis.
# Must be applied once per connection (ArmHandle.setup()).
# ---------------------------------------------------------------------------
TCP_OFFSET_ROBOTIQ_2F85 = [0.0, 0.0, 0.174, 0.0, 0.0, 0.0]
"""Wrist-3 flange to fingertip pinch point for the Robotiq 2F-85."""

TCP_OFFSET_HOOK: list = None  # type: ignore
"""TODO: measure from wrist-3 flange to the hook engagement point and set
here. Leaving None so ArmHandle.setup() raises a clear error if it's used
before calibration."""
