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

from .util.poses import Pose
from .util.rotations import Rotation


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
_LEFT_DZ_T = -0.815                     # task origin is 0.815 m BELOW base along task z
# -0.753 (original hand-measured) - 0.062 (correction from bowl perception 2026-05-03:
#  bowl base on table was reading -0.062 m instead of 0; shifted task origin down)

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
TCP_OFFSET_ROBOTIQ_2F85 = [0.0, 0.0, 0.184, 0.0, 0.0, 0.0]
"""Wrist-3 flange to fingertip pinch point for the Robotiq 2F-85."""

TCP_OFFSET_HOOK = [0.0, 0.0, 0.10275, 0.0, 1.5708, 0.0]
"""Wrist-3 flange to hook engagement point (the rim seat inside the throat).

Translation: 10.275 cm purely along flange +z. The engagement point sits
between the fixed jaw (at flange z = 9.1 cm) and the moving finger's inner
edge (at flange z = 12.7 cm) — closer to the fixed jaw, since the moving
finger retracts in flange -z toward the fixed jaw to clamp.

Rotation: R_y(+π/2). Maps tool +Z to flange +X. The hook is welded so
that, at a natural rim-grasp wrist orientation, flange +X points task -Z
(vertically down) — verified against a recorded grasp pose at angle π
(see grasps/bottle.py::bottle_hook_grasp). After this TCP rotation, tool
+Z aligns with task -Z (down), so the package convention "approach =
+tool_Z" gives a vertical descent and ``offset_along_tool_z(grasp, d)``
produces a pregrasp exactly d above the grasp in task z.
"""


# ---------------------------------------------------------------------------
# Home joint configurations — the "safe ready" pose each arm goes to at
# session start (before any pick/place is issued). Joint order is UR5e
# standard: [shoulder_pan, shoulder_lift, elbow, wrist_1, wrist_2, wrist_3].
# ---------------------------------------------------------------------------
HOME_Q_RAD_LEFT = np.radians([
    -48.24, -101.16, -107.03, -99.73, -120.90, -45.00
])
"""Left arm home, copied verbatim from ur_2026/object_grasp_example.py.
Verified by the team in previous runs as a reachable, collision-free
pose above the workspace."""

HOME_Q_RAD_RIGHT: "np.ndarray | None" = None
"""TODO: measure the right arm's home at the lab by manually moving to a
safe ready pose and reading joint angles via rtde_r.getActualQ(). Setting
``None`` here means ``Session.move_to_home()`` will SKIP homing the right
arm — safer than guessing at a pose and commanding a wrong motion."""
