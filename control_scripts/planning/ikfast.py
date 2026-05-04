"""Analytic UR5e IK via the ikfastpy extension.

We swap in the UR5e-generated ``ikfast61.cpp`` from
https://github.com/cambel/ur_ikfast (Apache 2.0) into Andy Zeng's
``ikfastpy`` (https://github.com/andyzeng/ikfastpy) build harness.
Verified to ~1-2 mm / ~1° agreement with Drake URDF FK; the residual
is the ~3-decimal link-length rounding in the source URDF that
generated the cpp. See ``verify_ikfast.py`` for measured errors.

Frame convention
----------------

ikfastpy's compiled FK/IK speaks the URDF's **REP-103 base frame**
(``base_link``, X+ forward). Our project's calibration and Drake plant
use the **UR controller's base frame** (welded ``base`` link, X+
backward — π-rotated around z). To keep the wrapper consistent with
the rest of the codebase (``X_BASE_TASK`` constants, RTDE-side poses,
``transit.py``, etc.), this module applies the π-z rotation internally
so callers always see the controller-frame convention:

  - ``forward(q)`` returns the wrist pose in the **controller** base.
  - ``solve_ik(X_base_tcp, ...)`` accepts a pose in the **controller**
    base and converts to ikfast's REP-103 frame internally.

API surface
-----------

>>> solutions = solve_ik(X_base_tcp, seed_q=current_q)
>>> closest   = closest_solution(X_base_tcp, seed_q=current_q)
>>> X_base    = forward(q)

All ``X_base_*`` poses use the **UR controller base** frame, matching
``calibration.X_LEFT_BASE_TASK`` / ``X_RIGHT_BASE_TASK`` and what
RTDE's ``getActualTCPPose`` returns.

Branch handling
---------------

``solve_ik`` returns up to 8 solutions sorted by joint-distance to
``seed_q`` (if provided), with each joint shifted by the nearest
2π-multiple to the seed (so chained planning naturally stays on the
same kinematic branch). ``closest_solution`` returns just the nearest,
or ``None`` if no IK solution exists.
"""

from __future__ import annotations

import sys
from pathlib import Path
from typing import List, Optional

import numpy as np
from pydrake.math import RigidTransform, RotationMatrix


# ---------------------------------------------------------------------------
#  Locate and load ikfastpy from the in-repo third_party clone
# ---------------------------------------------------------------------------

_IKFAST_DIR = (
    Path(__file__).resolve().parents[2] / "third_party" / "ikfastpy"
)


def _import_ikfastpy():
    """Import the compiled ikfastpy extension, ensuring the ``.so`` from
    ``third_party/ikfastpy/`` is on the Python path.

    Raises a clear error if the extension hasn't been built yet — the
    user has to run ``python3.11 setup.py build_ext --inplace`` once
    after cloning the repo (per Andy Zeng's README).
    """
    if not _IKFAST_DIR.exists():
        raise ImportError(
            f"third_party/ikfastpy/ not found at {_IKFAST_DIR}. "
            "Clone it: git clone https://github.com/andyzeng/ikfastpy.git "
            "into third_party/."
        )
    so_files = list(_IKFAST_DIR.glob("ikfastpy*.so"))
    if not so_files:
        raise ImportError(
            f"ikfastpy extension not built. From {_IKFAST_DIR}, run: "
            "python3.11 setup.py build_ext --inplace"
        )
    if str(_IKFAST_DIR) not in sys.path:
        sys.path.insert(0, str(_IKFAST_DIR))
    import ikfastpy  # noqa: E402
    return ikfastpy


_ikfastpy = _import_ikfastpy()
_KIN = _ikfastpy.PyKinematics()
N_DOF = _KIN.getDOF()
assert N_DOF == 6, f"ikfastpy kinematics has {N_DOF} DOF, expected 6"


# ikfastpy's compiled cpp speaks the URDF's REP-103 base_link frame
# (X+ forward). Our project speaks the UR controller's base frame
# (X+ backward, π-rotated around z). They share an origin, only
# orientation differs, so the transform is a pure z-rotation by π.
# It's its own inverse (R_z(π)^-1 = R_z(π)).
_R_BASE_TO_BASELINK = RotationMatrix.MakeZRotation(np.pi)
_X_BASE_TO_BASELINK = RigidTransform(_R_BASE_TO_BASELINK)
_X_BASELINK_TO_BASE = _X_BASE_TO_BASELINK.inverse()  # numerically identical


# ---------------------------------------------------------------------------
#  Public API
# ---------------------------------------------------------------------------


def forward(q: np.ndarray) -> RigidTransform:
    """FK: joint vector → wrist_3 pose in the **controller** base frame.

    Internally calls ikfastpy (which returns pose in REP-103 base_link)
    and applies the π-z rotation to land in our controller-frame
    convention. Verified accurate to ~1-2 mm against Drake URDF FK.
    """
    q = np.asarray(q, dtype=float)
    if q.shape != (N_DOF,):
        raise ValueError(f"q has shape {q.shape}, expected ({N_DOF},)")
    pose_3x4 = np.asarray(_KIN.forward(q.tolist())).reshape(3, 4)
    X_baselink_wrist = RigidTransform(
        RotationMatrix(pose_3x4[:, :3].copy()),
        pose_3x4[:, 3].copy(),
    )
    # X_base_wrist = X_base_to_baselink @ X_baselink_wrist
    return _X_BASE_TO_BASELINK @ X_baselink_wrist


def solve_ik(
    X_base_tcp: RigidTransform,
    *,
    seed_q: Optional[np.ndarray] = None,
    max_branch_dist: Optional[float] = None,
) -> List[np.ndarray]:
    """IK: TCP pose in BASE frame → list of joint solutions.

    Returns up to 8 solutions (closed-form roots of the inverse
    kinematics). When ``seed_q`` is provided, solutions are sorted by
    joint-space distance to the seed (closest first) — typical use is
    ``solutions[0]`` to grab the same-branch result.

    ``max_branch_dist``: drop solutions whose joint-distance from
    ``seed_q`` exceeds this (radians). Useful for filtering out
    branches that would require the wrist to flip 180°.
    """
    # Convert from controller base → REP-103 base_link for ikfast.
    X_baselink_tcp = _X_BASELINK_TO_BASE @ X_base_tcp
    pose_3x4 = np.empty((3, 4), dtype=float)
    pose_3x4[:, :3] = np.asarray(X_baselink_tcp.rotation().matrix())
    pose_3x4[:, 3] = np.asarray(X_baselink_tcp.translation())
    flat = pose_3x4.reshape(-1).tolist()

    raw = _KIN.inverse(flat)
    if not raw:
        return []
    n = len(raw) // N_DOF
    sols = np.asarray(raw, dtype=float).reshape(n, N_DOF)

    if seed_q is None:
        return [sol.copy() for sol in sols]

    seed = np.asarray(seed_q, dtype=float)
    # ikfastpy returns joint values in (-π, π]; shift each solution to
    # the "nearest equivalent angle" relative to the seed by adding
    # multiples of 2π per joint, then sort by distance.
    seed_aligned = sols.copy()
    for j in range(N_DOF):
        diff = seed_aligned[:, j] - seed[j]
        seed_aligned[:, j] -= 2 * np.pi * np.round(diff / (2 * np.pi))

    dists = np.linalg.norm(seed_aligned - seed, axis=1)
    order = np.argsort(dists)

    out: List[np.ndarray] = []
    for idx in order:
        if max_branch_dist is not None and dists[idx] > max_branch_dist:
            continue
        out.append(seed_aligned[idx].copy())
    return out


def closest_solution(
    X_base_tcp: RigidTransform,
    seed_q: np.ndarray,
    *,
    max_branch_dist: Optional[float] = None,
) -> Optional[np.ndarray]:
    """Convenience: nearest-to-seed IK solution, or ``None`` if
    unreachable / no branch within ``max_branch_dist``.

    Drop-in alternative to chained-SNOPT IK in motion planning: for a
    transit between consecutive task-frame poses, calling this with
    ``seed_q = previous_solution`` keeps consecutive waypoints on the
    same kinematic branch automatically.
    """
    sols = solve_ik(X_base_tcp, seed_q=seed_q, max_branch_dist=max_branch_dist)
    return sols[0] if sols else None
