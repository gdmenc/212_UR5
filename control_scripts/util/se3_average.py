"""Average rigid transforms (4x4 homogeneous) for noisy TCP / camera poses.

``_compute_T_task_camera`` historically averaged only translation while
keeping the last sample's rotation. Small orientation jitter on the wrist
camera couples into XY when deprojecting depth — averaging both parts
reduces that bias.
"""

from __future__ import annotations

import numpy as np


def _R_to_quat_wxyz(R: np.ndarray) -> np.ndarray:
    """Unit quaternion (w, x, y, z) from rotation matrix (Shepperd's method)."""
    R = np.asarray(R, dtype=float).reshape(3, 3)
    t = np.trace(R)
    if t > 0.0:
        s = 0.5 / np.sqrt(t + 1.0)
        w = 0.25 / s
        x = (R[2, 1] - R[1, 2]) * s
        y = (R[0, 2] - R[2, 0]) * s
        z = (R[1, 0] - R[0, 1]) * s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    q = np.array([w, x, y, z], dtype=float)
    n = np.linalg.norm(q)
    return q / n if n > 1e-12 else np.array([1.0, 0.0, 0.0, 0.0])


def _quat_wxyz_to_R(q: np.ndarray) -> np.ndarray:
    w, x, y, z = np.asarray(q, dtype=float).reshape(4)
    n = np.linalg.norm([w, x, y, z])
    if n < 1e-12:
        return np.eye(3)
    w, x, y, z = w / n, x / n, y / n, z / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


def average_se3(Ts: list[np.ndarray]) -> np.ndarray:
    """Mean of N rigid transforms: translation averaged, rotation via mean quaternion."""
    if not Ts:
        raise ValueError("average_se3: empty list")
    if len(Ts) == 1:
        return Ts[0].copy()
    t_mean = np.mean([np.asarray(T, dtype=float)[:3, 3] for T in Ts], axis=0)
    qs: list[np.ndarray] = []
    for T in Ts:
        R = np.asarray(T, dtype=float)[:3, :3]
        q = _R_to_quat_wxyz(R)
        if qs and float(np.dot(q, qs[-1])) < 0.0:
            q = -q
        qs.append(q)
    q_mean = np.mean(qs, axis=0)
    nq = np.linalg.norm(q_mean)
    R_mean = _quat_wxyz_to_R(q_mean / nq) if nq > 1e-9 else Ts[-1][:3, :3].copy()
    out = np.eye(4, dtype=float)
    out[:3, :3] = R_mean
    out[:3, 3] = t_mean
    return out
