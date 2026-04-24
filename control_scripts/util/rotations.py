"""Minimal rotation helper used by control_scripts.

This replaces the small subset of scipy Rotation functionality that the
real-control path uses:
    - Rotation.identity()
    - Rotation.from_rotvec(...)
    - Rotation.from_matrix(...)
    - rotation.as_rotvec()
    - rotation.apply(...)
    - rotation.inv()
    - composition via `A * B`

The implementation is matrix-backed and uses only numpy so the RTDE-side
tools stay lightweight.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np


_EPS = 1e-12


def _skew(v: np.ndarray) -> np.ndarray:
    x, y, z = v
    return np.array([
        [0.0, -z, y],
        [z, 0.0, -x],
        [-y, x, 0.0],
    ])


@dataclass
class Rotation:
    _matrix: np.ndarray = field(default_factory=lambda: np.eye(3))

    def __post_init__(self) -> None:
        self._matrix = np.asarray(self._matrix, dtype=float).reshape(3, 3)

    @classmethod
    def identity(cls) -> "Rotation":
        return cls(np.eye(3))

    @classmethod
    def from_matrix(cls, matrix) -> "Rotation":
        return cls(matrix)

    @classmethod
    def from_rotvec(cls, rotvec) -> "Rotation":
        r = np.asarray(rotvec, dtype=float).reshape(3)
        theta = np.linalg.norm(r)
        if theta < _EPS:
            return cls.identity()

        axis = r / theta
        K = _skew(axis)
        R = np.eye(3) + np.sin(theta) * K + (1.0 - np.cos(theta)) * (K @ K)
        return cls(R)

    def as_rotvec(self) -> np.ndarray:
        R = self._matrix
        cos_theta = np.clip((np.trace(R) - 1.0) * 0.5, -1.0, 1.0)
        theta = float(np.arccos(cos_theta))

        if theta < 1e-8:
            return np.zeros(3)

        if np.pi - theta < 1e-6:
            axis = np.array([
                np.sqrt(max((R[0, 0] + 1.0) * 0.5, 0.0)),
                np.sqrt(max((R[1, 1] + 1.0) * 0.5, 0.0)),
                np.sqrt(max((R[2, 2] + 1.0) * 0.5, 0.0)),
            ])

            if (R[2, 1] - R[1, 2]) < 0.0:
                axis[0] = -axis[0]
            if (R[0, 2] - R[2, 0]) < 0.0:
                axis[1] = -axis[1]
            if (R[1, 0] - R[0, 1]) < 0.0:
                axis[2] = -axis[2]

            axis_norm = np.linalg.norm(axis)
            if axis_norm < _EPS:
                axis = np.array([1.0, 0.0, 0.0])
            else:
                axis = axis / axis_norm
            return axis * theta

        axis = np.array([
            R[2, 1] - R[1, 2],
            R[0, 2] - R[2, 0],
            R[1, 0] - R[0, 1],
        ]) / (2.0 * np.sin(theta))
        return axis * theta

    def apply(self, vec):
        arr = np.asarray(vec, dtype=float)
        if arr.ndim == 1:
            return self._matrix @ arr
        return arr @ self._matrix.T

    def inv(self) -> "Rotation":
        return Rotation(self._matrix.T)

    def __mul__(self, other: "Rotation") -> "Rotation":
        return Rotation(self._matrix @ other._matrix)
