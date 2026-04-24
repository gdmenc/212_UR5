"""Grasp dataclass — the parcel a primitive consumes.

Wraps everything the pick primitive needs to execute one specific grasp
attempt: the task-frame TCP pose, a per-object pregrasp standoff, and the
gripper-close force. The ``description`` field is purely for logging and
debug viz — it should tell you at a glance what the grasp is ('plate rim
at 90°', 'bowl rim at 0°'), so failure messages name the attempt.

Each object module (plate.py, bowl.py, ...) exports factory functions
that return ``Grasp`` instances built from the object's task-frame pose
plus any object-specific parameters.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

from ..util.poses import Pose


@dataclass
class Grasp:
    grasp_pose: Pose
    """Task-frame TCP pose at which the gripper fingers close on the object."""

    pregrasp_offset: float
    """Distance (m) along tool -Z from grasp_pose to pregrasp. Per-object
    because a thin plate rim tolerates a short offset (≈ 2 cm) while a
    tall bowl needs more room (≈ 5 cm) to clear its curvature."""

    grasp_force: float = 20.0
    """Target holding force (N) for Robotiq close."""

    description: Optional[str] = None
    """Human-readable label for logging/debugging."""
