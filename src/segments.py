"""Segments: the command type sent from primitives to the execution backend.

A ``Segment`` is a small dataclass describing ONE atomic motion or gripper
action that a backend (sim or real) knows how to execute. Primitives compose
sequences of Segments; the sequencer orchestrates primitives; the backend
translates each Segment into Drake diagram updates (sim) or RTDE calls (real).

Why a dataclass seam? Keeping primitives and backends on opposite sides of a
serializable data type makes the sim/real swap a one-line choice, makes every
commanded motion trivially loggable and replayable, and keeps backends small
— each backend is basically a single dispatch function over the Segment union.

Note: most Segments map directly onto RTDE operations used in
``ur_2026/object_grasp_example.py``. That mapping is intentional — it lets the
real backend stay a thin wrapper.
"""

from dataclasses import dataclass
from typing import Optional, Sequence, Union

import numpy as np
from pydrake.math import RigidTransform


@dataclass
class MoveJ:
    """Joint-space move to a target configuration.

    Path is whatever the underlying controller chooses (usually time-scaled
    linear in joint space). Use for free-space transit when the exact path
    through the workspace does not matter — shortest joint distance is fine.

    Maps to ``rtde_c.moveJ(q_target, speed, accel)`` on real.
    """

    q_target: np.ndarray  # shape (6,), joint angles in radians
    speed: float = 1.05   # rad/s (RTDE default)
    accel: float = 1.4    # rad/s^2 (RTDE default)


@dataclass
class MoveL:
    """Cartesian linear move in tool space.

    TCP (tool center point) travels in a straight world-frame line to
    ``X_target``. Use for pre-grasp approach, retract-after-grasp, and any
    segment where the Cartesian path matters (e.g., avoiding a shelf edge).

    Maps to ``rtde_c.moveL(pose, speed, accel)`` on real.
    """

    X_target: RigidTransform
    speed: float = 0.1   # m/s
    accel: float = 0.25  # m/s^2


@dataclass
class ServoStream:
    """Stream a precomputed joint trajectory at high rate.

    Needed whenever a planner has produced a specific collision-free path we
    want to follow exactly — ``moveJ`` / ``moveL`` would discard it and plan
    their own. On real this is a tight ~125 Hz loop calling ``servoJ``.

    ``gain`` and ``lookahead_time`` map directly to the RTDE servoJ arguments
    (tradeoffs: higher gain = tighter tracking but more vibration; higher
    lookahead = smoother but laggy).
    """

    q_traj: np.ndarray          # shape (N, 6) joint waypoints
    dt: float = 0.008           # seconds between waypoints (125 Hz)
    lookahead_time: float = 0.1
    gain: int = 300             # RTDE range 100-2000


@dataclass
class ForceMode:
    """Cartesian compliant motion with commanded wrench along selected axes.

    Used for contact-rich tasks: pushing the tray corner, closing the
    microwave door. Arm becomes compliant along the ``selection_vector``
    axes (1 = compliant, 0 = stiff position) and tracks the commanded wrench
    on the compliant ones. This is the only way to safely apply a known
    force at the end-effector in real time.

    Maps to ``rtde_c.forceMode(task_frame, selection, wrench, type, limits)``.
    """

    task_frame: RigidTransform
    selection_vector: Sequence[int]  # length 6: 1 = compliant axis, 0 = stiff
    wrench: np.ndarray               # shape (6,), force (N) + torque (Nm)
    limits: np.ndarray               # shape (6,), max TCP speed per axis
    type: int = 2                    # 1=base, 2=tool, 3=base-aligned (UR docs)


@dataclass
class MoveUntilContact:
    """Linear TCP velocity until an external force threshold is crossed.

    Used for "place until surface hit" — command a downward velocity and let
    the controller stop the moment it feels resistance. Much more robust than
    hand-measuring table heights.

    Maps to ``rtde_c.moveUntilContact(v_base)`` on real.
    """

    v_base: np.ndarray      # shape (6,), linear + angular velocity, base frame
    accel: float = 0.5
    threshold: float = 10.0  # N, contact trigger


@dataclass
class GripperCommand:
    """Open / close / grasp-with-force on the gripper attached to this arm.

    Backend looks up the ``Gripper`` instance for the arm and dispatches.
    ``action="grasp"`` with a ``force`` is the intent-to-hold command
    (vs. ``"close"`` which just goes to minimum aperture).
    """

    action: str  # "open", "close", "grasp"
    force: Optional[float] = None  # N; only meaningful for "grasp"


@dataclass
class Wait:
    """Pause for a fixed duration.

    Mostly used for settling time after a contact event, or to let the
    gripper finish closing before the next Cartesian move begins.
    """

    duration: float  # seconds


Segment = Union[
    MoveJ, MoveL, ServoStream, ForceMode, MoveUntilContact, GripperCommand, Wait
]
