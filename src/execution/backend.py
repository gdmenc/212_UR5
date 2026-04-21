"""Execution backend protocol.

The backend is the seam between *what* to do (Segments emitted by primitives)
and *how* it runs (Drake diagram on sim, RTDE on real). Primitives talk to
the backend exclusively through three methods:

    execute(arm, segment)   # side-effect: drives the arm
    state(arm) -> dict      # read: current q, v, TCP pose, wrench, etc.
    gripper(arm) -> Gripper # lookup: the gripper attached to this arm

Anything not expressible this way (hardware-specific tuning, diagram
wiring, RTDE reconnect logic) lives inside the concrete backend, not in
primitives. That's the whole point of the seam.

Implementations:
  - ``SimBackend`` (sim_backend.py) drives a Drake ``Simulator``.
  - ``RealBackend`` (real_backend.py) drives two UR5e robots over RTDE.
"""

from typing import Any, Protocol

from ..grippers.base import Gripper
from ..segments import Segment


class Backend(Protocol):
    def execute(self, arm: str, segment: Segment) -> None:
        """Block-executes one ``Segment``. Raises on unrecoverable failure."""

    def state(self, arm: str) -> dict:
        """Return a dict with at least:
        ``q`` (joint positions), ``v`` (joint velocities),
        ``X_WTcp`` (current TCP pose), ``wrench`` (TCP wrench)."""

    def gripper(self, arm: str) -> Gripper:
        """Return the ``Gripper`` instance attached to ``arm``."""
