"""Abstract gripper interface.

Primitives talk to grippers through this ABC so swapping a Robotiq 2F-85 for
the team's custom hook gripper at assembly time requires changing zero lines
of primitive code. The ``GripperCommand`` segment is dispatched by the
backend to whichever ``Gripper`` instance is attached to the arm in question.

Each concrete subclass provides:
  - A sim side (loads/welds a geometry into the Drake plant; exposes finger
    or hook joints for control and sensing).
  - A real side (driver interface — TCP socket for Robotiq, TBD for hook).
  - A high-level API (``open``, ``close``, ``grasp``, ``status``) used by the
    primitives. ``grasp`` is the semantic "hold the object with a target
    force"; ``close`` is just "go to minimum aperture, no force target."
"""

from abc import ABC, abstractmethod
from typing import Optional


class Gripper(ABC):
    """One instance per arm. Constructor takes either the sim plant or a real
    driver handle depending on which side is being wired up — not both.

    Subclasses set ``type_name`` (short, stable string) so other modules can
    dispatch on gripper kind without ``isinstance`` — in particular the grasp
    candidate lookup in ``src/grasping/`` keys off this because a hook-latch
    pose and a two-finger pinch are genuinely different grasp geometries.
    """

    type_name: str  # e.g. "hook", "robotiq_2f85". Override in subclasses.

    @abstractmethod
    def open(self) -> None:
        """Drive to fully-open aperture and block until reached."""

    @abstractmethod
    def close(self) -> None:
        """Drive to fully-closed aperture and block until reached or stalled."""

    @abstractmethod
    def grasp(self, force: Optional[float] = None) -> bool:
        """Close with a target holding force (Newtons).

        Returns True if an object was detected (fingers stopped before fully
        closing). Used by pick primitives to branch on grasp success.
        """

    @abstractmethod
    def status(self) -> dict:
        """Current aperture, force, is_holding_object flag, etc.

        Shape kept loose so concrete grippers can expose whatever sensors
        they have without changing this ABC.
        """
