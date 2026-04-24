"""Gripper ABC — one instance per arm.

Primitives talk to grippers through this interface so swapping a Robotiq
2F-85 for the team's custom hook at connection time requires zero changes
in pick/place. The ``grasp(force)`` contract is the important one: it
returns ``True`` iff the gripper detected holding an object, so pick() can
branch on grasp success without polling extra sensors.

Concrete implementations set ``type_name`` to a short stable string
("robotiq_2f85", "hook") so other modules can key off the gripper kind
without isinstance checks — relevant for per-gripper grasp candidate
tables.
"""

from abc import ABC, abstractmethod
from typing import Optional


class Gripper(ABC):
    type_name: str = ""

    @abstractmethod
    def activate(self) -> None:
        """Prepare the gripper to execute commands. Blocks until ready.
        For Robotiq this is the calibration/activation sequence; for a
        simple DO-pin hook it is a no-op."""

    @abstractmethod
    def open(self) -> None:
        """Drive to fully-open aperture and block until reached."""

    @abstractmethod
    def close(self) -> None:
        """Drive to fully-closed aperture and block until reached or stalled."""

    @abstractmethod
    def grasp(self, force: Optional[float] = None) -> bool:
        """Close with a target holding force (Newtons).

        Returns True iff an object was detected (fingers stalled before
        fully closing). Used by pick primitives to branch on grasp
        success. ``force=None`` means 'use a sensible default for the
        gripper'."""

    @abstractmethod
    def status(self) -> dict:
        """Current aperture, force, is_holding_object flag, etc.
        Shape kept loose so concrete grippers expose whatever sensors
        they have without changing the ABC."""

    def disconnect(self) -> None:
        """Release any held resources (sockets, serial ports, ...).
        Default is no-op; override if the implementation needs cleanup."""
        pass

    # --- Optional tuning knobs. Default no-ops so pick/place can call
    # these unconditionally; only grippers that actually support speed/
    # force control (e.g. Robotiq) override them.

    def set_speed_pct(self, speed_pct: int) -> None:
        """Motion speed as a percentage [0, 100]. Default no-op."""
        pass

    def set_force_pct(self, force_pct: int) -> None:
        """Motion force (max contact envelope) as a percentage [0, 100].
        Default no-op. Note: per-grasp force still comes from Grasp.grasp_force
        (in Newtons) and overrides this at grasp time."""
        pass
