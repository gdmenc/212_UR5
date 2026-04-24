"""Custom hook gripper (welded to ur_left's wrist_3).

Single-DOF latch: the hook extends to engage a handle/edge and retracts
to disengage. Not a general-purpose pinch — semantics are "latched" vs.
"unlatched," not "holding at N Newtons."

Actuation is hardware-TBD. This class supports two paths:

    1. ``actuation="do_pin"`` (IMPLEMENTED): toggle a digital output pin
       on the UR control box via RTDE's ``setStandardDigitalOut``. Wire
       the hook's servo/solenoid to a pin on the tool I/O port. Simplest
       path; no extra host-side software.
    2. ``actuation="serial"`` (STUB): USB-serial to a microcontroller on
       the hook (hobby servo or smart motor). Gives position + force
       feedback at the cost of an extra cable + driver. Left raising
       NotImplementedError until the hook's electronics are finalized.

Pick the path at construction time so pick/place code stays identical.
"""

from __future__ import annotations

from typing import Any, Optional

from .base import Gripper


DEFAULT_DO_PIN = 0
"""Which standard digital output pin drives the hook. UPDATE TO MATCH
WIRING once the hook is installed on the rig."""

EXTEND_HIGH = True
"""Polarity: True means 'DO high = extend/latch'. Flip if the wiring
convention is inverted."""


class HookGripper(Gripper):
    type_name = "hook"

    def __init__(
        self,
        rtde_c: Any,
        actuation: str = "do_pin",
        do_pin: int = DEFAULT_DO_PIN,
    ) -> None:
        if actuation not in ("do_pin", "serial"):
            raise ValueError(
                f"actuation must be 'do_pin' or 'serial', got {actuation!r}"
            )
        self._rtde_c = rtde_c
        self._actuation = actuation
        self._do_pin = do_pin
        self._extended: Optional[bool] = None

    def activate(self) -> None:
        """No-op on DO-pin path (the pin is always available). Serial
        path needs a handshake that isn't built yet."""
        if self._actuation == "serial":
            raise NotImplementedError("serial actuation not yet implemented")

    def open(self) -> None:
        """Retract the hook (unlatch)."""
        if self._actuation == "do_pin":
            self._rtde_c.setStandardDigitalOut(self._do_pin, not EXTEND_HIGH)
            self._extended = False
        else:
            raise NotImplementedError("serial actuation not yet implemented")

    def close(self) -> None:
        """Extend the hook (latch)."""
        if self._actuation == "do_pin":
            self._rtde_c.setStandardDigitalOut(self._do_pin, EXTEND_HIGH)
            self._extended = True
        else:
            raise NotImplementedError("serial actuation not yet implemented")

    def grasp(self, force: Optional[float] = None) -> bool:
        """Latch. The ``force`` argument is ignored — DO-pin actuation has
        no load cell on the hook itself. Returns True unconditionally on
        DO-pin path (the UR controller acks the pin write; actual
        engagement must be verified via TCP wrench externally). When the
        serial path adds force feedback, this should return based on that.
        """
        self.close()
        return True

    def status(self) -> dict:
        return {
            "actuation": self._actuation,
            "extended": self._extended,
        }
