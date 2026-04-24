"""Robotiq 2F-85 via rtde_c.sendCustomScriptFunction + URScript preamble.

Direct adaptation of ``ur_2026/robotiq_gripper_control.py`` — same protocol
the team has already been running on the rig, just wrapped behind our
``Gripper`` ABC so pick/place can stay gripper-agnostic.

How it works: each call ships the big ``ROBOTIQ_PREAMBLE`` URScript plus a
short call (e.g. ``rq_close_and_wait()``) to the UR controller via
``rtde_c.sendCustomScriptFunction``. The controller executes the script
on its side; the ``*_and_wait`` variants block in URScript until the
gripper finishes its motion, so our Python methods are naturally
synchronous.

Why this instead of the direct-socket approach (port 63352): the URScript
path is what the lab's reference code uses and has been validated on
this exact hardware. Keeping the pattern identical makes the whole
control_scripts package recognisable to anyone who has read the
ur_2026/ examples. See git history if we ever need to revisit the
socket version for richer object-detection feedback.

Usage:

    from rtde_control import RTDEControlInterface
    rtde_c = RTDEControlInterface("192.168.1.102")
    g = Robotiq2F85(rtde_c)
    g.activate()        # ~5 s one-time calibration
    g.set_speed(100)    # percent
    g.set_force(50)     # percent
    g.open()
    g.close()
"""

from __future__ import annotations

import time
from typing import Any, Optional

from .base import Gripper
from .robotiq_preamble import ROBOTIQ_PREAMBLE


# 2F-85 advertised max force at setting 100%. Used to map Newton arguments
# to percentage for set_force(). Datasheet-ballpark; tune if calibrated
# grasps feel off.
_MAX_FORCE_NEWTONS = 235.0

_DEFAULT_FORCE_PCT = 50
_DEFAULT_SPEED_PCT = 100


def _force_newtons_to_pct(force: Optional[float]) -> int:
    if force is None:
        return _DEFAULT_FORCE_PCT
    pct = int(round(force / _MAX_FORCE_NEWTONS * 100.0))
    return max(0, min(pct, 100))


class Robotiq2F85(Gripper):
    """Robotiq 2F-85 controlled via the UR controller's script interface.

    One instance per arm. Constructor takes the arm's ``rtde_c`` so the
    gripper shares its controller channel — no separate network socket,
    no extra connection to manage.
    """

    type_name = "robotiq_2f85"

    def __init__(self, rtde_c: Any) -> None:
        self._rtde_c = rtde_c

    # --- low-level bridge to sendCustomScriptFunction ---

    def _call(self, script_name: str, script_function: str) -> bool:
        """Ship the preamble + a short call. Blocks until the UR
        controller acknowledges the script. Returns True on success."""
        return self._rtde_c.sendCustomScriptFunction(
            "ROBOTIQ_" + script_name,
            ROBOTIQ_PREAMBLE + script_function,
        )

    # --- Robotiq-specific convenience (not in the ABC; kept public for
    # tuning from scripts) ---

    def set_speed(self, speed_pct: int) -> bool:
        """Set gripper speed as a percentage [0, 100]."""
        return self._call("SET_SPEED", f"rq_set_speed_norm({int(speed_pct)})")

    def set_force(self, force_pct: int) -> bool:
        """Set gripper force as a percentage [0, 100]."""
        return self._call("SET_FORCE", f"rq_set_force_norm({int(force_pct)})")

    # Gripper-ABC compatibility: forward the percent-based knobs that
    # pick()/place() apply from PickPlaceConfig.
    def set_speed_pct(self, speed_pct: int) -> None:
        self.set_speed(speed_pct)

    def set_force_pct(self, force_pct: int) -> None:
        self.set_force(force_pct)

    def move_mm(self, pos_in_mm: float) -> bool:
        """Move to a specific aperture in millimetres (0 = closed, ~85 = open)."""
        return self._call("MOVE", f"rq_move_and_wait_mm({int(pos_in_mm)})")

    # --- Gripper ABC implementation ---

    def activate(self) -> None:
        """Run the gripper's activation sequence. Takes ~5 s the first
        time after power-on; subsequent calls are effectively no-ops on
        the controller side but still include the 5 s hold for safety."""
        self._call("ACTIVATE", "rq_activate()")
        time.sleep(5)

    def open(self) -> None:
        self._call("OPEN", "rq_open_and_wait()")

    def close(self) -> None:
        self._call("CLOSE", "rq_close_and_wait()")

    def grasp(self, force: Optional[float] = None) -> bool:
        """Close with the given target force (Newtons). Returns True
        unconditionally — the URScript path does not expose per-call
        object-detection feedback cleanly. Callers that need to verify a
        grasp should read the TCP wrench or the held-object flag via an
        explicit follow-up query. TODO: if we start failing silently on
        empty grasps, add an ``rq_is_object_detected()`` query here."""
        self.set_force(_force_newtons_to_pct(force))
        self.close()
        return True

    def status(self) -> dict:
        """No live state polled through this path — returns a minimal
        descriptor for logging. For richer status (position, OBJ flag),
        switch back to the direct-socket client (see git history)."""
        return {"type": self.type_name}
