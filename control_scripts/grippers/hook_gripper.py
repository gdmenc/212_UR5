"""Custom hook gripper (welded to ur_left's wrist_3).

Single-DOF clamp: the moving finger RETRACTS to close the throat (clamping
a rim against the fixed jaw) and EXTENDS to open the throat (release).
Semantics are "clamped" vs "released," not "holding at N Newtons" — there's
no force feedback on the do_pin path.

Geometric model lives below the imports — TCP offset, jaw positions, throat
width, and lateral extents are all measured from the wrist_3 flange. The
TCP itself is set in ``calibration.TCP_OFFSET_HOOK`` and must stay in sync
with ``HOOK_TCP_Z_M`` here.

Actuation is hardware-TBD. This class supports two paths:

    1. ``actuation="do_pin"`` (IMPLEMENTED): toggle a digital output pin
       on the tool I/O connector via ``rtde_io.RTDEIOInterface.setToolDigitalOut``.
       UR splits motion (``RTDEControlInterface``) from IO (``RTDEIOInterface``);
       Session opens both and passes the IO handle here. Wire the hook's servo
       or solenoid to tool digital output 0 or 1.
    2. ``actuation="serial"`` (STUB): USB-serial to a microcontroller on
       the hook (hobby servo or smart motor). Gives position + force
       feedback at the cost of an extra cable + driver. Left raising
       NotImplementedError until the hook's electronics are finalized.

Pick the path at construction time so pick/place code stays identical.
"""

from __future__ import annotations

import time
from typing import Any, Optional

from .base import Gripper


# =========================================================================
#  Physical geometry of the hook (welded gripper for ur_left)
# =========================================================================
# All distances are measured from the wrist_3 flange (mounting plate face).
#   Flange-z:  perpendicular to the flange face, positive into the workspace.
#   Flange-xy: lateral plane parallel to the flange face. The hook is
#              welded such that +x_flange points toward the throat opening
#              — the side a rim enters from. Verified against a recorded
#              grasp pose: with TCP_OFFSET_HOOK = R_y(+π/2), tool +Z (the
#              package's "approach" direction) lands on flange +X, which
#              the rig naturally orients to task -Z (vertical down) at a
#              rim-grasp wrist pose.
#
# Throat structure (cross-section perpendicular to the flange face):
#
#       flange face
#       │
#       │             ┌──── HOOK_FIXED_JAW_Z_M             (0.091)
#       │             │       fixed jaw
#       │   throat ──→│··············  HOOK_TCP_Z_M        (0.10275)
#       │             │       (rim seats here; matches
#       │             │        calibration.TCP_OFFSET_HOOK)
#       │             └──── HOOK_FINGER_INNER_EDGE_Z_M     (0.127, open)
#                              moving finger (retracts to clamp)
#
# Throat width at full open = 0.127 − 0.091 = 0.036 m. The finger retracts
# along flange-z toward the fixed jaw to clamp; "close()" = retract.

HOOK_FIXED_JAW_Z_M = 0.091
"""Flange-z of the fixed jaw — the stationary surface opposite the moving
finger across the throat. The moving finger clamps the rim against this
jaw on close."""

HOOK_FINGER_INNER_EDGE_Z_M = 0.127
"""Flange-z of the moving finger's inner edge when fully OPEN (extended).
Retracts toward the fixed jaw to clamp."""

HOOK_THROAT_WIDTH_M = HOOK_FINGER_INNER_EDGE_Z_M - HOOK_FIXED_JAW_Z_M
"""3.6 cm — gap between the two jaws when fully open. Maximum rim
thickness the throat can accept."""

HOOK_TCP_Z_M = 0.10275
"""Flange-z of the engagement point / TCP. Mirrors
``calibration.TCP_OFFSET_HOOK[2]`` — keep the two in sync if either
is re-measured."""

HOOK_CAMERA_Z_M = 0.090
"""Flange-z of the on-hook camera. Note this is LESS than HOOK_TCP_Z_M, so
on a flange-down descent the camera sits ABOVE the rim height, not below.
The leading edge during descent is the finger inner edge at 0.127 m."""

# ---- Lateral (xy-plane) extents from the mounting plate center -----------
# +x_flange = side the throat opens toward (the side a rim enters from).

HOOK_FINGER_TIP_LATERAL_M = 0.016
"""+x_flange offset of the finger tip — i.e., how far the moving finger
protrudes laterally on the throat-opening side. Doubles as the vertical
clearance from the TCP to the throat opening at the grasp pose: with
TCP_OFFSET_HOOK = R_y(+π/2) and the rig's natural rim-grasp wrist
orientation, +x_flange = task -z, so this is the descent distance from
"rim crosses throat opening" to "rim seated at TCP" — 1.6 cm."""

HOOK_TOP_OF_HOOK_LATERAL_M = 0.038
"""−x_flange offset of the top of the hook piece — lateral extent on the
side OPPOSITE the throat opening. Total lateral span of the hook structure
is 0.016 + 0.038 = 0.054 m."""

HOOK_FINGER_LATERAL_LENGTH_M = 0.055
"""Lateral length of the moving finger piece, top attachment to tip."""

HOOK_FINGER_TOP_TO_HOOK_BOTTOM_LATERAL_M = 0.063
"""Lateral distance from the top of the finger to the lowest point of the
hook curve. Approximates the lateral envelope of the closed hook."""

HOOK_FINGER_TOP_TO_CAMERA_LATERAL_M = 0.090
"""Lateral distance from the top of the finger to the camera. Camera sits
0.090 − 0.063 = 0.027 m past the bottom of the hook curve laterally."""


# =========================================================================
#  Actuation
# =========================================================================

DEFAULT_DO_PIN = 0
"""Which standard digital tool output pin drives the hook. UPDATE TO MATCH
WIRING once the hook is installed on the rig."""

RETRACT_HIGH = False
"""Polarity: True means 'DO high = finger retracts = throat closes/clamps'.
Flip if the wiring convention is inverted. ``close()`` writes RETRACT_HIGH
to the pin; ``open()`` writes its negation."""

DEFAULT_CLOSE_SETTLE_S = 1
"""Seconds to wait after commanding close before reporting grasp success.
The DO-pin path has no feedback, so this gives the hook actuator time to
finish retracting before the arm starts the post-grasp lift."""


class HookGripper(Gripper):
    type_name = "hook"

    def __init__(
        self,
        rtde_io: Any,
        actuation: str = "do_pin",
        do_pin: int = DEFAULT_DO_PIN,
        close_settle_s: float = DEFAULT_CLOSE_SETTLE_S,
    ) -> None:
        if actuation not in ("do_pin", "serial"):
            raise ValueError(
                f"actuation must be 'do_pin' or 'serial', got {actuation!r}"
            )
        if close_settle_s < 0.0:
            raise ValueError("close_settle_s must be non-negative")
        if actuation == "do_pin" and rtde_io is None:
            raise ValueError(
                "HookGripper(do_pin path) requires an RTDEIOInterface instance "
                "(use Session with needs_rtde_io=True)."
            )
        self._rtde_io = rtde_io
        self._actuation = actuation
        self._do_pin = do_pin
        self._close_settle_s = close_settle_s
        self._extended: Optional[bool] = None

    def activate(self) -> None:
        """No-op on DO-pin path (the pin is always available). Serial
        path needs a handshake that isn't built yet."""
        if self._actuation == "serial":
            raise NotImplementedError("serial actuation not yet implemented")

    def open(self) -> None:
        """Extend the finger — opens the throat, releases any held object."""
        if self._actuation == "do_pin":
            self._rtde_io.setToolDigitalOut(self._do_pin, not RETRACT_HIGH)
            self._extended = True
        else:
            raise NotImplementedError("serial actuation not yet implemented")

    def close(self) -> None:
        """Retract the finger — closes the throat, clamps the rim."""
        if self._actuation == "do_pin":
            self._rtde_io.setToolDigitalOut(self._do_pin, RETRACT_HIGH)
            self._extended = False
        else:
            raise NotImplementedError("serial actuation not yet implemented")

    def prepare_for_grasp(self, target_aperture_mm: Optional[float] = None) -> None:
        """Open the throat (extend the finger) regardless of the argument.
        The hook has no continuous aperture; it must be open before the
        final descent or the rim cannot enter the throat."""
        self.open()

    def grasp(self, force: Optional[float] = None) -> bool:
        """Latch. The ``force`` argument is ignored — DO-pin actuation has
        no load cell on the hook itself. Returns True unconditionally on
        DO-pin path (the UR controller acks the pin write; actual
        engagement must be verified via TCP wrench externally). When the
        serial path adds force feedback, this should return based on that.
        """
        self.close()
        if self._close_settle_s > 0.0:
            time.sleep(self._close_settle_s)
        return True

    def status(self) -> dict:
        return {
            "actuation": self._actuation,
            "extended": self._extended,
            "close_settle_s": self._close_settle_s,
        }

    def disconnect(self) -> None:
        """Close the RTDE IO connection opened for tool digital outputs."""
        if self._rtde_io is not None and hasattr(self._rtde_io, "disconnect"):
            self._rtde_io.disconnect()
