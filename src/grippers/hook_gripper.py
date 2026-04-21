"""Custom hook gripper (team-built end-effector, welded to ur_left).

An active, single-DOF hook: the hook extends to engage a handle/edge and
retracts to disengage. Used for tasks where a two-finger grasp is awkward
(microwave handle, tray edge). Not a general-purpose grasp — semantics are
"latch" and "unlatch," not "pinch at N Newtons."

Sim: loads ``src/assets/hook_gripper/hook_gripper.sdf``. In the current
scenario file it is already welded to ``ur_left::wrist_3_link`` with
``X_PC = translation [0, 0, 0.1], rpy [0, 90, 0] deg``.

Real: actuation hardware is TBD. Best guesses at the interface:
  - a hobby servo over USB-serial from the control PC, or
  - a relay wired to the UR controller's digital output so we can toggle
    via ``rtde_c.setStandardDigitalOut(pin, state)``.
Either path plugs in under ``close`` / ``open`` without changing callers.
"""

from pathlib import Path
from typing import Optional

from .base import Gripper


_REPO_ROOT = Path(__file__).resolve().parents[2]
HOOK_GRIPPER_SDF = str(
    _REPO_ROOT / "src" / "assets" / "hook_gripper" / "hook_gripper.sdf"
)


class HookGripper(Gripper):
    """Active hook end-effector. Constructor mirrors ``Robotiq2F85``: pass
    either a real driver handle or the sim plant, never both."""

    def __init__(
        self,
        rtde_control=None,
        sim_plant=None,
        sim_arm_name: Optional[str] = None,
    ) -> None:
        if (rtde_control is None) == (sim_plant is None):
            raise ValueError(
                "Provide exactly one of rtde_control or sim_plant."
            )
        self._rtde = rtde_control
        self._plant = sim_plant
        self._arm = sim_arm_name

    def open(self) -> None:
        """Retract the hook (un-latch)."""
        raise NotImplementedError("TODO: retract hook joint / DO pin low")

    def close(self) -> None:
        """Extend the hook (latch)."""
        raise NotImplementedError("TODO: extend hook joint / DO pin high")

    def grasp(self, force: Optional[float] = None) -> bool:
        """Latch and confirm contact.

        Force arg is ignored on the real hook (no load cell on the hook
        itself — infer latch success from TCP wrench after close). In sim,
        inspect contact results between hook tip and the target body.
        """
        raise NotImplementedError

    def status(self) -> dict:
        raise NotImplementedError
