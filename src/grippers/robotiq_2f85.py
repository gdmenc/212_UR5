"""Robotiq 2F-85 two-finger adaptive gripper.

Real side: driven over the UR controller via a TCP socket on port 63352 —
same protocol used by ``ur_2026/robotiq_gripper_control.py``. The UR
teach-pendant URCap exposes that port once the gripper is connected; we
send short ASCII commands (``SET POS``, ``GET STA``, etc.) and parse
responses.

Sim side: the current scenario YAML welds a ``wsg_right::body`` (WSG-50)
to the right wrist, not a Robotiq. Eventually the scene should swap that
for a Robotiq SDF; until then this class's sim methods are stubbed and
only the real side is expected to do real work.

TCP offset on real: 0.174 m along the flange z-axis (see
``ur_2026/object_grasp_example.py``). Callers typically set this once via
``rtde_c.setTcp([0, 0, 0.174, 0, 0, 0])``; it does not need to live here.
"""

from typing import Optional

from .base import Gripper


class Robotiq2F85(Gripper):
    """Robotiq 2F-85. Pass exactly one of ``rtde_control`` (real) or
    ``sim_plant``+``sim_arm_name`` (sim). Mixing is a configuration error."""

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
        if self._rtde is not None:
            raise NotImplementedError("TODO: socket write 'SET POS 0' + ack")
        raise NotImplementedError("TODO: drive sim finger_joint to open limit")

    def close(self) -> None:
        if self._rtde is not None:
            raise NotImplementedError("TODO: socket write 'SET POS 255' + ack")
        raise NotImplementedError("TODO: drive sim finger_joint to close limit")

    def grasp(self, force: Optional[float] = None) -> bool:
        # On real: set_force(force) then close(); parse object-detected flag
        # from the status word. On sim: close against the object and check
        # contact result between finger geometry and target.
        raise NotImplementedError

    def status(self) -> dict:
        raise NotImplementedError
