"""Real backend: executes ``Segment``s on physical UR5e's via RTDE.

Maps each Segment to the RTDE call pattern already used in
``ur_2026/object_grasp_example.py``:

  MoveJ            -> rtde_c.moveJ(q_target, speed, accel)
  MoveL            -> rtde_c.moveL(pose, speed, accel)     # pose in base frame
  ServoStream      -> tight loop: for each waypoint,
                      rtde_c.servoJ(q, 0, 0, dt, lookahead, gain);
                      hold the cadence with ``time.sleep`` between ticks.
  ForceMode        -> rtde_c.forceMode(task_frame, selection, wrench, type, limits)
                      in a loop; zero force on exit via rtde_c.forceModeStop().
  MoveUntilContact -> rtde_c.moveUntilContact(v_base, [accel], [direction])
  GripperCommand   -> delegate to the Gripper (socket for Robotiq,
                      TBD for hook).
  Wait             -> time.sleep(duration).

Real-time notes:
  - ``servoJ`` and ``forceMode`` both require the command to be refreshed
    every ~8 ms (125 Hz) or faster. Do NOT block between ticks.
  - Always call ``rtde_c.forceModeStop()`` before moving away, or the next
    ``moveJ`` / ``moveL`` will fight the lingering force commands.
  - Pose convention for RTDE is ``[x, y, z, rx, ry, rz]`` where ``(rx,
    ry, rz)`` is the axis-angle. ``MoveL.X_target`` is a ``RigidTransform``;
    convert via ``RigidTransform.rotation().ToAngleAxis()``.

This file is a stub until we are wired up on the hardware — production
implementation is blocked on RTDEControl/RTDEReceive imports being
available in the project env.
"""

from typing import Any

from ..grippers.base import Gripper
from ..segments import Segment


class RealBackend:
    """Two-arm RTDE backend. IPs match the class setup:
    left = 192.168.1.101, right = 192.168.1.102."""

    def __init__(
        self,
        left_ip: str = "192.168.1.101",
        right_ip: str = "192.168.1.102",
    ) -> None:
        self._left_ip = left_ip
        self._right_ip = right_ip
        # TODO:
        #   from rtde_control import RTDEControlInterface as RTDEControl
        #   from rtde_receive import RTDEReceiveInterface as RTDEReceive
        #   self._rtde_c = {"ur_left": RTDEControl(left_ip),
        #                   "ur_right": RTDEControl(right_ip)}
        #   self._rtde_r = {"ur_left": RTDEReceive(left_ip),
        #                   "ur_right": RTDEReceive(right_ip)}
        #   for c in self._rtde_c.values():
        #       c.setTcp([0, 0, 0.174, 0, 0, 0])  # Robotiq 2F-85 TCP
        #   self._grippers = {"ur_left": HookGripper(rtde_control=self._rtde_c["ur_left"]),
        #                     "ur_right": Robotiq2F85(rtde_control=self._rtde_c["ur_right"])}

    def execute(self, arm: str, segment: Segment) -> None:
        # TODO: isinstance dispatch mirroring sim_backend.
        raise NotImplementedError

    def state(self, arm: str) -> dict:
        # TODO: compose from RTDEReceive calls:
        #   q = rtde_r.getActualQ()
        #   v = rtde_r.getActualQd()
        #   X_WTcp = rtde_r.getActualTCPPose()
        #   wrench = rtde_r.getActualTCPForce()
        raise NotImplementedError

    def gripper(self, arm: str) -> Gripper:
        raise NotImplementedError

    def close(self) -> None:
        """Clean RTDE shutdown — call before exit."""
        # TODO: for c in self._rtde_c.values(): c.stopScript()
        raise NotImplementedError
