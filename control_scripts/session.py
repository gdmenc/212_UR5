"""Session: context-managed RTDE + gripper lifecycle for the bimanual rig.

Consolidates what ``ur_2026/object_grasp_example.py`` inlines (connect with
retry, setTcp, activate Robotiq, stopScript on teardown) into a single
context manager. Entry code becomes:

    with default_session(right=True) as session:
        pick(session.right, grasp, cfg)

and the connect / activate / clean shutdown sequence is guaranteed to
run regardless of what happens inside the ``with`` block — exceptions
in pick/place still release the controllers cleanly.

The module avoids importing ``rtde_control`` / ``rtde_receive`` at the
top level so the rest of the package can be imported on a machine
without the RTDE wheels (e.g. CI). The RTDE imports happen inside
``__enter__`` at actual connection time.
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import Any, Callable, Dict, List, Optional

import numpy as np

from .arm import ArmHandle
from .calibration import (
    HOME_Q_RAD_LEFT,
    HOME_Q_RAD_RIGHT,
    TCP_OFFSET_ROBOTIQ_2F85,
    X_LEFT_BASE_TASK,
    X_RIGHT_BASE_TASK,
)
from .grippers import Gripper, HookGripper, Robotiq2F85
from .util.poses import Pose


# --- Default network config for the 212 rig --------------------------------
DEFAULT_LEFT_IP = "192.168.1.101"
DEFAULT_RIGHT_IP = "192.168.1.102"
DEFAULT_CONNECT_TRIES = 3
DEFAULT_CONNECT_RETRY_DELAY = 0.1   # seconds between reconnect attempts


@dataclass
class ArmSpec:
    """Everything Session needs to stand up one arm."""

    name: str
    """Dict key the arm is stored under (``'ur_left'`` or ``'ur_right'``)."""

    ip: str
    """Robot controller IP. RTDE always uses port 30004 internally."""

    X_base_task: Pose
    """Per-arm calibration — pose of the shared task frame in this arm's base
    frame. See calibration.py."""

    tcp_offset: List[float]
    """6-vector [x, y, z, rx, ry, rz] for rtde_c.setTcp(). Must be set or
    ArmHandle.setup() raises."""

    gripper_factory: Optional[Callable[[Any], Gripper]] = None
    """``lambda rtde_c: Robotiq2F85(rtde_c)`` or equivalent. ``None`` means
    no gripper on this arm — ArmHandle.gripper stays None, pick/place fail
    with a clear message if called."""

    home_q_rad: Optional[np.ndarray] = None
    """Joint config to send at session.move_to_home(). ``None`` skips this
    arm during homing — safer than guessing."""


class Session:
    """Context manager that owns RTDE connections + grippers for N arms.

    Typical use is through the ``default_session`` factory below; direct
    construction is for custom rigs or unit tests with fake RTDE.

    Attributes (valid only inside the ``with`` block):
        arms   : dict[name, ArmHandle]
        left   : ArmHandle (KeyError if left arm not in session)
        right  : ArmHandle (KeyError if right arm not in session)
    """

    def __init__(
        self,
        specs: Dict[str, ArmSpec],
        connect_tries: int = DEFAULT_CONNECT_TRIES,
        connect_retry_delay: float = DEFAULT_CONNECT_RETRY_DELAY,
        *,
        rtde_control_cls: Optional[type] = None,
        rtde_receive_cls: Optional[type] = None,
    ) -> None:
        self._specs = specs
        self._connect_tries = connect_tries
        self._connect_retry_delay = connect_retry_delay
        self._arms: Dict[str, ArmHandle] = {}
        # Injectable for unit tests. None → resolve ur_rtde at __enter__ time.
        self._rtde_control_cls = rtde_control_cls
        self._rtde_receive_cls = rtde_receive_cls

    # --- public accessors -------------------------------------------------
    @property
    def arms(self) -> Dict[str, ArmHandle]:
        return self._arms

    @property
    def left(self) -> ArmHandle:
        return self._arms["ur_left"]

    @property
    def right(self) -> ArmHandle:
        return self._arms["ur_right"]

    # --- lifecycle --------------------------------------------------------
    def __enter__(self) -> "Session":
        ControlCls, ReceiveCls = self._resolve_rtde_classes()

        for name, spec in self._specs.items():
            rtde_c = ControlCls(spec.ip)
            rtde_r = ReceiveCls(spec.ip)

            self._ensure_connected(rtde_c, spec.ip)

            gripper: Optional[Gripper] = None
            if spec.gripper_factory is not None:
                gripper = spec.gripper_factory(rtde_c)
                gripper.activate()

            arm = ArmHandle(
                name=name,
                control=rtde_c,
                receive=rtde_r,
                gripper=gripper,
                X_base_task=spec.X_base_task,
                tcp_offset=spec.tcp_offset,
            )
            arm.setup()  # applies TCP offset via rtde_c.setTcp
            self._arms[name] = arm

        return self

    def __exit__(self, exc_type, exc_val, exc_tb) -> None:
        # Best-effort cleanup: try each step per-arm, swallow exceptions,
        # so one failed teardown doesn't skip the others.
        for name, arm in self._arms.items():
            if arm.gripper is not None:
                try:
                    arm.gripper.disconnect()
                except Exception as e:
                    print(f"[{name}] gripper disconnect failed: {e}")
            try:
                arm.control.stopScript()
            except Exception as e:
                print(f"[{name}] stopScript failed: {e}")
        self._arms.clear()

    # --- helpers ----------------------------------------------------------
    def move_to_home(self) -> None:
        """Move every arm whose ArmSpec has ``home_q_rad`` set to its home
        joint configuration. Blocking — returns once all arms have reached
        their targets. Safe to call multiple times."""
        for name, arm in self._arms.items():
            home = self._specs[name].home_q_rad
            if home is None:
                print(f"[{name}] no home configured — skipping")
                continue
            arm.control.moveJ(list(home))

    # --- internals --------------------------------------------------------
    def _resolve_rtde_classes(self) -> tuple:
        if self._rtde_control_cls is not None and self._rtde_receive_cls is not None:
            return self._rtde_control_cls, self._rtde_receive_cls
        # Import here so top-level import of this module doesn't require
        # ur_rtde to be installed (useful for CI / doc builds).
        from rtde_control import RTDEControlInterface
        from rtde_receive import RTDEReceiveInterface
        return RTDEControlInterface, RTDEReceiveInterface

    def _ensure_connected(self, rtde_c: Any, ip: str) -> None:
        """Retry reconnect up to ``connect_tries`` times. Mirrors the
        connection retry loop in ur_2026/object_grasp_example.py so
        transient power-cycle / network glitches don't fail the session."""
        if rtde_c.isConnected():
            return
        for attempt in range(self._connect_tries):
            rtde_c.reconnect()
            time.sleep(self._connect_retry_delay)
            if rtde_c.isConnected():
                return
        raise RuntimeError(
            f"Failed to connect to arm at {ip} after "
            f"{self._connect_tries} reconnect attempts."
        )


# =========================================================================
#  Standard rig factory
# =========================================================================

def default_session(
    *,
    left: bool = True,
    right: bool = True,
    left_ip: str = DEFAULT_LEFT_IP,
    right_ip: str = DEFAULT_RIGHT_IP,
    connect_tries: int = DEFAULT_CONNECT_TRIES,
) -> Session:
    """Build a Session for the 212 rig's bimanual UR5e setup.

    Skip arms you don't need — e.g. ``default_session(left=False)`` for
    pick-and-place tests with only the two-finger arm, avoiding an
    unnecessary connection to the unused hook arm.

    Gripper drivers are wired from the team's known rig state:
      ur_left  → HookGripper (TCP offset TODO, will raise on setup)
      ur_right → Robotiq 2F-85
    """
    specs: Dict[str, ArmSpec] = {}
    if left:
        specs["ur_left"] = ArmSpec(
            name="ur_left",
            ip=left_ip,
            X_base_task=X_LEFT_BASE_TASK,
            tcp_offset=TCP_OFFSET_ROBOTIQ_2F85,  # TODO: replace with TCP_OFFSET_HOOK once measured
            gripper_factory=lambda rtde_c: HookGripper(rtde_c),
            home_q_rad=HOME_Q_RAD_LEFT,
        )
    if right:
        specs["ur_right"] = ArmSpec(
            name="ur_right",
            ip=right_ip,
            X_base_task=X_RIGHT_BASE_TASK,
            tcp_offset=TCP_OFFSET_ROBOTIQ_2F85,
            gripper_factory=lambda rtde_c: Robotiq2F85(rtde_c),
            home_q_rad=HOME_Q_RAD_RIGHT,
        )
    return Session(specs, connect_tries=connect_tries)
