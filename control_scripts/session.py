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

import atexit
import signal
import time
from dataclasses import dataclass
from typing import Any, Callable, Dict, List, Optional, Tuple

import numpy as np

from .arm import ArmHandle
from .calibration import (
    HOME_Q_RAD_LEFT,
    HOME_Q_RAD_RIGHT,
    TCP_OFFSET_ROBOTIQ_2F85,
    TCP_OFFSET_HOOK,
    X_LEFT_BASE_TASK,
    X_RIGHT_BASE_TASK,
)
from .grippers import Gripper, HookGripper, Robotiq2F85
from .util.poses import Pose


# --- Default network config for the 212 rig --------------------------------
DEFAULT_LEFT_IP = "192.168.2.103"
# DEFAULT_LEFT_IP = "192.168.1.101"
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

    gripper_factory: Optional[Callable[[Any, Any, Optional[Any]], Gripper]] = None
    """``lambda rtde_c, rtde_r, rtde_io=None: Robotiq2F85(rtde_c)`` or
    ``HookGripper(rtde_io)``. Third argument is ``RTDEIOInterface`` when
    ``needs_rtde_io`` is True, else ``None``. ``None`` means no gripper."""

    needs_rtde_io: bool = False
    """If True, Session opens ``rtde_io.RTDEIOInterface`` for this arm's IP
    (required for tool digital outputs / ``HookGripper``)."""

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
        rtde_io_cls: Optional[type] = None,
    ) -> None:
        self._specs = specs
        self._connect_tries = connect_tries
        self._connect_retry_delay = connect_retry_delay
        self._arms: Dict[str, ArmHandle] = {}
        # Injectable for unit tests. None → resolve ur_rtde at __enter__ time.
        self._rtde_control_cls = rtde_control_cls
        self._rtde_receive_cls = rtde_receive_cls
        self._rtde_io_cls = rtde_io_cls
        self._closed = False
        self._previous_signal_handlers: Dict[int, Any] = {}

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
        self._install_shutdown_hooks()
        needs_io = any(spec.needs_rtde_io for spec in self._specs.values())
        ControlCls, ReceiveCls, IOCls = self._resolve_rtde_classes(needs_io)

        try:
            for name, spec in self._specs.items():
                rtde_c = ControlCls(spec.ip)
                rtde_r = ReceiveCls(spec.ip)

                self._ensure_connected(rtde_c, spec.ip)

                rtde_io = None
                if spec.needs_rtde_io:
                    if IOCls is None:
                        raise RuntimeError(
                            "RTDEIOInterface was requested but not resolved."
                        )
                    rtde_io = IOCls(spec.ip)
                    self._ensure_connected(rtde_io, spec.ip)

                gripper: Optional[Gripper] = None
                if spec.gripper_factory is not None:
                    gripper = spec.gripper_factory(rtde_c, rtde_r, rtde_io)
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
        except BaseException:
            self.close(reason="session setup failed")
            raise

        return self

    def __exit__(self, exc_type, exc_val, exc_tb) -> None:
        self.close(reason="session exit")

    def close(self, *, reason: str = "manual close") -> None:
        """Best-effort emergency teardown for all connected arms.

        This is intentionally idempotent so it can be called from normal
        ``with`` teardown, failed setup, atexit, or a signal handler.  It does
        not open/release grippers; it only stops controller-side motion/script
        state and closes RTDE resources so the Python process gives control
        back cleanly.
        """
        if self._closed:
            return
        self._closed = True

        if self._arms:
            print(f"[session] cleanup: {reason}")

        for name, arm in list(self._arms.items()):
            self._stop_arm_control(name, arm)
            if arm.gripper is not None:
                self._try_call(name, "gripper.disconnect", arm.gripper.disconnect)
            self._disconnect_interface(name, "receive", arm.receive)
            self._disconnect_interface(name, "control", arm.control)

        self._arms.clear()
        self._restore_shutdown_hooks()

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
    def _install_shutdown_hooks(self) -> None:
        atexit.register(self.close)
        for signum in (signal.SIGINT, signal.SIGTERM):
            self._previous_signal_handlers[signum] = signal.getsignal(signum)
            signal.signal(signum, self._signal_handler)

    def _restore_shutdown_hooks(self) -> None:
        try:
            atexit.unregister(self.close)
        except Exception:
            pass
        for signum, handler in self._previous_signal_handlers.items():
            try:
                signal.signal(signum, handler)
            except Exception:
                pass
        self._previous_signal_handlers.clear()

    def _signal_handler(self, signum: int, _frame: Any) -> None:
        signame = signal.Signals(signum).name
        self.close(reason=f"received {signame}")
        if signum == signal.SIGINT:
            raise KeyboardInterrupt
        raise SystemExit(128 + signum)

    def _stop_arm_control(self, name: str, arm: ArmHandle) -> None:
        control = arm.control
        # Stop the controller modes that can outlive the Python call path.
        for label, args_variants in (
            ("forceModeStop", ((),)),
            ("servoStop", ((),)),
            ("speedStop", ((0.5,), ())),
            ("stopL", ((0.5,), ())),
            ("stopJ", ((0.5,), ())),
            ("stopScript", ((),)),
        ):
            fn = getattr(control, label, None)
            if fn is None:
                continue
            for args in args_variants:
                try:
                    fn(*args)
                    break
                except TypeError:
                    continue
                except Exception as exc:
                    print(f"[{name}] {label} failed: {exc}")
                    break

    def _disconnect_interface(self, name: str, label: str, interface: Any) -> None:
        disconnect = getattr(interface, "disconnect", None)
        if disconnect is not None:
            self._try_call(name, f"{label}.disconnect", disconnect)

    @staticmethod
    def _try_call(name: str, label: str, fn: Callable[[], Any]) -> None:
        try:
            fn()
        except Exception as exc:
            print(f"[{name}] {label} failed: {exc}")

    def _resolve_rtde_classes(self, needs_io: bool) -> Tuple[type, type, Optional[type]]:
        if self._rtde_control_cls is not None and self._rtde_receive_cls is not None:
            ctl, rcv = self._rtde_control_cls, self._rtde_receive_cls
        else:
            # Import here so top-level import of this module doesn't require
            # ur_rtde to be installed (useful for CI / doc builds).
            from rtde_control import RTDEControlInterface
            from rtde_receive import RTDEReceiveInterface
            ctl, rcv = RTDEControlInterface, RTDEReceiveInterface
        io_cls = self._rtde_io_cls
        if needs_io and io_cls is None:
            from rtde_io import RTDEIOInterface
            io_cls = RTDEIOInterface
        return ctl, rcv, io_cls

    def _ensure_connected(self, interface: Any, ip: str) -> None:
        """Retry reconnect for RTDE interfaces that expose connection status.

        ``RTDEIOInterface`` in some ur_rtde versions has ``disconnect()`` but
        no ``isConnected()`` / ``reconnect()`` API. Its constructor either
        succeeds or raises, so there is nothing useful to probe here.
        """
        if not hasattr(interface, "isConnected"):
            return
        if interface.isConnected():
            return
        for attempt in range(self._connect_tries):
            if not hasattr(interface, "reconnect"):
                break
            interface.reconnect()
            time.sleep(self._connect_retry_delay)
            if interface.isConnected():
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
      ur_left  → Hook Gripper (tool DO via the existing RTDE control channel)
      ur_right → Robotiq 2F-85
    We avoid opening a separate ``RTDEIOInterface`` for the hook because some
    UR controllers reserve those RTDE input registers for fieldbus adapters.
    """
    specs: Dict[str, ArmSpec] = {}
    if left:
        specs["ur_left"] = ArmSpec(
            name="ur_left",
            ip=left_ip,
            X_base_task=X_LEFT_BASE_TASK,
            tcp_offset=TCP_OFFSET_HOOK,
            gripper_factory=lambda rtde_c, rtde_r, rtde_io=None: HookGripper(
                rtde_c,
            ),
            home_q_rad=HOME_Q_RAD_LEFT,
        )
    if right:
        specs["ur_right"] = ArmSpec(
            name="ur_right",
            ip=right_ip,
            X_base_task=X_RIGHT_BASE_TASK,
            tcp_offset=TCP_OFFSET_ROBOTIQ_2F85,
            gripper_factory=lambda rtde_c, rtde_r, rtde_io=None: Robotiq2F85(
                rtde_c,
            ),
            home_q_rad=HOME_Q_RAD_RIGHT,
        )
    return Session(specs, connect_tries=connect_tries)
