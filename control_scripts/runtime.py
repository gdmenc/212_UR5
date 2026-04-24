"""Shared live-runtime helpers for the real UR rig.

This file is the single place to define which arms exist on the rig, their
IP addresses, calibration, TCP offsets, and attached grippers.

Every live mode (`state`, `trial`, `manual`) connects through this module so
`run.py` can decide how many arms to connect and which of them are allowed to
move.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable, Optional, Sequence

import numpy as np

from .arm import ArmHandle
from .calibration import (
    TCP_OFFSET_HOOK,
    TCP_OFFSET_ROBOTIQ_2F85,
    X_LEFT_BASE_TASK,
    X_RIGHT_BASE_TASK,
)
from .grippers.hook_gripper import DEFAULT_DO_PIN, HookGripper
from .grippers.robotiq_2f85 import Robotiq2F85
from .util.poses import Pose
from .util.rtde_convert import rtde_to_pose


@dataclass(frozen=True)
class ArmRuntimeDefinition:
    name: str
    ip: str
    x_base_task: Pose
    tcp_offset: Optional[Sequence[float]]
    gripper_kind: Optional[str] = None
    description: str = ""
    hook_do_pin: int = DEFAULT_DO_PIN

    @property
    def motion_ready(self) -> bool:
        return self.tcp_offset is not None


ARM_DEFINITIONS = {
    "ur_left": ArmRuntimeDefinition(
        name="ur_left",
        ip="192.168.1.101",
        x_base_task=X_LEFT_BASE_TASK,
        tcp_offset=TCP_OFFSET_HOOK,
        gripper_kind="hook",
        description="Left arm with hook end effector.",
    ),
    "ur_right": ArmRuntimeDefinition(
        name="ur_right",
        ip="192.168.1.102",
        x_base_task=X_RIGHT_BASE_TASK,
        tcp_offset=TCP_OFFSET_ROBOTIQ_2F85,
        gripper_kind="robotiq_2f85",
        description="Right arm with Robotiq 2F-85.",
    ),
}

DEFAULT_CONNECTED_ARMS = ("ur_right",)


def format_vec(vec: Sequence[float], digits: int = 4) -> str:
    values = [float(v) for v in vec]
    return "[" + ", ".join(f"{v:+.{digits}f}" for v in values) + "]"


def print_pose(prefix: str, pose: Pose) -> None:
    print(f"{prefix} xyz   : {format_vec(pose.translation)}")
    print(f"{prefix} rotvec: {format_vec(pose.rotation.as_rotvec())}")


def list_arm_definitions() -> None:
    for name, arm in ARM_DEFINITIONS.items():
        tcp_status = "ready" if arm.motion_ready else "missing"
        print(
            f"{name}: ip={arm.ip}, gripper={arm.gripper_kind or 'none'}, "
            f"tcp={tcp_status}, {arm.description}"
        )


def parse_arm_names(csv_names: Optional[str]) -> list[str]:
    if not csv_names:
        return list(DEFAULT_CONNECTED_ARMS)

    names = [name.strip() for name in csv_names.split(",") if name.strip()]
    if not names:
        raise ValueError("no arm names were provided")

    seen: set[str] = set()
    ordered: list[str] = []
    for name in names:
        if name not in ARM_DEFINITIONS:
            raise ValueError(
                f"unknown arm {name!r}; choose from {sorted(ARM_DEFINITIONS)}"
            )
        if name not in seen:
            ordered.append(name)
            seen.add(name)
    return ordered


def current_base_pose(arm: ArmHandle) -> Pose:
    return rtde_to_pose(arm.receive.getActualTCPPose())


def current_task_pose(arm: ArmHandle) -> Pose:
    return arm.to_task(current_base_pose(arm))


def print_arm_state(arm: ArmHandle, label: Optional[str] = None) -> None:
    title = label or arm.name
    print(f"\n== {title} ==")
    q_actual = np.asarray(arm.receive.getActualQ(), dtype=float)
    print(f"joints_deg : {format_vec(np.degrees(q_actual), 2)}")
    base_pose = current_base_pose(arm)
    task_pose = arm.to_task(base_pose)
    print_pose("base", base_pose)
    print_pose("task", task_pose)


def _build_gripper(defn: ArmRuntimeDefinition, control):
    if defn.gripper_kind == "robotiq_2f85":
        return Robotiq2F85(control)
    if defn.gripper_kind == "hook":
        return HookGripper(control, do_pin=defn.hook_do_pin)
    return None


def connect_arms(
    arm_names: Sequence[str],
    motion_arms: Optional[Iterable[str]] = None,
    gripper_arms: Optional[Iterable[str]] = None,
) -> dict[str, ArmHandle]:
    from rtde_control import RTDEControlInterface as RTDEControlInterface
    from rtde_receive import RTDEReceiveInterface as RTDEReceiveInterface

    requested = list(arm_names)
    moving = set(motion_arms or ())
    with_grippers = set(gripper_arms or ())

    arms: dict[str, ArmHandle] = {}
    try:
        for name in requested:
            if name not in ARM_DEFINITIONS:
                raise ValueError(
                    f"unknown arm {name!r}; choose from {sorted(ARM_DEFINITIONS)}"
                )

            defn = ARM_DEFINITIONS[name]
            control = RTDEControlInterface(defn.ip)
            receive = RTDEReceiveInterface(defn.ip)

            if hasattr(control, "isConnected") and not control.isConnected():
                attempts = 0
                while attempts < 3 and not control.isConnected():
                    control.reconnect()
                    attempts += 1
                if not control.isConnected():
                    raise RuntimeError(f"failed to connect RTDE control to {defn.ip}")

            gripper = _build_gripper(defn, control) if name in with_grippers else None
            arm = ArmHandle(
                name=name,
                control=control,
                receive=receive,
                gripper=gripper,
                X_base_task=defn.x_base_task,
                tcp_offset=defn.tcp_offset,
            )

            if name in moving:
                if defn.tcp_offset is None:
                    raise RuntimeError(
                        f"arm {name!r} is not motion-ready: TCP offset is unset in "
                        "control_scripts/calibration.py"
                    )
                arm.setup()

            if gripper is not None:
                gripper.activate()

            arms[name] = arm

        return arms
    except Exception:
        close_arms(arms)
        raise


def close_arms(arms: dict[str, ArmHandle]) -> None:
    for arm in arms.values():
        try:
            if arm.gripper is not None:
                arm.gripper.disconnect()
        finally:
            try:
                arm.control.stopScript()
            except Exception:
                pass
