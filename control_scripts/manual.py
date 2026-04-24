"""Manual teleoperation for a selected arm.

Primary path uses a pygame joystick. The teleop loop prints live joint/TCP
state and logs commands + robot state to CSV so you can replay what happened
after a manual probing session.
"""

from __future__ import annotations

import csv
import datetime as dt
import os
import time
from dataclasses import dataclass
from typing import Optional, Sequence

import numpy as np

from .arm import ArmHandle
from .runtime import current_task_pose, format_vec, print_arm_state
from .trials.definitions import TRIALS


@dataclass(frozen=True)
class ManualTeleopOptions:
    arm_name: str
    safety_trial_name: Optional[str] = None
    input_source: str = "auto"
    joystick_index: int = 0
    linear_speed: float = 0.08
    angular_speed: float = 0.50
    accel: float = 0.75
    loop_dt: float = 0.05
    state_period: float = 0.5
    log_file: Optional[str] = None


@dataclass
class ManualCommand:
    v_task: np.ndarray
    quit_requested: bool = False
    print_state_requested: bool = False
    open_gripper_requested: bool = False
    close_gripper_requested: bool = False


@dataclass(frozen=True)
class PygameJoystickMapping:
    axis_task_x: int = 0
    axis_task_y: int = 1
    axis_task_z: int = 4
    axis_task_rz: int = 3
    button_task_rx_neg: int = 4
    button_task_rx_pos: int = 5
    hat_task_ry_axis: int = 1
    button_open_gripper: int = 0
    button_close_gripper: int = 1
    button_print_state: int = 7
    button_quit: int = 6
    deadband: float = 0.12


class ManualCSVLogger:
    def __init__(self, path: str, arm_name: str) -> None:
        directory = os.path.dirname(path)
        if directory:
            os.makedirs(directory, exist_ok=True)
        self._fh = open(path, "w", newline="", encoding="ascii")
        self._writer = csv.writer(self._fh)
        self._writer.writerow([
            "timestamp_iso",
            "arm",
            "q0_deg",
            "q1_deg",
            "q2_deg",
            "q3_deg",
            "q4_deg",
            "q5_deg",
            "task_x",
            "task_y",
            "task_z",
            "task_rx",
            "task_ry",
            "task_rz",
            "cmd_vx_task",
            "cmd_vy_task",
            "cmd_vz_task",
            "cmd_wx_task",
            "cmd_wy_task",
            "cmd_wz_task",
            "cmd_vx_base",
            "cmd_vy_base",
            "cmd_vz_base",
            "cmd_wx_base",
            "cmd_wy_base",
            "cmd_wz_base",
            "event",
        ])
        self._arm_name = arm_name

    def log(self, arm: ArmHandle, v_task, v_base, event: str = "") -> None:
        q_deg = np.degrees(np.asarray(arm.receive.getActualQ(), dtype=float))
        task_pose = current_task_pose(arm)
        row = [
            dt.datetime.now().isoformat(timespec="milliseconds"),
            self._arm_name,
            *[f"{x:.6f}" for x in q_deg.tolist()],
            *[f"{x:.6f}" for x in task_pose.translation.tolist()],
            *[f"{x:.6f}" for x in task_pose.rotation.as_rotvec().tolist()],
            *[f"{x:.6f}" for x in np.asarray(v_task, dtype=float).tolist()],
            *[f"{x:.6f}" for x in np.asarray(v_base, dtype=float).tolist()],
            event,
        ]
        self._writer.writerow(row)
        self._fh.flush()

    def close(self) -> None:
        self._fh.close()


def _default_log_file(arm_name: str) -> str:
    stamp = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
    return os.path.join("logs", "manual", f"{arm_name}_{stamp}.csv")


class PygameJoystickInput:
    def __init__(self, joystick_index: int, mapping: PygameJoystickMapping) -> None:
        import pygame

        self._pygame = pygame
        self._mapping = mapping
        self._pygame.init()
        self._pygame.joystick.init()
        count = self._pygame.joystick.get_count()
        if count == 0:
            raise RuntimeError("pygame is installed, but no joystick was detected")
        if joystick_index >= count:
            raise RuntimeError(
                f"joystick index {joystick_index} is unavailable; detected {count} device(s)"
            )
        self._joy = self._pygame.joystick.Joystick(joystick_index)
        self._joy.init()
        self._prev_buttons: dict[int, bool] = {}

    def _button_edge(self, index: int) -> bool:
        pressed = bool(self._joy.get_button(index))
        prev = self._prev_buttons.get(index, False)
        self._prev_buttons[index] = pressed
        return pressed and not prev

    def _axis(self, index: int) -> float:
        value = float(self._joy.get_axis(index))
        if abs(value) < self._mapping.deadband:
            return 0.0
        return value

    def poll(self) -> ManualCommand:
        self._pygame.event.pump()
        hat = self._joy.get_hat(0) if self._joy.get_numhats() else (0, 0)
        v_task = np.array([
            self._axis(self._mapping.axis_task_x),
            -self._axis(self._mapping.axis_task_y),
            -self._axis(self._mapping.axis_task_z),
            float(self._joy.get_button(self._mapping.button_task_rx_pos))
            - float(self._joy.get_button(self._mapping.button_task_rx_neg)),
            float(hat[self._mapping.hat_task_ry_axis]),
            -self._axis(self._mapping.axis_task_rz),
        ], dtype=float)
        return ManualCommand(
            v_task=v_task,
            quit_requested=self._button_edge(self._mapping.button_quit),
            print_state_requested=self._button_edge(self._mapping.button_print_state),
            open_gripper_requested=self._button_edge(self._mapping.button_open_gripper),
            close_gripper_requested=self._button_edge(self._mapping.button_close_gripper),
        )

    def close(self) -> None:
        try:
            self._joy.quit()
        finally:
            self._pygame.joystick.quit()
            self._pygame.quit()


class TerminalVelocityInput:
    """Fallback input: keys latch axis signs until changed.

    Controls:
    - `a` / `d`: x - / +
    - `s` / `w`: y - / +
    - `f` / `r`: z - / +
    - `j` / `l`: rx - / +
    - `k` / `i`: ry - / +
    - `u` / `o`: rz - / +
    - `space`: zero all velocities
    - `p`: print state
    - `g`: open gripper
    - `h`: close gripper
    - `x`: quit
    """

    def __init__(self) -> None:
        import select
        import sys
        import termios
        import tty

        if not sys.stdin.isatty():
            raise RuntimeError("terminal input fallback requires an interactive TTY")
        self._select = select
        self._sys = sys
        self._termios = termios
        self._tty = tty
        self._fd = sys.stdin.fileno()
        self._old_settings = termios.tcgetattr(self._fd)
        tty.setcbreak(self._fd)
        self._state = np.zeros(6, dtype=float)

    def _drain_chars(self) -> list[str]:
        chars: list[str] = []
        while self._select.select([self._sys.stdin], [], [], 0.0)[0]:
            chars.append(self._sys.stdin.read(1))
        return chars

    def poll(self) -> ManualCommand:
        command = ManualCommand(v_task=self._state.copy())
        for ch in self._drain_chars():
            if ch == "a":
                self._state[0] = -1.0
            elif ch == "d":
                self._state[0] = 1.0
            elif ch == "s":
                self._state[1] = -1.0
            elif ch == "w":
                self._state[1] = 1.0
            elif ch == "f":
                self._state[2] = -1.0
            elif ch == "r":
                self._state[2] = 1.0
            elif ch == "j":
                self._state[3] = -1.0
            elif ch == "l":
                self._state[3] = 1.0
            elif ch == "k":
                self._state[4] = -1.0
            elif ch == "i":
                self._state[4] = 1.0
            elif ch == "u":
                self._state[5] = -1.0
            elif ch == "o":
                self._state[5] = 1.0
            elif ch == " ":
                self._state[:] = 0.0
            elif ch == "p":
                command.print_state_requested = True
            elif ch == "g":
                command.open_gripper_requested = True
            elif ch == "h":
                command.close_gripper_requested = True
            elif ch == "x":
                command.quit_requested = True
            command.v_task = self._state.copy()
        return command

    def close(self) -> None:
        self._termios.tcsetattr(self._fd, self._termios.TCSADRAIN, self._old_settings)


def _build_input_device(options: ManualTeleopOptions):
    source = options.input_source.lower()
    if source not in {"auto", "pygame", "terminal"}:
        raise ValueError("input source must be one of: auto, pygame, terminal")

    if source in {"auto", "pygame"}:
        try:
            return (
                PygameJoystickInput(options.joystick_index, PygameJoystickMapping()),
                "pygame",
            )
        except Exception:
            if source == "pygame":
                raise

    return TerminalVelocityInput(), "terminal"


def _print_controls(resolved_source: str) -> None:
    if resolved_source == "pygame":
        print("Manual controls (pygame default mapping):")
        print("  left stick     : task x / y")
        print("  right stick Y  : task z")
        print("  right stick X  : task yaw (rz)")
        print("  LB / RB        : task roll (rx) - / +")
        print("  D-pad up/down  : task pitch (ry) + / -")
        print("  A / B          : open / close gripper")
        print("  Start          : print state")
        print("  Back           : quit")
        return

    print("Manual controls (terminal fallback):")
    print("  a/d x-,x+  s/w y-,y+  f/r z-,z+")
    print("  j/l rx-,rx+  k/i ry-,ry+  u/o rz-,rz+")
    print("  space zero all  p print state  g/h open/close gripper  x quit")


def _trial_violations(trial, xyz: Sequence[float]) -> list[str]:
    violations: list[str] = []
    if trial.workspace_limits is not None and not trial.workspace_limits.contains(xyz):
        violations.append(f"outside workspace {trial.workspace_limits.name}")
    for box in trial.keepout_boxes:
        if box.contains(xyz):
            violations.append(f"inside keep-out box {box.name}")
    return violations


def _apply_manual_safety(
    arm: ArmHandle,
    v_task: np.ndarray,
    loop_dt: float,
    safety_trial_name: Optional[str],
) -> tuple[np.ndarray, Optional[str]]:
    if safety_trial_name is None:
        return v_task, None

    trial = TRIALS[safety_trial_name]
    if trial.arm != arm.name:
        raise RuntimeError(
            f"safety trial {trial.name!r} is defined for {trial.arm}, not {arm.name}"
        )

    current_pose = current_task_pose(arm)
    predicted_xyz = current_pose.translation + np.asarray(v_task[:3], dtype=float) * loop_dt
    violations = _trial_violations(trial, predicted_xyz)
    if not violations:
        return v_task, None

    clipped = np.asarray(v_task, dtype=float).copy()
    clipped[:3] = 0.0
    return clipped, "; ".join(violations)


def run_manual_teleop(arm: ArmHandle, options: ManualTeleopOptions) -> None:
    if arm.name != options.arm_name:
        raise RuntimeError(f"connected arm is {arm.name}, but manual target is {options.arm_name}")

    device, resolved_source = _build_input_device(options)
    log_path = options.log_file or _default_log_file(arm.name)
    logger = ManualCSVLogger(log_path, arm.name)
    print(f"Logging manual teleop to {log_path}")
    _print_controls(resolved_source)
    print_arm_state(arm, "initial state")

    last_state_print = 0.0
    last_safety_note: Optional[str] = None

    try:
        while True:
            loop_start = time.monotonic()
            command = device.poll()

            if command.print_state_requested:
                print_arm_state(arm, "manual snapshot")

            if command.open_gripper_requested:
                if arm.gripper is None:
                    print("No gripper attached to this arm.")
                else:
                    arm.gripper.open()
            if command.close_gripper_requested:
                if arm.gripper is None:
                    print("No gripper attached to this arm.")
                else:
                    arm.gripper.close()

            v_task = np.asarray(command.v_task, dtype=float).copy()
            v_task[:3] *= options.linear_speed
            v_task[3:] *= options.angular_speed
            v_task, safety_note = _apply_manual_safety(
                arm,
                v_task,
                options.loop_dt,
                options.safety_trial_name,
            )
            if safety_note and safety_note != last_safety_note:
                print(f"Safety stop: {safety_note}")
                last_safety_note = safety_note
            if safety_note is None:
                last_safety_note = None

            v_base = np.asarray(arm.task_velocity_to_base(v_task), dtype=float)
            arm.control.speedL(v_base.tolist(), options.accel, 0)
            logger.log(arm, v_task, v_base, event=("safety_stop" if safety_note else ""))

            now = time.monotonic()
            if now - last_state_print >= options.state_period:
                q_deg = np.degrees(np.asarray(arm.receive.getActualQ(), dtype=float))
                pose_task = current_task_pose(arm)
                print(
                    f"q_deg={format_vec(q_deg, 1)} "
                    f"task_xyz={format_vec(pose_task.translation, 3)} "
                    f"cmd_task={format_vec(v_task, 3)}"
                )
                last_state_print = now

            if command.quit_requested:
                break

            elapsed = time.monotonic() - loop_start
            if elapsed < options.loop_dt:
                time.sleep(options.loop_dt - elapsed)
    finally:
        try:
            arm.control.speedL([0.0] * 6, options.accel, 0)
        except Exception:
            pass
        if hasattr(arm.control, "speedStop"):
            try:
                arm.control.speedStop(options.accel)
            except Exception:
                pass
        logger.close()
        device.close()
