"""Runner for hard-coded autonomous trials."""

from __future__ import annotations

import time
from typing import Optional, Sequence

import numpy as np

from ..arm import ArmHandle
from ..runtime import (
    close_arms,
    connect_arms,
    current_task_pose,
    format_vec,
    print_arm_state,
    print_pose,
)
from ..util.rtde_convert import pose_to_rtde
from .definitions import DEFAULT_TRIAL, TRIALS
from .models import TrialDefinition


def _resolve_waypoint_pose(
    trial: TrialDefinition,
    waypoint_name: str,
    rotation_hint: Optional[Sequence[float]] = None,
):
    waypoint = trial.waypoints[waypoint_name]
    fallback = rotation_hint
    if fallback is None:
        fallback = trial.default_tool_rotvec_task
    return waypoint.to_pose(fallback_rotvec=fallback)


def point_violations(trial: TrialDefinition, xyz: Sequence[float]) -> list[str]:
    violations: list[str] = []
    if trial.workspace_limits is not None and not trial.workspace_limits.contains(xyz):
        violations.append(f"outside workspace {trial.workspace_limits.name}")
    for box in trial.keepout_boxes:
        if box.contains(xyz):
            violations.append(f"inside keep-out box {box.name}")
    return violations


def segment_violations(
    trial: TrialDefinition,
    start_xyz: Sequence[float],
    end_xyz: Sequence[float],
) -> list[str]:
    start = np.asarray(start_xyz, dtype=float).reshape(3)
    end = np.asarray(end_xyz, dtype=float).reshape(3)
    for alpha in np.linspace(0.0, 1.0, trial.samples_per_segment + 1):
        probe = (1.0 - alpha) * start + alpha * end
        violations = point_violations(trial, probe)
        if violations:
            joined = "; ".join(violations)
            return [
                f"segment {format_vec(start, 3)} -> {format_vec(end, 3)} violates: {joined} "
                f"at alpha={alpha:.2f}, xyz={format_vec(probe, 3)}"
            ]
    return []


def print_trial_summary(trial: TrialDefinition) -> None:
    print(f"Trial: {trial.name}")
    print(f"Arm  : {trial.arm}")
    print(f"About: {trial.description}")
    if trial.workspace_limits is not None:
        print(
            "Task workspace: "
            f"{trial.workspace_limits.name} "
            f"min={format_vec(trial.workspace_limits.xyz_min, 3)} "
            f"max={format_vec(trial.workspace_limits.xyz_max, 3)}"
        )
    if trial.keepout_boxes:
        print("Keep-out boxes:")
        for box in trial.keepout_boxes:
            print(
                f"  - {box.name}: "
                f"min={format_vec(box.xyz_min, 3)} max={format_vec(box.xyz_max, 3)}"
            )
    else:
        print("Keep-out boxes: none configured")


def list_trials() -> None:
    for name, trial in TRIALS.items():
        print(f"{name}: {trial.description}")


def dry_run_trial(trial: TrialDefinition) -> None:
    print_trial_summary(trial)
    print("Mode : dry-run (no RTDE connection)")

    last_cartesian_pose_task = None
    for index, step in enumerate(trial.sequence, start=1):
        title = step.label or step.target or step.kind
        print(f"\n[{index:02d}] {step.kind}: {title}")

        if step.kind in {"move_home", "move_j"}:
            joint_target = trial.joint_targets[step.target]
            print(f"  joints_deg: {format_vec(joint_target.joints_deg, 2)}")
            print(f"  joints_rad: {format_vec(joint_target.joints_rad(), 3)}")
            print("  note      : joint path itself is not collision-checked by this runner")
            last_cartesian_pose_task = None
            continue

        if step.kind == "move_l":
            target_pose_task = _resolve_waypoint_pose(
                trial,
                step.target,
                rotation_hint=(
                    last_cartesian_pose_task.rotation.as_rotvec()
                    if last_cartesian_pose_task is not None
                    else None
                ),
            )
            target_pose_base = trial.x_base_task @ target_pose_task
            print_pose("  task", target_pose_task)
            print(f"  base  rtde: {format_vec(pose_to_rtde(target_pose_base), 4)}")

            violations = point_violations(trial, target_pose_task.translation)
            if violations:
                print(f"  ERROR     : {'; '.join(violations)}")
            elif last_cartesian_pose_task is not None:
                found = segment_violations(
                    trial,
                    last_cartesian_pose_task.translation,
                    target_pose_task.translation,
                )
                if found:
                    print(f"  ERROR     : {found[0]}")
                else:
                    print("  path check: point and straight-line segment are inside limits")
            else:
                print("  path check: endpoint is valid; segment check skipped after joint move")

            last_cartesian_pose_task = target_pose_task
            continue

        if step.kind == "report_state":
            print("  live only : prints actual joint angles and TCP pose on the robot")
            continue

        if step.kind == "wait":
            print(f"  duration  : {step.duration:.2f} s")
            continue

        if step.kind in {"open_gripper", "close_gripper"}:
            print("  gripper   : live only")
            continue

        raise ValueError(f"unsupported step kind {step.kind!r}")


def _validate_live_move_l(
    trial: TrialDefinition,
    start_pose_task,
    target_pose_task,
) -> None:
    violations = point_violations(trial, target_pose_task.translation)
    violations.extend(
        segment_violations(
            trial,
            start_pose_task.translation,
            target_pose_task.translation,
        )
    )
    if violations:
        raise RuntimeError(" | ".join(violations))


def execute_trial(trial: TrialDefinition, arm: ArmHandle) -> None:
    print_trial_summary(trial)
    print("Mode : live")

    for index, step in enumerate(trial.sequence, start=1):
        title = step.label or step.target or step.kind
        print(f"\n[{index:02d}] {step.kind}: {title}")

        if step.kind == "report_state":
            print_arm_state(arm, step.label or "state")
            continue

        if step.kind in {"move_home", "move_j"}:
            joint_target = trial.joint_targets[step.target]
            speed = step.speed if step.speed is not None else joint_target.speed
            accel = step.accel if step.accel is not None else joint_target.accel
            print(f"  moveJ target deg: {format_vec(joint_target.joints_deg, 2)}")
            arm.control.moveJ(joint_target.joints_rad().tolist(), speed, accel)
            continue

        if step.kind == "move_l":
            start_pose_task = current_task_pose(arm)
            target_pose_task = _resolve_waypoint_pose(
                trial,
                step.target,
                rotation_hint=start_pose_task.rotation.as_rotvec(),
            )
            _validate_live_move_l(trial, start_pose_task, target_pose_task)

            speed = step.speed if step.speed is not None else trial.default_linear_speed
            accel = step.accel if step.accel is not None else trial.default_linear_accel
            print(f"  start task xyz : {format_vec(start_pose_task.translation, 3)}")
            print(f"  target task xyz: {format_vec(target_pose_task.translation, 3)}")
            arm.control.moveL(pose_to_rtde(arm.to_base(target_pose_task)), speed, accel)
            continue

        if step.kind == "wait":
            print(f"  sleeping for {step.duration:.2f} s")
            time.sleep(step.duration)
            continue

        if step.kind == "open_gripper":
            if arm.gripper is None:
                raise RuntimeError("gripper step requested, but no gripper is attached")
            arm.gripper.open()
            continue

        if step.kind == "close_gripper":
            if arm.gripper is None:
                raise RuntimeError("gripper step requested, but no gripper is attached")
            arm.gripper.close()
            continue

        raise ValueError(f"unsupported step kind {step.kind!r}")


def run_trial(
    trial_name: str = DEFAULT_TRIAL,
    live: bool = False,
    connected_arm_names: Optional[Sequence[str]] = None,
    state_only: bool = False,
) -> int:
    if trial_name not in TRIALS:
        raise ValueError(f"unknown trial {trial_name!r}; choose from {sorted(TRIALS)}")

    trial = TRIALS[trial_name]
    if not live:
        dry_run_trial(trial)
        return 0

    selected_arms = list(connected_arm_names or (trial.arm,))
    if trial.arm not in selected_arms:
        raise RuntimeError(
            f"trial {trial.name!r} runs on {trial.arm}; add that arm to the live connection set"
        )

    needs_gripper = any(
        step.kind in {"open_gripper", "close_gripper"}
        for step in trial.sequence
    )
    gripper_arms = {trial.arm} if needs_gripper else set()
    arms = connect_arms(
        selected_arms,
        motion_arms={trial.arm},
        gripper_arms=gripper_arms,
    )
    try:
        for name in selected_arms:
            print_arm_state(arms[name], f"{name} connected state")
        if state_only:
            return 0
        execute_trial(trial, arms[trial.arm])
        return 0
    finally:
        close_arms(arms)
