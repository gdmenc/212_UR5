from __future__ import annotations

import argparse
import sys

from control_scripts.manual import ManualTeleopOptions, run_manual_teleop
from control_scripts.routines import ROUTINES
from control_scripts.runtime import (
    close_arms,
    connect_arms,
    list_arm_definitions,
    parse_arm_names,
    print_arm_state,
)
from control_scripts.tasks import TASKS
from control_scripts.trials.definitions import DEFAULT_TRIAL
from control_scripts.trials.runner import list_trials, run_trial


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Main entrypoint for UR real-arm trials, state readout, and manual teleop."
    )
    subparsers = parser.add_subparsers(dest="command", required=True)

    subparsers.add_parser("list-arms", help="show available arm definitions")
    subparsers.add_parser("list-trials", help="show available hard-coded trials")
    subparsers.add_parser("list-tasks", help="show registered single-arm tasks")
    subparsers.add_parser("list-routines", help="show registered end-to-end routines")

    task_parser = subparsers.add_parser(
        "task",
        help="run a registered single-arm task (see `list-tasks`)",
    )
    task_parser.add_argument("name", help="task name from the TASKS registry")
    task_parser.add_argument(
        "--dry",
        action="store_true",
        help="plan and print without commanding motion (task must support it)",
    )

    routine_parser = subparsers.add_parser(
        "routine",
        help="run a registered end-to-end routine (see `list-routines`)",
    )
    routine_parser.add_argument("name", help="routine name from the ROUTINES registry")
    routine_parser.add_argument(
        "--dry",
        action="store_true",
        help="plan and print without commanding motion (routine must support it)",
    )

    state_parser = subparsers.add_parser(
        "state",
        help="connect the selected arms and print their current joint/TCP state",
    )
    state_parser.add_argument(
        "--arms",
        default=None,
        help="comma-separated arm list, e.g. ur_right or ur_left,ur_right",
    )

    trial_parser = subparsers.add_parser(
        "trial",
        help="dry-run or execute a hard-coded autonomous trial",
    )
    trial_parser.add_argument("--trial", default=DEFAULT_TRIAL)
    trial_parser.add_argument("--live", action="store_true")
    trial_parser.add_argument("--state-only", action="store_true")
    trial_parser.add_argument(
        "--arms",
        default=None,
        help="comma-separated live connection set; trial arm must be included",
    )

    manual_parser = subparsers.add_parser(
        "manual",
        help="manual teleoperation for one selected arm",
    )
    manual_parser.add_argument(
        "--arms",
        default=None,
        help="comma-separated connected arm list; selected manual arm must be included",
    )
    manual_parser.add_argument(
        "--arm",
        default=None,
        help="which connected arm to manually control; defaults to the first connected arm",
    )
    manual_parser.add_argument(
        "--safety-trial",
        default=DEFAULT_TRIAL,
        help="trial whose workspace/keep-out boxes should be used for teleop safety",
    )
    manual_parser.add_argument(
        "--input-source",
        default="auto",
        choices=["auto", "pygame", "terminal"],
        help="teleop input source",
    )
    manual_parser.add_argument("--joystick-index", type=int, default=0)
    manual_parser.add_argument("--linear-speed", type=float, default=0.08)
    manual_parser.add_argument("--angular-speed", type=float, default=0.50)
    manual_parser.add_argument("--accel", type=float, default=0.75)
    manual_parser.add_argument("--loop-dt", type=float, default=0.05)
    manual_parser.add_argument("--state-period", type=float, default=0.5)
    manual_parser.add_argument("--log-file", default=None)

    return parser


def _legacy_argv(argv: list[str]) -> list[str]:
    if not argv or argv[0].startswith("-"):
        return ["trial", *argv]
    return argv


def main(argv: list[str] | None = None) -> int:
    raw_argv = list(sys.argv[1:] if argv is None else argv)
    parser = _build_parser()
    args = parser.parse_args(_legacy_argv(raw_argv))

    try:
        if args.command == "list-arms":
            list_arm_definitions()
            return 0

        if args.command == "list-trials":
            list_trials()
            return 0

        if args.command == "list-tasks":
            if not TASKS:
                print("(no tasks registered)")
            for name in sorted(TASKS):
                print(name)
            return 0

        if args.command == "list-routines":
            if not ROUTINES:
                print("(no routines registered)")
            for name in sorted(ROUTINES):
                print(name)
            return 0

        if args.command == "task":
            if args.name not in TASKS:
                parser.error(
                    f"unknown task {args.name!r}; choose from {sorted(TASKS) or 'none'}"
                )
            return TASKS[args.name](dry=args.dry)

        if args.command == "routine":
            if args.name not in ROUTINES:
                parser.error(
                    f"unknown routine {args.name!r}; choose from {sorted(ROUTINES) or 'none'}"
                )
            return ROUTINES[args.name](dry=args.dry)

        if args.command == "state":
            arm_names = parse_arm_names(args.arms)
            arms = connect_arms(arm_names)
            try:
                for name in arm_names:
                    print_arm_state(arms[name], name)
                return 0
            finally:
                close_arms(arms)

        if args.command == "trial":
            if args.state_only and not args.live:
                parser.error("trial --state-only requires --live")
            arm_names = parse_arm_names(args.arms) if args.arms else None
            return run_trial(
                trial_name=args.trial,
                live=args.live,
                connected_arm_names=arm_names,
                state_only=args.state_only,
            )

        if args.command == "manual":
            arm_names = parse_arm_names(args.arms)
            controlled_arm = args.arm or arm_names[0]
            if controlled_arm not in arm_names:
                parser.error("manual --arm must be included in --arms")

            arms = connect_arms(
                arm_names,
                motion_arms={controlled_arm},
                gripper_arms={controlled_arm},
            )
            try:
                for name in arm_names:
                    print_arm_state(arms[name], f"{name} connected state")
                options = ManualTeleopOptions(
                    arm_name=controlled_arm,
                    safety_trial_name=args.safety_trial,
                    input_source=args.input_source,
                    joystick_index=args.joystick_index,
                    linear_speed=args.linear_speed,
                    angular_speed=args.angular_speed,
                    accel=args.accel,
                    loop_dt=args.loop_dt,
                    state_period=args.state_period,
                    log_file=args.log_file,
                )
                run_manual_teleop(arms[controlled_arm], options)
                return 0
            finally:
                close_arms(arms)

        parser.error(f"unknown command {args.command!r}")
        return 2
    except KeyboardInterrupt:
        print("Interrupted.", file=sys.stderr)
        return 130
    except Exception as exc:
        print(f"Error: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
