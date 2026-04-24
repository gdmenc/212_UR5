"""Sanity-check calibration by reading each arm's TCP in both frames.

Usage (at the lab, with both arms powered on and RTDE reachable):

    python -m control_scripts.examples.verify_calibration

Reads current TCP from each arm in base frame, converts to task frame via
the calibration in ``control_scripts/calibration.py``, prints both. If the
calibration is correct, the task-frame reports from both arms should
agree on the location of any shared reference point (e.g., if both arms
are at the same physical table location, the task-frame XYZ numbers
should match within a few mm).

Offline dry run (no connection): pass ``--dry`` to print the expected
task-frame origin in each base frame — useful for eyeballing the
calibration arithmetic without connecting.
"""

from __future__ import annotations

import argparse

from ..arm import ArmHandle
from ..calibration import (
    TCP_OFFSET_ROBOTIQ_2F85,
    X_LEFT_BASE_TASK,
    X_RIGHT_BASE_TASK,
)
from ..util.poses import Pose
from ..util.rtde_convert import pose_to_rtde, rtde_to_pose


def _pretty(pose: Pose) -> str:
    t = pose.translation
    rotvec = pose.rotation.as_rotvec()
    return (f"  xyz  = [{t[0]:+.4f}, {t[1]:+.4f}, {t[2]:+.4f}]\n"
            f"  rvec = [{rotvec[0]:+.4f}, {rotvec[1]:+.4f}, {rotvec[2]:+.4f}]")


def dry_run() -> None:
    """Print the task-origin pose as each arm sees it — pure calibration math.
    The task origin is Pose() (identity); its base-frame pose is X_base_task
    itself, so this just prints the calibration."""
    print("LEFT arm — task origin expressed in left base frame:")
    print(_pretty(X_LEFT_BASE_TASK))
    print("  rtde pose:", pose_to_rtde(X_LEFT_BASE_TASK))
    print()
    print("RIGHT arm — task origin expressed in right base frame:")
    print(_pretty(X_RIGHT_BASE_TASK))
    print("  rtde pose:", pose_to_rtde(X_RIGHT_BASE_TASK))


def live(left_ip: str, right_ip: str) -> None:
    """Connect to both arms and print current TCP in both frames."""
    from rtde_control import RTDEControlInterface
    from rtde_receive import RTDEReceiveInterface

    left = ArmHandle(
        name="ur_left",
        control=RTDEControlInterface(left_ip),
        receive=RTDEReceiveInterface(left_ip),
        X_base_task=X_LEFT_BASE_TASK,
        tcp_offset=TCP_OFFSET_ROBOTIQ_2F85,  # TODO: swap for hook once measured
    )
    right = ArmHandle(
        name="ur_right",
        control=RTDEControlInterface(right_ip),
        receive=RTDEReceiveInterface(right_ip),
        X_base_task=X_RIGHT_BASE_TASK,
        tcp_offset=TCP_OFFSET_ROBOTIQ_2F85,
    )
    left.setup()
    right.setup()

    for arm in (left, right):
        tcp_base = rtde_to_pose(arm.receive.getActualTCPPose())
        tcp_task = arm.to_task(tcp_base)
        print(f"{arm.name} current TCP:")
        print("  base frame:")
        print(_pretty(tcp_base))
        print("  task frame:")
        print(_pretty(tcp_task))
        print()

    left.control.stopScript()
    right.control.stopScript()


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--dry", action="store_true",
                    help="skip RTDE — just print the calibration arithmetic")
    ap.add_argument("--left-ip",  default="192.168.1.101")
    ap.add_argument("--right-ip", default="192.168.1.102")
    args = ap.parse_args()

    if args.dry:
        dry_run()
    else:
        live(args.left_ip, args.right_ip)


if __name__ == "__main__":
    main()
