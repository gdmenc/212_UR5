"""Record waypoints by hand-moving the arm in UR freedrive (teachMode).

Variant for the **standing / table-mounted** setup: single UR at a fixed IP,
hook gripper, table-mount task frame (no Vention lab X_LEFT_BASE_TASK).

Workflow
--------
1. Connect to the hook arm via --robot-ip.
2. Enable freedrive — the joints release so you can physically pose the arm.
3. Move the arm to a pose you want.
4. At the prompt, type a label and press ENTER (or just ENTER for an
   auto-numbered name) to capture the current joints + task pose.
5. Repeat. Type 'q' + ENTER to finish.

Output
------
A JSON file at ``logs/waypoints/<stamp>.json`` containing every snapshot.
On quit the script also prints ready-to-paste Python for both
``JointTarget`` and ``TaskWaypoint`` — drop those into
``control_scripts/trials/definitions.py``.

Safety
------
Freedrive releases the joints. Gravity compensation stays on so the arm
doesn't drop, but it WILL swing if you let go suddenly with momentum.
Keep one hand on the arm and the other near the e-stop. End the session
cleanly with 'q' so endTeachMode() runs — if you Ctrl-C, the finally
block still tries to end teach mode but the controller may need a
manual reset.

Usage
-----
    python -m control_scripts.examples.record_waypoints_standing
    python -m control_scripts.examples.record_waypoints_standing --robot-ip 169.254.9.43
    python -m control_scripts.examples.record_waypoints_standing --task-origin-offset 0.1 0.0 0.0 --task-yaw-deg 0
    python -m control_scripts.examples.record_waypoints_standing --no-freedrive
"""

from __future__ import annotations

import argparse
import datetime as dt
import json
import os
from typing import List, Optional

import numpy as np

from ..arm import ArmHandle
from ..calibration import TCP_OFFSET_HOOK
from ..grippers import HookGripper
from ..runtime import current_base_pose, print_arm_state
from ..session import DEFAULT_CONNECT_TRIES, ArmSpec, Session
from ..util.poses import Pose
from ..util.rotations import Rotation


_DEFAULT_OUT_DIR = os.path.join("logs", "waypoints")

# Matches hook_grasp_from_vision — Drake planner arm id stays "ur_left".
_SESSION_HOOK_ARM = "ur_left"
_DEFAULT_ROBOT_IP = "169.254.9.43"


def _default_out_path() -> str:
    stamp = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
    return os.path.join(_DEFAULT_OUT_DIR, f"standing_{stamp}.json")


def _x_base_task_table_mount(
    dx_t: float,
    dy_t: float,
    dz_t: float,
    yaw_deg: float,
) -> Pose:
    """Pose of task origin in base frame for a table-mounted UR.

    (dx_t, dy_t, dz_t) is the vector from base origin to task origin in
    task coordinates. yaw_deg rotates task X about vertical (task Z ∥ base Z).
    """
    yaw = float(np.radians(yaw_deg))
    c, s = np.cos(yaw), np.sin(yaw)
    r_tb = np.array(
        [[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]],
        dtype=float,
    )
    v_task = np.array([dx_t, dy_t, dz_t], dtype=float)
    v_base = r_tb @ v_task
    return Pose(
        translation=v_base,
        rotation=Rotation.from_matrix(r_tb),
    )


def _standing_session(
    robot_ip: str,
    *,
    X_base_task: Pose,
    connect_tries: int = DEFAULT_CONNECT_TRIES,
) -> Session:
    """Single hook-arm session for the table-mounted / standing configuration."""
    return Session(
        {
            _SESSION_HOOK_ARM: ArmSpec(
                name=_SESSION_HOOK_ARM,
                ip=robot_ip,
                X_base_task=X_base_task,
                tcp_offset=TCP_OFFSET_HOOK,
                gripper_factory=lambda rtde_c, rtde_r, rtde_io=None: HookGripper(rtde_c),
                home_q_rad=None,
            )
        },
        connect_tries=connect_tries,
    )


def _capture(arm: ArmHandle, name: str) -> dict:
    q_rad = np.asarray(arm.receive.getActualQ(), dtype=float)
    base_pose = current_base_pose(arm)
    task_pose = arm.to_task(base_pose)
    return {
        "name": name,
        "timestamp_iso": dt.datetime.now().isoformat(timespec="seconds"),
        "joints_deg": np.degrees(q_rad).tolist(),
        "joints_rad": q_rad.tolist(),
        "task_pose": {
            "translation": task_pose.translation.tolist(),
            "rotvec":      task_pose.rotation.as_rotvec().tolist(),
        },
        "base_pose": {
            "translation": base_pose.translation.tolist(),
            "rotvec":      base_pose.rotation.as_rotvec().tolist(),
        },
    }


def _print_capture(snap: dict) -> None:
    t = snap["base_pose"]["translation"]
    r = snap["base_pose"]["rotvec"]
    j = snap["joints_deg"]
    print(f"  captured {snap['name']!r}")
    print(f"    joints_deg : [{', '.join(f'{v:+7.2f}' for v in j)}]")
    print(f"    base xyz   : [{t[0]:+.4f}, {t[1]:+.4f}, {t[2]:+.4f}]")
    print(f"    base rvec  : [{r[0]:+.4f}, {r[1]:+.4f}, {r[2]:+.4f}]")
    print(
        "    calib pose : "
        f"{t[0]:+.6f} {t[1]:+.6f} {t[2]:+.6f} {r[0]:+.6f} {r[1]:+.6f} {r[2]:+.6f}  "
        "(paste into calibration/calibrate_camera.cpp prompt; metres + radians)"
    )


def _save(snapshots: List[dict], out_path: str) -> None:
    parent = os.path.dirname(out_path)
    if parent:
        os.makedirs(parent, exist_ok=True)
    payload = {
        "arm": _SESSION_HOOK_ARM,
        "setup": "standing_table_mount",
        "saved_at": dt.datetime.now().isoformat(timespec="seconds"),
        "count": len(snapshots),
        "snapshots": snapshots,
    }
    with open(out_path, "w") as f:
        json.dump(payload, f, indent=2)


def _print_paste_snippets(snapshots: List[dict]) -> None:
    if not snapshots:
        return
    print("\n" + "=" * 70)
    print(" Paste into control_scripts/trials/definitions.py:")
    print("=" * 70)

    print("\n# JointTarget(...) entries — for move_home / move_j steps")
    for s in snapshots:
        j = s["joints_deg"]
        joints = ", ".join(f"{v:+.4f}" for v in j)
        print(f'    "{s["name"]}": JointTarget("{s["name"]}", joints_deg=[{joints}]),')

    print("\n# TaskWaypoint(...) entries — for move_l steps (task frame, not base)")
    for s in snapshots:
        t = s["task_pose"]["translation"]
        r = s["task_pose"]["rotvec"]
        xyz = " ".join(f"{v:+.4f}" for v in t)
        rvec = " ".join(f"{v:+.4f}" for v in r)
        print(f'    "{s["name"]}": TaskWaypoint("{s["name"]}", '
              f'xyz=[{xyz}], rotvec=[{rvec}]),')
    print()


def _help() -> None:
    print()
    print("Commands at the prompt:")
    print("  <label> + ENTER → capture current pose with that label")
    print("  ENTER (no text) → capture with auto-name 'wp_NN'")
    print("  l               → list captured snapshots so far")
    print("  d               → delete last snapshot")
    print("  s               → save now (also auto-saves on each capture)")
    print("  c               → close the attached gripper")
    print("  o               → open the attached gripper")
    print("  ?               → this help")
    print("  q               → quit (saves, ends teach mode, prints paste snippets)")
    print()


def _run_gripper_command(
    arm: ArmHandle,
    command: str,
    done_label: str,
    no_freedrive: bool,
) -> None:
    if arm.gripper is None:
        print("  (arm has no gripper attached)")
        return

    try:
        getattr(arm.gripper, command)()
        print(f"  gripper {done_label}")
    finally:
        if not no_freedrive:
            arm.control.teachMode()
            print("  freedrive re-enabled")


def run(
    robot_ip: str,
    out_path: Optional[str],
    no_freedrive: bool,
    task_origin_offset_m: tuple[float, float, float],
    task_yaw_deg: float,
) -> int:
    out_path = out_path or _default_out_path()
    snapshots: List[dict] = []

    ox, oy, oz = task_origin_offset_m
    X_base_task = _x_base_task_table_mount(ox, oy, oz, task_yaw_deg)

    print(f"Recording waypoints — standing/table-mount setup @ {robot_ip}")
    print(
        f"Task frame: origin offset (task) = ({ox}, {oy}, {oz}) m, "
        f"yaw = {task_yaw_deg}°"
    )
    print(f"Output file: {out_path}")
    if no_freedrive:
        print("--no-freedrive: arm will NOT be released. Move it from the pendant.")
    else:
        print("Freedrive (teachMode) WILL be enabled — keep the e-stop reachable.")
    _help()

    with _standing_session(robot_ip, X_base_task=X_base_task) as session:
        arm = session.arms[_SESSION_HOOK_ARM]
        print_arm_state(arm, "initial state")

        if not no_freedrive:
            print("\nEntering freedrive — joints released. Move the arm by hand.")
            arm.control.teachMode()

        try:
            wp_index = 0
            while True:
                try:
                    line = input(
                        "\n[wp] label (ENTER=auto, 'l' list, 'd' del, "
                        "'s' save, 'c' close, 'o' open, '?' help, 'q' quit): "
                    ).strip()
                except EOFError:
                    line = "q"

                if line == "q":
                    break
                if line == "?":
                    _help()
                    continue
                if line == "l":
                    if not snapshots:
                        print("  (no snapshots yet)")
                    for i, s in enumerate(snapshots):
                        t = s["base_pose"]["translation"]
                        print(f"  [{i:02d}] {s['name']:<20s}  "
                              f"base xyz=[{t[0]:+.3f}, {t[1]:+.3f}, {t[2]:+.3f}]")
                    continue
                if line == "d":
                    if snapshots:
                        dropped = snapshots.pop()
                        _save(snapshots, out_path)
                        print(f"  dropped {dropped['name']!r} (file updated)")
                    else:
                        print("  (nothing to drop)")
                    continue
                if line == "s":
                    _save(snapshots, out_path)
                    print(f"  saved {len(snapshots)} snapshot(s) to {out_path}")
                    continue
                if line == "c":
                    _run_gripper_command(arm, "close", "closed", no_freedrive)
                    continue
                if line == "o":
                    _run_gripper_command(arm, "open", "opened", no_freedrive)
                    continue

                name = line if line else f"wp_{wp_index:02d}"
                snap = _capture(arm, name)
                snapshots.append(snap)
                _save(snapshots, out_path)
                _print_capture(snap)
                wp_index += 1
        finally:
            if not no_freedrive:
                try:
                    arm.control.endTeachMode()
                    print("\nExited freedrive.")
                except Exception as e:
                    print(f"\nWARNING: endTeachMode failed: {e}")
            _save(snapshots, out_path)
            print(f"Saved {len(snapshots)} snapshot(s) to {out_path}")
            _print_paste_snippets(snapshots)

    return 0


def main() -> None:
    ap = argparse.ArgumentParser(
        description=(
            "Record waypoints in freedrive — standing/table-mount hook arm "
            "(single UR IP, table-mount task frame, no Vention lab calibration)."
        )
    )
    ap.add_argument(
        "--robot-ip",
        type=str,
        default=_DEFAULT_ROBOT_IP,
        metavar="ADDR",
        help=f"UR controller IP for RTDE (default {_DEFAULT_ROBOT_IP}).",
    )
    ap.add_argument(
        "--out",
        default=None,
        help="Output JSON path (default: logs/waypoints/standing_<stamp>.json)",
    )
    ap.add_argument(
        "--no-freedrive",
        action="store_true",
        help="Don't enable teachMode — move the arm from the pendant instead.",
    )
    ap.add_argument(
        "--task-origin-offset",
        type=float,
        nargs=3,
        default=[0.0, 0.0, 0.0],
        metavar=("DX", "DY", "DZ"),
        help=(
            "Vector from UR base origin to task origin [m], in task axes "
            "(table Z up; default 0 0 0 = origins coincident when yaw=0). "
            "Same convention as hook_grasp_from_vision --task-origin-offset."
        ),
    )
    ap.add_argument(
        "--task-yaw-deg",
        type=float,
        default=0.0,
        help="Yaw [deg] about vertical between task frame and base frame (default 0).",
    )
    args = ap.parse_args()
    raise SystemExit(
        run(
            robot_ip=args.robot_ip.strip(),
            out_path=args.out,
            no_freedrive=args.no_freedrive,
            task_origin_offset_m=tuple(args.task_origin_offset),
            task_yaw_deg=float(args.task_yaw_deg),
        )
    )


if __name__ == "__main__":
    main()
