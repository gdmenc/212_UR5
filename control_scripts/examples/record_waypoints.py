"""Record waypoints by hand-moving the arm in UR freedrive (teachMode).

Workflow
--------
1. Connect to one arm.
2. Enable freedrive — the joints release so you can physically pose the arm.
3. Move the arm to a pose you want.
4. At the prompt, type a label and press ENTER (or just ENTER for an
   auto-numbered name) to capture the current joints + task pose.
5. Repeat. Type 'q' + ENTER to finish.

Output
------
A JSON file at ``logs/waypoints/<arm>_<stamp>.json`` containing every
snapshot. On quit the script also prints ready-to-paste Python for both
``JointTarget`` (use with ``move_j`` / ``move_home`` steps) and
``TaskWaypoint`` (use with ``move_l`` steps) — drop those into
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
    python -m control_scripts.examples.record_waypoints --arm ur_right
    python -m control_scripts.examples.record_waypoints --arm ur_left --out logs/wps.json
    python -m control_scripts.examples.record_waypoints --arm ur_right --no-freedrive
"""

from __future__ import annotations

import argparse
import datetime as dt
import json
import os
from typing import List, Optional

import numpy as np

from ..arm import ArmHandle
from ..runtime import current_base_pose, current_task_pose, print_arm_state
from ..session import default_session


_DEFAULT_OUT_DIR = os.path.join("logs", "waypoints")


def _default_out_path(arm_name: str) -> str:
    stamp = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
    return os.path.join(_DEFAULT_OUT_DIR, f"{arm_name}_{stamp}.json")


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
    t = snap["task_pose"]["translation"]
    r = snap["task_pose"]["rotvec"]
    j = snap["joints_deg"]
    print(f"  ✓ captured {snap['name']!r}")
    print(f"    joints_deg : [{', '.join(f'{v:+7.2f}' for v in j)}]")
    print(f"    task xyz   : [{t[0]:+.4f}, {t[1]:+.4f}, {t[2]:+.4f}]")
    print(f"    task rvec  : [{r[0]:+.4f}, {r[1]:+.4f}, {r[2]:+.4f}]")


def _save(snapshots: List[dict], out_path: str, arm_name: str) -> None:
    parent = os.path.dirname(out_path)
    if parent:
        os.makedirs(parent, exist_ok=True)
    payload = {
        "arm": arm_name,
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

    print("\n# TaskWaypoint(...) entries — for move_l steps")
    for s in snapshots:
        t = s["task_pose"]["translation"]
        r = s["task_pose"]["rotvec"]
        xyz = ", ".join(f"{v:+.4f}" for v in t)
        rvec = ", ".join(f"{v:+.4f}" for v in r)
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
    print("  ?               → this help")
    print("  q               → quit (saves, ends teach mode, prints paste snippets)")
    print()


def run(arm_name: str, out_path: Optional[str], no_freedrive: bool) -> int:
    if arm_name not in {"ur_left", "ur_right"}:
        raise ValueError(f"unknown arm {arm_name!r}; choose ur_left or ur_right")

    out_path = out_path or _default_out_path(arm_name)
    snapshots: List[dict] = []

    print(f"Recording waypoints for {arm_name}.")
    print(f"Output file: {out_path}")
    if no_freedrive:
        print("--no-freedrive: arm will NOT be released. Move it from the pendant.")
    else:
        print("Freedrive (teachMode) WILL be enabled — keep the e-stop reachable.")
    _help()

    left  = (arm_name == "ur_left")
    right = (arm_name == "ur_right")

    with default_session(left=left, right=right) as session:
        arm = session.arms[arm_name]
        print_arm_state(arm, f"{arm_name} initial state")

        if not no_freedrive:
            print("\nEntering freedrive — joints released. Move the arm by hand.")
            arm.control.teachMode()

        try:
            wp_index = 0
            while True:
                try:
                    line = input(
                        "\n[wp] label (ENTER=auto, 'l' list, 'd' del, "
                        "'s' save, '?' help, 'q' quit): "
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
                        t = s["task_pose"]["translation"]
                        print(f"  [{i:02d}] {s['name']:<20s}  "
                              f"task xyz=[{t[0]:+.3f}, {t[1]:+.3f}, {t[2]:+.3f}]")
                    continue
                if line == "d":
                    if snapshots:
                        dropped = snapshots.pop()
                        _save(snapshots, out_path, arm_name)
                        print(f"  ✗ dropped {dropped['name']!r} (file updated)")
                    else:
                        print("  (nothing to drop)")
                    continue
                if line == "s":
                    _save(snapshots, out_path, arm_name)
                    print(f"  ↳ saved {len(snapshots)} snapshot(s) to {out_path}")
                    continue

                name = line if line else f"wp_{wp_index:02d}"
                snap = _capture(arm, name)
                snapshots.append(snap)
                _save(snapshots, out_path, arm_name)
                _print_capture(snap)
                wp_index += 1
        finally:
            if not no_freedrive:
                try:
                    arm.control.endTeachMode()
                    print("\nExited freedrive.")
                except Exception as e:
                    print(f"\nWARNING: endTeachMode failed: {e}")
            _save(snapshots, out_path, arm_name)
            print(f"Saved {len(snapshots)} snapshot(s) to {out_path}")
            _print_paste_snippets(snapshots)

    return 0


def main() -> None:
    ap = argparse.ArgumentParser(
        description="Record waypoints by hand-moving the arm in freedrive."
    )
    ap.add_argument("--arm", required=True, choices=["ur_left", "ur_right"],
                    help="Which arm to record on.")
    ap.add_argument("--out", default=None,
                    help="Output JSON path (default: logs/waypoints/<arm>_<stamp>.json)")
    ap.add_argument("--no-freedrive", action="store_true",
                    help="Don't enable teachMode — useful if you'd rather move the "
                         "arm from the pendant or remote control isn't authorized.")
    args = ap.parse_args()
    raise SystemExit(run(args.arm, args.out, args.no_freedrive))


if __name__ == "__main__":
    main()
