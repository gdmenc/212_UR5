"""Open the microwave door with the left arm's hook gripper — thin
wrapper around ``tasks.open_microwave``.

The door geometry (HANDLE_ENGAGE_POSE_TASK, PRE_ENGAGE_POSE_TASK,
PRE_ENGAGE_JOINTS_RAD, HINGE_POSITION_TASK, DOOR_WIDTH_M) plus the
``DOOR_SPEC`` and ``CONFIG`` defaults now live in
``tasks/open_microwave.py`` so the main script can import them
alongside the primitive. This file just imports the spec, prints a
summary, and runs against the rig.

Add ``--motion-planning`` to route Phase 1 + Phase 3 through Drake KTO
(uses the rig's ACTUAL current joints as the planner's start state,
not SIM_HOME). Otherwise the legacy moveJ-to-recorded-joints + arc-of-
moveLs flow is used.

Run::

    python3.11 -m control_scripts.examples.open_microwave           # arc mode
    python3.11 -m control_scripts.examples.open_microwave --dry     # plan-only
    python3.11 -m control_scripts.examples.open_microwave --motion-planning
"""

from __future__ import annotations

import argparse

import numpy as np

from ..session import default_session
from ..tasks.open_microwave import (
    ARM,
    CONFIG,
    DOOR_SPEC,
    HANDLE_ENGAGE_POSE_TASK,
    HINGE_POSITION_TASK,
    PRE_ENGAGE_POSE_TASK,
    SUPPORTS_SIM,
    _arc_waypoints,
    dry_run_motion_planning,
    open_microwave_door,
    run_sim,
)


def run(dry: bool, motion_planning: bool, mode: str = "real") -> int:
    door = DOOR_SPEC
    if motion_planning:
        # Shallow copy + flip the flag so we don't mutate the module
        # default. Other fields stay as-is.
        from dataclasses import replace
        door = replace(DOOR_SPEC, use_motion_planning=True)

    arc_waypoints = _arc_waypoints(door)
    final_arc_pos = arc_waypoints[-1].translation

    radius = float(np.linalg.norm(
        (HANDLE_ENGAGE_POSE_TASK.translation - HINGE_POSITION_TASK)[:2]
    ))
    arc_length = radius * door.arc_open_angle_rad

    mode_str = "MOTION-PLANNED (KTO)" if door.use_motion_planning else "arc mode"
    print("=" * 60)
    print("  Arm               :", ARM)
    print("  Mode              :", mode_str)
    print("  Pre-engage (task) :", np.round(PRE_ENGAGE_POSE_TASK.translation, 4))
    print("  Engage pose(task) :", np.round(HANDLE_ENGAGE_POSE_TASK.translation, 4))
    print("  Hinge (task)      :", np.round(HINGE_POSITION_TASK, 4))
    print("  Arc radius        :", f"{radius:.3f} m")
    print("  Arc angle         :", f"{np.degrees(door.arc_open_angle_rad):.1f}°")
    print("  Arc length        :", f"{arc_length:.3f} m")
    print("  Arc steps         :", door.n_arc_steps)
    print("  Final handle pos  :", np.round(final_arc_pos, 4))
    print("  Transit Z         :", CONFIG.transit_z, "m (task frame)")
    if door.use_motion_planning:
        print(f"  Plan waypoints    : {door.motion_plan_n_waypoints}  "
              f"(moveJ(path=...) sample count)")
        print(f"  Plan TCP speed cap: {door.motion_plan_tcp_speed:.3f} m/s")
    print("-" * 60)
    print("  Arc waypoints (task XY):")
    for i, wp in enumerate(arc_waypoints):
        print(f"    [{i+1}] {np.round(wp.translation[:2], 4)}")
    print("=" * 60)

    if dry:
        if door.use_motion_planning:
            # Actually run the planner offline against the cached plant
            # and dump the moveJ(path=[...]) payloads it would send.
            # Seed is SIM_HOME (no rig to query getActualQ from).
            dry_run_motion_planning(door)
        else:
            print("[dry run] skipping RTDE connection. No motion commanded.")
        return 0

    if mode == "sim":
        if not SUPPORTS_SIM:
            print("[sim] this task does not support sim mode")
            return 1
        if not door.use_motion_planning:
            print("[sim] sim mode requires --motion-planning (only the "
                  "motion-planned phases can be visualized; arc/force modes "
                  "go through hand-coded RTDE calls).")
            return 1
        return run_sim(door)
    if mode != "real":
        raise ValueError(f"unknown mode {mode!r}; choose 'real' or 'sim'")

    with default_session(left=True, right=False) as session:
        arm = session.arms[ARM]

        print(f"\n→ open microwave door ({mode_str})")
        result = open_microwave_door(arm, door, CONFIG)

        if result.success:
            print(f"  ✓ door opened — arc length {result.door_opened_distance:.3f} m")
            return 0
        print(f"  ✗ FAILED: {result.reason}")
        print(f"    TCP moved {result.door_opened_distance:.3f} m before stopping")
        return 1


def main() -> int:
    ap = argparse.ArgumentParser(
        description="Open microwave door with the left arm hook gripper."
    )
    ap.add_argument(
        "--dry", action="store_true",
        help="Print planned poses and arc waypoints without connecting to RTDE.",
    )
    ap.add_argument(
        "--motion-planning", action="store_true",
        help="Route Phase 1 + Phase 3 through Drake KTO motion planning. "
             "Uses the rig's actual current joints (not SIM_HOME) as the "
             "planner start state.",
    )
    ap.add_argument(
        "--mode",
        choices=["real", "sim"],
        default="real",
        help=(
            "Execution mode. 'real' (default) runs on the rig via RTDE. "
            "'sim' plans the motion-planned phases (requires "
            "--motion-planning) and replays them in meshcat with a "
            "leg-by-leg stepper; the hand-coded engage / release phases "
            "are not simulated."
        ),
    )
    args = ap.parse_args()
    return run(dry=args.dry, motion_planning=args.motion_planning, mode=args.mode)


if __name__ == "__main__":
    raise SystemExit(main())
