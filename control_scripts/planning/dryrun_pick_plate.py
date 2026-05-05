"""Dry-run motion planning for pick_place_plate_microwave: 4-leg test.

Plans the four task-frame transits the real task would issue, animates
each in Meshcat, and prints the ``moveJ(path=...)`` payload that
``execute_plan`` *would* send to RTDE — without connecting to the arm.

Legs (right arm only — left stays parked at sim HOME):
    1. HOME            -> hover above grasp        (free-space)
    2. hover           -> grasp pose               (descent; sim ignores plate)
    3. grasp pose      -> hover (transit_z lift)
    4. hover           -> midpoint @ transit_z     (carry across workspace)

Each leg is an independent ``plan_transit`` call. Legs 2-4 seed their
``current_q`` from the previous leg's terminal IK so the joint branch
stays consistent across the chain — same semantics as chained IK, just
spread across separate planner calls so we get a clean per-leg RTDE
payload (matching the natural stop-grip-lift flow on the real arm).

Run::

    python3.11 -m control_scripts.planning.dryrun_pick_plate
    python3.11 -m control_scripts.planning.dryrun_pick_plate --animate-seconds 3
"""

from __future__ import annotations

import argparse
import time
from typing import List, Optional

import numpy as np
from pydrake.geometry import (
    Rgba,
    Sphere,
    StartMeshcat,
)
from pydrake.math import RigidTransform
from pydrake.systems.analysis import Simulator

from . import default_home_q
from .build_scene import build_scene
from .transit import (
    InfeasiblePlanError,
    TransitPlan,
    _arm_model_instance,
    _arm_position_indices,
    plan_transit,
)
from ..grasps.plate import plate_rim_grasp_edge
from ..tasks.pick_place_plate_microwave import (
    ARM, CONFIG,
    PLATE_MIDPOINT_POSE_TASK, PLATE_PICK_POSE_TASK,
)
from ..util.poses import Pose
from ..util.rotations import Rotation


# Local overrides for this dry-run — matches the commented-out config in
# pick_place_plate_microwave.py (PICK_FROM="outside", PLACE_TO="microwave",
# grasp angle 0, place angle -55°). Done here so we can iterate on the
# planning test without editing the task file or touching the live config.
GRASP_ANGLE_OVERRIDE = 0.0
PLACE_ANGLE_OVERRIDE = -np.radians(55)

# Test: set the midpoint orientation to match the PLACE orientation
# (instead of the task module's MIDPOINT_ANGLE_RAD = 0). This forces leg 4
# to rotate the wrist from grasp (0°) to place (-55°) during the carry —
# a planner sanity check for whether that wrist sweep is feasible from
# a hover-altitude transit between the two XY locations.
MIDPOINT_ANGLE_OVERRIDE = PLACE_ANGLE_OVERRIDE


_DEFAULT_SPEED = 1.0    # rad/s — joint speed cap for moveJ_path
_DEFAULT_ACCEL = 1.4    # rad/s²
_BLEND_R = 0.02         # m — TCP-distance blend


def _set_marker(meshcat, path, xyz, color, radius=0.012):
    meshcat.SetObject(path, Sphere(radius), color)
    meshcat.SetTransform(path, RigidTransform(np.asarray(xyz, dtype=float)))


def _fk_tcp_pose(plant, plant_ctx, arm_name) -> Pose:
    """Task-frame TCP pose of the named arm at the current context."""
    inst = _arm_model_instance(plant, arm_name)
    tcp_frame = plant.GetFrameByName(f"tcp_{arm_name.removeprefix('ur_')}")
    X = tcp_frame.CalcPoseInWorld(plant_ctx)
    return Pose(
        translation=np.asarray(X.translation()),
        rotation=Rotation.from_matrix(X.rotation().matrix()),
    )


def _animate_plan(diagram, plant, sim_ctx, plan: TransitPlan,
                  seconds: float):
    """Replay a TransitPlan trajectory in real time via SetPositions."""
    plant_ctx = plant.GetMyMutableContextFromRoot(sim_ctx)
    t0_real = time.time()
    t0, t1 = plan.trajectory.start_time(), plan.trajectory.end_time()
    while True:
        elapsed = time.time() - t0_real
        if elapsed >= seconds:
            break
        s = min(1.0, elapsed / seconds)
        t = t0 + s * (t1 - t0)
        q = np.asarray(plan.trajectory.value(t)).flatten()
        plant.SetPositions(plant_ctx, q)
        diagram.ForcedPublish(sim_ctx)
        time.sleep(0.02)


def _print_movej_payload(plan: TransitPlan, leg_label: str,
                         n_waypoints: int = 20):
    """Format the moveJ(path=...) call execute_plan would send.

    Matches ``_execute_moveJ_path`` in execute.py: sparse-sample the
    full-plant trajectory at ``n_waypoints``, slice out the planning
    arm's 6 joints, append speed/accel/blend per row, terminal blend=0.
    """
    arm_inst_idx = (
        np.arange(0, 6) if plan.arm == "ur_left" else np.arange(6, 12)
    )
    t0, t1 = plan.trajectory.start_time(), plan.trajectory.end_time()
    sample_times = np.linspace(t0, t1, n_waypoints)
    rows: List[list] = []
    for t in sample_times:
        q_full = np.asarray(plan.trajectory.value(t)).flatten()
        q6 = q_full[arm_inst_idx]
        rows.append(list(q6) + [_DEFAULT_SPEED, _DEFAULT_ACCEL, _BLEND_R])
    rows[-1][-1] = 0.0   # terminal blend = 0 (controller requires)

    print(f"  # >>> RTDE: would call arm.control.moveJ(path=[")
    for i, row in enumerate(rows):
        q_str = "[" + ", ".join(f"{x:+.4f}" for x in row[:6]) + "]"
        tail = f"{row[6]:.2f}, {row[7]:.2f}, {row[8]:.4f}"
        marker = "  # final" if i == len(rows) - 1 else ""
        print(f"  #     [{q_str[1:-1]}, {tail}],{marker}")
    print(f"  # ])  # leg: {leg_label}, duration={plan.duration_s:.2f}s,"
          f" min_clearance={plan.min_clearance_m * 1000:.1f}mm")


def _terminal_q_for_arm(plan: TransitPlan, plant) -> np.ndarray:
    """Pull the planning arm's 6-vector at the trajectory's end."""
    arm_inst = _arm_model_instance(plant, plan.arm)
    arm_idx = _arm_position_indices(plant, arm_inst)
    q_full = np.asarray(plan.trajectory.value(plan.trajectory.end_time())).flatten()
    return q_full[arm_idx]


def _plan_leg(plant, plant_ctx, label: str, waypoints: List[Pose],
              current_q: dict, **kwargs) -> Optional[TransitPlan]:
    print()
    print(f"--- Leg: {label} ---")
    for i, wp in enumerate(waypoints):
        rv = wp.rotation.as_rotvec()
        print(f"  waypoint {i}: xyz={np.round(wp.translation,3)} "
              f"rotvec={np.round(rv, 3)}")
    try:
        plan = plan_transit(
            plant=plant, arm=ARM,
            waypoints=waypoints,
            plant_context=plant_ctx,
            current_q=current_q,
            **kwargs,
        )
    except InfeasiblePlanError as exc:
        print(f"  PLAN INFEASIBLE: {exc}")
        return None
    print(f"  planner={plan.metadata.get('planner')}, "
          f"duration={plan.duration_s:.2f}s, "
          f"clearance={plan.min_clearance_m*1000:.1f}mm")
    fallback = plan.metadata.get("spline_fallback_reason")
    if fallback:
        print(f"  (spline fell back to KTO: {fallback})")
    return plan


def _resolve_task_poses(plant, plant_ctx):
    """Build the 4 task-frame TCP poses + the FK'd HOME TCP pose."""
    grasp_pose = plate_rim_grasp_edge(
        PLATE_PICK_POSE_TASK, angle_rad=GRASP_ANGLE_OVERRIDE,
    ).grasp_pose
    midpoint_pose = plate_rim_grasp_edge(
        PLATE_MIDPOINT_POSE_TASK, angle_rad=MIDPOINT_ANGLE_OVERRIDE,
    ).grasp_pose
    transit_z = float(CONFIG.transit_z)

    hover_pose = Pose(
        translation=np.array([grasp_pose.translation[0],
                              grasp_pose.translation[1], transit_z]),
        rotation=grasp_pose.rotation,
    )
    midpoint_carry_pose = Pose(
        translation=np.array([midpoint_pose.translation[0],
                              midpoint_pose.translation[1], transit_z]),
        rotation=midpoint_pose.rotation,
    )
    home_tcp_pose = _fk_tcp_pose(plant, plant_ctx, ARM)
    return home_tcp_pose, hover_pose, grasp_pose, midpoint_carry_pose, transit_z


def _plan_all_legs(plant, plant_ctx, home_q, poses):
    """Plan all 4 legs once, chaining each leg's start q from the
    previous leg's terminal IK. Returns a list of (label, plan) tuples
    or None if any leg fails."""
    home_tcp, hover, grasp, midpoint = poses
    other_q = home_q[_arm_position_indices(plant, _arm_model_instance(plant, "ur_left"))]
    right_q = home_q[_arm_position_indices(plant, _arm_model_instance(plant, "ur_right"))]

    legs_spec = [
        ("1: HOME -> hover (free-space)", [home_tcp, hover]),
        ("2: hover -> grasp (descent)",   [hover, grasp]),
        ("3: grasp -> hover (lift)",      [grasp, hover]),
        ("4: hover -> midpoint (carry)",  [hover, midpoint]),
    ]
    out = []
    for label, wps in legs_spec:
        plan = _plan_leg(
            plant, plant_ctx, label, waypoints=wps,
            current_q={"ur_left": other_q, "ur_right": right_q},
        )
        if plan is None:
            return None
        out.append((label, plan))
        right_q = _terminal_q_for_arm(plan, plant)
    return out


def _print_all_payloads(legs):
    """Per-leg moveJ(path) payload that execute_plan would send."""
    print()
    print("=" * 78)
    print("  Would-be RTDE payloads (NOT sent — dry-run)")
    print("=" * 78)
    for label, plan in legs:
        _print_movej_payload(plan, label)
        if label.startswith("2:"):
            print("  # >>> RTDE: would call arm.gripper.close()  (placeholder)")


def _run_interactive(meshcat, diagram, plant, sim_ctx, legs,
                     animate_seconds: float):
    """Hold the visualizer alive; replay any leg via Meshcat controls.

    Slider 'leg' (1..N) plus Replay / Prev / Next / Quit buttons.
    Selecting a new leg auto-plays it once; the arm stays at the leg's
    end pose until you act again. No mesh re-upload — switches are
    near-instant.
    """
    plant_ctx = plant.GetMyMutableContextFromRoot(sim_ctx)
    n = len(legs)

    meshcat.AddSlider("leg", min=1, max=n, step=1, value=1)
    meshcat.AddButton("Replay")
    meshcat.AddButton("Prev")
    meshcat.AddButton("Next")
    meshcat.AddButton("Quit")

    def play(idx):
        label, plan = legs[idx - 1]
        # Start the arm at the leg's start q so the animation is the
        # full motion from beginning to end every time, regardless of
        # what was on screen before.
        q0 = np.asarray(plan.trajectory.value(plan.trajectory.start_time())).flatten()
        plant.SetPositions(plant_ctx, q0)
        diagram.ForcedPublish(sim_ctx)
        print(f"[play] leg {idx}: {label}  (animating {animate_seconds:.1f}s)")
        _animate_plan(diagram, plant, sim_ctx, plan, animate_seconds)

    last_idx = 0
    last_replay = meshcat.GetButtonClicks("Replay")
    last_prev = meshcat.GetButtonClicks("Prev")
    last_next = meshcat.GetButtonClicks("Next")

    print()
    print("Interactive mode — Meshcat controls panel:")
    print(f"  Slider 'leg' picks index 1..{n}")
    print(f"  Prev / Next steps through; Replay re-plays current leg")
    print(f"  Quit exits cleanly (or Ctrl-C)")

    try:
        while True:
            if meshcat.GetButtonClicks("Quit") > 0:
                break

            cur = int(round(meshcat.GetSliderValue("leg")))
            replay_clicks = meshcat.GetButtonClicks("Replay")
            prev_clicks = meshcat.GetButtonClicks("Prev")
            next_clicks = meshcat.GetButtonClicks("Next")

            if next_clicks > last_next:
                cur = ((cur - 1 + 1) % n) + 1
                meshcat.SetSliderValue("leg", cur)
            elif prev_clicks > last_prev:
                cur = ((cur - 1 - 1) % n) + 1
                meshcat.SetSliderValue("leg", cur)
            last_prev, last_next = prev_clicks, next_clicks

            should_play = (cur != last_idx) or (replay_clicks > last_replay)
            if should_play:
                play(cur)
                last_idx = cur
                last_replay = replay_clicks

            time.sleep(0.05)
    except KeyboardInterrupt:
        print("Closing.")
    finally:
        for name in ("Replay", "Prev", "Next", "Quit"):
            try:
                meshcat.DeleteButton(name)
            except Exception:
                pass
        try:
            meshcat.DeleteSlider("leg")
        except Exception:
            pass
    return 0


def main(animate_seconds: float = 2.5, cycle: bool = False) -> int:
    meshcat = StartMeshcat()
    print(f"[dryrun] Meshcat → {meshcat.web_url()}")

    scene = build_scene(meshcat=meshcat)
    diagram, plant = scene.diagram, scene.plant
    sim = Simulator(diagram)
    sim.Initialize()
    sim_ctx = sim.get_mutable_context()
    plant_ctx = plant.GetMyMutableContextFromRoot(sim_ctx)

    home_q = default_home_q(plant)
    plant.SetPositions(plant_ctx, home_q)
    diagram.ForcedPublish(sim_ctx)

    home_tcp_pose, hover_pose, grasp_pose, midpoint_carry_pose, transit_z = (
        _resolve_task_poses(plant, plant_ctx)
    )

    _set_marker(meshcat, "/tcp/home",     home_tcp_pose.translation,
                Rgba(0.6, 0.6, 0.6, 0.8))
    _set_marker(meshcat, "/tcp/hover",    hover_pose.translation,
                Rgba(0.2, 0.8, 0.2, 0.9))
    _set_marker(meshcat, "/tcp/grasp",    grasp_pose.translation,
                Rgba(0.95, 0.6, 0.1, 0.9))
    _set_marker(meshcat, "/tcp/midpoint", midpoint_carry_pose.translation,
                Rgba(0.2, 0.4, 0.95, 0.9))

    print()
    print("=" * 78)
    print(f"  Dry-run pick_place_plate_microwave   (arm={ARM})")
    print(f"  HOME      TCP xyz = {np.round(home_tcp_pose.translation, 3)}")
    print(f"  hover     TCP xyz = {np.round(hover_pose.translation, 3)}")
    print(f"  grasp     TCP xyz = {np.round(grasp_pose.translation, 3)}")
    print(f"  midpoint  TCP xyz = {np.round(midpoint_carry_pose.translation, 3)}")
    print(f"  transit_z         = {transit_z}")
    print(f"  Mode              = {'cycle' if cycle else 'interactive'}")
    print("=" * 78)

    legs = _plan_all_legs(
        plant, plant_ctx, home_q,
        (home_tcp_pose, hover_pose, grasp_pose, midpoint_carry_pose),
    )
    if legs is None:
        return 1

    _print_all_payloads(legs)

    if cycle:
        for label, plan in legs:
            print(f"[cycle] leg: {label}")
            # Reset to leg start so each animation is the full motion.
            q0 = np.asarray(plan.trajectory.value(plan.trajectory.start_time())).flatten()
            plant.SetPositions(plant_ctx, q0)
            diagram.ForcedPublish(sim_ctx)
            _animate_plan(diagram, plant, sim_ctx, plan, animate_seconds)
        print()
        print("Done — held final pose. Ctrl-C to exit.")
        try:
            while True:
                time.sleep(0.5)
        except KeyboardInterrupt:
            return 0
        return 0

    return _run_interactive(meshcat, diagram, plant, sim_ctx, legs,
                            animate_seconds)


if __name__ == "__main__":
    ap = argparse.ArgumentParser(
        description="Dry-run pick_place_plate_microwave: plan once, "
                    "replay any leg interactively in Meshcat."
    )
    ap.add_argument(
        "--animate-seconds", type=float, default=2.5,
        help="Seconds per leg animation (each replay).",
    )
    ap.add_argument(
        "--cycle", action="store_true",
        help="Auto-cycle through all legs instead of interactive mode.",
    )
    args = ap.parse_args()
    raise SystemExit(main(animate_seconds=args.animate_seconds,
                          cycle=args.cycle))
