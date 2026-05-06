"""Dry-run motion planning for pick_place_bowl_hook_microwave: entrance test.

Plans the four task-frame transits the real bowl task would issue, animates
each in Meshcat, and prints the ``moveJ(path=...)`` payload that
``execute_plan`` *would* send to RTDE — without connecting to the arm.

Lighter alternative
-------------------
``python3.11 -m control_scripts.tasks.pick_place_bowl_hook_microwave --mode sim``
plans only the post-pick carry + post-place return (the segments the
real task actually routes through ``plan_transit``) and uses the same
``WORLD`` declared in the task. This file plans more legs and prints
RTDE payloads — keep it for full-trajectory rehearsal.

Legs (left arm only — right stays parked at sim HOME):
    1. HOME            -> hover above grasp              (free-space)
    2. hover           -> pregrasp -> grasp pose         (descent; sim ignores bowl)
    3. grasp pose      -> pregrasp -> hover              (transit_z lift)
    4. hover           -> midpoint? -> microwave entrance hover

Each leg is an independent ``plan_transit`` call. Legs 2-4 seed their
``current_q`` from the previous leg's terminal IK so the joint branch
stays consistent across the chain — same semantics as chained IK, just
spread across separate planner calls so we get a clean per-leg RTDE
payload (matching the natural stop-grip-lift flow on the real arm).

Planner stack
-------------
``plan_transit`` IKs the task-frame TCP waypoints into joint targets. For the
held-bowl carry it tries Drake ``KinematicTrajectoryOptimization`` first, then
falls back to constrained RRT if KTO cannot find a valid path. The live task
streams the planned trajectory with ``execute_plan(..., method="servoJ")`` for
smoother KTO carries; this dry-run still prints a ``moveJ(path=...)`` payload
for inspection.

The first three legs are planned in a scene without a held object. Leg 4 is
planned in a second scene with the bowl welded to the hook TCP, matching the
live task's post-pick carry. Meshcat uses that attached-bowl scene so the
carry-to-entrance path shows the bowl in hand. The cup-with-stick is kept on
the table as a static obstacle in every planning scene so the carry routes
around it (matching the live task's ``INCLUDE_CUP_WITH_STICK_OBSTACLE``).

With ``KEEP_BOWL_LEVEL_DURING_CARRY`` enabled in the task module, leg 4 passes
an axis-alignment constraint to ``plan_transit``: the bowl's body-fixed +Z
axis, expressed in the TCP frame at carry start, must stay close to task +Z
during the carry. This uses Drake's ``AngleBetweenVectorsConstraint`` at KTO
path samples, so yaw/rotation around vertical remains free while the bowl tilt
established at pick time stays put.

Run::

    python3.11 -m control_scripts.planning.dryrun_pick_bowl
    python3.11 -m control_scripts.planning.dryrun_pick_bowl --animate-seconds 3
    python3.11 -m control_scripts.planning.dryrun_pick_bowl --direct-to-entry
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
from .rrt import build_planning_scene
from .transit import (
    InfeasiblePlanError,
    TransitPlan,
    _arm_model_instance,
    _arm_position_indices,
    plan_transit,
)
from ..tasks.pick_place_bowl_hook_microwave import (
    ARM, CONFIG,
    CARRY_BOWL_LEVEL_TOLERANCE_RAD,
    CARRY_MIN_CLEARANCE_M,
    INCLUDE_CUP_WITH_STICK_OBSTACLE,
    KEEP_BOWL_LEVEL_DURING_CARRY,
    MICROWAVE_DOOR_OPEN_ANGLE_RAD,
    MIDPOINT_ANGLE_RAD,
    MOTION_PLAN_BLEND_R_M,
    MOTION_PLAN_EXECUTION_METHOD,
    MOTION_PLAN_RRT_FALLBACK,
    MOTION_PLAN_SERVO_TIME_SCALE,
    USE_MIDPOINT,
    _bowl_up_axis_in_tcp,
    _hover_before_place,
    plan_midpoint,
    plan_pick,
    plan_place,
)
from ..util.poses import Pose, offset_along_tool_z, pose_at_altitude
from ..util.rotations import Rotation


_DEFAULT_SPEED = 1.0    # rad/s — joint speed cap for moveJ_path
_DEFAULT_ACCEL = 1.4    # rad/s²
_BLEND_R = MOTION_PLAN_BLEND_R_M


# Static-object skip list mirroring the live task's _build_planning_context.
# Keeping the cup-with-stick on the table so the planner routes around it.
_SCENE_SKIP_OBJECTS = (
    ("plate", "cup", "bowl", "bottle", "tray")
    if INCLUDE_CUP_WITH_STICK_OBSTACLE
    else ()
)
_SCENE_INCLUDE_OBJECTS = INCLUDE_CUP_WITH_STICK_OBSTACLE


def _set_marker(meshcat, path, xyz, color, radius=0.012):
    meshcat.SetObject(path, Sphere(radius), color)
    meshcat.SetTransform(path, RigidTransform(np.asarray(xyz, dtype=float)))


def _fk_tcp_pose(plant, plant_ctx, arm_name) -> Pose:
    """Task-frame TCP pose of the named arm at the current context."""
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

    print("  # >>> RTDE: would call arm.control.moveJ(path=[")
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
              current_q: dict, rrt_diagram=None, **kwargs) -> Optional[TransitPlan]:
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
            use_rrt_fallback=(
                rrt_diagram is not None and MOTION_PLAN_RRT_FALLBACK
            ),
            rrt_diagram=rrt_diagram,
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
    kto_fallback = plan.metadata.get("kto_fallback_reason")
    if kto_fallback:
        print(f"  (KTO fell back to RRT: {kto_fallback})")
    return plan


def _resolve_task_poses(plant, plant_ctx):
    """Build task-frame TCP poses matching the live bowl task."""
    grasp = plan_pick()
    grasp_pose = grasp.grasp_pose
    pregrasp_pose = offset_along_tool_z(grasp_pose, grasp.pregrasp_offset)
    place_pose = plan_place()
    midpoint_pose = plan_midpoint(angle_rad=MIDPOINT_ANGLE_RAD)
    entrance_hover_pose = _hover_before_place(place_pose, CONFIG)
    transit_z = float(CONFIG.transit_z)

    hover_pose = pose_at_altitude(pregrasp_pose, transit_z)
    midpoint_carry_pose = Pose(
        translation=np.array([midpoint_pose.translation[0],
                              midpoint_pose.translation[1], transit_z]),
        rotation=midpoint_pose.rotation,
    )
    home_tcp_pose = _fk_tcp_pose(plant, plant_ctx, ARM)
    return (
        home_tcp_pose,
        hover_pose,
        pregrasp_pose,
        grasp_pose,
        midpoint_carry_pose,
        entrance_hover_pose,
        transit_z,
    )


def _plan_all_legs(
    plant,
    plant_ctx,
    pre_carry_rrt_diagram,
    carry_plant,
    carry_plant_ctx,
    carry_rrt_diagram,
    home_q,
    poses,
    *,
    direct_to_entry: bool,
):
    """Plan all 4 legs once, chaining each leg's start q from the
    previous leg's terminal IK. Returns a list of (label, plan) tuples
    or None if any leg fails."""
    home_tcp, hover, pregrasp, grasp, midpoint, entrance_hover = poses

    # The bowl task plans on ur_left, so freeze ur_right at HOME and chain
    # ur_left's terminal q across legs.
    other_arm = "ur_right"
    other_q = home_q[
        _arm_position_indices(plant, _arm_model_instance(plant, other_arm))
    ]
    left_q = home_q[
        _arm_position_indices(plant, _arm_model_instance(plant, ARM))
    ]

    if direct_to_entry or not USE_MIDPOINT:
        carry_waypoints = [hover, entrance_hover]
        carry_label = "4: hover -> entrance hover (direct)"
    else:
        carry_waypoints = [hover, midpoint, entrance_hover]
        carry_label = "4: hover -> midpoint -> entrance hover"

    pre_carry_specs = [
        ("1: HOME -> hover (free-space)", [home_tcp, hover]),
        ("2: hover -> pregrasp -> grasp (descent)", [hover, pregrasp, grasp]),
        ("3: grasp -> pregrasp -> hover (lift)", [grasp, pregrasp, hover]),
    ]
    out = []
    for label, wps in pre_carry_specs:
        # Pass the no-bowl RRT diagram so KTO can fall back to RRT if the
        # spline path collides with the (open) microwave door or the cup-
        # with-stick — e.g. the wide HOME -> hover sweep on the left arm
        # routes around the door instead of straight through it.
        plan = _plan_leg(
            plant, plant_ctx, label, waypoints=wps,
            current_q={ARM: left_q, other_arm: other_q},
            rrt_diagram=pre_carry_rrt_diagram,
        )
        if plan is None:
            return None
        out.append((label, plan))
        left_q = _terminal_q_for_arm(plan, plant)

    plan = _plan_leg(
        carry_plant,
        carry_plant_ctx,
        carry_label,
        waypoints=carry_waypoints,
        current_q={ARM: left_q, other_arm: other_q},
        rrt_diagram=carry_rrt_diagram,
        align_tcp_axis=(
            _bowl_up_axis_in_tcp(hover)
            if KEEP_BOWL_LEVEL_DURING_CARRY else None
        ),
        align_tcp_axis_world=np.array([0.0, 0.0, 1.0]),
        align_tcp_axis_tolerance_rad=CARRY_BOWL_LEVEL_TOLERANCE_RAD,
        min_clearance_m=CARRY_MIN_CLEARANCE_M,
    )
    if plan is None:
        return None
    out.append((carry_label, plan))
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
    print("  Prev / Next steps through; Replay re-plays current leg")
    print("  Quit exits cleanly (or Ctrl-C)")

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


def main(animate_seconds: float = 2.5, cycle: bool = False,
         gripper_mode: str = "closed", direct_to_entry: bool = False) -> int:
    meshcat = StartMeshcat()
    print(f"[dryrun] Meshcat → {meshcat.web_url()}")

    # Approach/lift legs share a planning scene with no in-hand bowl yet but
    # WITH the cup-with-stick on the table (matching the live task setup).
    scene = build_scene(
        include_objects=_SCENE_INCLUDE_OBJECTS,
        skip_static_objects=_SCENE_SKIP_OBJECTS,
        robotiq_mode=gripper_mode,
        microwave_door_open_angle_rad=MICROWAVE_DOOR_OPEN_ANGLE_RAD,
    )
    plant = scene.plant
    root_ctx = scene.diagram.CreateDefaultContext()
    plant_ctx = plant.GetMyMutableContextFromRoot(root_ctx)

    # Carry scene: same arm/microwave/cup-with-stick layout, with the bowl
    # welded to the hook TCP. This is both the leg-4 visualization scene and
    # the source of the carry render in Meshcat.
    carry_scene = build_scene(
        include_objects=_SCENE_INCLUDE_OBJECTS,
        skip_static_objects=_SCENE_SKIP_OBJECTS,
        robotiq_mode=gripper_mode,
        microwave_door_open_angle_rad=MICROWAVE_DOOR_OPEN_ANGLE_RAD,
        attached_objects=(("bowl", ARM, None),),
        meshcat=meshcat,
    )
    diagram, carry_plant = carry_scene.diagram, carry_scene.plant
    sim = Simulator(diagram)
    sim.Initialize()
    sim_ctx = sim.get_mutable_context()
    carry_plant_ctx = carry_plant.GetMyMutableContextFromRoot(sim_ctx)

    # Pre-carry RRT scene: RobotDiagram-backed plant with the cup-with-stick
    # on the table but no in-hand bowl yet. Used as the RRT fallback for the
    # HOME -> hover and approach legs so they can route around the open door.
    pre_carry_rrt_diagram, _, _, _ = build_planning_scene(
        include_objects=_SCENE_INCLUDE_OBJECTS,
        skip_static_objects=_SCENE_SKIP_OBJECTS,
        robotiq_mode=gripper_mode,
        microwave_door_open_angle_rad=MICROWAVE_DOOR_OPEN_ANGLE_RAD,
    )

    # Carry RRT scene: RobotDiagram-backed plant with the bowl welded and the
    # cup-with-stick still on the table. Matches the live ``_build_planning_context``.
    carry_rrt_diagram, carry_rrt_plant, _, _ = build_planning_scene(
        include_objects=_SCENE_INCLUDE_OBJECTS,
        skip_static_objects=_SCENE_SKIP_OBJECTS,
        robotiq_mode=gripper_mode,
        microwave_door_open_angle_rad=MICROWAVE_DOOR_OPEN_ANGLE_RAD,
        attached_objects=(("bowl", ARM, None),),
    )
    carry_rrt_root_ctx = carry_rrt_diagram.CreateDefaultContext()
    carry_rrt_plant_ctx = carry_rrt_plant.GetMyMutableContextFromRoot(
        carry_rrt_root_ctx,
    )

    home_q = default_home_q(plant)
    plant.SetPositions(plant_ctx, home_q)
    carry_plant.SetPositions(carry_plant_ctx, default_home_q(carry_plant))
    diagram.ForcedPublish(sim_ctx)

    (
        home_tcp_pose,
        hover_pose,
        pregrasp_pose,
        grasp_pose,
        midpoint_carry_pose,
        entrance_hover_pose,
        transit_z,
    ) = (
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
    _set_marker(meshcat, "/tcp/entrance_hover", entrance_hover_pose.translation,
                Rgba(0.8, 0.2, 0.95, 0.9))

    print()
    print("=" * 78)
    print(f"  Dry-run pick_place_bowl_hook_microwave   (arm={ARM})")
    print(f"  HOME      TCP xyz = {np.round(home_tcp_pose.translation, 3)}")
    print(f"  hover     TCP xyz = {np.round(hover_pose.translation, 3)}")
    print(f"  pregrasp  TCP xyz = {np.round(pregrasp_pose.translation, 3)}")
    print(f"  grasp     TCP xyz = {np.round(grasp_pose.translation, 3)}")
    print(f"  midpoint  TCP xyz = {np.round(midpoint_carry_pose.translation, 3)}")
    print(f"  entrance  TCP xyz = {np.round(entrance_hover_pose.translation, 3)}")
    print(f"  transit_z         = {transit_z}")
    print(f"  Carry path         = {'direct to entrance' if (direct_to_entry or not USE_MIDPOINT) else 'via midpoint'}")
    print("  Motion planner     = plan_transit (IK -> KTO first -> RRT fallback)")
    print(f"  Microwave door     = open ({np.degrees(MICROWAVE_DOOR_OPEN_ANGLE_RAD):.0f}°)")
    print(f"  Cup-with-stick     = {'on table (obstacle)' if INCLUDE_CUP_WITH_STICK_OBSTACLE else 'omitted'}")
    attitude = (
        f"bowl-up axis within ±{np.degrees(CARRY_BOWL_LEVEL_TOLERANCE_RAD):.1f}° of task +Z"
        if KEEP_BOWL_LEVEL_DURING_CARRY else "unconstrained"
    )
    print(f"  Bowl attitude      = {attitude}")
    print(f"  Execution method   = execute_plan(method='{MOTION_PLAN_EXECUTION_METHOD}')")
    if MOTION_PLAN_EXECUTION_METHOD == "servoJ":
        print(f"  Servo time scale   = {MOTION_PLAN_SERVO_TIME_SCALE:.1f}x")
    print("  Held bowl model    = welded to TCP for leg 4 and Meshcat display")
    print(f"  Carry clearance    = {CARRY_MIN_CLEARANCE_M * 1000:.1f} mm")
    print(f"  Mode              = {'cycle' if cycle else 'interactive'}")
    print("=" * 78)

    legs = _plan_all_legs(
        plant, plant_ctx, pre_carry_rrt_diagram,
        carry_rrt_plant, carry_rrt_plant_ctx,
        carry_rrt_diagram, home_q,
        (
            home_tcp_pose,
            hover_pose,
            pregrasp_pose,
            grasp_pose,
            midpoint_carry_pose,
            entrance_hover_pose,
        ),
        direct_to_entry=direct_to_entry,
    )
    if legs is None:
        return 1

    _print_all_payloads(legs)

    if cycle:
        for label, plan in legs:
            print(f"[cycle] leg: {label}")
            q0 = np.asarray(plan.trajectory.value(plan.trajectory.start_time())).flatten()
            carry_plant.SetPositions(carry_plant_ctx, q0)
            diagram.ForcedPublish(sim_ctx)
            _animate_plan(diagram, carry_plant, sim_ctx, plan, animate_seconds)
        print()
        print("Done — held final pose. Ctrl-C to exit.")
        try:
            while True:
                time.sleep(0.5)
        except KeyboardInterrupt:
            return 0
        return 0

    return _run_interactive(meshcat, diagram, carry_plant, sim_ctx, legs,
                            animate_seconds)


if __name__ == "__main__":
    ap = argparse.ArgumentParser(
        description="Dry-run pick_place_bowl_hook_microwave: plan once, "
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
    ap.add_argument(
        "--gripper-mode", choices=["closed", "open"], default="closed",
        help="Robotiq 2F-85 finger configuration to load (default: closed).",
    )
    ap.add_argument(
        "--direct-to-entry", action="store_true",
        help="Skip the midpoint and plan the carry directly to the microwave entrance hover.",
    )
    args = ap.parse_args()
    raise SystemExit(main(animate_seconds=args.animate_seconds,
                          cycle=args.cycle,
                          gripper_mode=args.gripper_mode,
                          direct_to_entry=args.direct_to_entry))
