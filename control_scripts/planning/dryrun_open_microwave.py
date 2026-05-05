"""Dry-run motion planning for the open-microwave sequence.

Loads recorded waypoints from ``logs/waypoints/ur_left_open_microwave_1.json``
and replays each leg in sim using ``plan_transit``. The headline goal is
the door-arc leg: the recorded handle endpoints are joined by N

Lighter alternative
-------------------
``python3.11 -m control_scripts.examples.open_microwave --motion-planning --mode sim``
plans only Phase 1 (HOME→pre_engage) and Phase 3 (door arc) — the
segments the live task actually routes through ``plan_transit`` — and
uses the same ``WORLD`` declared in ``tasks/open_microwave.py``. This
file plans more legs (engage, slide-out, retreat) from recorded
waypoints — keep it for full-sequence rehearsal.
geometrically-correct intermediate arc waypoints (handle traces a circle
about the microwave hinge), and KTO smooths the joint trajectory through
all of them. Free-space transits before/after use the same planner.

Legs (only meaningful motion legs are planned; gripper open/close are
in-place state changes and are skipped):

    1.  HOME           -> pregrasp                    (free-space approach)
    2.  pregrasp       -> graspopen                   (slide hook into handle)
    3.  graspclose     -> opendoor    (via N arc waypoints; KTO smoothes)
    4.  opendoor       -> slideoutdoorhandle          (slide hook out)
    5.  slideoutdoorhandle -> HOME                    (retreat to HOME)

Scene caveat
------------
Single planning scene (microwave with door CLOSED) for all legs. The arc
geometry sweeps OUTSIDE the closed-door box (hook moves AWAY from the
front face throughout), so this is fine for legs 1-3. For legs 4-5 the
real door is open ~90° — the planner won't see the swept-open door as
an obstacle, so a path it likes here might collide with the real
swung-out door. Re-plan those two against an open-door scene if that
matters; for the user's stated goal (smooth the arc) this is sufficient.

Run::

    python3.11 -m control_scripts.planning.dryrun_open_microwave
    python3.11 -m control_scripts.planning.dryrun_open_microwave --animate-seconds 4
"""

from __future__ import annotations

import argparse
import json
import time
from pathlib import Path
from typing import List, Optional, Tuple

import numpy as np
from pydrake.geometry import Rgba, Sphere, StartMeshcat
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
from ..microwave import MICROWAVE_HINGE_X, door_plane_y
from ..tasks.open_microwave import MicrowaveDoorSpec, _arc_waypoints
from ..util.poses import Pose
from ..util.rotations import Rotation


WAYPOINTS_PATH = Path("logs/waypoints/ur_left_open_microwave_1.json")
ARM = "ur_left"
N_ARC_STEPS = 14
"""Intermediate arc waypoints between graspclose and opendoor. Same as
``examples/open_microwave.py``'s rig-side default."""

ARC_TCP_SPEED_M_S = 0.05
"""TCP linear speed cap on the door-arc leg (m/s). Hard-shape constraint
that forces ``plan_transit`` to run KTO instead of falling back to a
plain cubic spline — KTO smooths the joint trajectory through all 16
arc waypoints under this speed cap. Match the rig-side
``CONFIG.approach_speed`` used by ``examples/open_microwave.py`` (0.04
m/s); a hair more here so KTO has slack to satisfy the constraint."""

ARC_RTDE_N_WAYPOINTS = 30
"""How many waypoints to sample from each leg's trajectory for the
``moveJ(path=...)`` RTDE preview. Bumped past ``execute_plan``'s
default of 20 so the controller-side blending sees finer chord
segments along the door arc — keeps the blended TCP path closer to
the geometric circle, especially at the higher-curvature mid-arc
section where 20 chords starts to bow inward. Same count is used for
every leg's preview (overkill on the short legs, but keeps the dump
format uniform and easy to scan)."""


# ---------------------------------------------------------------------------
#  Waypoint loading
# ---------------------------------------------------------------------------

def _load_waypoints(path: Path) -> dict:
    if not path.exists():
        raise FileNotFoundError(
            f"Waypoint snapshot not found at {path}. "
            "Run the script from the repo root, or edit WAYPOINTS_PATH."
        )
    raw = path.read_text()
    # The snapshot file in this repo ships missing its OPENING brace
    # (it has a closing one). Prepend a brace if needed; don't touch
    # the rest.
    if not raw.lstrip().startswith("{"):
        raw = "{\n" + raw.lstrip()
    data = json.loads(raw)
    return {snap["name"]: snap for snap in data["snapshots"]}


def _pose_from_snap(snap: dict) -> Pose:
    t = snap["task_pose"]["translation"]
    rv = snap["task_pose"]["rotvec"]
    return Pose(translation=np.array(t, dtype=float),
                rotation=Rotation.from_rotvec(rv))


# ---------------------------------------------------------------------------
#  Arc waypoint generation
# ---------------------------------------------------------------------------

def _build_arc_waypoints(
    graspclose_pose: Pose,
    opendoor_pose: Pose,
    n_steps: int,
) -> Tuple[List[Pose], float]:
    """Geometric arc waypoints from graspclose to opendoor about the hinge.

    Hinge xy comes from ``microwave.py`` constants; hinge z is taken
    from the graspclose recording (the hinge axis is vertical, so z
    doesn't affect the arc — but using the recorded z keeps the
    intermediate poses on the same horizontal slice as the recording).

    The total opening angle is computed from the recorded endpoints
    (NOT a hard-coded constant), so the arc lands exactly at the
    recorded ``opendoor`` pose if the recording was on the door's
    actual circle. Returns the N intermediate poses (excluding start)
    and the total angle in radians.
    """
    hinge = np.array([MICROWAVE_HINGE_X, door_plane_y(),
                      graspclose_pose.translation[2]])

    r_start = graspclose_pose.translation - hinge
    r_end = opendoor_pose.translation - hinge
    angle_start = float(np.arctan2(r_start[1], r_start[0]))
    angle_end = float(np.arctan2(r_end[1], r_end[0]))
    delta = angle_end - angle_start
    while delta > np.pi:
        delta -= 2 * np.pi
    while delta < -np.pi:
        delta += 2 * np.pi

    spec = MicrowaveDoorSpec(
        handle_engage_pose_task=graspclose_pose,
        hinge_position_task=hinge,
        arc_open_angle_rad=abs(delta),
        n_arc_steps=n_steps,
        pull_direction_task=(opendoor_pose.translation - graspclose_pose.translation),
    )
    return _arc_waypoints(spec), delta


# ---------------------------------------------------------------------------
#  Planning helpers
# ---------------------------------------------------------------------------

def _plan_leg(
    plant, plant_ctx,
    label: str,
    waypoints: List[Pose],
    current_q: dict,
    *,
    avoid_collisions: bool = True,
    max_tcp_speed: Optional[float] = None,
) -> Optional[TransitPlan]:
    """Plan one leg. ``avoid_collisions=False`` is used for the engage
    and arc legs because the hook is in continuous contact with the
    door's collision body throughout — that contact is intended, not a
    failure. The user explicitly OK'd ignoring hook-handle interaction.

    ``max_tcp_speed`` (m/s) caps the TCP linear speed; this is a "hard
    shape" constraint in ``plan_transit`` that bypasses the spline-first
    fast path and forces a KTO solve. Use it on the arc leg to get
    KTO smoothing through the multi-waypoint trajectory.
    """
    print()
    print(f"--- Leg: {label}  ({len(waypoints)} waypoint{'s' if len(waypoints) != 1 else ''}) ---")
    for i, wp in enumerate(waypoints):
        print(f"  wp {i}: xyz={np.round(wp.translation, 3)}  "
              f"rotvec={np.round(wp.rotation.as_rotvec(), 2)}")
    if not avoid_collisions:
        print("  (collision avoidance OFF — hook intentionally contacts door)")
    if max_tcp_speed is not None:
        print(f"  (TCP speed cap {max_tcp_speed:.3f} m/s — forces KTO)")
    try:
        plan = plan_transit(
            plant=plant, arm=ARM,
            waypoints=waypoints,
            plant_context=plant_ctx,
            current_q=current_q,
            avoid_collisions=avoid_collisions,
            self_collision=avoid_collisions,
            max_tcp_linear_speed_m_per_s=max_tcp_speed,
        )
    except InfeasiblePlanError as exc:
        print(f"  PLAN INFEASIBLE: {exc}")
        return None
    clr = plan.min_clearance_m * 1000 if np.isfinite(plan.min_clearance_m) else float("nan")
    print(f"  planner={plan.metadata.get('planner')}  "
          f"duration={plan.duration_s:.2f}s  "
          f"clearance={clr:+.1f}mm")
    fallback = plan.metadata.get("spline_fallback_reason")
    if fallback:
        print(f"  (spline fell back to KTO: {fallback})")
    return plan


# ---------------------------------------------------------------------------
#  RTDE payload preview — what execute_plan would send
# ---------------------------------------------------------------------------

# moveJ(path=[...]) per-row defaults. Mirror execute.py:_DEFAULT_SPEED/_ACCEL.
_DEFAULT_SPEED = 1.0    # rad/s
_DEFAULT_ACCEL = 1.4    # rad/s²
_BLEND_R = 0.02         # m TCP-distance blend


def _print_movej_payload(
    plan: TransitPlan,
    leg_label: str,
    n_waypoints: int = 20,
) -> None:
    """Format the ``arm.control.moveJ(path=[[q1..q6, v, a, blend], ...])``
    call ``execute_plan`` would issue. Mirrors
    ``execute._execute_moveJ_path`` byte-for-byte: sparse-sample the
    trajectory at ``n_waypoints``, slice out the planning arm's 6
    joints, append speed/accel/blend per row, terminal blend = 0.
    """
    arm_idx = np.arange(0, 6) if plan.arm == "ur_left" else np.arange(6, 12)
    t0, t1 = plan.trajectory.start_time(), plan.trajectory.end_time()
    sample_times = np.linspace(t0, t1, n_waypoints)
    rows: List[list] = []
    for t in sample_times:
        q_full = np.asarray(plan.trajectory.value(t)).flatten()
        q6 = q_full[arm_idx]
        rows.append(list(q6) + [_DEFAULT_SPEED, _DEFAULT_ACCEL, _BLEND_R])
    rows[-1][-1] = 0.0   # terminal blend = 0 (UR controller requires)

    clr = plan.min_clearance_m * 1000 if np.isfinite(plan.min_clearance_m) else float("nan")
    print()
    print(f"  # >>> RTDE: would call arm.control.moveJ(path=[")
    for i, row in enumerate(rows):
        q_str = ", ".join(f"{x:+.4f}" for x in row[:6])
        tail = f"{row[6]:.2f}, {row[7]:.2f}, {row[8]:.4f}"
        marker = "  # final" if i == len(rows) - 1 else ""
        print(f"  #     [{q_str}, {tail}],{marker}")
    print(f"  # ])  # leg: {leg_label}  duration={plan.duration_s:.2f}s  "
          f"clearance={clr:+.1f}mm  planner={plan.metadata.get('planner')}")


def _terminal_arm_q(plant, plan: TransitPlan, arm_name: str) -> np.ndarray:
    """Pull the planning arm's 6-vector at the trajectory's end."""
    arm_idx = _arm_position_indices(plant, _arm_model_instance(plant, arm_name))
    q_full = np.asarray(
        plan.trajectory.value(plan.trajectory.end_time())
    ).flatten()
    return q_full[arm_idx]


def _fk_tcp_pose(plant, plant_ctx, arm_name: str) -> Pose:
    """Task-frame TCP pose of the named arm at the current context."""
    tcp_frame = plant.GetFrameByName(f"tcp_{arm_name.removeprefix('ur_')}")
    X = tcp_frame.CalcPoseInWorld(plant_ctx)
    return Pose(
        translation=np.asarray(X.translation()),
        rotation=Rotation.from_matrix(X.rotation().matrix()),
    )


# ---------------------------------------------------------------------------
#  Animation
# ---------------------------------------------------------------------------

# Meshcat publish rate during animation. 45 Hz balances motion
# smoothness against the WebSocket bandwidth needed to push the full
# URDF arm meshes + microwave on every frame.
_ANIM_PUBLISH_HZ = 50.0
_ANIM_PUBLISH_DT = 1.0 / _ANIM_PUBLISH_HZ


def _animate_plan(diagram, plant, sim_ctx, plan: TransitPlan,
                  seconds: float):
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
        time.sleep(_ANIM_PUBLISH_DT)


def _set_marker(meshcat, path, xyz, color, radius=0.012):
    meshcat.SetObject(path, Sphere(radius), color)
    meshcat.SetTransform(path, RigidTransform(np.asarray(xyz, dtype=float)))


# ---------------------------------------------------------------------------
#  Interactive replay loop
# ---------------------------------------------------------------------------

def _run_interactive(
    meshcat,
    diagram,
    plant,
    sim_ctx,
    legs: List[Tuple[str, TransitPlan]],
    animate_seconds: float,
) -> int:
    """Hold the Meshcat viewer alive; replay any leg via on-screen controls.

    Slider 'leg' (1..N) plus Replay / Prev / Next / Play-All / Quit
    buttons. Selecting a new leg auto-plays it once; the arm stays at
    the leg's end pose until you act again. No diagram rebuild between
    plays — switches are near-instant and the scene only loads once,
    avoiding the leak that hits when you re-launch the whole script.
    """
    plant_ctx = plant.GetMyMutableContextFromRoot(sim_ctx)
    n = len(legs)

    meshcat.AddSlider("leg", min=1, max=n, step=1, value=1)
    meshcat.AddButton("Replay")
    meshcat.AddButton("Prev")
    meshcat.AddButton("Next")
    meshcat.AddButton("Play All")
    meshcat.AddButton("Quit")

    def play(idx: int) -> None:
        label, plan = legs[idx - 1]
        q0 = np.asarray(
            plan.trajectory.value(plan.trajectory.start_time())
        ).flatten()
        plant.SetPositions(plant_ctx, q0)
        diagram.ForcedPublish(sim_ctx)
        print(f"[play] leg {idx}: {label}  "
              f"(plan {plan.duration_s:.2f}s, replay {animate_seconds:.1f}s)")
        _animate_plan(diagram, plant, sim_ctx, plan, animate_seconds)

    def play_all() -> None:
        for i in range(1, n + 1):
            meshcat.SetSliderValue("leg", i)
            play(i)

    last_idx = 0
    last_replay = meshcat.GetButtonClicks("Replay")
    last_prev = meshcat.GetButtonClicks("Prev")
    last_next = meshcat.GetButtonClicks("Next")
    last_play_all = meshcat.GetButtonClicks("Play All")

    print()
    print("Interactive mode — Meshcat controls panel:")
    print(f"  Slider 'leg' picks index 1..{n}")
    print(f"  Prev / Next steps through")
    print(f"  Replay re-plays current leg")
    print(f"  Play All cycles through every leg in order")
    print(f"  Quit exits cleanly (or Ctrl-C)")

    try:
        while True:
            if meshcat.GetButtonClicks("Quit") > 0:
                break

            cur = int(round(meshcat.GetSliderValue("leg")))
            replay_clicks = meshcat.GetButtonClicks("Replay")
            prev_clicks = meshcat.GetButtonClicks("Prev")
            next_clicks = meshcat.GetButtonClicks("Next")
            play_all_clicks = meshcat.GetButtonClicks("Play All")

            if play_all_clicks > last_play_all:
                play_all()
                last_play_all = play_all_clicks
                last_idx = n
                last_replay = meshcat.GetButtonClicks("Replay")
                continue

            if next_clicks > last_next:
                cur = (cur % n) + 1
                meshcat.SetSliderValue("leg", cur)
            elif prev_clicks > last_prev:
                cur = ((cur - 2) % n) + 1
                meshcat.SetSliderValue("leg", cur)
            last_prev, last_next = prev_clicks, next_clicks

            should_play = (cur != last_idx) or (replay_clicks > last_replay)
            if should_play:
                play(cur)
                last_idx = cur
                last_replay = replay_clicks

            time.sleep(0.05)
    except KeyboardInterrupt:
        print("\nClosing.")
    finally:
        for name in ("Replay", "Prev", "Next", "Play All", "Quit"):
            try:
                meshcat.DeleteButton(name)
            except Exception:
                pass
        try:
            meshcat.DeleteSlider("leg")
        except Exception:
            pass
    return 0


# ---------------------------------------------------------------------------
#  Main
# ---------------------------------------------------------------------------

def main(
    animate_seconds: float = 3.5,
    cycle: bool = False,
    show_arc_markers: bool = False,
) -> int:
    meshcat = StartMeshcat()
    print(f"[dryrun] Meshcat → {meshcat.web_url()}")

    # Don't load workspace demo objects (plate / bowl / cup_with_stick /
    # ...) — they clutter the cell and the open-microwave task doesn't
    # interact with them. Several of them sit where the left arm wants
    # to be at pregrasp and would block planning otherwise.
    #
    # Note: arm visual meshes are auto-swapped to the lightweight
    # collision OBJs at URDF-load time (see scene/arms.py), so the
    # default illustration visualizer already renders cheap arms +
    # full-fidelity microwave/gripper/objects.
    scene = build_scene(meshcat=meshcat, robotiq_mode="closed",
                        include_objects=False)
    diagram, plant = scene.diagram, scene.plant
    sim = Simulator(diagram)
    sim.Initialize()
    sim_ctx = sim.get_mutable_context()
    plant_ctx = plant.GetMyMutableContextFromRoot(sim_ctx)

    home_q_full = default_home_q(plant)
    plant.SetPositions(plant_ctx, home_q_full)
    diagram.ForcedPublish(sim_ctx)

    snaps = _load_waypoints(WAYPOINTS_PATH)
    pre_pose = _pose_from_snap(snaps["pregrasp"])
    grp_open = _pose_from_snap(snaps["graspopen"])
    grp_close = _pose_from_snap(snaps["graspclose"])
    open_door = _pose_from_snap(snaps["opendoor"])
    slideout = _pose_from_snap(snaps["slideoutdoorhandle"])

    home_pose = _fk_tcp_pose(plant, plant_ctx, ARM)

    print()
    print("=" * 72)
    print(f"  Dry-run open microwave  (arm={ARM})")
    print("=" * 72)
    print(f"  HOME left TCP : {np.round(home_pose.translation, 3)}")
    print(f"  pregrasp      : {np.round(pre_pose.translation, 3)}")
    print(f"  graspclose    : {np.round(grp_close.translation, 3)}")
    print(f"  opendoor      : {np.round(open_door.translation, 3)}")
    print(f"  slideout      : {np.round(slideout.translation, 3)}")

    arc_intermediate, delta_angle = _build_arc_waypoints(
        grp_close, open_door, N_ARC_STEPS,
    )
    print(f"  arc           : {np.degrees(delta_angle):+.1f}°  "
          f"({N_ARC_STEPS} intermediate steps)")
    print("=" * 72)

    # Markers for each waypoint on the meshcat scene.
    _set_marker(meshcat, "/wp/home",     home_pose.translation,
                Rgba(0.6, 0.6, 0.6, 0.8))
    _set_marker(meshcat, "/wp/pregrasp", pre_pose.translation,
                Rgba(0.2, 0.8, 0.2, 0.9))
    _set_marker(meshcat, "/wp/graspclose", grp_close.translation,
                Rgba(0.95, 0.6, 0.1, 0.9))
    _set_marker(meshcat, "/wp/opendoor", open_door.translation,
                Rgba(0.95, 0.1, 0.1, 0.9))
    _set_marker(meshcat, "/wp/slideout", slideout.translation,
                Rgba(0.2, 0.4, 0.95, 0.9))
    if show_arc_markers:
        for i, wp in enumerate(arc_intermediate):
            _set_marker(meshcat, f"/wp/arc/{i:02d}", wp.translation,
                        Rgba(0.6, 0.2, 0.6, 0.6), radius=0.008)

    # Build arc-leg waypoint list: [graspclose, arc[0..N-2], opendoor].
    # The helper's last intermediate is approximately at the recorded
    # opendoor pose; we replace it with the recorded value so KTO ends
    # exactly where the rig observed.
    arc_leg_waypoints = [grp_close, *arc_intermediate[:-1], open_door]

    arm_idx_left = _arm_position_indices(
        plant, _arm_model_instance(plant, "ur_left"))
    arm_idx_right = _arm_position_indices(
        plant, _arm_model_instance(plant, "ur_right"))
    cur_q = {
        "ur_left": home_q_full[arm_idx_left].copy(),
        "ur_right": home_q_full[arm_idx_right].copy(),
    }

    legs: List[Tuple[str, TransitPlan]] = []

    plan = _plan_leg(plant, plant_ctx,
                     "1: HOME -> pregrasp",
                     [home_pose, pre_pose], cur_q)
    if plan is not None:
        legs.append(("1: HOME -> pregrasp", plan))
        cur_q["ur_left"] = _terminal_arm_q(plant, plan, "ur_left")

    plan = _plan_leg(plant, plant_ctx,
                     "2: pregrasp -> graspopen -> graspclose",
                     [pre_pose, grp_open, grp_close], cur_q,
                     avoid_collisions=False)
    if plan is not None:
        legs.append(("2: engage", plan))
        cur_q["ur_left"] = _terminal_arm_q(plant, plan, "ur_left")

    plan = _plan_leg(plant, plant_ctx,
                     "3: door arc (graspclose -> opendoor, N waypoints)",
                     arc_leg_waypoints, cur_q,
                     avoid_collisions=False,
                     max_tcp_speed=ARC_TCP_SPEED_M_S)
    if plan is not None:
        legs.append(("3: door arc", plan))
        cur_q["ur_left"] = _terminal_arm_q(plant, plan, "ur_left")

    plan = _plan_leg(plant, plant_ctx,
                     "4: opendoor -> slideoutdoorhandle",
                     [open_door, slideout], cur_q,
                     avoid_collisions=False)
    if plan is not None:
        legs.append(("4: slide out", plan))
        cur_q["ur_left"] = _terminal_arm_q(plant, plan, "ur_left")

    plan = _plan_leg(plant, plant_ctx,
                     "5: slideout -> HOME",
                     [slideout, home_pose], cur_q)
    if plan is not None:
        legs.append(("5: retreat", plan))

    if not legs:
        print("\nAll legs failed.")
        return 1

    # RTDE payload preview — what ``execute_plan(method='moveJ_path')``
    # would send for EACH leg if wired to the real arm. Sparse-sampled
    # to N rows + per-row speed/accel/blend, matching
    # ``execute._execute_moveJ_path``.
    print()
    print("=" * 72)
    print("  RTDE previews — what would be sent to the controller per leg")
    print("=" * 72)
    print(f"  Sample count per leg : {ARC_RTDE_N_WAYPOINTS}")
    print(f"  Per-row format       : [q1..q6, speed, accel, blend]  "
          "(speed=1.00 rad/s, accel=1.40 rad/s², blend=0.0200 m, "
          "terminal blend=0)")
    print()

    # Per-leg summary table first (lets you scan KTO vs spline / clearance
    # at a glance before reading the joint dumps).
    print("  Leg                                       planner               "
          "duration   clearance")
    print("  " + "-" * 78)
    for label, plan in legs:
        clr = (plan.min_clearance_m * 1000
               if np.isfinite(plan.min_clearance_m) else float("nan"))
        clr_str = f"{clr:+6.1f} mm" if np.isfinite(clr) else "    n/a"
        print(f"  {label:42s}{plan.metadata.get('planner', '?'):18s}  "
              f"{plan.duration_s:5.2f} s    {clr_str}")
    print()

    # Then the full moveJ(path=...) joint dump for each leg.
    for label, plan in legs:
        _print_movej_payload(plan, label, ARC_RTDE_N_WAYPOINTS)

    print()
    print(f"Planned {len(legs)} legs successfully.")

    if cycle:
        # One-shot animation in order, then hold final pose. Useful for
        # headless / scripted runs where the interactive controls aren't
        # needed.
        for label, plan in legs:
            print(f"[anim] {label}  ({plan.duration_s:.2f}s plan, "
                  f"replay {animate_seconds:.1f}s)")
            q0 = np.asarray(
                plan.trajectory.value(plan.trajectory.start_time())
            ).flatten()
            plant.SetPositions(plant_ctx, q0)
            diagram.ForcedPublish(sim_ctx)
            _animate_plan(diagram, plant, sim_ctx, plan, animate_seconds)
        print("\nDone — holding final pose. Ctrl-C to exit.")
        try:
            while True:
                time.sleep(0.5)
        except KeyboardInterrupt:
            pass
        return 0

    # Default: interactive replay. Plan once, scene stays loaded —
    # replay any leg as many times as you want from the Meshcat panel.
    return _run_interactive(meshcat, diagram, plant, sim_ctx, legs,
                            animate_seconds)


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--animate-seconds", type=float, default=3.5,
                    help="Replay duration per leg (s).")
    ap.add_argument("--cycle", action="store_true",
                    help="Play all legs once in order, then hold "
                         "(no interactive controls). Default is "
                         "interactive Meshcat replay.")
    ap.add_argument("--show-arc-markers", action="store_true",
                    help="Show the 14 small spheres marking the arc "
                         "intermediate waypoints. Off by default — "
                         "they add browser-side render load.")
    args = ap.parse_args()
    raise SystemExit(main(animate_seconds=args.animate_seconds,
                          cycle=args.cycle,
                          show_arc_markers=args.show_arc_markers))
