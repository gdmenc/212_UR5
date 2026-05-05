"""Verify RRT-Connect on the planning scene.

Builds the scene, picks two test cases (one easy, one obstructed), runs
RRT-Connect on each, prints stats, and animates the result in Meshcat
behind a slider so you can switch between the cases interactively.

Run::

    python3.11 -m control_scripts.planning.verify_rrt
    python3.11 -m control_scripts.planning.verify_rrt --case obstructed
    python3.11 -m control_scripts.planning.verify_rrt --animate-seconds 5

Test cases
----------

  easy        — sim HOME → a hover above the outside-pick location.
                Spline can do this; RRT should also do it trivially
                via the direct-connect early-out.
  obstructed  — sim HOME → a configuration on the FAR side of the
                microwave. The direct joint-space line probably plows
                through the microwave housing; RRT must route around.

For the obstructed case we deliberately pick a goal q whose straight-
line path from HOME crosses the microwave geometry, which forces the
planner to actually grow trees instead of returning the direct shot.
"""

from __future__ import annotations

import argparse
import time
from typing import Dict, List, Tuple

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
from .rrt import (
    RRTFailure, build_planning_scene, make_collision_checker,
    path_length, rrt_connect, shortcut_path, _arm_position_indices,
)
from .transit import _arm_model_instance


PLANNING_ARM = "ur_right"


# ---------------------------------------------------------------------------
#  Test cases
# ---------------------------------------------------------------------------


def _case_easy(plant) -> Tuple[np.ndarray, np.ndarray]:
    """sim HOME → a hover-altitude q above the outside-pick location.

    A straight cubic spline already handles this transit; we use it to
    confirm RRT's "direct connect" early-out fires and produces a
    2-node path."""
    home = default_home_q(plant)
    arm_idx = _arm_position_indices(plant, _arm_model_instance(plant, PLANNING_ARM))
    goal = home.copy()
    # Reach pose: shoulder pan slightly forward, lift down a bit, elbow bent
    goal[arm_idx] = np.radians([-90, -110, -70, -90, -90, 0])
    return home, goal


def _case_obstructed(plant) -> Tuple[np.ndarray, np.ndarray]:
    """sim HOME → a config whose straight-line C-space edge from HOME
    is in collision (verified empirically).

    Picked by sampling random goal q's against the collision checker
    and selecting one where the goal config IS collision-free but the
    straight-line edge from HOME is NOT — i.e., the linear joint-space
    interpolation between HOME and goal sweeps the arm through the
    microwave/table/itself. Forces RRT to actually grow trees."""
    home = default_home_q(plant)
    arm_idx = _arm_position_indices(plant, _arm_model_instance(plant, PLANNING_ARM))
    goal = home.copy()
    goal[arm_idx] = np.radians([60.6, -10.4, 23.5, 95.4, 48.5, 19.3])
    return home, goal


CASES = {"easy": _case_easy, "obstructed": _case_obstructed}


# ---------------------------------------------------------------------------
#  Visualizer
# ---------------------------------------------------------------------------


def _interpolate_path(path_full: List[np.ndarray], n_per_segment: int = 25
                      ) -> List[np.ndarray]:
    """Linearly interpolate a piecewise path into a dense q sequence
    suitable for animation (RRT output is typically jagged with long
    segments — interpolating gives the eye something smooth to track)."""
    out: List[np.ndarray] = []
    for q0, q1 in zip(path_full[:-1], path_full[1:]):
        for s in np.linspace(0.0, 1.0, n_per_segment, endpoint=False):
            out.append(q0 + s * (q1 - q0))
    out.append(path_full[-1].copy())
    return out


def _animate_path(diagram, plant, sim_ctx, path_full, *, seconds: float):
    """Replay the path at constant time across the dense interpolation."""
    plant_ctx = plant.GetMyMutableContextFromRoot(sim_ctx)
    dense = _interpolate_path(path_full)
    if not dense:
        return
    t0 = time.time()
    while True:
        elapsed = time.time() - t0
        if elapsed >= seconds:
            break
        s = min(1.0, elapsed / seconds)
        idx = min(int(s * (len(dense) - 1)), len(dense) - 1)
        plant.SetPositions(plant_ctx, dense[idx])
        diagram.ForcedPublish(sim_ctx)
        time.sleep(0.02)
    plant.SetPositions(plant_ctx, dense[-1])
    diagram.ForcedPublish(sim_ctx)


# ---------------------------------------------------------------------------
#  Entry point
# ---------------------------------------------------------------------------


def _plan_case(case_name: str, max_iters: int) -> Tuple[List[np.ndarray], Dict]:
    """Build the (planner-side) scene, run RRT-Connect for one case,
    return (smoothed path, stats)."""
    diagram_p, plant_p, _, _ = build_planning_scene()
    home, goal = CASES[case_name](plant_p)
    checker = make_collision_checker(diagram_p, plant_p, PLANNING_ARM)

    t0 = time.time()
    plan = rrt_connect(
        checker, home, goal,
        planning_arm_name=PLANNING_ARM,
        max_iters=max_iters,
    )
    rrt_ms = (time.time() - t0) * 1000
    raw_len = path_length(plan.path_full, plan.arm_indices)
    raw_nodes = len(plan.path_full)

    t0 = time.time()
    smoothed = shortcut_path(checker, plan.path_full, attempts=200)
    smooth_ms = (time.time() - t0) * 1000
    smoothed_len = path_length(smoothed, plan.arm_indices)

    stats = {
        "case": case_name,
        "planner": plan.metadata.get("planner"),
        "iterations": plan.iterations,
        "tree_sizes": plan.tree_sizes,
        "rrt_ms": rrt_ms,
        "smooth_ms": smooth_ms,
        "raw_nodes": raw_nodes,
        "smoothed_nodes": len(smoothed),
        "raw_path_length_rad": raw_len,
        "smoothed_path_length_rad": smoothed_len,
    }
    return smoothed, stats


def _print_stats(stats: Dict) -> None:
    print(f"  case             : {stats['case']}")
    print(f"  planner returned : {stats['planner']}")
    print(f"  RRT iterations   : {stats['iterations']}")
    print(f"  tree sizes       : start={stats['tree_sizes'][0]}, "
          f"goal={stats['tree_sizes'][1]}")
    print(f"  RRT-Connect time : {stats['rrt_ms']:6.1f} ms")
    print(f"  smoothing time   : {stats['smooth_ms']:6.1f} ms")
    print(f"  nodes            : {stats['raw_nodes']} → "
          f"{stats['smoothed_nodes']} (after shortcut)")
    print(f"  joint path length: {stats['raw_path_length_rad']:.2f} → "
          f"{stats['smoothed_path_length_rad']:.2f} rad")


def main(case: str, animate_seconds: float, max_iters: int) -> int:
    if case not in CASES and case != "all":
        raise SystemExit(f"unknown case {case!r}; pick from {list(CASES)} or 'all'")

    cases_to_run = list(CASES) if case == "all" else [case]

    # Plan all cases up front (each rebuilds its own RobotDiagram for
    # the checker side; cheap).
    case_results: Dict[str, Tuple[List[np.ndarray], Dict]] = {}
    for c in cases_to_run:
        print(f"\n=== planning case: {c} ===")
        try:
            smoothed, stats = _plan_case(c, max_iters)
        except RRTFailure as exc:
            print(f"  FAILED: {exc}")
            return 1
        _print_stats(stats)
        case_results[c] = (smoothed, stats)

    # --- Visualization ---
    meshcat = StartMeshcat()
    print(f"\n[verify_rrt] Meshcat → {meshcat.web_url()}")

    scene = build_scene(meshcat=meshcat)
    diagram, plant = scene.diagram, scene.plant
    sim = Simulator(diagram)
    sim.Initialize()
    sim_ctx = sim.get_mutable_context()
    plant_ctx = plant.GetMyMutableContextFromRoot(sim_ctx)
    plant.SetPositions(plant_ctx, default_home_q(plant))
    diagram.ForcedPublish(sim_ctx)

    # Drop a marker at the FK'd start and goal TCP for each case.
    arm_idx = _arm_position_indices(plant, _arm_model_instance(plant, PLANNING_ARM))
    tcp_frame = plant.GetFrameByName("tcp_right")
    for i, (case_name, (smoothed, _)) in enumerate(case_results.items()):
        for tag, q, color in (
            ("start", smoothed[0],  Rgba(0.6, 0.6, 0.6, 0.8)),
            ("goal",  smoothed[-1], Rgba(0.95, 0.6, 0.1, 0.9)),
        ):
            plant.SetPositions(plant_ctx, q)
            xyz = np.asarray(tcp_frame.CalcPoseInWorld(plant_ctx).translation())
            meshcat.SetObject(f"/{case_name}/{tag}", Sphere(0.012), color)
            meshcat.SetTransform(f"/{case_name}/{tag}", RigidTransform(xyz))

    # Reset to home so the slider start state isn't whichever case we
    # just FK'd.
    plant.SetPositions(plant_ctx, default_home_q(plant))
    diagram.ForcedPublish(sim_ctx)

    # Interactive controls if multiple cases, otherwise auto-play once
    # and hold.
    case_list = list(case_results)
    if len(case_list) == 1:
        print(f"\n[play] case '{case_list[0]}' "
              f"(animating {animate_seconds:.1f}s)")
        _animate_path(diagram, plant, sim_ctx, case_results[case_list[0]][0],
                      seconds=animate_seconds)
        print("Done — held final pose. Ctrl-C to exit.")
        try:
            while True:
                time.sleep(0.5)
        except KeyboardInterrupt:
            return 0
        return 0

    # Multi-case interactive: slider + Replay/Prev/Next/Quit.
    n = len(case_list)
    meshcat.AddSlider("case_idx", min=1, max=n, step=1, value=1)
    meshcat.AddButton("Replay")
    meshcat.AddButton("Prev")
    meshcat.AddButton("Next")
    meshcat.AddButton("Quit")

    def play(idx):
        name = case_list[idx - 1]
        smoothed, stats = case_results[name]
        plant.SetPositions(plant_ctx, smoothed[0])
        diagram.ForcedPublish(sim_ctx)
        print(f"[play] case {idx} '{name}' ({stats['planner']}, "
              f"{stats['smoothed_nodes']} nodes, "
              f"animating {animate_seconds:.1f}s)")
        _animate_path(diagram, plant, sim_ctx, smoothed, seconds=animate_seconds)

    last_idx = 0
    last_replay = meshcat.GetButtonClicks("Replay")
    last_prev = meshcat.GetButtonClicks("Prev")
    last_next = meshcat.GetButtonClicks("Next")

    print("\nInteractive mode — Meshcat controls panel:")
    print(f"  Slider 'case_idx' picks index 1..{n}")
    print(f"  Prev / Next steps; Replay re-plays current case; Quit exits")

    try:
        while True:
            if meshcat.GetButtonClicks("Quit") > 0:
                break
            cur = int(round(meshcat.GetSliderValue("case_idx")))
            replay_clicks = meshcat.GetButtonClicks("Replay")
            prev_clicks = meshcat.GetButtonClicks("Prev")
            next_clicks = meshcat.GetButtonClicks("Next")
            if next_clicks > last_next:
                cur = ((cur - 1 + 1) % n) + 1
                meshcat.SetSliderValue("case_idx", cur)
            elif prev_clicks > last_prev:
                cur = ((cur - 1 - 1) % n) + 1
                meshcat.SetSliderValue("case_idx", cur)
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
            meshcat.DeleteSlider("case_idx")
        except Exception:
            pass
    return 0


if __name__ == "__main__":
    ap = argparse.ArgumentParser(
        description="Verify RRT-Connect on the planning scene "
                    "(easy + obstructed cases, interactive Meshcat replay)."
    )
    ap.add_argument(
        "--case", choices=list(CASES) + ["all"], default="all",
        help="Which test case(s) to plan and animate.",
    )
    ap.add_argument(
        "--animate-seconds", type=float, default=4.0,
        help="Seconds per animation playthrough.",
    )
    ap.add_argument(
        "--max-iters", type=int, default=2000,
        help="RRT-Connect iteration budget.",
    )
    args = ap.parse_args()
    raise SystemExit(main(case=args.case, animate_seconds=args.animate_seconds,
                          max_iters=args.max_iters))
