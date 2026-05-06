"""Meshcat playback for ``TransitPlan`` segments + interactive leg stepper.

Two helpers, both extracted from the near-identical ``_animate_plan`` /
``_run_interactive`` loops in ``dryrun_pick_plate.py`` and
``dryrun_open_microwave.py`` (the open-microwave version had Play All
and cleaner Prev/Next wraparound, so that's the canonical one here).

These functions are deliberately small and stateless — they take a
prebuilt diagram + plant + context and don't construct anything on
their own. Tasks that want a sim ``--mode`` can build a sim scene
(via ``World.build_sim_scene``), call ``plan_transit`` to produce
legs, and hand both to ``run_interactive_legs``.
"""

from __future__ import annotations

import time
from typing import List, Tuple

import numpy as np

from .transit import TransitPlan


# Meshcat publish rate during animation. 50 Hz balances motion
# smoothness against the WebSocket bandwidth needed to push the full
# URDF arm meshes + microwave on every frame.
_ANIM_PUBLISH_HZ = 50.0
_ANIM_PUBLISH_DT = 1.0 / _ANIM_PUBLISH_HZ


def animate_plan(
    diagram,
    plant,
    sim_ctx,
    plan: TransitPlan,
    seconds: float,
) -> None:
    """Drive ``plant`` through ``plan.trajectory`` over ``seconds`` of
    wall time, publishing meshcat at ~50 Hz.

    Mutates the plant's position state inside ``sim_ctx``. Returns when
    the animation finishes. ``seconds`` is replay time, not the
    trajectory's planned duration — pass a smaller value to fast-forward
    or a larger one to slow-mo.
    """
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


def run_interactive_legs(
    meshcat,
    diagram,
    plant,
    sim_ctx,
    legs: List[Tuple[str, TransitPlan]],
    *,
    animate_seconds: float = 2.5,
) -> int:
    """Hold the meshcat viewer alive; let the user step through legs.

    Adds a 'leg' slider (1..N) plus Replay / Prev / Next / Play All /
    Quit buttons. Picking a new leg auto-plays it once; the arm rests
    at the leg's terminal pose between actions. No diagram rebuild
    between plays — switches are near-instant.

    Blocks until Quit (button) or Ctrl-C. Returns 0 on clean exit.
    """
    plant_ctx = plant.GetMyMutableContextFromRoot(sim_ctx)
    n = len(legs)
    if n == 0:
        print("[preview] no legs to step through")
        return 0

    meshcat.AddSlider("leg", min=1, max=n, step=1, value=1)
    meshcat.AddButton("Replay")
    meshcat.AddButton("Prev")
    meshcat.AddButton("Next")
    meshcat.AddButton("Play All")
    meshcat.AddButton("Quit")

    def play(idx: int) -> None:
        label, plan = legs[idx - 1]
        # Reset the arm to the leg's start q so each animation is the
        # full motion regardless of what was on screen before.
        q0 = np.asarray(
            plan.trajectory.value(plan.trajectory.start_time())
        ).flatten()
        plant.SetPositions(plant_ctx, q0)
        diagram.ForcedPublish(sim_ctx)
        print(f"[play] leg {idx}: {label}  "
              f"(plan {plan.duration_s:.2f}s, replay {animate_seconds:.1f}s)")
        animate_plan(diagram, plant, sim_ctx, plan, animate_seconds)

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
    print("  Prev / Next steps through")
    print("  Replay re-plays current leg")
    print("  Play All cycles through every leg in order")
    print("  Quit exits cleanly (or Ctrl-C)")

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
