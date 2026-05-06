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

    Sliders:
      - ``leg``    picks index 1..N. Changing it auto-plays the new leg once.
      - ``scrub``  (0..1) jumps the arm to that fractional time along the
                   currently-selected leg. Drag to inspect any moment of the
                   trajectory frame-by-frame; useful for catching collisions.
      - ``speed``  (0.1..4.0) scales auto-playback duration. 1.0 ≈
                   ``animate_seconds`` of wall-clock; 0.25 → 4× slower.

    Buttons: Replay / Prev / Next / Play All / Quit.

    Blocks until Quit (button) or Ctrl-C. Returns 0 on clean exit.
    """
    plant_ctx = plant.GetMyMutableContextFromRoot(sim_ctx)
    n = len(legs)
    if n == 0:
        print("[preview] no legs to step through")
        return 0

    meshcat.AddSlider("leg", min=1, max=n, step=1, value=1)
    meshcat.AddSlider("scrub", min=0.0, max=1.0, step=0.005, value=0.0)
    meshcat.AddSlider("speed", min=0.1, max=4.0, step=0.1, value=1.0)
    meshcat.AddButton("Replay")
    meshcat.AddButton("Prev")
    meshcat.AddButton("Next")
    meshcat.AddButton("Play All")
    meshcat.AddButton("Quit")

    def _set_q_at_fraction(plan: TransitPlan, frac: float) -> None:
        t0, t1 = plan.trajectory.start_time(), plan.trajectory.end_time()
        t = t0 + max(0.0, min(1.0, frac)) * (t1 - t0)
        q = np.asarray(plan.trajectory.value(t)).flatten()
        plant.SetPositions(plant_ctx, q)
        diagram.ForcedPublish(sim_ctx)

    def play(idx: int) -> None:
        label, plan = legs[idx - 1]
        speed = max(0.1, float(meshcat.GetSliderValue("speed")))
        replay_s = animate_seconds / speed
        # Reset the arm to the leg's start q so each animation is the
        # full motion regardless of what was on screen before.
        _set_q_at_fraction(plan, 0.0)
        meshcat.SetSliderValue("scrub", 0.0)
        print(f"[play] leg {idx}: {label}  "
              f"(plan {plan.duration_s:.2f}s, replay {replay_s:.2f}s, "
              f"speed {speed:.2f}×)")
        animate_plan(diagram, plant, sim_ctx, plan, replay_s)
        meshcat.SetSliderValue("scrub", 1.0)

    def play_all() -> None:
        for i in range(1, n + 1):
            meshcat.SetSliderValue("leg", i)
            play(i)

    last_idx = 0
    last_replay = meshcat.GetButtonClicks("Replay")
    last_prev = meshcat.GetButtonClicks("Prev")
    last_next = meshcat.GetButtonClicks("Next")
    last_play_all = meshcat.GetButtonClicks("Play All")
    last_scrub = float(meshcat.GetSliderValue("scrub"))

    print()
    print("Interactive mode — Meshcat controls panel:")
    print(f"  Slider 'leg'   picks index 1..{n} (auto-plays on change)")
    print("  Slider 'scrub' (0..1) drags the arm to any fraction of the current leg")
    print("  Slider 'speed' (0.1..4.0) scales auto-playback rate")
    print("  Prev / Next steps through legs")
    print("  Replay re-plays current leg at current speed")
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
                last_scrub = float(meshcat.GetSliderValue("scrub"))
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
                last_scrub = float(meshcat.GetSliderValue("scrub"))
                continue

            # Manual scrub: jump the arm to the requested fraction of the
            # current leg's trajectory without re-playing.
            scrub = float(meshcat.GetSliderValue("scrub"))
            if abs(scrub - last_scrub) > 1e-6:
                _, plan = legs[cur - 1]
                _set_q_at_fraction(plan, scrub)
                last_scrub = scrub

            time.sleep(0.05)
    except KeyboardInterrupt:
        print("\nClosing.")
    finally:
        for name in ("Replay", "Prev", "Next", "Play All", "Quit"):
            try:
                meshcat.DeleteButton(name)
            except Exception:
                pass
        for name in ("leg", "scrub", "speed"):
            try:
                meshcat.DeleteSlider(name)
            except Exception:
                pass
    return 0
