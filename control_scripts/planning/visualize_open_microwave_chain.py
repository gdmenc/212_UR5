"""Step through the recorded open-microwave waypoints in Meshcat.

Loads ``logs/waypoints/ur_left_open_microwave_1.json`` and lets you
flip through the 6 recorded snapshots one at a time using a slider in
the Meshcat controls panel. For each snapshot we:

  * SetPositions on the planning arm to the recorded ``joints_rad``
  * drop a small marker at the recorded TCP world position
  * print the snapshot name + ‖q − previous_q‖ delta

This bypasses ``plan_transit`` entirely so it's guaranteed to display
exactly what was recorded — useful when the planner-driven sim mode is
hitting cold-start IK issues.

Run::

    python3.11 -m control_scripts.planning.visualize_open_microwave_chain
"""

from __future__ import annotations

import json
import time
from pathlib import Path
from typing import List, Optional

import numpy as np
from pydrake.geometry import Rgba, Sphere, StartMeshcat
from pydrake.math import RigidTransform
from pydrake.systems.analysis import Simulator

from . import default_home_q
from .build_scene import build_scene
from .transit import _arm_model_instance, _arm_position_indices


JSON_PATH = (
    Path(__file__).resolve().parents[2]
    / "logs" / "waypoints" / "ur_left_open_microwave_1.json"
)


def _load_open_microwave_json() -> dict:
    raw = JSON_PATH.read_text()
    try:
        return json.loads(raw)
    except json.JSONDecodeError:
        return json.loads("{\n" + raw)


def main(arm_name: str = "ur_left") -> int:
    data = _load_open_microwave_json()
    snapshots = [s for s in data["snapshots"] if s.get("joints_rad")]
    if not snapshots:
        print(f"No snapshots with joints_rad in {JSON_PATH}")
        return 1

    meshcat = StartMeshcat()
    print(f"\nMeshcat → {meshcat.web_url()}")
    scene = build_scene(meshcat=meshcat)
    diag = scene.diagram
    plant = scene.plant
    sim = Simulator(diag)
    sim.Initialize()
    sim_ctx = sim.get_mutable_context()
    plant_ctx = plant.GetMyMutableContextFromRoot(sim_ctx)

    arm_inst = _arm_model_instance(plant, arm_name)
    arm_idx = _arm_position_indices(plant, arm_inst)
    home_full = default_home_q(plant)

    short = arm_name.removeprefix("ur_")
    tcp_frame = plant.GetFrameByName(f"tcp_{short}")

    # Pre-compute world TCP for each recorded q via Drake FK so the marker
    # tracks where the SIMULATION puts the gripper at that q, not where
    # the controller said it was. (We expect them to match — see
    # verify_ik_recorded.)
    full_qs: List[np.ndarray] = []
    tcp_world: List[np.ndarray] = []
    names: List[str] = []
    for snap in snapshots:
        q_arm = np.asarray(snap["joints_rad"], dtype=float)
        q_full = home_full.copy()
        q_full[arm_idx] = q_arm
        plant.SetPositions(plant_ctx, q_full)
        X = tcp_frame.CalcPoseInWorld(plant_ctx)
        full_qs.append(q_full)
        tcp_world.append(np.asarray(X.translation()))
        names.append(snap["name"])

    # Plant back to HOME for the initial render.
    plant.SetPositions(plant_ctx, home_full)
    diag.ForcedPublish(sim_ctx)

    # Drop a numbered marker at each recorded TCP location.
    colors = [
        Rgba(0.6, 0.6, 0.6, 0.8),     # pregrasp (gray)
        Rgba(0.4, 0.7, 0.4, 0.9),     # graspopen (green)
        Rgba(0.95, 0.6, 0.1, 0.9),    # graspclose (orange)
        Rgba(0.2, 0.4, 0.95, 0.9),    # opendoor (blue)
        Rgba(0.5, 0.2, 0.95, 0.9),    # opengraspopendoor (purple)
        Rgba(0.95, 0.2, 0.5, 0.9),    # slideoutdoorhandle (pink)
    ]
    for i, (xyz, name) in enumerate(zip(tcp_world, names)):
        c = colors[i] if i < len(colors) else Rgba(0.7, 0.7, 0.7, 0.9)
        meshcat.SetObject(f"/recorded/{i}_{name}", Sphere(0.014), c)
        meshcat.SetTransform(f"/recorded/{i}_{name}", RigidTransform(xyz))

    n = len(full_qs)
    meshcat.AddSlider("snapshot", min=1, max=n, step=1, value=1)
    meshcat.AddButton("Prev")
    meshcat.AddButton("Next")
    meshcat.AddButton("Quit")

    print()
    print(f"  {n} recorded snapshots loaded:")
    for i, name in enumerate(names):
        print(f"    [{i+1}] {name}")
    print()
    print("  Use the 'snapshot' slider, or Prev/Next buttons.")
    print("  TCP markers are colored by snapshot.")

    last_idx = 0
    last_prev = meshcat.GetButtonClicks("Prev")
    last_next = meshcat.GetButtonClicks("Next")
    last_q: Optional[np.ndarray] = None
    try:
        while meshcat.GetButtonClicks("Quit") == 0:
            cur = int(round(meshcat.GetSliderValue("snapshot")))
            prev_clicks = meshcat.GetButtonClicks("Prev")
            next_clicks = meshcat.GetButtonClicks("Next")
            if next_clicks > last_next:
                cur = ((cur - 1 + 1) % n) + 1
                meshcat.SetSliderValue("snapshot", cur)
            elif prev_clicks > last_prev:
                cur = ((cur - 1 - 1) % n) + 1
                meshcat.SetSliderValue("snapshot", cur)
            last_prev, last_next = prev_clicks, next_clicks

            if cur != last_idx:
                q_full = full_qs[cur - 1]
                plant.SetPositions(plant_ctx, q_full)
                diag.ForcedPublish(sim_ctx)
                q_arm = q_full[arm_idx]
                if last_q is None:
                    delta = ""
                else:
                    dq = float(np.linalg.norm(q_arm - last_q))
                    delta = f"  ‖Δq from prev‖={dq*1000:.0f} mrad"
                print(f"  → [{cur}] {names[cur-1]:<24} "
                      f"q={np.round(np.degrees(q_arm), 1)}°{delta}")
                last_idx = cur
                last_q = q_arm
            time.sleep(0.05)
    except KeyboardInterrupt:
        print("\nClosing.")
    finally:
        for nm in ("Prev", "Next", "Quit"):
            try: meshcat.DeleteButton(nm)
            except Exception: pass
        try: meshcat.DeleteSlider("snapshot")
        except Exception: pass
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
