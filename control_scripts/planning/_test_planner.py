"""Non-interactive planner test sweep.

Runs ``plan_transit`` against the current scene with every supported
constraint configuration and reports pass/fail per case. No Meshcat,
no animation, no Ctrl-C wait — exits cleanly so it can be wired into
CI later.

Run::

    python3.11 -m control_scripts.planning._test_planner
"""

from __future__ import annotations

import sys
import time
import traceback
from typing import Callable, Dict, List

import numpy as np

from ..calibration import HOME_Q_RAD_LEFT
from ..util.poses import Pose
from ..util.rotations import Rotation
from . import default_home_q
from .build_scene import build_scene
from .transit import (
    InfeasiblePlanError,
    _arm_model_instance,
    plan_transit,
)


# Test poses derived inside _setup() once we have an FK plant.
START_OFFSET_M = np.array([-0.05, +0.00, 0.0])
END_OFFSET_M = np.array([+0.05, +0.10, 0.0])


def _setup():
    """Build the scene + extract a sane start/end pair around left HOME."""
    scene = build_scene()
    plant = scene.plant
    diag_ctx = scene.diagram.CreateDefaultContext()
    plant_ctx = plant.GetMyMutableContextFromRoot(diag_ctx)

    plant.SetPositions(plant_ctx, default_home_q(plant))

    left = _arm_model_instance(plant, "ur_left")
    home_X = plant.EvalBodyPoseInWorld(
        plant_ctx, plant.GetBodyByName("wrist_3_link", left)
    )
    home_xyz = np.asarray(home_X.translation())
    home_R = Rotation.from_matrix(home_X.rotation().matrix())

    start_pose = Pose(translation=home_xyz + START_OFFSET_M, rotation=home_R)
    end_pose = Pose(translation=home_xyz + END_OFFSET_M, rotation=home_R)

    current_q = {
        "ur_left": np.asarray(HOME_Q_RAD_LEFT),
        "ur_right": np.zeros(6),
    }

    return plant, plant_ctx, start_pose, end_pose, current_q, home_xyz


def _make_cases(home_xyz, start_pose):
    """List of (name, kwargs, expected_outcome) per constraint set."""
    return [
        (
            "simple",
            dict(
                avoid_collisions=False, avoid_arm_singularity=False,
                self_collision=False, min_z_task=None,
                max_tcp_linear_speed_m_per_s=None,
            ),
            "pass",
        ),
        (
            "kto_minimal",
            dict(
                avoid_collisions=False, avoid_arm_singularity=False,
                self_collision=False, min_z_task=None,
                max_tcp_linear_speed_m_per_s=None,
                stay_in_workspace_box=((-2.0, -2.0, -2.0), (2.0, 2.0, 2.0)),
            ),
            "pass",
        ),
        (
            "kto_only_floor",
            dict(
                avoid_collisions=False, avoid_arm_singularity=False,
                self_collision=False, min_z_task=0.02,
                max_tcp_linear_speed_m_per_s=None,
            ),
            "pass",
        ),
        (
            "fix_z",
            dict(
                fix_z_task=float(home_xyz[2]), z_tolerance_m=0.01,
                avoid_collisions=False, avoid_arm_singularity=False,
                self_collision=False, min_z_task=None,
            ),
            "pass",
        ),
        (
            "fix_orient",
            dict(
                fix_orientation=start_pose.rotation,
                avoid_collisions=False, avoid_arm_singularity=False,
                self_collision=False, min_z_task=None,
            ),
            "pass",
        ),
        (
            "kto_only_collisions",
            dict(
                avoid_collisions=True, avoid_arm_singularity=False,
                self_collision=False, min_z_task=None,
                max_tcp_linear_speed_m_per_s=None,
            ),
            "pass-or-known-fail",   # may fail due to home pose vs vention model
        ),
        (
            "default",
            dict(),
            "pass-or-known-fail",
        ),
    ]


def _run_case(plant, plant_ctx, start, end, current_q, kwargs):
    t0 = time.time()
    try:
        plan = plan_transit(
            plant=plant, plant_context=plant_ctx,
            arm="ur_left",
            waypoints=[start, end],
            current_q=current_q,
            **kwargs,
        )
    except InfeasiblePlanError as e:
        return ("INFEASIBLE", str(e)[:120], time.time() - t0, None)
    except Exception as e:
        return ("ERROR", f"{type(e).__name__}: {e}"[:120], time.time() - t0, None)
    return ("OK", "", time.time() - t0, plan)


def main() -> int:
    plant, plant_ctx, start, end, current_q, home_xyz = _setup()
    cases = _make_cases(home_xyz, start)

    print("=" * 78)
    print(f"  Planner test sweep — left arm transit, {len(cases)} cases")
    print(f"  Home TCP: {np.round(home_xyz, 3)}")
    print(f"  Start:    {np.round(start.translation, 3)}")
    print(f"  End:      {np.round(end.translation, 3)}")
    print("=" * 78)

    results: List[tuple] = []
    for name, kwargs, expected in cases:
        status, detail, dt, plan = _run_case(
            plant, plant_ctx, start, end, current_q, kwargs,
        )
        ok = status == "OK"
        symbol = "✓" if ok else ("⚠" if expected == "pass-or-known-fail" else "✗")
        suffix = ""
        if ok:
            suffix = (
                f"  [{plan.metadata.get('planner', '?')}, "
                f"{plan.duration_s:.2f} s, "
                f"clearance {plan.min_clearance_m * 1000:.1f} mm]"
            )
        elif detail:
            suffix = f"  →  {detail}"
        print(f"  {symbol} {name:25s} {status:11s} {dt:5.2f}s{suffix}")
        results.append((name, status, expected, plan))

    print("=" * 78)
    n_pass = sum(1 for _, s, _, _ in results if s == "OK")
    n_fail = len(results) - n_pass
    n_unexpected = sum(
        1 for _, s, exp, _ in results
        if s != "OK" and exp == "pass"
    )
    print(f"  total: {len(results)}   pass: {n_pass}   fail: {n_fail}   "
          f"unexpected fail: {n_unexpected}")
    return 0 if n_unexpected == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
