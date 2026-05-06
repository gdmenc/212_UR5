"""Smoke test: plan a transit, animate it in Meshcat, no RTDE.

Builds the full planning scene, picks a start TCP pose and an end TCP
pose in task frame, runs ``plan_transit`` with one constraint enabled
at a time, and replays the resulting trajectory in Meshcat.

Run::

    python3.11 -m control_scripts.planning.smoke_transit
    python3.11 -m control_scripts.planning.smoke_transit --constraints simple
    python3.11 -m control_scripts.planning.smoke_transit --constraints fix_z

No real arm needed; use this to verify a plan is sane before pointing
``execute.py`` at the hardware.
"""

from __future__ import annotations

import argparse
import time
from typing import Optional

import numpy as np
from pydrake.geometry import StartMeshcat

from ..calibration import HOME_Q_RAD_LEFT
from ..util.poses import Pose
from ..util.rotations import Rotation
from .build_scene import build_scene
from .transit import InfeasiblePlanError, plan_transit


# Two task-frame TCP offsets relative to the home pose. We FK
# HOME_Q_RAD_LEFT inside _run() to get the home TCP pose, then pick
# start/end as small XY translations around it — guarantees the
# poses are reachable since HOME is the seed.
START_OFFSET_M = np.array([-0.05, +0.00, 0.0])
END_OFFSET_M   = np.array([+0.05, +0.10, 0.0])


def _animate(diagram, plant, traj, meshcat, *, seconds: float = 4.0):
    """Replay the trajectory once at real time. No simulation — just
    SetPositions and forced publish."""
    diag_ctx = diagram.CreateDefaultContext()
    plant_ctx = plant.GetMyMutableContextFromRoot(diag_ctx)

    t0_real = time.time()
    t0_traj = traj.start_time()
    t1_traj = traj.end_time()
    while True:
        elapsed = time.time() - t0_real
        if elapsed >= seconds:
            break
        s = min(1.0, elapsed / seconds)
        t = t0_traj + s * (t1_traj - t0_traj)
        q = np.asarray(traj.value(t)).flatten()
        plant.SetPositions(plant_ctx, q)
        diagram.ForcedPublish(diag_ctx)
        time.sleep(0.02)


def _run(constraint_set: str, animate_seconds: float,
         gripper_mode: str = "closed"):
    meshcat = StartMeshcat()
    print(f"[smoke] Meshcat → {meshcat.web_url()}")

    scene = build_scene(meshcat=meshcat, robotiq_mode=gripper_mode)
    diagram, plant = scene.diagram, scene.plant
    diag_ctx = diagram.CreateDefaultContext()
    plant_ctx = plant.GetMyMutableContextFromRoot(diag_ctx)

    # Seed the planning arm at HOME and FK to get the home TCP pose;
    # define start/end as offsets around it so IK has trivial seed → soln.
    from .transit import _arm_model_instance, _tcp_frame
    left = _arm_model_instance(plant, "ur_left")
    plant.SetPositions(plant_ctx, left, np.asarray(HOME_Q_RAD_LEFT))
    home_X_world = plant.EvalBodyPoseInWorld(
        plant_ctx, plant.GetBodyByName("wrist_3_link", left),
    )
    home_xyz = np.asarray(home_X_world.translation())
    home_R = home_X_world.rotation()
    home_rotvec = home_R.ToAngleAxis()
    start_pose = Pose(
        translation=home_xyz + START_OFFSET_M,
        rotation=Rotation.from_matrix(home_R.matrix()),
    )
    end_pose = Pose(
        translation=home_xyz + END_OFFSET_M,
        rotation=Rotation.from_matrix(home_R.matrix()),
    )
    print(f"[smoke] home TCP at {np.round(home_xyz, 3)} (task)")
    print(f"[smoke] start / end TCP: "
          f"{np.round(start_pose.translation, 3)} → "
          f"{np.round(end_pose.translation, 3)}")

    # In a real task this dict would be ``{name: arm.receive.getActualQ()}``
    # for both arms. Here we hand-pick HOME for left, zeros for right
    # (right arm's HOME isn't measured yet — see calibration.py).
    current_q = {
        "ur_left": np.asarray(HOME_Q_RAD_LEFT),
        "ur_right": np.zeros(6),
    }

    common_kwargs = dict(
        plant=plant,
        plant_context=plant_ctx,
        arm="ur_left",
        waypoints=[start_pose, end_pose],
        current_q=current_q,
    )

    if constraint_set == "simple":
        kwargs = dict(
            avoid_collisions=False,
            avoid_arm_singularity=False,
            self_collision=False,
            min_z_task=None,
            max_tcp_linear_speed_m_per_s=None,
        )
        label = "simple cubic spline (no constraints)"
    elif constraint_set == "kto_minimal":
        # KTO path with everything turned off except endpoints.
        # If this fails, the problem is in the KTO setup itself
        # (initial guess, partner-arm pin, joint bounds), not a path
        # constraint. Force KTO via fix_z=None but tag a trivial
        # workspace box to enable KTO routing.
        kwargs = dict(
            avoid_collisions=False,
            avoid_arm_singularity=False,
            self_collision=False,
            min_z_task=None,
            max_tcp_linear_speed_m_per_s=None,
            stay_in_workspace_box=((-2.0, -2.0, -2.0), (2.0, 2.0, 2.0)),  # huge box → trivial
        )
        label = "kto with trivial workspace box (smoke-only)"
    elif constraint_set == "kto_only_collisions":
        kwargs = dict(
            avoid_collisions=True, avoid_arm_singularity=False,
            self_collision=False, min_z_task=None,
            max_tcp_linear_speed_m_per_s=None,
        )
        label = "kto + only avoid_collisions"
    elif constraint_set == "kto_only_floor":
        kwargs = dict(
            avoid_collisions=False, avoid_arm_singularity=False,
            self_collision=False, min_z_task=0.02,
            max_tcp_linear_speed_m_per_s=None,
        )
        label = "kto + only min_z_task=0.02"
    elif constraint_set == "default":
        kwargs = dict()   # all defaults — KTO with safety constraints
        label = "default constraints (collisions + arm singularity + z floor)"
    elif constraint_set == "fix_z":
        # Use the home TCP z as the target so endpoints are consistent
        # with the constraint. Otherwise the constraint contradicts
        # the endpoint position constraints and KTO reports infeasible.
        kwargs = dict(
            fix_z_task=float(home_xyz[2]),
            z_tolerance_m=0.01,
            avoid_collisions=False,        # turn off the auto-on collision
            avoid_arm_singularity=False,
            self_collision=False,
            min_z_task=None,
        )
        label = f"fix_z_task={home_xyz[2]:.3f} m ± 10 mm"
    elif constraint_set == "fix_orient":
        kwargs = dict(
            fix_orientation=start_pose.rotation,
            avoid_collisions=False,
            avoid_arm_singularity=False,
            self_collision=False,
            min_z_task=None,
        )
        label = "fix tool orientation throughout"
    else:
        raise SystemExit(f"unknown constraint set: {constraint_set!r}")

    print(f"[smoke] planning: {label}")
    try:
        plan = plan_transit(**common_kwargs, **kwargs)
    except InfeasiblePlanError as exc:
        print(f"[smoke] PLAN INFEASIBLE: {exc}")
        return

    print(f"[smoke] plan ok — {plan.metadata.get('planner', '?')}, "
          f"duration {plan.duration_s:.2f} s, "
          f"clearance {plan.min_clearance_m * 1000:.1f} mm")

    print(f"[smoke] animating for {animate_seconds:.1f} s in Meshcat …")
    _animate(diagram, plant, plan.trajectory, meshcat, seconds=animate_seconds)
    print("[smoke] done.  Ctrl-C to exit, or refresh the browser to inspect.")
    try:
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        return


def main() -> None:
    ap = argparse.ArgumentParser(
        description="Plan a transit, replay it in Meshcat (no RTDE)."
    )
    ap.add_argument(
        "--constraints",
        choices=(
            "simple", "default",
            "kto_minimal", "kto_only_collisions", "kto_only_floor",
            "fix_z", "fix_orient",
        ),
        default="default",
    )
    ap.add_argument("--animate-seconds", type=float, default=4.0)
    ap.add_argument(
        "--gripper-mode", choices=["closed", "open"], default="closed",
        help="Robotiq 2F-85 finger configuration to load (default: closed).",
    )
    args = ap.parse_args()
    _run(args.constraints, args.animate_seconds, args.gripper_mode)


if __name__ == "__main__":
    main()
