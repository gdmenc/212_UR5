"""Shared core for ``pick_place_cup*`` tasks.

The ``pick_place_cup`` task started as a single file with module-level
constants pinning every knob (pick/place pose, angles, world, config).
As variants appeared (plain → pour station, cup → tray, cup-with-stick
→ microwave top), the file would have either grown a ``--scenario``
switch or been duplicated end-to-end. This module extracts the gnarly
helpers — motion-planned transits, sim leg planning, print/run wiring —
so each variant file only declares its own ``CupTaskCfg`` and a thin
``main`` that calls into ``run_main``.

A variant file looks like::

    from ._pick_place_cup_core import CupTaskCfg, parse_args, run_main

    CFG = CupTaskCfg(
        arm="ur_right",
        pick_pose_task=Pose(translation=[...]),
        place_pose_task=Pose(translation=[...]),
        final_xy=(0.4, 0.0),
        grasp_angle_rad=0.0,
        place_angle_rad=0.0,
        in_hand_kind="cup",
        world=World(...),
        config=PickPlaceConfig(...),
    )

    def main(dry=False, mode="real", motion_planning=True) -> int:
        return run_main(CFG, dry=dry, mode=mode, motion_planning=motion_planning)

    if __name__ == "__main__":
        raise SystemExit(main(**parse_args()))
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass, replace
from typing import Optional, Tuple

import numpy as np

from ..arm import ArmHandle
from ..config import PickPlaceConfig
from ..grasps.cup import cup_rim_grasp
from ..moves import transit_xy
from ..pick import pick
from ..place import place
from ..session import Session, default_session
from ..util.poses import Pose, pose_at_altitude
from ..util.rtde_convert import rtde_to_pose
from ..world import World


# ---------------------------------------------------------------------------
#  Per-variant config
# ---------------------------------------------------------------------------

@dataclass
class CupTaskCfg:
    """Bundle of per-variant knobs for a cup pick-and-place task.

    Treat as immutable — ``run_main`` rebuilds it via ``dataclasses.replace``
    when the CLI overrides ``use_motion_planning``."""

    arm: str
    pick_pose_task: Pose
    place_pose_task: Pose
    final_xy: Tuple[float, float]
    grasp_angle_rad: float
    place_angle_rad: float
    in_hand_kind: str
    """``"cup"`` welds a plain cup body to the gripper for collision checks
    on the carry leg; ``"cup_with_stick"`` welds the dowel-in-cup composite
    so the stick (~0.31 m above the cup base) is also planned around."""

    world: World
    config: PickPlaceConfig

    # ----- Shared motion-planning knobs (defaults match the original task)
    use_motion_planning: bool = True
    motion_plan_pre_pick_approach: bool = True
    motion_plan_rrt_fallback: bool = True
    motion_plan_rrt_max_iters: int = 2000
    motion_plan_rrt_shortcut_attempts: int = 20
    motion_plan_auto_fallback: bool = True
    carry_min_clearance_m: float = 0.005
    motion_plan_n_waypoints: int = 30
    motion_plan_blend_r_m: float = 0.005
    supports_sim: bool = True


# ---------------------------------------------------------------------------
#  Plan factories
# ---------------------------------------------------------------------------

def plan_pick(cfg: CupTaskCfg):
    return cup_rim_grasp(cfg.pick_pose_task, angle_rad=cfg.grasp_angle_rad)


def plan_place(cfg: CupTaskCfg) -> Pose:
    """TCP pose at release. Re-uses the rim-grasp factory at the place
    pose so the gripper orientation stays consistent between pick and
    place — easier on the wrist than re-solving a new orientation."""
    grasp_at_dest = cup_rim_grasp(
        cfg.place_pose_task, angle_rad=cfg.place_angle_rad,
    )
    return grasp_at_dest.grasp_pose


def plan_final_pose(cfg: CupTaskCfg, reference_pose: Pose) -> Pose:
    """Pose used only for the final XY transit after the cup is released."""
    return Pose(
        translation=[float(cfg.final_xy[0]), float(cfg.final_xy[1]), 0.0],
        rotation=reference_pose.rotation,
    )


def _print_plan(cfg: CupTaskCfg, grasp, place_pose: Pose) -> None:
    print("=" * 60)
    print("  Arm             :", cfg.arm)
    print("  In-hand kind    :", cfg.in_hand_kind)
    print("  Pick location   :", cfg.pick_pose_task.translation, "(task frame)")
    print("  Place location  :", cfg.place_pose_task.translation, "(task frame)")
    print("  Grasp angle     :", f"{np.degrees(cfg.grasp_angle_rad):+.0f}°")
    print("  Place angle     :", f"{np.degrees(cfg.place_angle_rad):+.0f}°")
    print("  Grasp pose      :", grasp.grasp_pose.translation, "(task frame)")
    print("  Place pose      :", place_pose.translation, "(task frame)")
    print("  Final XY        :", cfg.final_xy,
          f"(task frame @ z={cfg.config.transit_z})")
    print("  Transit Z       :", cfg.config.transit_z, "m")
    print("  Release aperture:", cfg.config.release_aperture_mm, "mm")
    print("  Gripper speeds  :",
          f"open={cfg.config.gripper_open_speed_pct}%,",
          f"close={cfg.config.gripper_close_speed_pct}%")
    print("  Grasp force     :", grasp.grasp_force, "N")
    print("=" * 60)


# ---------------------------------------------------------------------------
#  Live-arm helpers
# ---------------------------------------------------------------------------

def _current_q(session: Session) -> dict[str, np.ndarray]:
    """Current connected-arm joint positions for seeding Drake IK."""
    return {
        name: np.asarray(arm.receive.getActualQ(), dtype=float)
        for name, arm in session.arms.items()
    }


def _current_tcp_pose_task(arm: ArmHandle) -> Pose:
    """Read the real TCP pose and convert it into the shared task frame."""
    return arm.to_task(rtde_to_pose(arm.receive.getActualTCPPose()))


def _hover_before_pick(grasp, config: PickPlaceConfig) -> Pose:
    """The transit-altitude pose ``pick`` will target before descending."""
    return pose_at_altitude(grasp.grasp_pose, config.transit_z)


def _hover_before_place(place_pose: Pose, config: PickPlaceConfig) -> Pose:
    """The transit-altitude pose ``place`` will target before descending."""
    return pose_at_altitude(place_pose, config.transit_z)


def _build_planning_context(cfg: CupTaskCfg, *, attached: bool):
    """Build a planning scene for a single live planned transit. During
    the post-pick carry, weld the held object to the active TCP so
    collision checks include it."""
    leg_world = (
        replace(cfg.world, in_hand={cfg.arm: (cfg.in_hand_kind, None)})
        if attached else cfg.world
    )
    diagram, plant, _, _ = leg_world.build_planning_scene()
    root_context = diagram.CreateDefaultContext()
    plant_context = plant.GetMyMutableContextFromRoot(root_context)
    return diagram, plant, plant_context


def _planned_or_linear_transit(
    cfg: CupTaskCfg,
    session: Session,
    arm: ArmHandle,
    label: str,
    waypoints: list,
    *,
    attached: bool,
) -> bool:
    """Plan a free-space transit through ``waypoints`` and execute it.

    Routes through Drake ``plan_transit`` + ``execute_plan`` when
    ``cfg.use_motion_planning`` is True; falls back to sequential moveL
    via ``transit_xy`` otherwise OR on planner/exec failure when
    ``cfg.motion_plan_auto_fallback`` is True."""
    config = cfg.config
    print(f"\n→ planned transit: {label}")
    for i, wp in enumerate(waypoints):
        print(f"  wp {i}: xyz={np.round(wp.translation, 3)}")

    def _run_movel_fallback(reason: str) -> bool:
        print(f"  ➜ moveL fallback ({reason}); routing through "
              f"sequential transit_xy")
        for wp in waypoints[1:]:
            transit_xy(
                arm, wp, config.transit_z,
                config.transit_speed, config.transit_accel,
            )
        return True

    if not cfg.use_motion_planning:
        return _run_movel_fallback("use_motion_planning=False")

    from ..planning.execute import execute_plan
    from ..planning.transit import (
        InfeasiblePlanError, make_rtde_ik, plan_transit,
    )

    diagram, plant, plant_context = _build_planning_context(
        cfg, attached=attached,
    )
    try:
        plan = plan_transit(
            plant=plant,
            arm=cfg.arm,
            waypoints=waypoints,
            plant_context=plant_context,
            current_q=_current_q(session),
            use_rrt_fallback=cfg.motion_plan_rrt_fallback,
            rrt_diagram=diagram,
            rrt_max_iters=cfg.motion_plan_rrt_max_iters,
            rrt_shortcut_attempts=cfg.motion_plan_rrt_shortcut_attempts,
            min_clearance_m=cfg.carry_min_clearance_m if attached else 0.01,
            rtde_ik=make_rtde_ik(arm),
        )
    except InfeasiblePlanError as exc:
        print(f"  ✗ motion plan infeasible: {exc}")
        if cfg.motion_plan_auto_fallback:
            return _run_movel_fallback("planner raised InfeasiblePlanError")
        return False

    print(f"  planner={plan.metadata.get('planner')}  "
          f"duration={plan.duration_s:.2f}s  "
          f"clearance={plan.min_clearance_m * 1000:.1f}mm")

    result = execute_plan(
        plan, session,
        method="moveJ_path",
        n_waypoints=cfg.motion_plan_n_waypoints,
        blend_r_m=cfg.motion_plan_blend_r_m,
    )
    if not result.success:
        print(f"  ✗ planned transit execution failed: {result.reason}")
        if cfg.motion_plan_auto_fallback:
            return _run_movel_fallback(
                f"execute_plan failed: {result.reason}",
            )
        return False
    print("  ✓ planned transit reached.")
    return True


# ---------------------------------------------------------------------------
#  Sim-mode leg planning (no live RTDE)
# ---------------------------------------------------------------------------

def _plan_sim_legs(cfg: CupTaskCfg, grasp, place_pose: Pose):
    """Plan the same motion-planned segments the real path plans, but
    from synthetic start states.

    Three legs (when ``cfg.motion_plan_pre_pick_approach=True``):
      1. pre-pick approach — HOME (FK) → grasp hover.
      2. post-pick carry   — grasp hover (object in hand) → place hover.
      3. post-place return — place hover → final-XY hover.
    """
    from ..planning.transit import (
        InfeasiblePlanError,
        _arm_model_instance,
        _arm_position_indices,
        _tcp_frame,
        plan_transit,
        plan_transit_chained,
    )
    from ..util.rotations import Rotation as RotUtil

    config = cfg.config
    legs: list[tuple[str, object]] = []
    chained_arm_q = None
    prev_terminal_pose: Optional[Pose] = None

    # ---- Leg 1: pre-pick approach (no held object) ------------------
    if cfg.motion_plan_pre_pick_approach:
        approach_diagram, approach_plant, _, _ = cfg.world.build_planning_scene()
        approach_root = approach_diagram.CreateDefaultContext()
        approach_plant_ctx = approach_plant.GetMyMutableContextFromRoot(
            approach_root,
        )
        approach_arm_idx = _arm_position_indices(
            approach_plant, _arm_model_instance(approach_plant, cfg.arm),
        )
        approach_tcp_frame = _tcp_frame(
            approach_plant, _arm_model_instance(approach_plant, cfg.arm),
        )
        X = approach_tcp_frame.CalcPoseInWorld(approach_plant_ctx)
        home_pose = Pose(
            translation=np.asarray(X.translation()),
            rotation=RotUtil.from_matrix(X.rotation().matrix()),
        )
        approach_waypoints = [home_pose, _hover_before_pick(grasp, config)]
        print("\n[sim] planning pre-pick approach...")
        try:
            approach_plan = plan_transit(
                plant=approach_plant,
                arm=cfg.arm,
                waypoints=approach_waypoints,
                plant_context=approach_plant_ctx,
                use_rrt_fallback=cfg.motion_plan_rrt_fallback,
                rrt_diagram=approach_diagram,
                rrt_max_iters=cfg.motion_plan_rrt_max_iters,
                rrt_shortcut_attempts=cfg.motion_plan_rrt_shortcut_attempts,
                min_clearance_m=0.01,
            )
            legs.append(("pre-pick approach to grasp hover", approach_plan))
            chained_arm_q = np.asarray(
                approach_plan.trajectory.value(
                    approach_plan.trajectory.end_time(),
                )
            ).flatten()[approach_arm_idx]
            prev_terminal_pose = _hover_before_pick(grasp, config)
        except InfeasiblePlanError as exc:
            print(f"  ✗ pre-pick approach infeasible: {exc}")
            return legs

    # ---- Leg 2: post-pick carry (object in hand) --------------------
    carry_world = replace(
        cfg.world, in_hand={cfg.arm: (cfg.in_hand_kind, None)},
    )
    carry_diagram, carry_plant, _, _ = carry_world.build_planning_scene()
    carry_root = carry_diagram.CreateDefaultContext()
    carry_plant_ctx = carry_plant.GetMyMutableContextFromRoot(carry_root)
    carry_arm_idx = _arm_position_indices(
        carry_plant, _arm_model_instance(carry_plant, cfg.arm),
    )

    carry_waypoints = [
        _hover_before_pick(grasp, config),
        _hover_before_place(place_pose, config),
    ]
    try:
        carry_plan = plan_transit_chained(
            arm=cfg.arm,
            log_label="post-pick carry",
            prev_terminal_pose=prev_terminal_pose,
            chained_arm_q=chained_arm_q,
            plant=carry_plant,
            waypoints=carry_waypoints,
            plant_context=carry_plant_ctx,
            current_q=({cfg.arm: chained_arm_q}
                       if chained_arm_q is not None else None),
            use_rrt_fallback=cfg.motion_plan_rrt_fallback,
            rrt_diagram=carry_diagram,
            rrt_max_iters=cfg.motion_plan_rrt_max_iters,
            rrt_shortcut_attempts=cfg.motion_plan_rrt_shortcut_attempts,
            min_clearance_m=cfg.carry_min_clearance_m,
        )
    except InfeasiblePlanError as exc:
        print(f"  ✗ post-pick carry plan infeasible: {exc}")
        return legs
    legs.append(("post-pick carry to place hover", carry_plan))
    chained_arm_q = np.asarray(
        carry_plan.trajectory.value(carry_plan.trajectory.end_time())
    ).flatten()[carry_arm_idx]
    prev_terminal_pose = carry_waypoints[-1]

    # ---- Leg 3: post-place return → final XY hover -----------------
    return_diagram, return_plant, _, _ = cfg.world.build_planning_scene()
    return_root = return_diagram.CreateDefaultContext()
    return_plant_ctx = return_plant.GetMyMutableContextFromRoot(return_root)

    final_pose = pose_at_altitude(
        plan_final_pose(cfg, place_pose), config.transit_z,
    )
    return_waypoints = [
        _hover_before_place(place_pose, config),
        final_pose,
    ]

    try:
        return_plan = plan_transit_chained(
            arm=cfg.arm,
            log_label="post-place return to final XY",
            prev_terminal_pose=prev_terminal_pose,
            chained_arm_q=chained_arm_q,
            plant=return_plant,
            waypoints=return_waypoints,
            plant_context=return_plant_ctx,
            current_q=({cfg.arm: chained_arm_q}
                       if chained_arm_q is not None else None),
            use_rrt_fallback=cfg.motion_plan_rrt_fallback,
            rrt_diagram=return_diagram,
            rrt_max_iters=cfg.motion_plan_rrt_max_iters,
            rrt_shortcut_attempts=cfg.motion_plan_rrt_shortcut_attempts,
            min_clearance_m=0.01,
        )
    except InfeasiblePlanError as exc:
        print(f"  ✗ post-place return plan infeasible: {exc}")
        return legs
    legs.append(("post-place return to final XY", return_plan))

    return legs


def _run_sim(cfg: CupTaskCfg, grasp, place_pose: Pose) -> int:
    """Plan the carry legs, build a meshcat scene, hand off to the
    leg-by-leg stepper. Hand-coded ``pick`` / ``place`` / final-transit
    primitives are not simulated."""
    from pydrake.geometry import StartMeshcat
    from pydrake.systems.analysis import Simulator

    from ..planning import preview

    legs = _plan_sim_legs(cfg, grasp, place_pose)
    if not legs:
        print("[sim] no legs planned; aborting")
        return 1

    print()
    print("[sim] plan summary:")
    for label, plan in legs:
        clr = plan.min_clearance_m
        s = (f"{clr * 1000:.1f}mm" if np.isfinite(clr) else "n/a")
        print(f"  - {label}: planner={plan.metadata.get('planner')}  "
              f"duration={plan.duration_s:.2f}s  clearance={s}")

    sim_world = replace(cfg.world, in_hand={cfg.arm: (cfg.in_hand_kind, None)})
    meshcat = StartMeshcat()
    print(f"\n[sim] meshcat → {meshcat.web_url()}")
    scene = sim_world.build_sim_scene(meshcat=meshcat)

    simulator = Simulator(scene.diagram)
    simulator.Initialize()
    sim_ctx = simulator.get_mutable_context()

    return preview.run_interactive_legs(
        meshcat,
        scene.diagram,
        scene.plant,
        sim_ctx,
        legs,
    )


# ---------------------------------------------------------------------------
#  Real-arm execution
# ---------------------------------------------------------------------------

def run_on_arm(
    cfg: CupTaskCfg,
    session: Session,
    arm: ArmHandle,
    grasp,
    place_pose: Pose,
) -> bool:
    """Execute the pick + place on a live ArmHandle. Returns True on success.

    Use this from a routine that owns the Session — it does not connect
    or disconnect on its own."""
    config = cfg.config
    if cfg.motion_plan_pre_pick_approach:
        if not _planned_or_linear_transit(
            cfg, session, arm,
            "pre-pick approach to grasp hover",
            [_current_tcp_pose_task(arm), _hover_before_pick(grasp, config)],
            attached=False,
        ):
            return False

    print(f"\n→ pick: {grasp.description}")
    pick_result = pick(arm, grasp, config)
    if not pick_result.success:
        print(f"  ✗ pick FAILED: {pick_result.reason}")
        return False
    print("  ✓ pick succeeded.")

    if not _planned_or_linear_transit(
        cfg, session, arm,
        "post-pick carry to place hover",
        [_current_tcp_pose_task(arm),
         _hover_before_place(place_pose, config)],
        attached=True,
    ):
        return False

    print(f"\n→ place @ {cfg.place_pose_task.translation}")
    place_result = place(arm, place_pose, config)
    if not place_result.success:
        print(f"  ✗ place FAILED: {place_result.reason}")
        return False
    print("  ✓ place succeeded.")

    final_pose = plan_final_pose(cfg, place_pose)
    if not _planned_or_linear_transit(
        cfg, session, arm,
        "post-place return to final XY",
        [_current_tcp_pose_task(arm),
         pose_at_altitude(final_pose, config.transit_z)],
        attached=False,
    ):
        return False

    print("\nDone — arm retracted to transit altitude.")
    return True


# ---------------------------------------------------------------------------
#  CLI / registry entry point
# ---------------------------------------------------------------------------

def run_main(
    cfg: CupTaskCfg,
    *,
    dry: bool = False,
    mode: str = "real",
    motion_planning: bool = True,
) -> int:
    """Returns 0 on success, 1 on task failure; leaves exceptions to the
    caller."""
    if not motion_planning:
        cfg = replace(cfg, use_motion_planning=False)
        print("[CLI] motion planning DISABLED — every transit will use the "
              "sequential moveL fallback (the original pre-planner flow).")

    grasp = plan_pick(cfg)
    place_pose = plan_place(cfg)
    _print_plan(cfg, grasp, place_pose)

    if dry:
        print("[dry run] skipping RTDE connection. No motion commanded.")
        return 0

    if mode == "sim":
        if not cfg.supports_sim:
            print("[sim] this task does not support sim mode")
            return 1
        return _run_sim(cfg, grasp, place_pose)
    if mode != "real":
        raise ValueError(f"unknown mode {mode!r}; choose 'real' or 'sim'")

    left = cfg.arm == "ur_left"
    right = cfg.arm == "ur_right"
    with default_session(left=left, right=right) as session:
        arm = session.arms[cfg.arm]
        return 0 if run_on_arm(cfg, session, arm, grasp, place_pose) else 1


def parse_args(description: Optional[str] = None) -> dict:
    """Default argparse for the cup-task CLIs. Returns a dict of kwargs
    suitable for ``run_main``."""
    ap = argparse.ArgumentParser(description=description)
    ap.add_argument(
        "--dry",
        action="store_true",
        help="Plan and print the grasp/place poses without connecting to RTDE.",
    )
    ap.add_argument(
        "--mode",
        choices=["real", "sim"],
        default="real",
        help=(
            "Execution mode. 'real' (default) runs on the rig via RTDE. "
            "'sim' plans the planner-driven carry segments and replays them "
            "in meshcat with a leg-by-leg stepper; the hand-coded pick / "
            "place primitives are not simulated."
        ),
    )
    ap.add_argument(
        "--no-motion-planning",
        action="store_true",
        help=(
            "Disable Drake motion planning entirely. All transits route "
            "through sequential moveL Cartesian primitives — the pre-"
            "planner flow. ``motion_plan_auto_fallback`` already does "
            "this automatically on per-segment plan failures; this flag "
            "forces it for every segment up front."
        ),
    )
    args = ap.parse_args()
    return dict(
        dry=args.dry,
        mode=args.mode,
        motion_planning=not args.no_motion_planning,
    )
