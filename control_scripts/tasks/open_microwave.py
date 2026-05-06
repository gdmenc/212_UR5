"""Open the microwave door with the hook gripper (left arm).

Four-phase strategy:

    1. Approach   (moveL)      Hook tip to pre-engagement pose below handle.
    2. Engage     (moveL + gripper)
                               Short slide to seat hook under the handle,
                               then extend hook (close).
    3. Pull open               Two modes, selected by whether
                               ``MicrowaveDoorSpec.hinge_position_task`` is set:

                               ARC MODE (preferred when hinge is measured):
                               A series of ``moveL`` waypoints along the
                               computed arc. The TCP position follows the
                               hinge arc while orientation stays fixed at the
                               engaged hook pose, avoiding unnecessary wrist
                               rotation while pulling.
                               Deterministic and doesn't require force sensing.

                               FORCE MODE (fallback, no hinge needed):
                               ``forceMode`` with a commanded pull force along
                               the door's initial tangent direction. The arm is
                               compliant in both X and Y of the pull frame so
                               it can follow the arc passively. Exits when TCP
                               has moved past a position threshold OR time cap.

    4. Release    (moveL + gripper)
                               Open gripper (retract hook), lift to transit Z.

forceMode notes (phase 3 fallback):
  - ``rtde_c.forceModeStop()`` MUST fire before returning, even on exception —
    dangling force mode leaves the next RTDE call in an unpredictable state.
    This module wraps phase 3 in try/finally.
  - selection_vector [1, 1, 0, 0, 0, 0]: X and Y are force-controlled (Y force
    = 0 N, so the arm drifts freely along the arc), Z/Rx/Ry/Rz are
    position-controlled. This allows arc following without needing hinge geometry.
  - Force magnitude kept low (~20 N) so a hook slip doesn't pull hard.
    Tune based on the door's resistance.
  - Position threshold + time cap catch both "door reached stop" and
    "hook slipped / door got stuck."
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import List, Optional

import numpy as np
from scipy.spatial.transform import Rotation as ScipyRotation

from ..arm import ArmHandle
from ..config import DEFAULT, PickPlaceConfig
from ..moves import approach_to, lift_to_transit, retract_to
from ..reachability import is_task_pose_reachable
from ..util.poses import Pose, offset_along_tool_z
from ..util.rotations import Rotation
from ..util.rtde_convert import pose_to_rtde, rtde_to_pose
from ..world import World


SUPPORTS_SIM = True
"""``--mode sim`` plans the motion-planned phases (Phase 1 + Phase 3) and
replays them in meshcat with a leg-by-leg stepper. The arc-mode and
force-mode paths run hand-coded RTDE calls and are NOT simulated; sim
mode covers only the motion-planned branch."""


WORLD = World(
    include_microwave=True,
    include_objects=False,             # task uses recorded waypoints, not demo objects
    robotiq_mode="closed",
    microwave_door_open_rad=0.0,       # door starts closed (we are opening it)
)
"""Single source of env truth for this task's planning + sim scenes."""


@dataclass
class MicrowaveDoorSpec:
    """Hand-measured door / handle geometry.

    All poses and positions in task frame."""

    handle_engage_pose_task: Pose
    """Task-frame pose the TCP adopts when the hook is engaged under the
    door handle, before pulling. Hook pointing into the handle from
    below; tool orientation chosen so the hook latches when extended."""

    pre_engage_joints_rad: Optional[List[float]] = None
    """Recorded joint angles (6-element list, radians) at the pre-grasp
    waypoint. When set, phase 1 uses ``moveJ`` to this configuration
    instead of the ``lift → transit_xy → approach`` Cartesian chain.

    This avoids the wrist over-rotation / singularity that happens when
    ``moveL`` has to simultaneously move XY and rotate the end effector
    from its starting orientation (e.g. HOME) to the pre-engage
    orientation — a large orientation change in a single Cartesian move
    spins the wrist excessively. ``moveJ`` goes directly to the correct
    arm configuration in joint space with no orientation interpolation.

    Set from the ``joints_rad`` field of the recorded waypoint snapshot."""

    joint_speed: float = 0.1
    """Joint speed (rad/s) for the ``moveJ`` approach. Conservative default;
    speed up once the joint path is verified collision-free."""

    joint_accel: float = 0.3
    """Joint acceleration (rad/s²) for the ``moveJ`` approach."""

    pre_engage_pose_task: Optional[Pose] = None
    """Full task-frame Pose for the pre-engagement waypoint — both
    translation AND rotation from the recorded pre-grasp snapshot.
    Used only in the Cartesian fallback (when ``pre_engage_joints_rad``
    is None). If also None, falls back to engage pose + offset."""

    pre_engage_offset: np.ndarray = field(
        default_factory=lambda: np.array([0.0, 0.0, -0.03])
    )
    """Last-resort fallback: task-frame translation from engage to
    pre-engage, reusing the engage rotation. Prefer
    ``pre_engage_joints_rad`` + ``pre_engage_pose_task`` instead."""

    pull_direction_task: np.ndarray = field(
        default_factory=lambda: np.array([-1.0, -1.0, 0.0])
    )
    """Unit vector in task frame pointing along the INITIAL door-pull
    direction (tangent to the door's arc at the start of the swing).
    Default: -y (away from microwave, toward operator)."""

    # --- Arc mode (preferred) ---

    hinge_position_task: Optional[np.ndarray] = None
    """Position of the door hinge (rotation axis) in task frame.
    When set, phase 3 uses a series of moveL arc waypoints instead of
    force mode. Compute from handle position + door width:
        hinge = handle_translation + np.array([-door_width, 0, 0])
    (adjust sign based on which side the hinge is on)."""

    arc_open_angle_rad: float = 1.57
    """Total door opening angle (rad) for arc mode. 1.2 rad ≈ 69°.
    Adjust until the door is visually fully open."""

    n_arc_steps: int = 12
    """Number of intermediate moveL waypoints along the arc. More steps =
    smoother motion but more RTDE round-trips. 8 is a good start."""

    # --- Recorded-arc waypoints (preferred when set; bypasses math arc) ---

    recorded_arc_waypoints_task: Optional[List[Pose]] = None
    """When non-None, Phase 3 uses these task-frame poses **as-is**
    instead of generating an arc mathematically from
    ``hinge_position_task``. The list is the *intermediate* waypoints
    AFTER ``handle_engage_pose_task`` — the engage pose itself is
    prepended automatically when the planner builds its waypoint
    sequence.

    Use this when you have hand-recorded waypoints from the lab (e.g.
    ``logs/waypoints/ur_left_open_microwave_1.json``) — they are
    guaranteed-IK-feasible because the controller already executed
    them, whereas mathematically-generated arc poses can land at
    workspace boundaries that fail IK.
    """

    # --- Motion-planned mode (opt-in; preferred over arc/force when on) ---

    use_motion_planning: bool = False
    """When True, Phase 1 and Phase 3 are routed through Drake KTO
    instead of the recorded-joints moveJ + arc-of-moveLs flow:

      Phase 1 — collision-aware free-space transit from the rig's
        ACTUAL current joint configuration to ``pre_engage_pose_task``.
        Goes around the closed microwave instead of cutting through.
      Phase 3 — KTO smoothing through the same arc waypoints as the
        non-motion-planned path, but executed as a single blended
        ``moveJ(path=[...])`` call instead of N sequential moveLs.

    Both phases use ``arm.receive.getActualQ()`` as the planner's
    start state — NOT the sim's HOME pose — so the trajectory begins
    exactly where the arm is when called. Required: rig joints must
    be a feasible IK solution to ``pre_engage_pose_task`` (Phase 1 IK
    seeds from them) and the planning plant must have its UR5e at the
    matching pose."""

    motion_plan_n_waypoints: int = 30
    """N joint waypoints sampled from each KTO trajectory for the
    ``moveJ(path=[...])`` RTDE call. 30 keeps the controller-side
    blended path close to the planned curve at the door arc's
    high-curvature midsection."""

    motion_plan_rrt_max_iters: int = 2000
    """RRT fallback iteration budget for Phase 3 door-arc planning."""

    motion_plan_rrt_shortcut_attempts: int = 20
    """RRT fallback shortcut budget for Phase 3 door-arc post-processing."""

    motion_plan_tcp_speed: float = 0.05
    """TCP speed cap (m/s) on the door-arc leg. Hard-shape constraint
    that forces ``plan_transit`` to run KTO instead of falling back
    to a plain spline."""

    # --- Force mode (fallback when hinge_position_task is None) ---

    pull_force_n: float = 20.0
    """Force magnitude (N) commanded during force-mode pull. Tune so the
    door opens without slipping the hook."""

    pull_distance_task: float = 0.25
    """Target TCP displacement (m) along ``pull_direction_task`` before
    we consider the door 'open' in force mode."""

    pull_speed_limit: float = 0.05
    """Max TCP speed (m/s) during force mode. Low to avoid yanking."""

    pull_timeout_s: float = 5.0
    """Hard cap on force-mode phase 3. Stops even if the position
    threshold wasn't met (e.g., hook slipped, door stuck)."""

    disengage_offset: float = 0.03
    """Distance to back the hook out along its local approach axis after
    opening the throat at the end of the pull. This unseats the hook from
    the handle before the final lift to transit."""


@dataclass
class OpenMicrowaveResult:
    success: bool
    reason: Optional[str] = None
    door_opened_distance: float = 0.0
    """How far (m) the TCP moved — arc length in arc mode, projected
    displacement in force mode."""


# -----------------------------------------------------------------------
#  Lazy planning-plant cache
# -----------------------------------------------------------------------
# Built on the first motion-planned call and reused for every subsequent
# one. Drake plant + context only — no Meshcat, no diagram visualisation.
# Microwave is welded with the door CLOSED (matches the real state at the
# start of the open sequence).

_PLANNING_DIAGRAM = None
_PLANNING_PLANT = None
_PLANNING_PLANT_CONTEXT = None


def _get_planning_plant():
    """Lazy-build the bimanual scene used for motion planning. First
    call pays the build cost (~half a second); subsequent calls are
    instant. Disable workspace demo objects since they clutter the
    cell with bodies the open-microwave task doesn't interact with.

    Routes through ``WORLD.build_planning_scene()`` so this task's
    planning plant matches the env values declared in ``WORLD`` AND the
    returned diagram is a ``RobotDiagram`` — required for the RRT
    fallback's ``SceneGraphCollisionChecker``. Effective kwargs are
    unchanged from the previous direct call (include_objects=False,
    everything else default)."""
    global _PLANNING_DIAGRAM, _PLANNING_PLANT, _PLANNING_PLANT_CONTEXT
    if _PLANNING_PLANT is None:
        diagram, plant, _, _ = WORLD.build_planning_scene()
        diagram_ctx = diagram.CreateDefaultContext()
        _PLANNING_DIAGRAM = diagram
        _PLANNING_PLANT = plant
        _PLANNING_PLANT_CONTEXT = plant.GetMyContextFromRoot(diagram_ctx)
    return _PLANNING_DIAGRAM, _PLANNING_PLANT, _PLANNING_PLANT_CONTEXT


def _build_movej_path_rows(
    plant,
    plan,
    arm_name: str,
    n_waypoints: int,
    *,
    speed: float = 1.0,
    accel: float = 1.4,
    blend_r: float = 0.02,
) -> list:
    """Sample N joint waypoints from a KTO trajectory and pack into the
    ``[[q1..q6, speed, accel, blend], ...]`` list shape RTDE expects
    for ``moveJ(path=...)``. Terminal-row blend forced to 0 (UR
    controller requirement).

    Pure data — does NOT touch the rig. Shared between
    ``_execute_plan_as_movej_path`` (rig send) and
    ``dry_run_motion_planning`` (offline preview).
    """
    from ..planning.transit import _arm_model_instance, _arm_position_indices
    arm_idx = _arm_position_indices(plant, _arm_model_instance(plant, arm_name))
    t0, t1 = plan.trajectory.start_time(), plan.trajectory.end_time()
    sample_times = np.linspace(t0, t1, n_waypoints)
    rows = []
    for t in sample_times:
        q_full = np.asarray(plan.trajectory.value(t)).flatten()
        rows.append(list(q_full[arm_idx]) + [speed, accel, blend_r])
    rows[-1][-1] = 0.0
    return rows


def _execute_plan_as_movej_path(
    arm: ArmHandle,
    plant,
    plan,
    n_waypoints: int,
    *,
    speed: float = 1.0,
    accel: float = 1.4,
    blend_r: float = 0.02,
) -> None:
    """Sample N joint waypoints from a KTO trajectory and ship them to
    the controller as one blended ``moveJ(path=[...])`` call.

    Mirrors ``planning/execute.py:_execute_moveJ_path`` byte-for-byte
    for the joint-waypoint extraction. Inlined here to avoid a Session
    round-trip — we already have the ArmHandle.
    """
    rows = _build_movej_path_rows(
        plant, plan, arm.name, n_waypoints,
        speed=speed, accel=accel, blend_r=blend_r,
    )
    arm.control.moveJ(rows)


def _print_movej_payload(label: str, plan, rows: list) -> None:
    """Pretty-print a ``moveJ(path=[...])`` payload — same format as
    ``planning/dryrun_open_microwave._print_movej_payload``."""
    clr = (plan.min_clearance_m * 1000
           if np.isfinite(plan.min_clearance_m) else float("nan"))
    print()
    print(f"  # >>> RTDE: would call arm.control.moveJ(path=[")
    for i, row in enumerate(rows):
        q_str = ", ".join(f"{x:+.4f}" for x in row[:6])
        tail = f"{row[6]:.2f}, {row[7]:.2f}, {row[8]:.4f}"
        marker = "  # final" if i == len(rows) - 1 else ""
        print(f"  #     [{q_str}, {tail}],{marker}")
    print(f"  # ])  # leg: {label}  duration={plan.duration_s:.2f}s  "
          f"clearance={clr:+.1f}mm  planner={plan.metadata.get('planner')}")


def dry_run_motion_planning(door: "MicrowaveDoorSpec") -> int:
    """Run Phase 1 + Phase 3 motion plans against the cached planning
    plant and dump the ``moveJ(path=[...])`` payloads they'd send.
    No RTDE access required — uses ``SIM_HOME_Q_LEFT`` as the planner's
    start state since there's no rig to query.

    Returns 0 on success, 1 if any leg failed to plan.
    """
    from ..planning import SIM_HOME_Q_LEFT, SIM_HOME_Q_RIGHT
    from ..planning.transit import (
        InfeasiblePlanError,
        _arm_model_instance,
        _arm_position_indices,
        plan_transit,
    )

    if door.pre_engage_pose_task is None:
        print("[dry-run] pre_engage_pose_task is None — Phase 1 motion "
              "planning needs it. Aborting.")
        return 1
    if door.hinge_position_task is None:
        print("[dry-run] hinge_position_task is None — Phase 3 motion "
              "planning needs it. Aborting.")
        return 1

    _, plant, plant_ctx = _get_planning_plant()

    # Seed plant at the task-specific approach config (same IK branch as
    # PRE_ENGAGE_JOINTS_RAD) so Phase 1 IK doesn't jump branches the
    # way it does from SIM_HOME_Q_LEFT.
    arm_idx = _arm_position_indices(plant, _arm_model_instance(plant, "ur_left"))
    other_idx = _arm_position_indices(plant, _arm_model_instance(plant, "ur_right"))
    start_q_left = OPEN_MICROWAVE_APPROACH_Q_LEFT
    full_q = plant.GetPositions(plant_ctx).copy()
    full_q[arm_idx] = start_q_left
    full_q[other_idx] = SIM_HOME_Q_RIGHT
    plant.SetPositions(plant_ctx, full_q)

    tcp_frame = plant.GetFrameByName("tcp_left")
    X = tcp_frame.CalcPoseInWorld(plant_ctx)
    home_pose = Pose(translation=np.asarray(X.translation()),
                     rotation=Rotation.from_matrix(X.rotation().matrix()))

    cur_q = {"ur_left": np.asarray(start_q_left, dtype=float),
             "ur_right": np.asarray(SIM_HOME_Q_RIGHT, dtype=float)}

    print()
    print("=" * 72)
    print("  Motion-planning dry run — no RTDE, no rig")
    print(f"  Seed left  q = OPEN_MICROWAVE_APPROACH_Q_LEFT  "
          f"(rig will use getActualQ() instead)")
    print(f"  Seed right q = SIM_HOME_Q_RIGHT")
    print(f"  Sample count per phase = {door.motion_plan_n_waypoints}")
    print("=" * 72)

    plans: list = []
    failures: list = []

    # --- Phase 1: HOME -> pre_engage ---
    # Object collision is OFF; the simplified door box has no handle
    # relief and the hook gripper's AABB always clips into it at the
    # recorded pre-engage pose. Instead of relying on collision, we cap
    # TCP y at the pre-engage y (task frame) so the planner physically
    # can't drive the gripper past the closed-door volume.
    # Phase 3 still wants RRT fallback wiring, so build the RobotDiagram
    # here and reuse it below.
    from ..planning.build_scene import _compose_scene_fragments
    from pydrake.planning import RobotDiagramBuilder
    rdb = RobotDiagramBuilder(time_step=0.0)
    rdb_plant = rdb.plant()
    _compose_scene_fragments(
        rdb_plant,
        include_microwave=True,
        include_grippers=True,
        include_objects=False,
        robotiq_mode="closed",
    )
    rdb_plant.Finalize()
    rrt_diagram_dry = rdb.Build()

    pre_engage_y = float(door.pre_engage_pose_task.translation[1])
    phase1_workspace_box = (
        (-1e3, -1e3, -1e3),
        (+1e3, pre_engage_y, +1e3),
    )

    try:
        plan1 = plan_transit(
            plant=plant, arm="ur_left",
            waypoints=[home_pose, door.pre_engage_pose_task],
            plant_context=plant_ctx,
            current_q=cur_q,
            avoid_collisions=False, self_collision=False,
            stay_in_workspace_box=phase1_workspace_box,
        )
        print(f"\nPhase 1: HOME -> pre_engage  "
              f"planner={plan1.metadata.get('planner')}  "
              f"duration={plan1.duration_s:.2f}s")
        plans.append(("Phase 1: HOME -> pre_engage", plan1))
        # Update Phase 3 seed from Phase 1's terminal joints so the
        # planner sees the arm at the engage pose, not SIM_HOME.
        cur_q["ur_left"] = np.asarray(
            plan1.trajectory.value(plan1.trajectory.end_time())
        ).flatten()[arm_idx]
    except InfeasiblePlanError as exc:
        print(f"\nPhase 1 (HOME -> pre_engage) PLAN INFEASIBLE:\n  {exc}")
        failures.append("Phase 1")
        # Continue to Phase 3 anyway — Phase 3 starts from a known
        # task-frame pose (handle_engage_pose_task), so its IK is
        # independent of Phase 1's outcome.

    # --- Phase 3: arc through the hinge (collision off) ---
    # Same plan_transit shape as the live path: explicit duration_s
    # (path length / target TCP speed) routes through
    # simple_spline → KTO → RRT.
    arc_intermediate = _arc_waypoints(door, start_pose_task=door.handle_engage_pose_task)
    arc_waypoints = [door.handle_engage_pose_task, *arc_intermediate]
    try:
        plan3 = plan_transit(
            plant=plant, arm="ur_left",
            waypoints=arc_waypoints,
            plant_context=plant_ctx,
            current_q=cur_q,
            avoid_collisions=False, self_collision=False,
            duration_s=_phase3_duration_s(arc_waypoints, door.motion_plan_tcp_speed),
            use_rrt_fallback=True,
            rrt_diagram=rrt_diagram_dry,
            rrt_max_iters=door.motion_plan_rrt_max_iters,
            rrt_shortcut_attempts=door.motion_plan_rrt_shortcut_attempts,
        )
        print(f"\nPhase 3: door arc  "
              f"planner={plan3.metadata.get('planner')}  "
              f"duration={plan3.duration_s:.2f}s  "
              f"({len(arc_waypoints)} arc waypoints)")
        plans.append(("Phase 3: door arc", plan3))
    except InfeasiblePlanError as exc:
        print(f"\nPhase 3 (door arc) PLAN INFEASIBLE:\n  {exc}")
        failures.append("Phase 3")

    # --- Dump moveJ(path=[...]) payloads for whatever planned cleanly ---
    if plans:
        print()
        print("=" * 72)
        print(f"  RTDE moveJ(path=[...]) payloads — "
              f"{door.motion_plan_n_waypoints} samples each")
        print("=" * 72)
        print("  Per-row format: [q1..q6, speed, accel, blend]")
        print("  Defaults:        speed=1.00 rad/s  accel=1.40 rad/s²  "
              "blend=0.0200 m  (terminal blend=0)")
        for label, plan in plans:
            rows = _build_movej_path_rows(
                plant, plan, "ur_left", door.motion_plan_n_waypoints,
            )
            _print_movej_payload(label, plan, rows)

    if failures:
        print()
        print(f"[dry-run] {len(failures)} phase(s) failed to plan: "
              f"{', '.join(failures)}")
        return 1
    return 0


def run_sim(door: "MicrowaveDoorSpec") -> int:
    """Plan the motion-planned phases (Phase 1 + Phase 3) and replay
    them in meshcat with a leg-by-leg stepper.

    Phase 2 (engage / hook close) and Phase 4 (release / lift) are
    hand-coded RTDE calls and are NOT simulated. Sim mode covers only
    the planner-driven phases. Between legs the arm jumps to the next
    leg's start config — that jump represents the result of the skipped
    hand-coded primitive.

    Two scenes are built:
      - planning scene (RobotDiagram via ``WORLD.build_planning_scene``)
        — used for ``plan_transit`` and the RRT collision checker
      - sim scene (Diagram via ``WORLD.build_sim_scene(meshcat=...)``)
        — used for the meshcat-driven leg stepper

    Both go through ``_compose_scene_fragments`` so the q-vector layout
    matches; trajectories planned against one play correctly on the
    other.
    """
    from pydrake.geometry import StartMeshcat
    from pydrake.systems.analysis import Simulator

    from ..planning import SIM_HOME_Q_LEFT, SIM_HOME_Q_RIGHT, preview
    from ..planning.transit import (
        InfeasiblePlanError,
        _arm_model_instance,
        _arm_position_indices,
        plan_transit,
    )

    if door.pre_engage_pose_task is None:
        print("[sim] pre_engage_pose_task is None — Phase 1 needs it. Aborting.")
        return 1
    if door.hinge_position_task is None:
        print("[sim] hinge_position_task is None — Phase 3 needs it. Aborting.")
        return 1

    meshcat = StartMeshcat()
    print(f"[sim] meshcat → {meshcat.web_url()}")

    # Sim scene (meshcat-attached) — used for animation only.
    sim_scene = WORLD.build_sim_scene(meshcat=meshcat)

    # Planning scene (RobotDiagram) — used by plan_transit + RRT.
    # Built separately because SceneGraphCollisionChecker (RRT) requires
    # RobotDiagram, not a regular Diagram.
    plan_diagram, plan_plant, _, _ = WORLD.build_planning_scene()
    plan_root = plan_diagram.CreateDefaultContext()
    plan_plant_ctx = plan_plant.GetMyMutableContextFromRoot(plan_root)

    arm_idx = _arm_position_indices(
        plan_plant, _arm_model_instance(plan_plant, "ur_left"),
    )
    other_idx = _arm_position_indices(
        plan_plant, _arm_model_instance(plan_plant, "ur_right"),
    )
    # Use the task-specific approach config for the left arm (same IK
    # branch as PRE_ENGAGE_JOINTS_RAD); SIM_HOME_Q_LEFT is on a
    # different branch and Phase 1 IK can't reach the recorded pregrasp
    # from there without a wrist flip.
    start_q_left = OPEN_MICROWAVE_APPROACH_Q_LEFT
    full_q = plan_plant.GetPositions(plan_plant_ctx).copy()
    full_q[arm_idx] = start_q_left
    full_q[other_idx] = SIM_HOME_Q_RIGHT
    plan_plant.SetPositions(plan_plant_ctx, full_q)

    # Mirror that on the sim (meshcat) plant so the rendered arm starts
    # at the same approach pose, not at SIM_HOME.
    sim_plant_ctx = sim_scene.plant.GetMyMutableContextFromRoot(
        sim_scene.diagram.CreateDefaultContext()
    )
    sim_full_q = sim_scene.plant.GetPositions(sim_plant_ctx).copy()
    sim_arm_idx = _arm_position_indices(
        sim_scene.plant, _arm_model_instance(sim_scene.plant, "ur_left"),
    )
    sim_other_idx = _arm_position_indices(
        sim_scene.plant, _arm_model_instance(sim_scene.plant, "ur_right"),
    )
    sim_full_q[sim_arm_idx] = start_q_left
    sim_full_q[sim_other_idx] = SIM_HOME_Q_RIGHT
    sim_scene.plant.SetDefaultPositions(sim_full_q)

    tcp_frame = plan_plant.GetFrameByName("tcp_left")
    X = tcp_frame.CalcPoseInWorld(plan_plant_ctx)
    home_pose = Pose(translation=np.asarray(X.translation()),
                     rotation=Rotation.from_matrix(X.rotation().matrix()))

    cur_q = {"ur_left": np.asarray(start_q_left, dtype=float),
             "ur_right": np.asarray(SIM_HOME_Q_RIGHT, dtype=float)}

    legs: list = []

    pre_engage_y = float(door.pre_engage_pose_task.translation[1])
    phase1_workspace_box = (
        (-1e3, -1e3, -1e3),
        (+1e3, pre_engage_y, +1e3),
    )

    print("\n[sim] planning Phase 1 (HOME → pre_engage) — TCP y ≤ "
          f"{pre_engage_y:.3f} m, no env/self collision...")
    try:
        plan1 = plan_transit(
            plant=plan_plant, arm="ur_left",
            waypoints=[home_pose, door.pre_engage_pose_task],
            plant_context=plan_plant_ctx,
            current_q=cur_q,
            avoid_collisions=False, self_collision=False,
            stay_in_workspace_box=phase1_workspace_box,
        )
        legs.append(("Phase 1: HOME -> pre_engage", plan1))
        cur_q["ur_left"] = np.asarray(
            plan1.trajectory.value(plan1.trajectory.end_time())
        ).flatten()[arm_idx]
    except InfeasiblePlanError as exc:
        print(f"  ✗ Phase 1 plan infeasible: {exc}")

    print("\n[sim] planning Phase 3 (door arc)...")
    arc_intermediate = _arc_waypoints(door, start_pose_task=door.handle_engage_pose_task)
    arc_waypoints = [door.handle_engage_pose_task, *arc_intermediate]
    try:
        plan3 = plan_transit(
            plant=plan_plant, arm="ur_left",
            waypoints=arc_waypoints,
            plant_context=plan_plant_ctx,
            current_q=cur_q,
            avoid_collisions=False, self_collision=False,
            duration_s=_phase3_duration_s(arc_waypoints, door.motion_plan_tcp_speed),
            use_rrt_fallback=True,
            rrt_diagram=plan_diagram,
            rrt_max_iters=door.motion_plan_rrt_max_iters,
            rrt_shortcut_attempts=door.motion_plan_rrt_shortcut_attempts,
        )
        legs.append(("Phase 3: door arc", plan3))
    except InfeasiblePlanError as exc:
        print(f"  ✗ Phase 3 plan infeasible: {exc}")

    if not legs:
        print("[sim] no legs planned; aborting")
        return 1

    print()
    print("[sim] plan summary:")
    for label, plan in legs:
        clr = (plan.min_clearance_m * 1000
               if np.isfinite(plan.min_clearance_m) else float("nan"))
        print(f"  - {label}: planner={plan.metadata.get('planner')}  "
              f"duration={plan.duration_s:.2f}s  clearance={clr:.1f}mm")

    simulator = Simulator(sim_scene.diagram)
    simulator.Initialize()
    sim_ctx = simulator.get_mutable_context()

    return preview.run_interactive_legs(
        meshcat,
        sim_scene.diagram,
        sim_scene.plant,
        sim_ctx,
        legs,
    )


def _phase1_motion_planned(
    arm: ArmHandle,
    door: "MicrowaveDoorSpec",
) -> None:
    """Phase 1 (motion-planned approach): Drake KTO from the rig's
    actual current joints to ``door.pre_engage_pose_task``, executed
    as a single blended ``moveJ(path)``.

    Object/self collision avoidance is OFF — the simplified door box
    has no handle relief and the hook gripper's AABB always clips into
    it at the recorded pre-engage pose. We instead cap TCP y at the
    pre-engage y (task frame) so the planner physically cannot drive
    the gripper past the closed-door volume. This is a behavioural
    constraint that doesn't depend on URDF/TCP calibration drift."""
    from ..planning.transit import plan_transit
    _, plant, plant_ctx = _get_planning_plant()

    actual_q = np.array(arm.receive.getActualQ(), dtype=float)
    start_pose_task = arm.to_task(rtde_to_pose(arm.receive.getActualTCPPose()))

    if door.pre_engage_pose_task is None:
        raise ValueError(
            "use_motion_planning=True requires pre_engage_pose_task "
            "(the planner targets a Cartesian pose, not raw joints)."
        )

    pre_engage_y = float(door.pre_engage_pose_task.translation[1])
    phase1_workspace_box = (
        (-1e3, -1e3, -1e3),
        (+1e3, pre_engage_y, +1e3),
    )

    plan = plan_transit(
        plant=plant, arm=arm.name,
        waypoints=[start_pose_task, door.pre_engage_pose_task],
        plant_context=plant_ctx,
        current_q={arm.name: actual_q},
        avoid_collisions=False,
        self_collision=False,
        stay_in_workspace_box=phase1_workspace_box,
    )
    _execute_plan_as_movej_path(arm, plant, plan, door.motion_plan_n_waypoints)


def _phase3_motion_planned(
    arm: ArmHandle,
    door: "MicrowaveDoorSpec",
) -> float:
    """Phase 3 (motion-planned arc): plan_transit through the same TCP
    arc waypoints as ``_phase3_arc``, executed as one blended
    ``moveJ(path)``.

    Collision avoidance is OFF — the hook is in intentional contact
    with the door throughout the swing. TCP speed is shaped via an
    explicit ``duration_s`` (path length / target speed) rather than
    ``max_tcp_linear_speed_m_per_s`` so the planner can take the
    simple_spline → KTO → RRT chain (the speed cap would force KTO
    immediately and skip the spline path)."""
    from ..planning.transit import plan_transit
    if door.hinge_position_task is None:
        raise ValueError(
            "use_motion_planning=True for phase 3 requires "
            "hinge_position_task (the arc waypoint generator needs it)."
        )
    diagram, plant, plant_ctx = _get_planning_plant()

    actual_q = np.array(arm.receive.getActualQ(), dtype=float)
    start_pose_task = arm.to_task(rtde_to_pose(arm.receive.getActualTCPPose()))

    arc_intermediate = _arc_waypoints(door, start_pose_task=start_pose_task)
    waypoints = [start_pose_task, *arc_intermediate]

    plan = plan_transit(
        plant=plant, arm=arm.name,
        waypoints=waypoints,
        plant_context=plant_ctx,
        current_q={arm.name: actual_q},
        avoid_collisions=False,
        self_collision=False,
        duration_s=_phase3_duration_s(waypoints, door.motion_plan_tcp_speed),
        use_rrt_fallback=True,
        rrt_diagram=diagram,
        rrt_max_iters=door.motion_plan_rrt_max_iters,
        rrt_shortcut_attempts=door.motion_plan_rrt_shortcut_attempts,
    )
    _execute_plan_as_movej_path(arm, plant, plan, door.motion_plan_n_waypoints)

    r_vec = start_pose_task.translation - door.hinge_position_task
    return float(np.linalg.norm(r_vec[:2])) * door.arc_open_angle_rad


# -----------------------------------------------------------------------
#  Arc-mode helpers
# -----------------------------------------------------------------------

def _phase3_duration_s(waypoints: List[Pose], target_tcp_speed: float) -> float:
    """Total trajectory time so the average TCP speed ≈ ``target_tcp_speed``.

    Sum of consecutive-waypoint translation distances divided by the
    target speed. Used in place of ``max_tcp_linear_speed_m_per_s`` so
    Phase 3 doesn't trip ``plan_transit``'s hard-shape constraint and
    can take the simple_spline → KTO → RRT chain like Phase 1. The
    speed isn't enforced pointwise — it's an average over the whole
    arc — but for the door pull that's close enough."""
    total = 0.0
    for a, b in zip(waypoints[:-1], waypoints[1:]):
        total += float(np.linalg.norm(a.translation - b.translation))
    return max(1.0, total / max(target_tcp_speed, 1e-3))


def _arc_waypoints(
    door: MicrowaveDoorSpec,
    start_pose_task: Optional[Pose] = None,
) -> List[Pose]:
    """Return the intermediate task-frame TCP poses for Phase 3.

    Two sources, in priority order:
      1. ``door.recorded_arc_waypoints_task`` — recorded waypoints from
         the rig (e.g. ``logs/waypoints/ur_left_open_microwave_1.json``).
         Returned verbatim — these are guaranteed IK-feasible because
         they were physically executed.
      2. Math-generated arc — rotate the start pose around
         ``door.hinge_position_task`` by ``arc_open_angle_rad`` in
         ``n_arc_steps`` increments. Both translation and the orientation
         around task Z rotate together so the hook stays tangent to the
         arc. Rotation sign is chosen so the initial tangent aligns with
         ``pull_direction_task``.
    """
    if door.recorded_arc_waypoints_task is not None:
        return list(door.recorded_arc_waypoints_task)

    start = start_pose_task or door.handle_engage_pose_task
    hinge = door.hinge_position_task

    r_vec = start.translation - hinge  # hinge → current handle/TCP, task frame

    ccw_tangent_2d = np.array([r_vec[1], -r_vec[0]])
    pull_2d = door.pull_direction_task[:2]
    angle_sign = -1.0 if np.dot(ccw_tangent_2d, pull_2d) > 0.0 else +1.0

    total_angle = angle_sign * door.arc_open_angle_rad
    angles = np.linspace(0.0, total_angle, door.n_arc_steps + 1)[1:]

    waypoints: List[Pose] = []
    for theta in angles:
        c, s = np.cos(theta), np.sin(theta)
        # Rotate r_vec around task Z by theta.
        new_r = np.array([
            c * r_vec[0] - s * r_vec[1],
            s * r_vec[0] + c * r_vec[1],
            r_vec[2],
        ])
        new_translation = hinge + new_r

        # Rotate TCP orientation by the same angle around task Z so the
        # hook stays tangent to the arc throughout the sweep.
        dR = Rotation.from_rotvec(np.array([0.0, 0.0, theta]))
        new_rotation = dR * start.rotation

        waypoints.append(Pose(translation=new_translation, rotation=new_rotation))

    return waypoints


# -----------------------------------------------------------------------
#  Force-mode helpers (used only when hinge_position_task is None)
# -----------------------------------------------------------------------

def _build_task_frame_for_pull(
    engage_pose_task: Pose,
    pull_direction_task: np.ndarray,
    arm: ArmHandle,
) -> List[float]:
    """Build the rtde_c.forceMode ``task_frame`` argument.

    The task frame is a 6-vector [x,y,z, rx,ry,rz] in base frame coords
    (axis-angle rotation). We want its +x axis aligned with the desired
    pull direction, origin at the current TCP. Position doesn't matter
    mechanically (force mode uses the frame's orientation) but we set
    it to the TCP position for clarity in logs.
    """
    pull_dir_base = arm.X_base_task.rotation.apply(pull_direction_task)
    pull_dir_base = pull_dir_base / np.linalg.norm(pull_dir_base)

    world_up_base = np.array([0.0, 0.0, 1.0])
    if abs(np.dot(pull_dir_base, world_up_base)) > 0.95:
        world_up_base = np.array([0.0, 1.0, 0.0])
    z_axis = world_up_base - pull_dir_base * np.dot(pull_dir_base, world_up_base)
    z_axis = z_axis / np.linalg.norm(z_axis)
    y_axis = np.cross(z_axis, pull_dir_base)

    R = np.column_stack([pull_dir_base, y_axis, z_axis])
    rotvec = ScipyRotation.from_matrix(R).as_rotvec()

    tcp_base = rtde_to_pose(arm.receive.getActualTCPPose())

    return [
        tcp_base.translation[0],
        tcp_base.translation[1],
        tcp_base.translation[2],
        float(rotvec[0]),
        float(rotvec[1]),
        float(rotvec[2]),
    ]


# -----------------------------------------------------------------------
#  Primitive
# -----------------------------------------------------------------------

def open_microwave_door(
    arm: ArmHandle,
    door: MicrowaveDoorSpec,
    config: PickPlaceConfig = DEFAULT,
) -> OpenMicrowaveResult:
    """Open the microwave door end-to-end.

    Phase 3 automatically uses arc-waypoint mode when
    ``door.hinge_position_task`` is set, falling back to force mode
    otherwise. See module docstring for the full four-phase breakdown.

    Assumes ``arm`` has a ``HookGripper`` attached.
    """
    if config.transit_z is None:
        raise ValueError("config.transit_z is unset.")
    if arm.gripper is None:
        raise ValueError(f"arm {arm.name!r} has no gripper attached.")

    arm.gripper.prepare_for_grasp()

    engage_pose = door.handle_engage_pose_task

    # Use the full recorded pre-grasp Pose when available — this preserves
    # the exact hook orientation from the recording, which differs from the
    # engage orientation due to the hook gripper's R_y(π/2) TCP offset.
    if door.pre_engage_pose_task is not None:
        pre_engage_pose = door.pre_engage_pose_task
    else:
        pre_engage_pose = Pose(
            translation=engage_pose.translation + door.pre_engage_offset,
            rotation=engage_pose.rotation,
        )

    # Pre-flight reachability (only for the Cartesian fallback path).
    if door.pre_engage_joints_rad is None:
        for label, pose in [("pre_engage", pre_engage_pose), ("engage", engage_pose)]:
            if is_task_pose_reachable(arm, pose) is None:
                return OpenMicrowaveResult(
                    success=False,
                    reason=f"waypoint '{label}' is kinematically infeasible.",
                )

    # --- Phase 1: approach ---
    if door.use_motion_planning:
        # Drake KTO from the rig's CURRENT joints to pre_engage_pose,
        # collision-aware. Routes around the closed microwave instead
        # of cutting through it.
        _phase1_motion_planned(arm, door)
    elif door.pre_engage_joints_rad is not None:
        # Joint-space approach: go directly to the recorded pre-grasp joint
        # configuration. Avoids the wrist over-rotation that happens when a
        # single moveL tries to simultaneously move XY and rotate the end
        # effector from its starting orientation to the pre-engage orientation.
        arm.control.moveJ(
            list(door.pre_engage_joints_rad),
            door.joint_speed,
            door.joint_accel,
        )
        approach_to(arm, pre_engage_pose, config.approach_speed, config.approach_accel)
    # else:
    #     # Cartesian fallback: lift → transit at safe altitude → descend.
    #     # May cause wrist spin if the starting orientation is far from the
    #     # pre-engage orientation. Prefer providing pre_engage_joints_rad.
    #     lift_to_transit(arm, config.transit_z, config.transit_speed, config.transit_accel)
    #     transit_xy(arm, pre_engage_pose, config.transit_z,
    #                config.transit_speed, config.transit_accel)
    #     approach_to(arm, pre_engage_pose, config.approach_speed, config.approach_accel)

    # --- Phase 2: engage (short moveL slide to seat hook, then latch) ---
    arm.gripper.prepare_for_grasp()
    approach_to(arm, engage_pose, config.approach_speed, config.approach_accel)
    if not arm.gripper.grasp():
        return OpenMicrowaveResult(
            success=False,
            reason="hook gripper did not report a successful latch.",
        )
    gripper_status = arm.gripper.status()
    if gripper_status.get("extended") is True:
        return OpenMicrowaveResult(
            success=False,
            reason="hook gripper still reports open after close command.",
        )

    # --- Phase 3: pull open ---
    if door.use_motion_planning:
        distance_moved = _phase3_motion_planned(arm, door)
    elif door.hinge_position_task is not None:
        distance_moved = _phase3_arc(arm, door, config)
    else:
        distance_moved = _phase3_force(arm, door, engage_pose)

    # --- Phase 4: release and retract ---
    arm.gripper.open()
    current_pose_task = arm.to_task(rtde_to_pose(arm.receive.getActualTCPPose()))
    disengage_pose = offset_along_tool_z(current_pose_task, door.disengage_offset)
    retract_to(arm, disengage_pose, config.retract_speed, config.retract_accel)
    lift_to_transit(
        arm, config.transit_z, config.retract_speed, config.retract_accel
    )

    if door.hinge_position_task is not None:
        # Arc: distance is the arc length.
        r_vec = engage_pose.translation - door.hinge_position_task
        radius = float(np.linalg.norm(r_vec[:2]))
        arc_length = radius * door.arc_open_angle_rad
        opened_ok = True
        return OpenMicrowaveResult(
            success=opened_ok,
            door_opened_distance=arc_length,
        )
    else:
        opened_ok = distance_moved >= 0.8 * door.pull_distance_task
        return OpenMicrowaveResult(
            success=opened_ok,
            reason=None if opened_ok else "door did not reach target pull distance",
            door_opened_distance=distance_moved,
        )


# -----------------------------------------------------------------------
#  Phase 3 implementations (split out to keep open_microwave_door readable)
# -----------------------------------------------------------------------

def _phase3_arc(
    arm: ArmHandle,
    door: MicrowaveDoorSpec,
    config: PickPlaceConfig,
) -> float:
    """Phase 3 (arc mode): moveL through computed arc waypoints.

    Returns the arc length traveled (m).
    """
    start_pose_task = arm.to_task(rtde_to_pose(arm.receive.getActualTCPPose()))
    waypoints = _arc_waypoints(door, start_pose_task=start_pose_task)

    # Convert all waypoints to RTDE base-frame poses up-front, then enforce
    # rotvec continuity so the UR controller doesn't see axis-angle flips
    # between consecutive targets (which causes unnecessary wrist spins).
    rtde_poses = [pose_to_rtde(arm.to_base(wp)) for wp in waypoints]
    prev_rv = np.array(rtde_poses[0][3:6]) if rtde_poses else None
    for rp in rtde_poses:
        rv = np.array(rp[3:6])
        if prev_rv is not None and np.dot(rv, prev_rv) < 0.0:
            # Flip to equivalent representation: rotvec ± 2π*axis
            # For axis-angle, negating gives same rotation when angle → 2π-angle
            # but the simpler fix is to pick the closer of ±rv.
            rv = -rv
            rp[3], rp[4], rp[5] = float(rv[0]), float(rv[1]), float(rv[2])
        prev_rv = rv

    for rp in rtde_poses:
        arm.control.moveL(
            rp,
            config.approach_speed,
            config.approach_accel,
        )
    r_vec = start_pose_task.translation - door.hinge_position_task
    return float(np.linalg.norm(r_vec[:2])) * door.arc_open_angle_rad


def _phase3_force(
    arm: ArmHandle,
    door: MicrowaveDoorSpec,
    engage_pose_task: Pose,
) -> float:
    """Phase 3 (force mode): compliant pull along pull_direction_task.

    Compliant in both X and Y of the pull frame (selection_vector
    [1, 1, 0, 0, 0, 0]) so the arm can passively follow the door's arc
    without fighting the perpendicular motion.

    Returns the projected distance traveled along pull_direction_task (m).
    """
    start_tcp_base = rtde_to_pose(arm.receive.getActualTCPPose())
    task_frame = _build_task_frame_for_pull(engage_pose_task, door.pull_direction_task, arm)

    # Compliant in X (pull direction) and Y (arc drift), position-controlled
    # in Z and all rotations.
    selection_vector = [1, 1, 0, 0, 0, 0]
    wrench = [door.pull_force_n, 0.0, 0.0, 0.0, 0.0, 0.0]
    limits = [door.pull_speed_limit] * 3 + [0.5] * 3
    FORCE_MODE_TYPE = 2

    distance_moved = 0.0
    try:
        arm.control.forceMode(task_frame, selection_vector, wrench,
                              FORCE_MODE_TYPE, limits)
        t_start = time.time()
        while (time.time() - t_start) < door.pull_timeout_s:
            now_tcp = rtde_to_pose(arm.receive.getActualTCPPose())
            delta_base = now_tcp.translation - start_tcp_base.translation
            pull_base = arm.X_base_task.rotation.apply(door.pull_direction_task)
            pull_base = pull_base / np.linalg.norm(pull_base)
            distance_moved = float(np.dot(delta_base, pull_base))
            if distance_moved >= door.pull_distance_task:
                break
            time.sleep(0.01)
    finally:
        arm.control.forceModeStop()

    return distance_moved


# =======================================================================
#  Default door spec + motion config — moved here from
#  ``examples/open_microwave.py`` so the main script (or any other
#  caller) can import them directly. Tune these to match the rig.
# =======================================================================

ARM = "ur_left"


# Hand-recorded waypoints sourced from the 2026-05-05 calibration
# pass: ``logs/waypoints/ur_left_open_microwave_1.json``. The earlier
# 2026-04-30 recording is stale (different microwave position + hook
# TCP calibration) — its poses no longer IK cleanly under the current
# planning sim. Hardcoded here as the canonical fallback so callers
# don't have to load JSON at runtime.

OPEN_MICROWAVE_Z_ADJUST_M = -0.010
"""Calibration tweak applied to the recorded open-microwave hook poses.

Negative lowers the hook in task Z. Keep this as a named offset while tuning
on the rig so the original recorded waypoint values remain visible.
"""

# TCP pose when the hook is seated under the door handle, ready to
# pull. Source: snapshot "graspclose" — gripper closed on handle.
HANDLE_ENGAGE_POSE_TASK = Pose(
    translation=np.array([
        -0.03617530952655548,
        0.3478974379645616,
        0.24985614987175653 + OPEN_MICROWAVE_Z_ADJUST_M,
    ]),
    rotation=Rotation.from_rotvec(
        [-1.1542880082655, 1.334799257776183, -1.15436939064674]
    ),
)

# Pre-engage: hook tip clear of the handle before the slide that
# seats it. Uses the full recorded waypoint (translation + rotation)
# — the hook gripper's R_y(π/2) TCP offset means pre-grasp and engage
# have different orientations.
# Source: snapshot "pregrasp".
PRE_ENGAGE_POSE_TASK = Pose(
    translation=np.array([
        -0.07683383775230111,
        0.34557897993134873,
        0.22629524718131488 + OPEN_MICROWAVE_Z_ADJUST_M,
    ]),
    rotation=Rotation.from_rotvec(
        [-1.2072070942247553, 1.3184590951899269, -1.1333357024991244]
    ),
)

# Joint angles at the pre-grasp snapshot (radians). Used by the
# non-motion-planned Phase 1 (moveJ direct to these joints, which
# avoids wrist over-rotation when starting far from the pre-engage
# orientation). Source: snapshot "pregrasp" → joints_rad.
PRE_ENGAGE_JOINTS_RAD = [
    2.1254944801330566,
    -0.6607252520373841,
    1.069571320210592,
    -3.4712687931456507,
    -2.1583827177630823,
    -3.8029139677630823,
]


# Sim-mode "approach home" for Phase 1. The global SIM_HOME_Q_LEFT
# parks the left arm on a totally different IK branch from the recorded
# pregrasp; planning HOME → pregrasp from SIM_HOME has SNOPT pick a
# valid-but-different config (5+ rad of joint-space delta from the
# recorded pregrasp). Using a starting q that's already on the same
# branch — pregrasp's own joints, with the shoulder rotated back ~30°
# so Phase 1 has a visible "arm sweeps in" motion — fixes that.
OPEN_MICROWAVE_APPROACH_Q_LEFT = np.array([
    PRE_ENGAGE_JOINTS_RAD[0] - 0.5,    # shoulder pan back ~29°
    PRE_ENGAGE_JOINTS_RAD[1] - 0.3,    # shoulder lift up a bit
    PRE_ENGAGE_JOINTS_RAD[2],
    PRE_ENGAGE_JOINTS_RAD[3],
    PRE_ENGAGE_JOINTS_RAD[4],
    PRE_ENGAGE_JOINTS_RAD[5],
])
"""Approach config used by ``run_sim`` and ``dry_run_motion_planning``
in place of ``SIM_HOME_Q_LEFT`` for this task. Same IK branch as
``PRE_ENGAGE_JOINTS_RAD`` so Phase 1's IK chain converges to the
recorded pregrasp config instead of jumping branches."""


# -----------------------------------------------------------------------
#  Recorded Phase 3 (door-arc) waypoints
# -----------------------------------------------------------------------
# Three task-frame TCP poses recorded as the hook physically pulled the
# microwave door open during the May 5 calibration pass. Source:
# ``logs/waypoints/ur_left_open_microwave_1.json``. Snapshots ``opendoor``,
# ``opengraspopendoor``, ``slideoutdoorhandle`` — in chronological order.
#
# Use these in place of the math-generated arc (``_arc_waypoints``) so
# every Phase 3 waypoint is a configuration the rig has already reached
# without IK error. The mathematically-generated arc can land at
# workspace boundaries that fail Drake / ikfast IK — the recorded poses
# are guaranteed feasible because they were physically executed.

OPEN_DOOR_POSE_TASK = Pose(
    translation=np.array([
        -0.4083709932963583,
        0.06113811160757704,
        0.263433329622522 + OPEN_MICROWAVE_Z_ADJUST_M,
    ]),
    rotation=Rotation.from_rotvec(
        [0.06827493549060357, -2.131608284023123, 2.033609070101807]
    ),
)

OPEN_GRASP_OPEN_DOOR_POSE_TASK = Pose(
    translation=np.array([
        -0.4114166630036591,
        0.06657335691713845,
        0.2722179065729067 + OPEN_MICROWAVE_Z_ADJUST_M,
    ]),
    rotation=Rotation.from_rotvec(
        [0.06885526218077948, -2.150849042820588, 2.021959198088254]
    ),
)

SLIDE_OUT_DOOR_HANDLE_POSE_TASK = Pose(
    translation=np.array([
        -0.4125380104176084,
        0.09050882584839098,
        0.27441533228926074 + OPEN_MICROWAVE_Z_ADJUST_M,
    ]),
    rotation=Rotation.from_rotvec(
        [0.11437845602669702, -2.102808951678928, 2.1423445092868065]
    ),
)

RECORDED_ARC_WAYPOINTS_TASK: List[Pose] = [
    OPEN_DOOR_POSE_TASK,
    OPEN_GRASP_OPEN_DOOR_POSE_TASK,
    SLIDE_OUT_DOOR_HANDLE_POSE_TASK,
]
"""Phase 3 intermediate waypoints, in execution order. Engage pose is
prepended automatically by the planner."""


# Door geometry — see ``control_scripts/microwave.py`` for the full
# microwave constants. Door is 36 cm wide; full front face is 44 cm
# but the rightmost 8 cm is the fixed control-panel face.
DOOR_WIDTH_M = 0.36

# Hinge axis: vertical, at the outer-left edge of the door's front
# face, in the door plane. Derived from microwave geometry constants
# rather than from the recorded handle pose so it stays consistent
# when waypoints are re-recorded.
#   x = MICROWAVE_HINGE_X        (-0.380, outer-left edge)
#   y = door_plane_y()           (+0.375, front face plane)
#   z = handle z                 (planar arc; only xy matters)
# Hardcoded resolved value below as a backup so this file is self-
# contained; verify against the microwave constants if you change
# either.
from ..microwave import MICROWAVE_HINGE_X as _MICROWAVE_HINGE_X
from ..microwave import door_plane_y as _door_plane_y

HINGE_POSITION_TASK = np.array([
    _MICROWAVE_HINGE_X,                              # -0.380
    _door_plane_y(),                                 # +0.375
    HANDLE_ENGAGE_POSE_TASK.translation[2],          # +0.250 (handle z)
])
# Resolved (current values): [-0.380, +0.375, +0.250]
# Distance from this hinge to HANDLE_ENGAGE_POSE_TASK xy ≈ 0.345 m.

# Stale derivation kept for reference — DO NOT use.
# _HINGE_DIR = np.array([-1.0, 0.0, 0.0]) / np.sqrt(2.0)
# HINGE_POSITION_TASK_OLD = HANDLE_ENGAGE_POSE_TASK.translation + DOOR_WIDTH_M * _HINGE_DIR


CONFIG = PickPlaceConfig(
    # transit_z is in TASK frame. Handle is at ~0.158 m; 0.25 m gives
    # ~9 cm clearance above it and the microwave housing.
    transit_z=0.25,
    transit_speed=0.15,
    transit_accel=0.3,
    approach_speed=0.04,    # slow final slide onto handle + arc steps
    approach_accel=0.1,
    retract_speed=0.10,
    retract_accel=0.2,
)


DOOR_SPEC = MicrowaveDoorSpec(
    handle_engage_pose_task=HANDLE_ENGAGE_POSE_TASK,
    pre_engage_joints_rad=PRE_ENGAGE_JOINTS_RAD,
    pre_engage_pose_task=PRE_ENGAGE_POSE_TASK,

    # Phase-3 path: prefer the recorded waypoints from May 5 — they
    # round-trip cleanly through both Drake and ikfast IK. Math-generated
    # arc params (``hinge_position_task`` + ``arc_open_angle_rad`` + ``n_arc_steps``)
    # are kept as a fallback for hinge geometries we don't have a recording
    # for; ``recorded_arc_waypoints_task`` overrides them when set.
    recorded_arc_waypoints_task=RECORDED_ARC_WAYPOINTS_TASK,
    hinge_position_task=HINGE_POSITION_TASK,
    arc_open_angle_rad=1.8,    # ≈ 103° — tune until door is visually fully open
    n_arc_steps=14,             # moveL waypoints along the arc

    # pull_direction_task drives the arc-rotation sign check.
    # -X, -Y = door swings diagonally toward operator and to the left.
    pull_direction_task=np.array([-1.0, -1.0, 0.0]),

    # Force-mode params (only used if hinge_position_task is None).
    pull_force_n=20.0,
    pull_distance_task=0.25,
    pull_speed_limit=0.05,
    pull_timeout_s=8.0,
    disengage_offset=0.03,

    # Motion-planned mode — opt-in. When True, Phase 1 + Phase 3 run
    # through Drake KTO using the rig's actual current joints as the
    # planning start state.
    use_motion_planning=False,
    motion_plan_n_waypoints=30,
    motion_plan_tcp_speed=0.05,
)
