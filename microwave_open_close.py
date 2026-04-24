"""
Microwave door open/close demo using UR5e left arm + hook gripper.

Uses Drake InverseKinematics to solve Cartesian waypoints along the
microwave door's swing arc.  Kinematic playback — sets joint angles
directly and publishes to Meshcat (no dynamic simulation).

Usage:
    conda run -n puckbot python microwave_open_close.py
"""
from __future__ import annotations

import time
import numpy as np

from pydrake.all import (
    DiagramBuilder,
    InverseKinematics,
    MeshcatVisualizer,
    RigidTransform,
    RotationMatrix,
    RollPitchYaw,
    Simulator,
    StartMeshcat,
    Solve,
)
from pydrake.multibody.parsing import PackageMap
from pydrake.visualization import AddFrameTriadIllustration

from manipulation.station import LoadScenario, MakeHardwareStation


# ──────────────────────────────────────────────────────────────────────
# Microwave geometry
# ──────────────────────────────────────────────────────────────────────
# microwave.xml body at pos="0 1 0.71"
# Door hinge (microdoorroot) from body: (-0.345, -0.176, 0.192)
# Handle from hinge (in door frame): (0.475, -0.108, 0)
# Handle bar runs along world Z (vertical capsule)
#
# Hinge in world  = (-0.345, 0.824, 0.902)
# Handle (closed) = ( 0.130, 0.716, 0.902)

MICROWAVE_BODY_POS = np.array([0.0, 1.0, 0.71])
HINGE_LOCAL = np.array([-0.345, -0.176, 0.192])
HINGE_WORLD = MICROWAVE_BODY_POS + HINGE_LOCAL

HANDLE_FROM_HINGE = np.array([0.475, -0.108, 0.0])


def handle_world_pos(door_angle: float) -> np.ndarray:
    """World position of the microwave handle at a given door angle."""
    c, s = np.cos(door_angle), np.sin(door_angle)
    rotated = np.array([
        c * HANDLE_FROM_HINGE[0] - s * HANDLE_FROM_HINGE[1],
        s * HANDLE_FROM_HINGE[0] + c * HANDLE_FROM_HINGE[1],
        HANDLE_FROM_HINGE[2],
    ])
    return HINGE_WORLD + rotated


# ──────────────────────────────────────────────────────────────────────
# Gripper orientation for the task
# ──────────────────────────────────────────────────────────────────────
# The hook_gripper mesh extends in +X (local).  For the hook to wrap
# around the vertical handle bar:
#   • Gripper +X  →  toward the microwave  (direction changes with door arc)
#   • Gripper +Z  →  world -Z  (down, so the hook plane is horizontal)
#
# At door_angle=0, the approach direction from robot to handle is +Y.
# As the door swings open (negative angle), the approach direction rotates.

def gripper_orientation_for_door(door_angle: float) -> RotationMatrix:
    """Desired gripper orientation when the hook is at the given door angle.

    The approach direction rotates with the door so the hook stays aligned
    with the handle bar throughout the arc.
    """
    # Direction from hinge to handle (in XY plane), rotated by door_angle
    # This is the direction the handle sticks out from the hinge
    c, s = np.cos(door_angle), np.sin(door_angle)
    handle_dir = np.array([
        c * HANDLE_FROM_HINGE[0] - s * HANDLE_FROM_HINGE[1],
        s * HANDLE_FROM_HINGE[0] + c * HANDLE_FROM_HINGE[1],
        0.0,
    ])
    handle_dir /= np.linalg.norm(handle_dir)

    # Gripper X → handle direction (toward microwave body from handle)
    # Gripper Z → down
    # Gripper Y → cross(Z, X)
    gz = np.array([0.0, 0.0, -1.0])   # down
    gx = handle_dir                     # toward microwave from handle
    gy = np.cross(gz, gx)
    gy /= np.linalg.norm(gy)

    # Rotation matrix: columns are gripper axes in world frame
    R = np.column_stack([gx, gy, gz])
    return RotationMatrix(R)


def solve_ik(plant, plant_context, target_pos, ee_frame,
             q_seed, ur_joint_indices, other_joint_indices,
             orientation_target=None,
             pos_tol=0.01, orient_tol=0.25, n_retries=25):
    """Solve IK with locked non-arm joints and smooth-cost."""
    ik = InverseKinematics(plant, plant_context)
    prog = ik.prog()
    q = ik.q()

    # Position constraint
    ik.AddPositionConstraint(
        ee_frame, np.zeros(3), plant.world_frame(),
        target_pos - pos_tol, target_pos + pos_tol,
    )

    # Orientation constraint
    if orientation_target is not None:
        ik.AddOrientationConstraint(
            ee_frame, RotationMatrix(),
            plant.world_frame(), orientation_target,
            orient_tol,
        )

    # Lock non-arm joints
    for idx in other_joint_indices:
        prog.AddConstraint(q[idx] >= q_seed[idx] - 1e-6)
        prog.AddConstraint(q[idx] <= q_seed[idx] + 1e-6)

    # Smooth cost: stay near seed
    Q = np.eye(len(q)) * 10.0
    prog.AddQuadraticErrorCost(Q, q_seed, q)

    prog.SetInitialGuess(q, q_seed)
    result = Solve(prog)
    if result.is_success():
        return result.GetSolution(q)

    # Retries: perturb arm joints only
    for i in range(n_retries):
        perturbed = q_seed.copy()
        scale = 0.3 + 0.1 * i
        for idx in ur_joint_indices:
            perturbed[idx] += np.random.uniform(-scale, scale)
        prog.SetInitialGuess(q, perturbed)
        result = Solve(prog)
        if result.is_success():
            return result.GetSolution(q)

    return None


def interpolate_positions(q_waypoints, times, dt=0.02):
    """Linearly interpolate between joint-space waypoints."""
    t_out, q_out = [], []
    for i in range(len(times) - 1):
        t0, t1 = times[i], times[i + 1]
        q0, q1 = q_waypoints[i], q_waypoints[i + 1]
        n_steps = max(int((t1 - t0) / dt), 1)
        for j in range(n_steps):
            alpha = j / n_steps
            t_out.append(t0 + alpha * (t1 - t0))
            q_out.append(q0 + alpha * (q1 - q0))
    t_out.append(times[-1])
    q_out.append(q_waypoints[-1])
    return np.array(t_out), np.array(q_out)


def main():
    meshcat = StartMeshcat()

    # ── Load scenario ──────────────────────────────────────────────
    scenario = LoadScenario(
        filename="scenario_files/microwave_scenario.yaml"
    )
    builder = DiagramBuilder()
    station = builder.AddSystem(MakeHardwareStation(scenario))
    plant = station.GetSubsystemByName("plant")
    scene_graph = station.GetSubsystemByName("scene_graph")

    gripper_body = plant.GetBodyByName("hook_gripper")
    AddFrameTriadIllustration(
        scene_graph=scene_graph, body=gripper_body,
        length=0.1, radius=0.004,
    )

    visualizer = MeshcatVisualizer.AddToBuilder(
        builder, station.GetOutputPort("query_object"), meshcat
    )
    diagram = builder.Build()

    # ── Contexts ───────────────────────────────────────────────────
    diag_context = diagram.CreateDefaultContext()
    plant_context = plant.GetMyMutableContextFromRoot(diag_context)

    ee_frame = plant.GetFrameByName("hook_gripper")
    ur_left = plant.GetModelInstanceByName("ur_left")

    # ── Joint index map ────────────────────────────────────────────
    ur_names = [
        "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
        "wrist_1_joint", "wrist_2_joint", "wrist_3_joint",
    ]
    ur_idx = [plant.GetJointByName(n, ur_left).position_start() for n in ur_names]
    other_idx = sorted(set(range(plant.num_positions())) - set(ur_idx))

    # ── Diagnostics ────────────────────────────────────────────────
    base_pos = plant.CalcRelativeTransform(
        plant_context, plant.world_frame(),
        plant.GetFrameByName("base_link", ur_left)
    ).translation()

    h_closed = handle_world_pos(0.0)
    print(f"\n=== Workspace ===")
    print(f"  UR5e base     : {base_pos}")
    print(f"  Handle closed : {h_closed}")
    print(f"  Base→Handle   : {np.linalg.norm(h_closed - base_pos):.3f}m")

    q_default = plant.GetPositions(plant_context).copy()

    # ── Find task-ready seed ───────────────────────────────────────
    # Solve IK to pre-approach position WITH the correct orientation
    # so the arm starts in a reasonable downward-reaching config.
    pre_app_pos = h_closed + np.array([0.0, -0.10, 0.05])
    orient_closed = gripper_orientation_for_door(0.0)

    print(f"\n=== Finding task-ready seed ===")
    print(f"  Target orient (door=0):\n{orient_closed.matrix()}")

    task_seed = None
    for attempt in range(100):
        q_try = q_default.copy()
        for idx in ur_idx:
            q_try[idx] = np.random.uniform(-np.pi, np.pi)
        # Quick filter: check EE is in the right half of the workspace
        plant.SetPositions(plant_context, q_try)
        ee_p = plant.CalcRelativeTransform(
            plant_context, plant.world_frame(), ee_frame
        ).translation()
        if not (ee_p[2] < 1.2 and ee_p[1] > 0.3 and ee_p[2] > 0.5):
            continue  # EE not in useful region

        q_sol = solve_ik(
            plant, plant_context, pre_app_pos, ee_frame,
            q_try, ur_idx, other_idx,
            orientation_target=orient_closed,
            pos_tol=0.03, orient_tol=0.4,
        )
        if q_sol is not None:
            task_seed = q_sol
            plant.SetPositions(plant_context, q_sol)
            actual = plant.CalcRelativeTransform(
                plant_context, plant.world_frame(), ee_frame
            )
            print(f"  Found at attempt {attempt}")
            print(f"  EE pos: {actual.translation()}")
            print(f"  EE rot:\n{actual.rotation().matrix()}")
            break

    if task_seed is None:
        print("  WARNING: No task-ready seed found. Using default.")
        task_seed = q_default.copy()

    # ── Build waypoints ────────────────────────────────────────────
    door_angles = [0.0, -0.25, -0.50, -0.75, -1.0, -1.25]

    # (name, position, orientation, time)
    specs = []

    # Phase 1: Approach
    specs.append(("task-ready",   pre_app_pos,
                  orient_closed, 0.0))
    specs.append(("pre-approach", h_closed + np.array([0.0, -0.04, 0.0]),
                  orient_closed, 2.0))
    specs.append(("at-handle",    h_closed.copy(),
                  orient_closed, 3.5))
    specs.append(("hooked",       h_closed + np.array([0.0, 0.02, 0.0]),
                  orient_closed, 4.5))

    # Phase 2: Pull door open along arc
    for i, angle in enumerate(door_angles[1:], start=1):
        ori = gripper_orientation_for_door(angle)
        specs.append((f"arc-{i}", handle_world_pos(angle),
                      ori, 4.5 + i * 2.0))

    # Phase 3: Release and return
    t_last = specs[-1][3]
    h_open = handle_world_pos(door_angles[-1])
    ori_open = gripper_orientation_for_door(door_angles[-1])
    specs.append(("release",     h_open + np.array([0.0, 0.0, 0.10]),
                  ori_open, t_last + 1.5))
    specs.append(("task-return", pre_app_pos,
                  orient_closed, t_last + 4.0))

    # ── Solve IK ───────────────────────────────────────────────────
    print(f"\n=== Solving IK ({len(specs)} waypoints) ===")
    q_solutions = []
    q_seed = task_seed.copy()

    for name, pos, ori, t in specs:
        dist = np.linalg.norm(pos - base_pos)
        q_sol = solve_ik(
            plant, plant_context, pos, ee_frame,
            q_seed, ur_idx, other_idx,
            orientation_target=ori,
            pos_tol=0.015, orient_tol=0.3,
        )
        if q_sol is not None:
            q_solutions.append(q_sol)
            q_seed = q_sol.copy()
            plant.SetPositions(plant_context, q_sol)
            actual = plant.CalcRelativeTransform(
                plant_context, plant.world_frame(), ee_frame
            ).translation()
            err = np.linalg.norm(actual - pos)
            print(f"  ✓ [{name:14s}] t={t:5.1f}s  d={dist:.3f}m  err={err:.4f}m")
        else:
            print(f"  ✗ [{name:14s}] t={t:5.1f}s  d={dist:.3f}m  FAILED")
            q_solutions.append(q_seed.copy())

    times = [s[3] for s in specs]

    # ── Trajectory ─────────────────────────────────────────────────
    t_traj, q_traj = interpolate_positions(q_solutions, times, dt=0.02)
    total = t_traj[-1]
    print(f"\n=== Trajectory: {total:.1f}s, {len(t_traj)} pts ===")

    # ── Playback ───────────────────────────────────────────────────
    print(f"\n  ▶  Open Meshcat to watch.  Runs {total:.0f}s.\n")
    meshcat.AddButton("Stop Simulation")
    t0 = time.time()

    while meshcat.GetButtonClicks("Stop Simulation") < 1:
        elapsed = time.time() - t0
        idx = min(np.searchsorted(t_traj, elapsed), len(t_traj) - 1)
        plant.SetPositions(plant_context, q_traj[idx])
        diagram.ForcedPublish(diag_context)
        time.sleep(0.02)
        if elapsed > total + 1.0:
            print("  Done — press 'Stop Simulation'.")
            while meshcat.GetButtonClicks("Stop Simulation") < 1:
                time.sleep(0.5)
            break

    print("\nDone!")


if __name__ == "__main__":
    main()
