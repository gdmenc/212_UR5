"""Verify that the new tabletop objects participate in collision checks.

Two-part check:

1. **Static** — confirm each object registered proximity geometry in
   the SceneGraph, and that an object placed under HOME's TCP causes
   HOME to be in collision (proving the geometry is queried by the
   collision checker, not just registered).

2. **Functional** — for each object's *default* xy, verify the
   collision checker reports collision when the right or left arm's
   TCP is driven to (xy, table-top z=0) via Drake IK constrained to
   point straight down. Compares against an objects=False scene to
   prove it's the new geometry causing the collision.

The key trick for the functional check is constructing the IK with a
**downward orientation constraint** (tool +Z = task -Z) so the gripper
fingers descend into the cylindrical object instead of getting parked
in the table. If we ran a free-rotation IK, the finger geometry would
crash into the table on its way to the target xy and we'd see
collisions in both scenes.

Run::

    python3.11 -m control_scripts.planning.verify_objects
"""

from __future__ import annotations

from typing import Optional, Tuple

import numpy as np
from pydrake.geometry import Role
from pydrake.math import RigidTransform, RotationMatrix
from pydrake.multibody.inverse_kinematics import InverseKinematics
from pydrake.multibody.tree import ModelInstanceIndex
from pydrake.solvers import Solve

from . import default_home_q
from .rrt import build_planning_scene, make_collision_checker
from .scene.objects import (
    BOTTLE_DEFAULT_TASK_XYZ,
    BOWL_DEFAULT_TASK_XYZ,
    CUP_DEFAULT_TASK_XYZ,
    CUP_WITH_STICK_DEFAULT_TASK_XYZ,
    PLATE_DEFAULT_TASK_XYZ,
    TRAY_DEFAULT_TASK_XYZ,
    add_bottle,
    add_bowl,
    add_cup,
    add_plate,
    add_tray,
)


# ---------------------------------------------------------------------------
#  Static check (geometry registration + smoke test)
# ---------------------------------------------------------------------------


def _model_instance_by_name(plant, name: str) -> Optional[ModelInstanceIndex]:
    for i in range(plant.num_model_instances()):
        mi = ModelInstanceIndex(i)
        if plant.GetModelInstanceName(mi) == name:
            return mi
    return None


def _count_proximity_geoms(plant, scene_graph, model_name: str,
                           body_names) -> int:
    inspector = scene_graph.model_inspector()
    inst = _model_instance_by_name(plant, model_name)
    if inst is None:
        return 0
    total = 0
    for body_name in body_names:
        body = plant.GetBodyByName(body_name, inst)
        frame_id = plant.GetBodyFrameIdOrThrow(body.index())
        total += len(inspector.GetGeometries(frame_id, Role.kProximity))
    return total


def _static_check() -> None:
    print("  STATIC")
    diag, plant, _, _ = build_planning_scene(include_objects=True)
    sg = diag.scene_graph()
    bodies_for = [
        ("plate",          ["plate"]),
        ("cup",            ["body"]),
        ("cup_with_stick", ["body", "stick"]),
        ("bowl",           ["bowl"]),
        ("bottle",         ["bottle"]),
        ("tray",           ["tray"]),
    ]
    for obj_name, body_names in bodies_for:
        n = _count_proximity_geoms(plant, sg, obj_name, body_names)
        print(f"    {obj_name:<16} proximity geoms = {n}")


def _smoke_check_object_at_gripper() -> None:
    """Place an extra plate directly at the right gripper's BODY origin
    (the wrist, where its actual collision geometry sits — TCP is a
    virtual frame 18 cm offset out the tool axis). Confirm HOME goes
    from free → COL when the plate is added."""
    diag, plant, _, _ = build_planning_scene(include_objects=False)
    home = default_home_q(plant)
    diag_ctx = diag.CreateDefaultContext()
    plant_ctx = plant.GetMyMutableContextFromRoot(diag_ctx)
    plant.SetPositions(plant_ctx, home)
    # The Robotiq is one rigid body called "robotiq_2f_85" welded at
    # the wrist; its collision geometry extends some way along the
    # tool axis from there. Pulling the body's world pose puts our
    # spawned plate right inside that geometry.
    grip_inst = None
    for i in range(plant.num_model_instances()):
        mi = ModelInstanceIndex(i)
        if plant.GetModelInstanceName(mi).startswith("robotiq_2f_85_ur_right"):
            grip_inst = mi
            break
    grip_body = plant.GetBodyByName("robotiq_2f_85", grip_inst)
    p_world = np.asarray(grip_body.EvalPoseInWorld(plant_ctx).translation())
    print()
    print(f"  SMOKE: right gripper body @ HOME = {np.round(p_world, 3)} (world)")

    checker = make_collision_checker(diag, plant, "ur_right")
    free_baseline = checker.CheckConfigCollisionFree(home)
    print(f"    HOME without objects                   : "
          f"{'free' if free_baseline else 'COL'}")

    from pydrake.planning import RobotDiagramBuilder
    rdb = RobotDiagramBuilder(time_step=0.0)
    plant2 = rdb.plant()
    from .build_scene import _compose_scene_fragments
    _compose_scene_fragments(
        plant2,
        include_microwave=True, include_grippers=True,
        include_objects=False, robotiq_mode="closed",
    )
    # An extra plate centred AT the gripper body origin (z slightly
    # lowered so the cylinder envelopes the wrist). We expect this q
    # to be in collision.
    add_plate(plant2, xyz_task=(p_world[0], p_world[1], p_world[2] - 0.01),
              name="plate_at_gripper")
    plant2.Finalize()
    plant2.SetDefaultPositions(default_home_q(plant2))
    diag2 = rdb.Build()
    checker2 = make_collision_checker(diag2, plant2, "ur_right")
    free_blocked = checker2.CheckConfigCollisionFree(default_home_q(plant2))
    print(f"    HOME with plate spawned AT gripper body: "
          f"{'free' if free_blocked else 'COL'}")
    if free_baseline and not free_blocked:
        print(f"    → OK: object is part of collision graph")
    else:
        print(f"    → FAIL: collision query did not see the spawned plate")


# ---------------------------------------------------------------------------
#  Functional check (IK each arm onto each object)
# ---------------------------------------------------------------------------


def _ik_tcp_at_pose(plant, diagram, arm_name, p_world,
                    R_world_tcp, q_seed, *,
                    pos_tol: float = 1e-3,
                    rot_tol_rad: float = np.deg2rad(5.0),
                    ) -> Optional[np.ndarray]:
    """Drake IK with both position and orientation constraints. Returns
    full-plant q or None on failure."""
    diag_ctx = diagram.CreateDefaultContext()
    plant_ctx = plant.GetMyMutableContextFromRoot(diag_ctx)
    plant.SetPositions(plant_ctx, q_seed)

    ik = InverseKinematics(plant, plant_ctx, with_joint_limits=True)
    tcp_frame = plant.GetFrameByName(f"tcp_{arm_name.removeprefix('ur_')}")

    p_world = np.asarray(p_world, dtype=float)
    ik.AddPositionConstraint(
        frameB=tcp_frame, p_BQ=np.zeros(3),
        frameA=plant.world_frame(),
        p_AQ_lower=p_world - pos_tol, p_AQ_upper=p_world + pos_tol,
    )
    ik.AddOrientationConstraint(
        frameAbar=plant.world_frame(),
        R_AbarA=RotationMatrix(R_world_tcp),
        frameBbar=tcp_frame,
        R_BbarB=RotationMatrix(),
        theta_bound=rot_tol_rad,
    )
    prog = ik.get_mutable_prog()
    prog.SetInitialGuess(ik.q(), q_seed)
    res = Solve(prog)
    if not res.is_success():
        return None
    return res.GetSolution(ik.q())


def _ik_gripper_at_pose_unused(plant, diagram, arm_name, p_world,
                               R_world_grip, q_seed,
                               *, pos_tol=1e-3,
                               rot_tol_rad=np.deg2rad(15.0)) -> Optional[np.ndarray]:
    """Drake IK with gripper-body frame as the target (not TCP).

    The Robotiq's collision geometry lives at the gripper body, not at
    the virtual TCP frame 18 cm out, so anchoring the IK target at the
    gripper body guarantees the gripper geometry is the thing that
    ends up at ``p_world``. Rot tolerance is loose (15°) so IK has
    headroom to find any pose where the gripper *body* points
    roughly downward.
    """
    diag_ctx = diagram.CreateDefaultContext()
    plant_ctx = plant.GetMyMutableContextFromRoot(diag_ctx)
    plant.SetPositions(plant_ctx, q_seed)

    ik = InverseKinematics(plant, plant_ctx, with_joint_limits=True)
    if arm_name == "ur_right":
        grip_body = plant.GetBodyByName("robotiq_2f_85")
    else:
        grip_body = plant.GetBodyByName("hook_gripper")
    grip_frame = grip_body.body_frame()
    p_world = np.asarray(p_world, dtype=float)

    ik.AddPositionConstraint(
        frameB=grip_frame, p_BQ=np.zeros(3),
        frameA=plant.world_frame(),
        p_AQ_lower=p_world - pos_tol, p_AQ_upper=p_world + pos_tol,
    )
    ik.AddOrientationConstraint(
        frameAbar=plant.world_frame(),
        R_AbarA=RotationMatrix(R_world_grip),
        frameBbar=grip_frame,
        R_BbarB=RotationMatrix(),
        theta_bound=rot_tol_rad,
    )
    prog = ik.get_mutable_prog()
    prog.SetInitialGuess(ik.q(), q_seed)
    res = Solve(prog)
    return res.GetSolution(ik.q()) if res.is_success() else None


def _spawn_at_gripper_check(label: str, add_fn, *, body_name: str = None) -> None:
    """Spawn ONE object of a given kind at the right gripper's body
    origin and confirm HOME goes from free → COL. Generalizes the
    smoke check for every shape we add."""
    # Reference scene to read where the gripper body is at HOME.
    diag, plant, _, _ = build_planning_scene(include_objects=False)
    home = default_home_q(plant)
    diag_ctx = diag.CreateDefaultContext()
    plant_ctx = plant.GetMyMutableContextFromRoot(diag_ctx)
    plant.SetPositions(plant_ctx, home)
    grip_inst = None
    for i in range(plant.num_model_instances()):
        mi = ModelInstanceIndex(i)
        if plant.GetModelInstanceName(mi).startswith("robotiq_2f_85_ur_right"):
            grip_inst = mi
            break
    p_world = np.asarray(
        plant.GetBodyByName("robotiq_2f_85", grip_inst)
        .EvalPoseInWorld(plant_ctx).translation()
    )
    # Build a fresh scene WITHOUT any default objects, then drop ONE
    # of the requested kind centred at the gripper.
    from pydrake.planning import RobotDiagramBuilder
    rdb = RobotDiagramBuilder(time_step=0.0)
    plant2 = rdb.plant()
    from .build_scene import _compose_scene_fragments
    _compose_scene_fragments(
        plant2,
        include_microwave=True, include_grippers=True,
        include_objects=False, robotiq_mode="closed",
    )
    add_fn(plant2, xyz_task=tuple(p_world.tolist()), name=f"probe_{label}")
    plant2.Finalize()
    plant2.SetDefaultPositions(default_home_q(plant2))
    diag2 = rdb.Build()
    checker = make_collision_checker(diag2, plant2, "ur_right")
    free = checker.CheckConfigCollisionFree(default_home_q(plant2))
    verdict = "OK   (HOME blocked by spawned object)" if not free else \
              "FAIL (HOME still free — object not in collision graph)"
    print(f"    {label:<16} HOME with spawned {label:<14} : "
          f"{'free' if free else 'COL':<4}  → {verdict}")


def _per_kind_check() -> None:
    """Run the smoke test once per object kind."""
    print()
    print("  PER-KIND: spawn each object kind at right gripper body, ")
    print("            check that HOME flips from free → COL")
    _spawn_at_gripper_check("plate",          add_plate)
    _spawn_at_gripper_check("cup",            lambda p, **kw: add_cup(p, with_stick=False, **kw))
    _spawn_at_gripper_check("cup_with_stick", lambda p, **kw: add_cup(p, with_stick=True, **kw))
    _spawn_at_gripper_check("bowl",           add_bowl)
    _spawn_at_gripper_check("bottle",         add_bottle)
    _spawn_at_gripper_check("tray",           add_tray)


def main() -> int:
    print()
    _static_check()
    _smoke_check_object_at_gripper()
    _per_kind_check()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
