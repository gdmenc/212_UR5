"""Pick primitive: approach, grasp, lift.

Assumes the scene can provide:
  - ``scene.object_pose(name)`` -> world-frame pose of the object,
  - ``scene.grasp_candidates(name)`` -> list of X_OG (grasp pose in the
    object frame) to try in order. Single-grasp objects return a 1-element
    list; symmetric objects (bottle) return a rotation ring.

High-level flow:

    for each grasp candidate:
        X_WG = X_WO * X_OG
        X_WPre = X_WG * RigidTransform([0, 0, -standoff])
        q_goal = solve_ik(plant, X_WPre, q_seed=scene.q(arm))
        if q_goal is None: continue
        traj = plan_trajopt(plant, scene.q(arm), q_goal) or
               plan_rrt_connect(plant, scene.q(arm), q_goal)
        if traj is None: continue
        backend.execute(arm, ServoStream(sample(traj)))
        backend.execute(arm, MoveL(X_WG))            # straight-line approach
        backend.execute(arm, GripperCommand("grasp", force=20))
        if backend.gripper(arm).status()["holding"]:
            backend.execute(arm, MoveL(X_WG * lift))
            return PrimitiveResult(success=True, data={"grasp": X_WG})
    return PrimitiveResult(success=False, reason="no feasible grasp")
"""

from typing import Optional

from .base import PrimitiveResult


def pick(
    backend,
    planner,
    scene,
    object_name: str,
    arm: str = "ur_left",
    standoff: float = 0.12,   # meters; distance above grasp for pre-grasp
    grasp_force: float = 20,  # Newtons
    lift_height: float = 0.1,  # meters above the object after grasping
) -> PrimitiveResult:
    """Move to pre-grasp, approach straight-line, close gripper, lift.

    Returns a PrimitiveResult with ``data["grasp"]`` set to the X_WG that
    actually succeeded (useful for the subsequent place primitive).
    """
    # TODO
    raise NotImplementedError
