"""End-to-end IK feasibility check for the open_microwave Phase 1 + 3 chain.

The actual ``plan_transit`` IKs each waypoint sequentially, seeding each
IK with the *previous* waypoint's solution. We replicate that chaining
here against the May 5 recorded waypoints in
``logs/waypoints/ur_left_open_microwave_1.json``, plus an explicit
SIM_HOME → pregrasp leg for Phase 1.

For each chained step we run BOTH:
  * Drake ``InverseKinematics`` (SNOPT, the production path)
  * IKFast analytic IK (the new RRT-fallback path)

and report whether each step would succeed in practice.

A green row means the recorded pose is reachable under the current
plant + TCP calibration when seeded with the previous waypoint's
joints — exactly what would happen during a real run that started
near the recorded pregrasp pose.
"""

from __future__ import annotations

import json
import sys
from pathlib import Path
from typing import List, Optional, Tuple

import numpy as np
from pydrake.math import RigidTransform, RotationMatrix
from scipy.spatial.transform import Rotation as ScipyRotation

from . import SIM_HOME_Q_LEFT, SIM_HOME_Q_RIGHT, default_home_q
from .build_scene import build_scene
from .ikfast import solve_ik as ikfast_solve_ik
from .rrt import _arm_calibration
from .transit import (
    _arm_model_instance,
    _arm_position_indices,
    _ik_pose_to_joints as _drake_ik,
)
from .verify_ikfast_real import _pose_from_tx_rotvec, _tcp_offset_for
from ..util.poses import Pose
from ..util.rotations import Rotation


JSON_PATH = (
    Path(__file__).resolve().parents[2]
    / "logs" / "waypoints" / "ur_left_open_microwave_1.json"
)


def _load_open_microwave_json() -> dict:
    """Load the open-microwave waypoints (handles missing-brace bug)."""
    raw = JSON_PATH.read_text()
    try:
        return json.loads(raw)
    except json.JSONDecodeError:
        return json.loads("{\n" + raw)


def _world_pose_from_arm_base(plant, plant_ctx, arm_inst,
                              X_arm_base_pose: RigidTransform) -> RigidTransform:
    base_frame = plant.GetFrameByName("base", arm_inst)
    return base_frame.CalcPoseInWorld(plant_ctx) @ X_arm_base_pose


def main(arm_name: str = "ur_left",
         pos_tol: float = 0.005,
         rot_tol: float = 0.05) -> int:
    scene = build_scene()
    plant = scene.plant
    diag_ctx = scene.diagram.CreateDefaultContext()
    plant_ctx = plant.GetMyMutableContextFromRoot(diag_ctx)

    arm_inst = _arm_model_instance(plant, arm_name)
    other_arm = "ur_left" if arm_name == "ur_right" else "ur_right"
    other_inst = _arm_model_instance(plant, other_arm)
    arm_idx = _arm_position_indices(plant, arm_inst)
    other_idx = _arm_position_indices(plant, other_inst)

    home = default_home_q(plant)
    other_q = home[other_idx]

    # --- Build the chained-step test sequence -----------------------------
    # Step 0: Phase 1 cold-start  — SIM_HOME_Q_LEFT → pregrasp
    # Steps 1..n: Phase 3 chain    — graspclose → opendoor → … (per JSON)
    #
    # Seed for step k = q_chain after step k-1's IK succeeded.
    # Target for step k = the k-th recorded TCP base_pose.

    data = _load_open_microwave_json()
    snapshots = data["snapshots"]
    by_name = {s["name"]: s for s in snapshots}

    # Phase 1 leg (HOME -> pregrasp): seed = SIM_HOME_Q_LEFT, target = pregrasp.
    # Phase 3 chain: graspclose → opendoor → opengraspopendoor → slideoutdoorhandle.
    chain_names = ["graspclose", "opendoor", "opengraspopendoor",
                   "slideoutdoorhandle"]

    sim_home = (SIM_HOME_Q_LEFT if arm_name == "ur_left" else SIM_HOME_Q_RIGHT)

    steps: List[Tuple[str, np.ndarray, dict]] = [
        ("Phase 1: SIM_HOME → pregrasp", np.asarray(sim_home, dtype=float),
         by_name["pregrasp"]),
    ]
    # Phase 3 chain seeds with the previous recorded q each time. The
    # very first Phase 3 IK seeds with graspclose's own joints (i.e. the
    # arm has just landed there after Phase 2's hand-coded sequence).
    prev_seed_arm_q: Optional[np.ndarray] = None
    for nm in chain_names:
        snap = by_name[nm]
        if prev_seed_arm_q is None:
            seed_arm_q = np.asarray(snap["joints_rad"], dtype=float)
            label = f"Phase 3: (start at {nm})"
        else:
            seed_arm_q = prev_seed_arm_q
            label = f"Phase 3: chain → {nm}"
        steps.append((label, seed_arm_q, snap))
        prev_seed_arm_q = np.asarray(snap["joints_rad"], dtype=float)

    # Per-arm TCP offset for ikfast wrist conversion.
    tcp_t, tcp_r = _tcp_offset_for(JSON_PATH.name, arm_name)
    X_wrist_tcp = _pose_from_tx_rotvec(tcp_t, tcp_r)
    X_tcp_wrist = X_wrist_tcp.inverse()

    # Drake IK setup helpers.
    def run_drake(target_world_pose: Pose, seed_arm_q: np.ndarray
                  ) -> Tuple[Optional[np.ndarray], str]:
        seed_full = home.copy()
        seed_full[arm_idx] = seed_arm_q
        try:
            q_full = _drake_ik(
                plant, plant_ctx, arm_inst, other_inst, other_q,
                target_world_pose, seed_full,
                pos_tolerance_m=pos_tol, rot_tolerance_rad=rot_tol,
            )
            return q_full[arm_idx], "OK"
        except Exception as exc:
            return None, str(exc).splitlines()[0]

    def run_ikfast(X_base_wrist3: RigidTransform, seed_arm_q: np.ndarray
                   ) -> Tuple[Optional[np.ndarray], str]:
        sols = ikfast_solve_ik(X_base_wrist3, seed_q=seed_arm_q)
        if not sols:
            return None, "no-sol"
        return np.asarray(sols[0]), "OK"

    # --- Run the chain ----------------------------------------------------
    print()
    print("=" * 110)
    print(f"  Open-microwave IK chain  (arm={arm_name}, "
          f"pos_tol={pos_tol*1000:.0f}mm, rot_tol={np.degrees(rot_tol):.1f}°)")
    print("=" * 110)
    print(f"{'step':<38}  {'drake':<26}  {'ikfast':<26}  {'recorded q match':<16}")

    for label, seed_arm_q, snap in steps:
        q_rec = np.asarray(snap["joints_rad"], dtype=float)
        # Recorded TCP in arm base.
        t = snap["base_pose"]["translation"]
        r = snap["base_pose"]["rotvec"]
        X_base_tcp_recorded = _pose_from_tx_rotvec(t, r)
        X_base_wrist3_recorded = X_base_tcp_recorded @ X_tcp_wrist

        # World pose for Drake (arm-base → world via the welded base frame).
        X_world_tcp = _world_pose_from_arm_base(
            plant, plant_ctx, arm_inst, X_base_tcp_recorded,
        )
        target_world_pose = Pose(
            translation=np.asarray(X_world_tcp.translation()),
            rotation=Rotation.from_matrix(np.asarray(X_world_tcp.rotation().matrix())),
        )

        q_drake, drake_msg = run_drake(target_world_pose, seed_arm_q)
        q_ikfast, ikfast_msg = run_ikfast(X_base_wrist3_recorded, seed_arm_q)

        def fmt_dq(q: Optional[np.ndarray], status_msg: str) -> str:
            if q is None:
                return f"FAIL ({status_msg[:18]})"
            dq = np.linalg.norm(q - q_rec)
            return f"OK  ‖Δq‖={dq*1000:>5.1f}mrad"

        # Whether either solver returns a config near the recorded q.
        match_status = "—"
        if q_drake is not None:
            match_status = (f"drake≈qrec" if np.linalg.norm(q_drake - q_rec) < 0.05
                            else "drake≠qrec")
        if q_ikfast is not None:
            match_status = (match_status + " ✓ikfast"
                            if np.linalg.norm(q_ikfast - q_rec) < 0.05
                            else match_status + " ✗ikfast")

        print(f"  {label:<36}  "
              f"{fmt_dq(q_drake, drake_msg):<26}  "
              f"{fmt_dq(q_ikfast, ikfast_msg):<26}  "
              f"{match_status:<16}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
