"""Compare ikfast and Drake IK against recorded waypoint joint angles.

Each ``logs/waypoints/*.json`` snapshot has a recorded joint
configuration ``q_rec`` AND the controller's reported TCP pose
``base_pose`` (in arm base frame). The recorded pose is what the UR
controller said the TCP was at when the arm was at ``q_rec`` — i.e.,
``base_pose`` ≈ controller_FK(q_rec).

This script asks: given the controller's TCP pose, do our IKs
recover the controller's joint configuration?

For each snapshot we:
  1. Strip the per-arm/date TCP offset from base_pose to recover the
     wrist_3 pose in arm base frame.
  2. ``ikfast.solve_ik(X_base_wrist3, seed=q_rec)`` → ``q_ikfast``,
     the closest analytic IK branch to the seed.
  3. Drake ``InverseKinematics`` with PositionConstraint +
     OrientationConstraint at the TCP frame, seeded with q_rec full
     plant → ``q_drake``.
  4. Report ``‖q − q_rec‖`` (joint-space distance) and the largest
     per-joint deviation.

Run::

    python3.11 -m control_scripts.planning.verify_ik_recorded
    python3.11 -m control_scripts.planning.verify_ik_recorded --pos-tol 0.01 --rot-tol 0.10

Tight tolerances will produce more Drake "infeasible" failures even
when the round-trip is geometrically OK; loosening them isolates "is
this a tolerance issue or a real branch / FK mismatch issue?".
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import List, Optional, Tuple

import numpy as np
from pydrake.math import RigidTransform, RotationMatrix
from scipy.spatial.transform import Rotation as ScipyRotation

from . import default_home_q
from .build_scene import build_scene
from .ikfast import solve_ik as ikfast_solve_ik
from .transit import (
    _arm_model_instance,
    _arm_position_indices,
    _ik_pose_to_joints as _run_ik,
)
from .verify_ikfast_real import _tcp_offset_for, _pose_from_tx_rotvec


WP_DIR = Path(__file__).resolve().parents[2] / "logs" / "waypoints"


def _world_pose_from_arm_base(plant, plant_ctx, arm_inst,
                              X_arm_base_pose: RigidTransform) -> RigidTransform:
    """Convert a pose given in the arm's base frame to world frame.

    The arm's "base" link is welded to world (via the rig calibration),
    so its world pose is fixed regardless of joints. We just compose.
    """
    base_frame = plant.GetFrameByName("base", arm_inst)
    X_world_arm_base = base_frame.CalcPoseInWorld(plant_ctx)
    return X_world_arm_base @ X_arm_base_pose


def _max_per_joint_diff(q_a: np.ndarray, q_b: np.ndarray) -> float:
    return float(np.max(np.abs(np.asarray(q_a) - np.asarray(q_b))))


def main(pos_tol: float, rot_tol: float, only_arm: Optional[str]) -> int:
    scene = build_scene()
    plant = scene.plant
    diag_ctx = scene.diagram.CreateDefaultContext()
    plant_ctx = plant.GetMyMutableContextFromRoot(diag_ctx)

    # Pin the partner arm to HOME for the Drake IK so the bounding-box
    # constraint has a fixed pose to ground itself in.
    home = default_home_q(plant)

    rows = []
    skipped: List[Tuple[str, str]] = []
    for path in sorted(WP_DIR.glob("*.json")):
        raw = path.read_text()
        try:
            d = json.loads(raw)
        except json.JSONDecodeError:
            # ur_left_open_microwave_1.json was saved missing the
            # enclosing opening brace; the closing brace is present.
            # Try prepending '{' and reparsing.
            try:
                d = json.loads("{\n" + raw)
            except json.JSONDecodeError as exc2:
                skipped.append((path.name, f"bad JSON ({exc2})"))
                continue
        arm = d["arm"]
        if only_arm and arm != only_arm:
            continue
        if not d.get("snapshots"):
            continue

        try:
            arm_inst = _arm_model_instance(plant, arm)
        except KeyError as exc:
            skipped.append((path.name, f"unknown arm ({exc})"))
            continue
        # Partner arm = whichever isn't the planning arm; ok if absent.
        try:
            other_arm = "ur_left" if arm == "ur_right" else "ur_right"
            other_inst = _arm_model_instance(plant, other_arm)
        except KeyError:
            other_inst = None
        arm_idx = _arm_position_indices(plant, arm_inst)
        other_q = home[_arm_position_indices(plant, other_inst)] if other_inst else None

        tcp_t, tcp_r = _tcp_offset_for(path.name, arm)
        X_wrist_tcp = _pose_from_tx_rotvec(tcp_t, tcp_r)
        X_tcp_wrist = X_wrist_tcp.inverse()

        for snap in d["snapshots"]:
            q_rec = np.asarray(snap.get("joints_rad", []), dtype=float)
            if q_rec.shape != (6,):
                continue
            t = snap["base_pose"]["translation"]
            r = snap["base_pose"]["rotvec"]
            X_base_tcp_recorded = _pose_from_tx_rotvec(t, r)
            X_base_wrist_recorded = X_base_tcp_recorded @ X_tcp_wrist

            # 1) ikfast: closest branch to seed.
            sols = ikfast_solve_ik(X_base_wrist_recorded, seed_q=q_rec)
            if sols:
                q_ikfast = np.asarray(sols[0], dtype=float)
                ik_dq = float(np.linalg.norm(q_ikfast - q_rec))
                ik_max = _max_per_joint_diff(q_ikfast, q_rec)
                ik_status = "OK"
            else:
                q_ikfast = None
                ik_dq = float("nan")
                ik_max = float("nan")
                ik_status = "no-sol"

            # 2) Drake IK at TCP frame in world.
            #    Recorded pose is in arm base; convert to world.
            X_world_tcp_recorded = _world_pose_from_arm_base(
                plant, plant_ctx, arm_inst, X_base_tcp_recorded,
            )
            seed_full = home.copy()
            seed_full[arm_idx] = q_rec
            try:
                from ..util.poses import Pose
                from ..util.rotations import Rotation
                pose_world_pose = Pose(
                    translation=np.asarray(X_world_tcp_recorded.translation()),
                    rotation=Rotation.from_matrix(
                        np.asarray(X_world_tcp_recorded.rotation().matrix())
                    ),
                )
                q_full_drake = _run_ik(
                    plant, plant_ctx, arm_inst, other_inst, other_q,
                    pose_world_pose, seed_full,
                    pos_tolerance_m=pos_tol, rot_tolerance_rad=rot_tol,
                )
                q_drake = q_full_drake[arm_idx]
                drake_dq = float(np.linalg.norm(q_drake - q_rec))
                drake_max = _max_per_joint_diff(q_drake, q_rec)
                drake_status = "OK"
            except Exception as exc:
                q_drake = None
                drake_dq = float("nan")
                drake_max = float("nan")
                drake_status = "fail"

            rows.append({
                "file": path.name,
                "arm": arm,
                "snap": snap.get("name", "?")[:34],
                "ik_dq": ik_dq,
                "ik_max": ik_max,
                "ik_status": ik_status,
                "drake_dq": drake_dq,
                "drake_max": drake_max,
                "drake_status": drake_status,
            })

    # ---- Per-snapshot table ----
    print()
    print("=" * 122)
    print(f"  ikfast / Drake IK round-trip vs recorded q  "
          f"(pos_tol={pos_tol*1000:.0f}mm, rot_tol={np.degrees(rot_tol):.1f}°)")
    print("=" * 122)
    print(f"{'arm':<9} {'snapshot':<37}"
          f"  {'ikfast':>17}  {'drake':>17}")
    print(f"{'':9} {'':37}  {'‖Δq‖, max|Δq|':>17}  {'‖Δq‖, max|Δq|':>17}")
    for r in rows:
        ik_cell = (f"{r['ik_dq']*1e3:>5.2f},{r['ik_max']*1e3:>5.2f} mrad"
                   if r['ik_status'] == "OK" else f"{'no-sol':>17}")
        drake_cell = (f"{r['drake_dq']*1e3:>5.2f},{r['drake_max']*1e3:>5.2f} mrad"
                      if r['drake_status'] == "OK" else f"{r['drake_status']:>17}")
        print(f"{r['arm']:<9} {r['snap']:<37}  {ik_cell}  {drake_cell}")

    # ---- Aggregate ----
    print()
    print("=" * 122)
    print("  aggregate (median over snapshots, split by arm)")
    print("=" * 122)
    for arm in ("ur_left", "ur_right"):
        arm_rows = [r for r in rows if r["arm"] == arm]
        if not arm_rows:
            continue
        ik_ok = [r for r in arm_rows if r["ik_status"] == "OK"]
        drake_ok = [r for r in arm_rows if r["drake_status"] == "OK"]
        print(f"  {arm}  ({len(arm_rows)} snapshots, ikfast OK={len(ik_ok)}, drake OK={len(drake_ok)}):")
        if ik_ok:
            dqs = [r["ik_dq"] for r in ik_ok]
            mxs = [r["ik_max"] for r in ik_ok]
            print(f"    ikfast : median ‖Δq‖={np.median(dqs)*1e3:>6.2f} mrad,  "
                  f"max ‖Δq‖={np.max(dqs)*1e3:>6.2f} mrad,  "
                  f"max-per-joint median={np.median(mxs)*1e3:>6.2f} mrad")
        if drake_ok:
            dqs = [r["drake_dq"] for r in drake_ok]
            mxs = [r["drake_max"] for r in drake_ok]
            print(f"    drake  : median ‖Δq‖={np.median(dqs)*1e3:>6.2f} mrad,  "
                  f"max ‖Δq‖={np.max(dqs)*1e3:>6.2f} mrad,  "
                  f"max-per-joint median={np.median(mxs)*1e3:>6.2f} mrad")

    if skipped:
        print()
        print("Skipped files:")
        for fn, why in skipped:
            print(f"  {fn:<45}  {why}")

    return 0


if __name__ == "__main__":
    ap = argparse.ArgumentParser(
        description="Compare ikfast and Drake IK against recorded q "
                    "(round-trip via the controller's recorded TCP pose).",
    )
    ap.add_argument("--pos-tol", type=float, default=0.005,
                    help="Drake IK position tolerance (m). Default 5 mm.")
    ap.add_argument("--rot-tol", type=float, default=0.05,
                    help="Drake IK orientation tolerance (rad). Default 0.05 rad ≈ 2.86°.")
    ap.add_argument("--arm", default=None, choices=[None, "ur_left", "ur_right"],
                    help="Restrict to one arm.")
    args = ap.parse_args()
    raise SystemExit(main(
        pos_tol=args.pos_tol,
        rot_tol=args.rot_tol,
        only_arm=args.arm,
    ))
