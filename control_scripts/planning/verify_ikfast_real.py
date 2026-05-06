"""Quantify ikfast vs real-arm kinematics using existing recordings.

Each ``logs/waypoints/*.json`` snapshot has both the joint angles AND
the controller's reported TCP pose (``base_pose``, from
``arm.receive.getActualTCPPose()``). That's the closest thing we have
to ground truth without running the real arm again — if our software
FK matches what the controller reported, we know the kinematics is
right *for poses the team has actually moved through*.

For each snapshot we compute three FK estimates of wrist_3 in the
arm's BASE frame and report the deltas:

  - **ikfast(q)**                       — cambel/ur_ikfast UR5e cpp
  - **Drake URDF FK(q)**                — our planning plant
  - **recorded base_pose - TCP offset** — the controller's reading,
    with the (inferred) ``setTcp`` value subtracted to recover wrist_3

The TCP offset that was on the controller at recording time depends on
arm + date — see ``_tcp_offset_for`` for the lookup table. If we have
the wrong TCP value, the third estimate will be off by exactly the
offset magnitude, which lets us confirm the inference too.

Run::

    python3.11 -m control_scripts.planning.verify_ikfast_real
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Tuple

import numpy as np
from pydrake.math import RigidTransform, RotationMatrix
from scipy.spatial.transform import Rotation as ScipyRotation

from .build_scene import build_scene
from .ikfast import forward as ikfast_forward
from .transit import _arm_model_instance


WP_DIR = (
    Path(__file__).resolve().parents[2] / "logs" / "waypoints"
)


def _tcp_offset_for(file_name: str, arm: str) -> Tuple[np.ndarray, np.ndarray]:
    """Inferred ``setTcp`` value at the time this recording was made.

    The right arm's Robotiq TCP offset has been stable. The left arm's
    hook TCP picked up an R_y(π/2) rotation between Apr 30 and May 1
    (verified earlier by reading git history on calibration.py and
    seeing the 90° wrist mismatch on the older recordings). Returns
    (translation, rotvec) as np.ndarrays.
    """
    if arm == "ur_right":
        # TCP_OFFSET_ROBOTIQ_2F85 = [0, 0, 0.184, 0, 0, 0]
        return np.array([0.0, 0.0, 0.184]), np.zeros(3)
    if arm == "ur_left":
        # Apr 30 recordings used translation-only TCP; May 1 added R_y(π/2).
        if "20260430" in file_name:
            return np.array([0.0, 0.0, 0.10275]), np.zeros(3)
        # Default (May 1+): full TCP_OFFSET_HOOK with R_y(π/2)
        return np.array([0.0, 0.0, 0.10275]), np.array([0.0, np.pi / 2, 0.0])
    raise KeyError(f"unknown arm {arm!r}")


def _pose_from_tx_rotvec(t, r) -> RigidTransform:
    return RigidTransform(
        RotationMatrix(ScipyRotation.from_rotvec(r).as_matrix()),
        np.asarray(t, dtype=float),
    )


def _pose_delta(X_a: RigidTransform, X_b: RigidTransform) -> Tuple[float, float]:
    """Return (position error mm, rotation error deg) between two poses."""
    pos = float(np.linalg.norm(
        np.asarray(X_a.translation()) - np.asarray(X_b.translation())
    )) * 1000
    R_rel = (np.asarray(X_a.rotation().matrix()).T
             @ np.asarray(X_b.rotation().matrix()))
    cos_t = (np.trace(R_rel) - 1.0) / 2.0
    rot = float(np.degrees(np.arccos(np.clip(cos_t, -1.0, 1.0))))
    return pos, rot


def main() -> int:
    scene = build_scene()
    plant = scene.plant
    diag_ctx = scene.diagram.CreateDefaultContext()
    plant_ctx = plant.GetMyMutableContextFromRoot(diag_ctx)

    rows = []
    for path in sorted(WP_DIR.glob("*.json")):
        d = json.loads(path.read_text())
        arm = d["arm"]
        if not d["snapshots"]:
            continue
        inst = _arm_model_instance(plant, arm)
        base_frame = plant.GetFrameByName("base", inst)
        wrist3_frame = plant.GetFrameByName("wrist_3_link", inst)

        tcp_t, tcp_r = _tcp_offset_for(path.name, arm)
        X_wrist_tcp = _pose_from_tx_rotvec(tcp_t, tcp_r)
        X_tcp_wrist = X_wrist_tcp.inverse()

        for snap in d["snapshots"]:
            q = np.asarray(snap["joints_rad"], dtype=float)
            # Recorded TCP in base frame.
            t = snap["base_pose"]["translation"]
            r = snap["base_pose"]["rotvec"]
            X_base_tcp_recorded = _pose_from_tx_rotvec(t, r)
            # Recorded wrist_3 in base = TCP * inv(X_wrist_tcp)
            X_base_wrist_recorded = X_base_tcp_recorded @ X_tcp_wrist

            # Drake FK of wrist_3 in base.
            plant.SetPositions(plant_ctx, inst, q)
            X_base_wrist_drake = plant.CalcRelativeTransform(
                plant_ctx, base_frame, wrist3_frame,
            )

            # ikfast FK (already in our controller-base convention).
            if arm == "ur_right":
                X_base_wrist_ikfast = ikfast_forward(q)
            else:
                # ikfastpy was loaded once at module import for the
                # right arm's UR5e cpp; the cpp handles either UR5e arm
                # since their kinematics are identical (mounting differs
                # but that's in the world-frame transform, not in the
                # arm's local base→wrist chain).
                X_base_wrist_ikfast = ikfast_forward(q)

            ik_vs_real_pos, ik_vs_real_rot = _pose_delta(
                X_base_wrist_ikfast, X_base_wrist_recorded,
            )
            ik_vs_drake_pos, ik_vs_drake_rot = _pose_delta(
                X_base_wrist_ikfast, X_base_wrist_drake,
            )
            drake_vs_real_pos, drake_vs_real_rot = _pose_delta(
                X_base_wrist_drake, X_base_wrist_recorded,
            )
            rows.append({
                "file": path.name,
                "arm": arm,
                "snap": snap["name"][:34],
                "ik_vs_real_pos": ik_vs_real_pos,
                "ik_vs_real_rot": ik_vs_real_rot,
                "ik_vs_drake_pos": ik_vs_drake_pos,
                "ik_vs_drake_rot": ik_vs_drake_rot,
                "drake_vs_real_pos": drake_vs_real_pos,
                "drake_vs_real_rot": drake_vs_real_rot,
            })

    # ----- Per-row table -----
    print()
    print("=" * 110)
    print("  ikfast / Drake / real-arm wrist_3 agreement (mm position, deg rotation)")
    print("=" * 110)
    print(f"{'arm':<9} {'snapshot':<37}"
          f"  {'ik vs real':>16}"
          f"  {'ik vs drake':>16}"
          f"  {'drake vs real':>16}")
    for row in rows:
        print(f"{row['arm']:<9} {row['snap']:<37}"
              f"  {row['ik_vs_real_pos']:>6.1f} mm {row['ik_vs_real_rot']:>5.2f}°"
              f"  {row['ik_vs_drake_pos']:>6.1f} mm {row['ik_vs_drake_rot']:>5.2f}°"
              f"  {row['drake_vs_real_pos']:>6.1f} mm {row['drake_vs_real_rot']:>5.2f}°")

    # ----- Aggregate -----
    print()
    print("=" * 110)
    print("  aggregate (median across all snapshots, split by arm)")
    print("=" * 110)
    for arm in ("ur_left", "ur_right"):
        arm_rows = [r for r in rows if r["arm"] == arm]
        if not arm_rows:
            continue
        print(f"  {arm} ({len(arm_rows)} snapshots):")
        for label, key in (
            ("ik vs real ", "ik_vs_real"),
            ("ik vs drake", "ik_vs_drake"),
            ("drake vs real", "drake_vs_real"),
        ):
            ps = [r[f"{key}_pos"] for r in arm_rows]
            rs = [r[f"{key}_rot"] for r in arm_rows]
            print(f"    {label:<14} : median {np.median(ps):>6.1f} mm "
                  f"{np.median(rs):>5.2f}°  "
                  f"max {np.max(ps):>6.1f} mm {np.max(rs):>5.2f}°")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
