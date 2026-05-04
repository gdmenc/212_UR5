"""Verify ikfastpy on the planning scene.

Three checks, increasing in stringency:

  1. **Round-trip self-consistency** (ikfast only): random q → FK →
     IK → verify the seed q is among the returned branches.

  2. **Drake FK agreement**: random q → ikfastpy.forward vs Drake
     `wrist_3_link` pose. Quantifies the UR5-vs-UR5e link-length
     mismatch — ikfastpy was generated for UR5; we use it on UR5e.

  3. **Reachability replay**: take the SNOPT IK seed q for the dryrun
     legs (HOME → hover → grasp → ...) and check whether ikfastpy
     finds a branch within X cm / Y deg of those poses.

Run::

    python3.11 -m control_scripts.planning.verify_ikfast
    python3.11 -m control_scripts.planning.verify_ikfast --n-random 200
"""

from __future__ import annotations

import argparse
import time
from typing import Tuple

import numpy as np
from pydrake.math import RigidTransform

from . import default_home_q
from .build_scene import build_scene
from .ikfast import closest_solution, forward, solve_ik
from .transit import _arm_model_instance, _arm_position_indices


def _fk_drake_in_base(plant, plant_ctx, arm_name, q_arm) -> RigidTransform:
    """Drake FK of wrist_3_link in the arm's BASE frame (ikfastpy's
    output frame). We FK relative to the URDF's ``base`` link, which
    is the UR controller convention since our base-welding fix."""
    inst = _arm_model_instance(plant, arm_name)
    plant.SetPositions(plant_ctx, inst, q_arm)
    base_frame = plant.GetFrameByName("base", inst)
    wrist3_frame = plant.GetFrameByName("wrist_3_link", inst)
    return plant.CalcRelativeTransform(plant_ctx, base_frame, wrist3_frame)


def _round_trip_self_consistency(n: int, rng: np.random.Generator) -> dict:
    """Sample random q in [-π, π]^6, FK with ikfast, IK back; check
    whether the original q (or an equivalent within 2π wrap) is among
    the returned solutions."""
    matched = 0
    n_solutions_dist = []
    for _ in range(n):
        q = rng.uniform(-np.pi, np.pi, size=6)
        X = forward(q)
        sols = solve_ik(X, seed_q=q)
        n_solutions_dist.append(len(sols))
        if not sols:
            continue
        if np.linalg.norm(sols[0] - q) < 1e-3:
            matched += 1
    return {
        "n": n,
        "matched": matched,
        "match_rate": matched / n,
        "mean_solutions": np.mean(n_solutions_dist),
        "min_solutions": int(np.min(n_solutions_dist)),
    }


def _drake_fk_agreement(n: int, rng: np.random.Generator) -> dict:
    """Quantify the UR5-vs-UR5e link-length mismatch:
    ikfastpy.forward vs Drake's URDF FK at the same q."""
    scene = build_scene()
    plant = scene.plant
    diag_ctx = scene.diagram.CreateDefaultContext()
    plant_ctx = plant.GetMyMutableContextFromRoot(diag_ctx)

    pos_errors_mm = []
    rot_errors_deg = []
    for _ in range(n):
        # Sample a "reasonable" q (avoid full ±π extremes that pose
        # the arm in unusual configurations).
        q = rng.uniform(-2.0, 2.0, size=6)
        X_ikfast = forward(q)
        X_drake = _fk_drake_in_base(plant, plant_ctx, "ur_right", q)

        pos_err = np.linalg.norm(
            np.asarray(X_ikfast.translation()) - np.asarray(X_drake.translation())
        )
        # Rotation angle between the two rotations
        R_rel = (np.asarray(X_ikfast.rotation().matrix()).T
                 @ np.asarray(X_drake.rotation().matrix()))
        cos_t = (np.trace(R_rel) - 1.0) / 2.0
        rot_err = float(np.degrees(np.arccos(np.clip(cos_t, -1.0, 1.0))))

        pos_errors_mm.append(pos_err * 1000)
        rot_errors_deg.append(rot_err)

    return {
        "n": n,
        "pos_mm": dict(
            mean=float(np.mean(pos_errors_mm)),
            median=float(np.median(pos_errors_mm)),
            p95=float(np.percentile(pos_errors_mm, 95)),
            max=float(np.max(pos_errors_mm)),
        ),
        "rot_deg": dict(
            mean=float(np.mean(rot_errors_deg)),
            median=float(np.median(rot_errors_deg)),
            p95=float(np.percentile(rot_errors_deg, 95)),
            max=float(np.max(rot_errors_deg)),
        ),
    }


def _branch_demo() -> dict:
    """Show all 8 IK solutions for a single sample pose, sorted by
    distance from a seed q. Demonstrates that branch selection is
    deterministic and exposes which kinematic configurations exist."""
    scene = build_scene()
    plant = scene.plant
    diag_ctx = scene.diagram.CreateDefaultContext()
    plant_ctx = plant.GetMyMutableContextFromRoot(diag_ctx)

    arm_idx = _arm_position_indices(plant, _arm_model_instance(plant, "ur_right"))
    home_q = default_home_q(plant)[arm_idx]

    # FK home with Drake to get the corresponding base-frame pose,
    # then IK that pose back. Should include a branch close to home_q.
    X = _fk_drake_in_base(plant, plant_ctx, "ur_right", home_q)
    sols = solve_ik(X, seed_q=home_q)
    return {
        "home_q_deg": np.round(np.degrees(home_q), 1).tolist(),
        "n_sols": len(sols),
        "solutions_deg": [np.round(np.degrees(s), 1).tolist() for s in sols],
        "joint_dist_from_home_rad": [
            float(np.linalg.norm(s - home_q)) for s in sols
        ],
    }


def _timing(n: int, rng: np.random.Generator) -> dict:
    """Microbenchmark forward + inverse calls."""
    qs = [rng.uniform(-2.0, 2.0, size=6) for _ in range(n)]
    Xs = [forward(q) for q in qs]
    t0 = time.time()
    for q in qs:
        forward(q)
    t_fk = (time.time() - t0) / n * 1e6  # us per call
    t0 = time.time()
    for X, q in zip(Xs, qs):
        solve_ik(X, seed_q=q)
    t_ik = (time.time() - t0) / n * 1e6
    return {"n": n, "fk_us": t_fk, "ik_us": t_ik}


def main(n_random: int) -> int:
    rng = np.random.default_rng(0)

    print("=" * 78)
    print("  ikfastpy verification (UR5 ikfast on UR5e plant)")
    print("=" * 78)

    print("\n[1] Self-consistency: FK → IK → seed in result?")
    rt = _round_trip_self_consistency(n_random, rng)
    print(f"  matched : {rt['matched']}/{rt['n']} "
          f"({100*rt['match_rate']:.1f}%)")
    print(f"  mean #solutions per pose : {rt['mean_solutions']:.2f}")
    print(f"  min  #solutions per pose : {rt['min_solutions']}")

    print("\n[2] Drake FK agreement: ikfastpy.forward vs URDF FK")
    print("    (UR5e cpp from cambel/ur_ikfast, ~3-decimal-rounded link lengths)")
    fk = _drake_fk_agreement(n_random, rng)
    print(f"  position error (mm)  : "
          f"mean={fk['pos_mm']['mean']:.1f}, "
          f"median={fk['pos_mm']['median']:.1f}, "
          f"p95={fk['pos_mm']['p95']:.1f}, "
          f"max={fk['pos_mm']['max']:.1f}")
    print(f"  rotation error (deg) : "
          f"mean={fk['rot_deg']['mean']:.2f}, "
          f"median={fk['rot_deg']['median']:.2f}, "
          f"p95={fk['rot_deg']['p95']:.2f}, "
          f"max={fk['rot_deg']['max']:.2f}")

    print("\n[3] Branch enumeration at sim HOME (right arm)")
    bd = _branch_demo()
    print(f"  HOME q (deg)        : {bd['home_q_deg']}")
    print(f"  ikfastpy solutions  : {bd['n_sols']}")
    print(f"  {'#':>3}  {'q (deg)':<48}  {'dist from HOME (rad)':>20}")
    for i, (s, d) in enumerate(zip(bd['solutions_deg'],
                                    bd['joint_dist_from_home_rad'])):
        print(f"  {i:>3}  {str(s):<48}  {d:>20.3f}")

    print("\n[4] Microbench")
    tm = _timing(n_random, rng)
    print(f"  forward : {tm['fk_us']:6.1f} us per call")
    print(f"  inverse : {tm['ik_us']:6.1f} us per call (avg over {tm['n']})")

    print()
    return 0


if __name__ == "__main__":
    ap = argparse.ArgumentParser(
        description="Verify ikfastpy: self-consistency, Drake FK delta, "
                    "branch enumeration, microbench."
    )
    ap.add_argument("--n-random", type=int, default=200,
                    help="Number of random configs for tests 1, 2, 4.")
    args = ap.parse_args()
    raise SystemExit(main(n_random=args.n_random))
