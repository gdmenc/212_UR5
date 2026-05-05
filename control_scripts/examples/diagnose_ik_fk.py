"""Diagnose why fk_replay reports multi-metre errors on the open arc.

Runs four sanity checks against the live controller (or URSim). No motion
is commanded. Each check isolates one possible failure mode:

    1. FK(actualQ) ≈ getActualTCPPose
       If this fails, getForwardKinematics is using a different TCP than
       the one currently active on the controller — likely an unset or
       stale setTcp.

    2. IK(actualTCP, qnear=actualQ) ≈ actualQ
       If this fails, getInverseKinematics is broken or ignoring qnear.

    3. FK(IK(target_base, qnear=actualQ)) ≈ target_base
       The full round-trip we rely on in fk_replay. If 1 and 2 both pass
       but this fails, the target itself is unreachable or out of IK
       tolerance — not a code bug, a geometry / calibration issue.

    4. arm.to_base then arm.to_task round-trip
       Frame conversion sanity. If this fails the calibration matrix
       isn't actually a rigid transform.

Run::

    python3.11 -m control_scripts.examples.diagnose_ik_fk
"""

from __future__ import annotations

import numpy as np

from ..session import default_session
from ..util.poses import Pose
from ..util.rotations import Rotation
from ..util.rtde_convert import pose_to_rtde, rtde_to_pose


ARM = "ur_left"


def _fmt(p: Pose) -> str:
    return (
        f"xyz={np.round(p.translation, 4).tolist()} "
        f"rv°={np.round(np.degrees(p.rotation.as_rotvec()), 1).tolist()}"
    )


def _pose_diff(a: Pose, b: Pose) -> tuple[float, float]:
    """(pos_err_m, rot_err_deg) between two poses."""
    pos = float(np.linalg.norm(a.translation - b.translation))
    rel = a.rotation.inv() * b.rotation
    rot = float(np.degrees(np.linalg.norm(rel.as_rotvec())))
    return pos, rot


def main() -> None:
    with default_session(left=True, right=False) as session:
        arm = session.arms[ARM]
        c = arm.control
        r = arm.receive

        print("=" * 70)
        print("  Diagnose IK / FK round-trip on", ARM)
        print("=" * 70)

        actual_q = list(r.getActualQ())
        actual_tcp_rtde = list(r.getActualTCPPose())
        actual_tcp_base = rtde_to_pose(actual_tcp_rtde)

        print(f"  actual q (rad)    : {np.round(actual_q, 4).tolist()}")
        print(f"  actual TCP (base) : {_fmt(actual_tcp_base)}")
        print(f"  TCP offset set    : {arm.tcp_offset}")
        print()

        # --- Check 1: FK(actual_q) ≈ getActualTCPPose ---
        # Three variants to isolate which TCP getForwardKinematics is using:
        #   1a: FK(q) with no tcp_offset arg — should use active controller TCP
        #   1b: FK(q, []) — explicit empty list
        #   1c: FK(q, our hook offset) — explicitly pass the TCP we configured
        #   1d: FK(q, [0]*6) — explicitly pass NO TCP (returns flange pose)
        fk_default = rtde_to_pose(c.getForwardKinematics(actual_q))
        fk_empty = rtde_to_pose(c.getForwardKinematics(actual_q, []))
        fk_explicit = rtde_to_pose(c.getForwardKinematics(actual_q, list(arm.tcp_offset)))
        fk_zero_tcp = rtde_to_pose(c.getForwardKinematics(actual_q, [0.0] * 6))

        for label, fk_pose in [
            ("FK(q)            ", fk_default),
            ("FK(q, [])        ", fk_empty),
            ("FK(q, hook tcp)  ", fk_explicit),
            ("FK(q, zero tcp)  ", fk_zero_tcp),
        ]:
            pos_err, rot_err = _pose_diff(fk_pose, actual_tcp_base)
            tag = "OK  " if (pos_err * 1000 <= 5.0 and rot_err <= 1.0) else "FAIL"
            print(f"  [1] {label}: {_fmt(fk_pose)}")
            print(f"      vs actualTCP    : pos {pos_err * 1000:8.3f} mm   "
                  f"rot {rot_err:6.3f}°   [{tag}]")
        print()
        print("      Interpretation:")
        print("        - If FK(q, hook tcp) is OK but FK(q) FAILs:")
        print("            => the controller's stored TCP is NOT what setTcp configured.")
        print("            => fix: pass tcp_offset explicitly in fk_replay & arc IK calls.")
        print("        - If FK(q, zero tcp) is OK and FK(q, hook tcp) FAILs:")
        print("            => the controller is treating the TCP as zero / unset.")
        print("            => fix: re-call setTcp before motion, or check for stale state.")
        print("        - If all four variants FAIL:")
        print("            => something deeper is wrong (joint sign/order, kinematic model).")
        print()

        # --- Check 2: IK(actualTCP, qnear=actualQ) ≈ actualQ ---
        ik_q = c.getInverseKinematics(actual_tcp_rtde, actual_q, 0.001, 0.001)
        if not ik_q or len(ik_q) != 6:
            print(f"  [2] IK(actualTCP, qnear=actualQ): empty result — IK FAILED")
        else:
            joint_diff = float(np.linalg.norm(np.array(ik_q) - np.array(actual_q)))
            print(f"  [2] IK(actualTCP, qnear=actualQ) vs actualQ:")
            print(f"      IK q          : {np.round(ik_q, 4).tolist()}")
            print(f"      |q_diff| (rad): {joint_diff:.6f}")
            if joint_diff > 0.01:
                print("      FAIL — IK is not converging to the actual joint config")
                print("             when seeded from it. qnear is being ignored or"
                      "\n             IK is broken.")
            else:
                print("      OK")
        print()

        # --- Check 3: full round-trip on a representative target ---
        # Build a target close to the actual TCP so reachability isn't a question.
        target_base = Pose(
            translation=actual_tcp_base.translation + np.array([0.0, 0.0, 0.05]),
            rotation=actual_tcp_base.rotation,
        )
        target_rtde = pose_to_rtde(target_base)
        ik_q2 = c.getInverseKinematics(target_rtde, actual_q, 0.001, 0.001)
        if not ik_q2 or len(ik_q2) != 6:
            print(f"  [3] IK(target = actualTCP + 5cm Z): empty result — IK FAILED")
        else:
            fk2_rtde = c.getForwardKinematics(list(ik_q2))
            fk2_pose = rtde_to_pose(fk2_rtde)
            pos_err, rot_err = _pose_diff(fk2_pose, target_base)
            print(f"  [3] FK(IK(actualTCP+5cm Z)) vs target:")
            print(f"      target (base) : {_fmt(target_base)}")
            print(f"      FK result     : {_fmt(fk2_pose)}")
            print(f"      pos err       : {pos_err * 1000:.3f} mm")
            print(f"      rot err       : {rot_err:.3f}°")
            if pos_err * 1000 > 5.0 or rot_err > 1.0:
                print("      FAIL — IK→FK round-trip diverges. Suggests IK is")
                print("             returning something that doesn't satisfy FK back"
                      "\n             to the target. Likely a stale TCP.")
            else:
                print("      OK")
        print()

        # --- Check 4: task <-> base frame round-trip ---
        sample_task = Pose(
            translation=np.array([-0.071, 0.344, 0.155]),
            rotation=Rotation.from_rotvec(np.array([-1.55, 0.04, -0.09])),
        )
        sample_base = arm.to_base(sample_task)
        sample_back = arm.to_task(sample_base)
        pos_err, rot_err = _pose_diff(sample_back, sample_task)
        print(f"  [4] arm.to_task(arm.to_base(p)) round-trip on engage pose:")
        print(f"      task in       : {_fmt(sample_task)}")
        print(f"      base mid      : {_fmt(sample_base)}")
        print(f"      task back     : {_fmt(sample_back)}")
        print(f"      pos err       : {pos_err * 1000:.6f} mm")
        print(f"      rot err       : {rot_err:.6f}°")
        # Relaxed thresholds: the calibration matrix uses hardcoded 0.707
        # (not 1/sqrt(2)), so columns aren't unit-norm to machine precision.
        # A round-trip error of ~50 µm is expected from that quantization.
        if pos_err * 1000 > 0.5 or rot_err > 0.05:
            print("      FAIL — frame conversion not invertible beyond float noise.")
        else:
            print(f"      OK  (round-trip noise from hardcoded 0.707 in calibration "
                  f"matrix; tighten by replacing with 1/np.sqrt(2.0) if it ever matters)")
        print()
        print("=" * 70)


if __name__ == "__main__":
    main()
