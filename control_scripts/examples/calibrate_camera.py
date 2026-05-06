"""Hand-eye calibration for a wrist-mounted RealSense camera (eye-in-hand).

Solves **T_ee_camera** — the fixed rigid transform from the camera frame
to the robot's TCP (tool-centre-point) frame.  Once this is known, the
real-time camera-to-task transform is:

    X_task_camera = X_task_ee  @  T_ee_camera

where X_task_ee changes every control cycle (read from RTDE) and
T_ee_camera is constant until the camera mount is disturbed.

Algorithm
---------
OpenCV ``calibrateHandEye`` (AX = XB formulation, eye-in-hand variant):

    A_ij = T_ee_i^{-1} * T_ee_j      ← relative EE motion (from RTDE)
    B_ij = T_cam_i_board * T_cam_j_board^{-1} ← relative board motion (from camera)
    solve: A * X = X * B   where X = T_ee_camera

At least **10 well-distributed samples** are required for a reliable
solution (aim for 15–20).  Good sample diversity means:

    ✓ Large tilt variation (±30° about board normal)
    ✓ Large pan variation (±30° side-to-side)
    ✓ Varying standoff (30–60 cm from board)
    ✓ Some poses that differ ONLY in translation, some ONLY in rotation
    ✗ Avoid poses that are almost identical (wasted sample)
    ✗ Avoid moving only one joint between every capture

Procedure
---------
1.  Print the calibration board (run once):
        python -m control_scripts.examples.calibrate_camera --print-board

    The board is saved as ``charuco_board.png`` at ~100 DPI.
    Print at 100 % scale (no "fit to page") on a rigid, flat surface.
    Verify the printed square side measures exactly 25 mm.

2.  Fix the board flat on the workspace table.  It must NOT move.

3.  Run the calibration:
        python -m control_scripts.examples.calibrate_camera --arm left

4.  In the live window:
    - Move the robot (teach pendant or free-drive) to a pose where the
      board is visible and all four board edges are in frame.
    - When the green axis overlay appears (board detected) and the robot
      is stationary, press SPACE to record the sample.
    - Repeat ≥ 15 times with varied poses.  The status bar shows a
      diversity score — aim to keep it green (> 0.6) for all samples.
    - Press Q when done.

5.  The result is saved to ``calibration/build/T_ee_camera.json`` by default
    (same path vision tasks load).  **Before capturing samples**, set the
    controller TCP to the same tool frame used in production (e.g. run
    ``Session`` / ``ArmHandle.setup()`` once, or set TCP on the pendant to
    match ``TCP_OFFSET_*`` in ``control_scripts/calibration.py``) — this script
    only reads ``getActualTCPPose()`` and does not call ``setTcp`` itself.

Usage
-----
    python -m control_scripts.examples.calibrate_camera [--arm left|right]
    python -m control_scripts.examples.calibrate_camera --print-board
    python -m control_scripts.examples.calibrate_camera --arm right --ip 192.168.1.102
"""

from __future__ import annotations

import argparse
import json
import time
from pathlib import Path
from typing import Optional

import cv2
import numpy as np
import pyrealsense2 as rs


# ---------------------------------------------------------------------------
# ChArUco board parameters — must match what is physically printed
# ---------------------------------------------------------------------------

_SQUARES_X   = 6      # chessboard columns
_SQUARES_Y   = 5      # chessboard rows
_SQUARE_M    = 0.025  # 25 mm per square side
_MARKER_M    = 0.019  # 19 mm per ArUco marker side
_ARUCO_DICT  = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_100)
_BOARD       = cv2.aruco.CharucoBoard(
    (_SQUARES_X, _SQUARES_Y), _SQUARE_M, _MARKER_M, _ARUCO_DICT
)

# Stationary detection threshold: max joint velocity (rad/s) to accept a sample
_STATIONARY_THRESHOLD = 0.005

# Minimum cosine-distance rotation spread to consider two captures "different"
_MIN_ROTATION_SPREAD_DEG = 10.0

# ---------------------------------------------------------------------------
# Board image generation
# ---------------------------------------------------------------------------

def print_board(output_path: Path = Path("charuco_board.png")) -> None:
    """Generate and save the ChArUco board image for printing."""
    # Image size: board footprint + 1 cm margin on each side.
    # At 10 px/mm: 6×25mm = 150px wide + 20px margin = 170px.
    px_per_mm = 10
    margin_px = int(10 * px_per_mm)  # 10 mm margin
    w_px = int(_SQUARES_X * _SQUARE_M * 1000 * px_per_mm) + 2 * margin_px
    h_px = int(_SQUARES_Y * _SQUARE_M * 1000 * px_per_mm) + 2 * margin_px

    img = _BOARD.generateImage((w_px, h_px), marginSize=margin_px, borderBits=1)
    cv2.imwrite(str(output_path), img)

    w_cm = _SQUARES_X * _SQUARE_M * 100
    h_cm = _SQUARES_Y * _SQUARE_M * 100
    print(f"Board saved to: {output_path}")
    print(f"Physical size:  {w_cm:.1f} cm × {h_cm:.1f} cm")
    print(f"Print at 100 % (no 'fit to page').  Verify square = 25 mm after printing.")


# ---------------------------------------------------------------------------
# Camera intrinsics from RealSense
# ---------------------------------------------------------------------------

def _rs_intrinsics_to_cv(intr: rs.intrinsics):
    """Convert RealSense intrinsics to OpenCV camera_matrix + dist_coeffs."""
    K = np.array([
        [intr.fx,  0.0,     intr.ppx],
        [ 0.0,    intr.fy,  intr.ppy],
        [ 0.0,     0.0,      1.0    ],
    ], dtype=np.float64)
    # RealSense reports [k1, k2, p1, p2, k3]; OpenCV order matches.
    D = np.array(intr.coeffs, dtype=np.float64)
    return K, D


# ---------------------------------------------------------------------------
# ChArUco detection
# ---------------------------------------------------------------------------

def _detect_charuco(
    gray: np.ndarray,
    K: np.ndarray,
    D: np.ndarray,
):
    """Return (rvec, tvec, n_corners, vis_frame) for the ChArUco board.

    Returns (None, None, 0, vis_frame) when detection fails.  ``vis_frame``
    is the annotated version of ``gray`` converted to BGR for display.
    """
    bgr = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)

    # Detect ArUco markers (new API, available from OpenCV 4.6)
    detector_params = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(_ARUCO_DICT, detector_params)
    corners, ids, _ = detector.detectMarkers(gray)

    if ids is None or len(ids) < 4:
        return None, None, 0, bgr

    cv2.aruco.drawDetectedMarkers(bgr, corners, ids)

    # Refine to ChArUco corners
    retval, charuco_corners, charuco_ids = cv2.aruco.interpolateCornersCharuco(
        corners, ids, gray, _BOARD
    )
    if retval < 4 or charuco_corners is None:
        return None, None, retval, bgr

    cv2.aruco.drawDetectedCornersCharuco(bgr, charuco_corners, charuco_ids)

    # Estimate 6-DoF board pose in camera frame
    ok, rvec, tvec = cv2.aruco.estimatePoseCharucoBoard(
        charuco_corners, charuco_ids, _BOARD,
        K, D,
        np.zeros((3, 1), dtype=np.float64),
        np.zeros((3, 1), dtype=np.float64),
    )
    if not ok:
        return None, None, retval, bgr

    cv2.drawFrameAxes(bgr, K, D, rvec, tvec, _SQUARE_M * 2)
    return rvec, tvec, retval, bgr


# ---------------------------------------------------------------------------
# Pose transform helpers
# ---------------------------------------------------------------------------

def _rotvec_to_Rt(rotvec: np.ndarray, tvec: np.ndarray):
    """Convert axis-angle rotvec + tvec → (R 3×3, t 3×1)."""
    R, _ = cv2.Rodrigues(np.asarray(rotvec, dtype=np.float64).ravel())
    t = np.asarray(tvec, dtype=np.float64).reshape(3, 1)
    return R, t


def _Rt_to_mat4(R: np.ndarray, t: np.ndarray) -> np.ndarray:
    M = np.eye(4)
    M[:3, :3] = R
    M[:3,  3] = t.ravel()
    return M


def _inv4(M: np.ndarray) -> np.ndarray:
    """Invert a 4×4 rigid-body transform efficiently."""
    R = M[:3, :3]
    t = M[:3,  3]
    M_inv = np.eye(4)
    M_inv[:3, :3] = R.T
    M_inv[:3,  3] = -(R.T @ t)
    return M_inv


# ---------------------------------------------------------------------------
# Calibration residual
# ---------------------------------------------------------------------------

def _hand_eye_residual(
    R_ee_list, t_ee_list,
    R_target_list, t_target_list,
    R_cam2ee: np.ndarray,
    t_cam2ee: np.ndarray,
) -> tuple[float, float]:
    """Mean rotation (deg) and translation (mm) residual of AX = XB.

    For each pair (i, j):
        A_ij = T_ee_i^{-1} * T_ee_j
        B_ij = T_cam_board_i * T_cam_board_j^{-1}
        residual: ‖A*X − X*B‖ in rotation and translation separately
    """
    n = len(R_ee_list)
    X = _Rt_to_mat4(R_cam2ee, t_cam2ee)

    rot_errs = []
    trans_errs = []
    for i in range(n):
        for j in range(i + 1, n):
            T_ee_i  = _Rt_to_mat4(R_ee_list[i],     t_ee_list[i])
            T_ee_j  = _Rt_to_mat4(R_ee_list[j],     t_ee_list[j])
            T_b_i   = _Rt_to_mat4(R_target_list[i], t_target_list[i])
            T_b_j   = _Rt_to_mat4(R_target_list[j], t_target_list[j])

            A = _inv4(T_ee_i) @ T_ee_j
            B = T_b_i @ _inv4(T_b_j)

            LHS = A @ X   # should equal X @ B
            RHS = X @ B

            dR = LHS[:3, :3] @ RHS[:3, :3].T
            angle = np.arccos(np.clip((np.trace(dR) - 1.0) / 2.0, -1, 1))
            rot_errs.append(np.degrees(angle))

            dt = LHS[:3, 3] - RHS[:3, 3]
            trans_errs.append(np.linalg.norm(dt) * 1000)  # → mm

    return float(np.mean(rot_errs)), float(np.mean(trans_errs))


# ---------------------------------------------------------------------------
# Diversity score (0–1): how spread are the captured EE rotations?
# ---------------------------------------------------------------------------

def _rotation_spread(R_list: list) -> float:
    """Returns mean pairwise rotation-angle difference (normalised to 1 at 45°)."""
    if len(R_list) < 2:
        return 0.0
    angles = []
    for i in range(len(R_list)):
        for j in range(i + 1, len(R_list)):
            dR = R_list[i].T @ R_list[j]
            cos_a = np.clip((np.trace(dR) - 1.0) / 2.0, -1, 1)
            angles.append(abs(np.degrees(np.arccos(cos_a))))
    mean_angle = np.mean(angles)
    return min(1.0, mean_angle / 45.0)


# ---------------------------------------------------------------------------
# Main calibration loop
# ---------------------------------------------------------------------------

def _run_live(
    arm_name: str,
    ip: str,
    min_samples: int,
    output_path: Path,
    n_warmup: int = 15,
) -> None:
    # --- connect to robot (receive only — no motion commands) ---------------
    from rtde_receive import RTDEReceiveInterface
    print(f"[INFO] Connecting to {arm_name} ({ip}) ...")
    rtde_r = RTDEReceiveInterface(ip)
    if not rtde_r.isConnected():
        raise RuntimeError(f"Could not connect to {ip}")
    print("[INFO] Robot connected.")

    # --- open RealSense -----------------------------------------------------
    pipe = rs.pipeline()
    cfg  = rs.config()
    cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    cfg.enable_stream(rs.stream.depth, 640, 480, rs.format.z16,  30)
    profile = pipe.start(cfg)

    color_intr = (
        profile.get_stream(rs.stream.color)
               .as_video_stream_profile()
               .get_intrinsics()
    )
    K, D = _rs_intrinsics_to_cv(color_intr)
    align = rs.align(rs.stream.color)

    print(f"[INFO] Warming up camera ({n_warmup} frames) ...")
    for _ in range(n_warmup):
        pipe.wait_for_frames()

    # --- sample collection --------------------------------------------------
    R_ee_list, t_ee_list         = [], []   # EE poses in base frame
    R_target_list, t_target_list = [], []   # board poses in camera frame

    print("\n" + "=" * 60)
    print("  SPACE : capture sample (board must be detected + robot still)")
    print("  Q     : finish and run calibration")
    print("  ESC   : abort")
    print("=" * 60 + "\n")

    window = "Hand-Eye Calibration — move robot, then SPACE to capture"
    cv2.namedWindow(window, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(window, 960, 540)

    while True:
        # Grab aligned frame
        frameset = pipe.wait_for_frames()
        aligned  = align.process(frameset)
        color_f  = aligned.get_color_frame()
        if not color_f:
            continue

        bgr  = np.asanyarray(color_f.get_data())
        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)

        rvec_board, tvec_board, n_corners, vis = _detect_charuco(gray, K, D)

        # Read EE state
        tcp_rtde = rtde_r.getActualTCPPose()    # [x,y,z,rx,ry,rz] base frame
        qd_rtde  = rtde_r.getActualQd()         # joint velocities
        max_vel  = max(abs(v) for v in qd_rtde)
        is_still = max_vel < _STATIONARY_THRESHOLD
        board_ok = rvec_board is not None

        n_samp    = len(R_ee_list)
        diversity = _rotation_spread([r for r in R_ee_list]) if n_samp > 1 else 0.0

        # Status text
        div_color   = (0, 255, 0) if diversity >= 0.5 else (0, 200, 255)
        still_color = (0, 255, 0) if is_still        else (0, 0, 255)
        board_color = (0, 255, 0) if board_ok        else (0, 0, 255)

        def _put(img, text, row, color=(200, 200, 200)):
            cv2.putText(img, text, (10, 28 + 28 * row),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.70, color, 2, cv2.LINE_AA)

        _put(vis, f"Samples : {n_samp}  (need {min_samples})", 0)
        _put(vis, f"Board   : {'FOUND (' + str(n_corners) + ' corners)' if board_ok else 'NOT FOUND'}",
             1, board_color)
        _put(vis, f"Robot   : {'STILL' if is_still else f'MOVING  ({max_vel:.3f} rad/s)'}",
             2, still_color)
        _put(vis, f"Diversity: {diversity:.2f}  (aim > 0.5)", 3, div_color)
        _put(vis, "SPACE=capture   Q=finish   ESC=abort", 5, (180, 180, 180))

        cv2.imshow(window, vis)
        key = cv2.waitKey(1) & 0xFF

        if key == 27:  # ESC
            print("\n[ABORT] User pressed ESC.")
            pipe.stop()
            cv2.destroyAllWindows()
            return

        if key == ord('q'):
            if n_samp < min_samples:
                print(f"[WARN] Only {n_samp} samples — need at least {min_samples}. Keep going.")
            else:
                break

        if key == ord(' '):
            if not board_ok:
                print("[SKIP] Board not detected — reposition so the board is fully visible.")
                continue
            if not is_still:
                print(f"[SKIP] Robot is moving ({max_vel:.4f} rad/s) — wait for it to stop.")
                continue

            R_ee, t_ee = _rotvec_to_Rt(tcp_rtde[3:], tcp_rtde[:3])

            # Reject if too similar to an existing sample (< 10° rotation change)
            if R_ee_list:
                dR = R_ee_list[-1].T @ R_ee
                cos_a = np.clip((np.trace(dR) - 1.0) / 2.0, -1, 1)
                angle_deg = abs(np.degrees(np.arccos(cos_a)))
                if angle_deg < _MIN_ROTATION_SPREAD_DEG:
                    print(f"[SKIP] Too similar to previous sample "
                          f"(Δrot = {angle_deg:.1f}° < {_MIN_ROTATION_SPREAD_DEG}°). "
                          f"Move to a more different pose.")
                    continue

            R_board, t_board = _rotvec_to_Rt(rvec_board, tvec_board)

            R_ee_list.append(R_ee)
            t_ee_list.append(t_ee)
            R_target_list.append(R_board)
            t_target_list.append(t_board)

            t = tcp_rtde[:3]
            print(f"[CAPTURE #{n_samp + 1}] "
                  f"EE xyz=({t[0]:+.3f},{t[1]:+.3f},{t[2]:+.3f}) m  "
                  f"board dist={float(np.linalg.norm(tvec_board)):.3f} m  "
                  f"diversity={_rotation_spread(R_ee_list):.2f}")

    pipe.stop()
    cv2.destroyAllWindows()
    print(f"\n[INFO] Collected {len(R_ee_list)} samples.  Running calibration ...\n")

    # --- hand-eye calibration -----------------------------------------------
    methods = {
        "Tsai":       cv2.CALIB_HAND_EYE_TSAI,
        "Horaud":     cv2.CALIB_HAND_EYE_HORAUD,
        "Park":       cv2.CALIB_HAND_EYE_PARK,
        "Andreff":    cv2.CALIB_HAND_EYE_ANDREFF,
        "Daniilidis": cv2.CALIB_HAND_EYE_DANIILIDIS,
    }

    print(f"{'Method':<14} {'Rot err (°)':>12} {'Trans err (mm)':>15}")
    print("-" * 44)

    best_name   = None
    best_R      = None
    best_t      = None
    best_rot_e  = float("inf")

    for name, method in methods.items():
        try:
            R, t = cv2.calibrateHandEye(
                R_ee_list, t_ee_list,
                R_target_list, t_target_list,
                method=method,
            )
            rot_e, trans_e = _hand_eye_residual(
                R_ee_list, t_ee_list,
                R_target_list, t_target_list,
                R, t,
            )
            print(f"{name:<14} {rot_e:>12.3f} {trans_e:>15.2f}")
            if rot_e < best_rot_e:
                best_rot_e  = rot_e
                best_name   = name
                best_R      = R
                best_t      = t
        except Exception as e:
            print(f"{name:<14} FAILED: {e}")

    print("-" * 44)
    print(f"Best: {best_name}  (rot err = {best_rot_e:.3f}°)\n")

    if best_R is None:
        raise RuntimeError("All calibration methods failed — check your samples.")

    # --- save result --------------------------------------------------------
    rvec_result, _ = cv2.Rodrigues(best_R)
    result = {
        "description": (
            "T_ee_camera: pose of the camera in the robot TCP (EE) frame. "
            "Produced by hand-eye calibration (eye-in-hand, AX=XB). "
            f"Method: {best_name}, rot_residual_deg: {best_rot_e:.3f}."
        ),
        "arm": arm_name,
        "method": best_name,
        "rot_residual_deg": round(best_rot_e, 4),
        "R_ee_camera": best_R.tolist(),
        "t_ee_camera_m": best_t.flatten().tolist(),
        "rotvec_ee_camera": rvec_result.flatten().tolist(),
        "n_samples": len(R_ee_list),
    }

    output_path.parent.mkdir(parents=True, exist_ok=True)
    with open(output_path, "w") as f:
        json.dump(result, f, indent=2)

    print(f"T_ee_camera saved to: {output_path}")
    print()
    print("  t_ee_camera_m  =", [f"{v:+.4f}" for v in result["t_ee_camera_m"]])
    print("  rotvec_ee_cam  =", [f"{v:+.4f}" for v in result["rotvec_ee_camera"]])
    print()
    print("Update make_T_task_camera() in the perception scripts to load this file,")
    print("or use load_T_ee_camera() from control_scripts.examples.calibrate_camera.")


# ---------------------------------------------------------------------------
# Public helper for perception scripts and task scripts
# ---------------------------------------------------------------------------

def load_T_ee_camera(
    path: Optional[Path] = None,
) -> np.ndarray:
    """Load the calibrated T_ee_camera from JSON and return as a 4×4 matrix.

    This is the camera pose in the TCP frame (fixed extrinsic).  Combine
    with the real-time TCP pose to get the camera pose in task frame:

        X_task_camera = X_task_ee  @  load_T_ee_camera()

    Parameters
    ----------
    path : optional override.  Defaults to
           ``<repo_root>/control_scripts/T_ee_camera.json``.
    """
    if path is None:
        path = Path(__file__).parent.parent / "T_ee_camera.json"
    with open(path) as f:
        data = json.load(f)
    R = np.array(data["R_ee_camera"], dtype=np.float64)
    t = np.array(data["t_ee_camera_m"], dtype=np.float64).reshape(3, 1)
    M = np.eye(4)
    M[:3, :3] = R
    M[:3,  3] = t.ravel()
    return M


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main() -> None:
    ap = argparse.ArgumentParser(
        description="Hand-eye calibration for wrist-mounted RealSense."
    )
    ap.add_argument(
        "--arm", choices=["left", "right"], default="left",
        help="Which arm carries the camera (default: left).",
    )
    ap.add_argument(
        "--ip", default=None,
        help="Override robot IP (default: 192.168.1.101 for left, .102 for right).",
    )
    ap.add_argument(
        "--min-samples", type=int, default=12,
        help="Minimum captures required before Q is accepted (default: 12).",
    )
    ap.add_argument(
        "--print-board", action="store_true",
        help="Save charuco_board.png and exit (no robot connection needed).",
    )
    ap.add_argument(
        "--output", default=None,
        help="Output JSON path (default: calibration/build/T_ee_camera.json).",
    )
    args = ap.parse_args()

    if args.print_board:
        print_board()
        return

    # Resolve IP and output path
    default_ips = {"left": "192.168.1.101", "right": "192.168.1.102"}
    ip = args.ip if args.ip else default_ips[args.arm]

    if args.output:
        output_path = Path(args.output)
    else:
        repo_root = Path(__file__).resolve().parent.parent.parent
        output_path = repo_root / "calibration" / "build" / "T_ee_camera.json"
        output_path.parent.mkdir(parents=True, exist_ok=True)

    _run_live(
        arm_name=args.arm,
        ip=ip,
        min_samples=args.min_samples,
        output_path=output_path,
    )


if __name__ == "__main__":
    main()
