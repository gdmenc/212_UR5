"""pick_place_bowl_hook_full — perception-driven autonomous bowl pick-and-place.

Interactive checkpoints are ON by default.  Pass --auto to skip all of them.

Pipeline
--------
1. Move left arm to observation pose (candidate3).
2. Read live TCP → compute T_task_camera.
3. Run C++ perception (YOLO bowl prompt + SAM2 mask when ONNX models present,
   librealsense2) → bowl position in task frame.
   Saves an annotated JPEG to /tmp/bowl_detection_preview.jpg and opens it
   in Preview.app so you can verify the detection before continuing.
4. Print detected position + grasp plan, wait for confirmation.
5. Execute pick, wait for confirmation.
6. Execute place.

Task-frame / grasp-angle consistency
--------------------------------------
Observation pose "candidate3" (2026-05-01):
  EE rotvec ≈ [−0.047, −2.173, 0.031] → hook tip points roughly +X.
  GRASP_ANGLE_RAD = π places the hook contact on the −X rim of the bowl
  and approaches from further −X — consistent with the arm's natural posture
  at the observation pose.  Do not change the grasp angle without re-verifying
  reachability from this pose.

Running
-------
    sudo python -m control_scripts.tasks.pick_place_bowl_hook_full           # interactive (default)
    sudo python -m control_scripts.tasks.pick_place_bowl_hook_full --auto    # no stops
    sudo python -m control_scripts.tasks.pick_place_bowl_hook_full --dry     # plan only, no robot
    sudo python -m control_scripts.tasks.pick_place_bowl_hook_full --build   # cmake build

Sudo note
---------
The C++ subprocess inherits the parent UID.  sudo on the Python process is
sufficient for RealSense USB access — no separate sudo needed inside the command.
cv::imshow does NOT work under sudo on macOS (window server restriction), so the
C++ binary saves the annotated frame to a JPEG file instead.
"""

from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
import time
from pathlib import Path

import numpy as np

from ...arm import ArmHandle
from ...config import PickPlaceConfig
from ...grasps.bowl import bowl_hook_grasp
from ...session import default_session
from ...util.poses import Pose
from ...util.rotations import Rotation
from ...util.rtde_convert import rtde_to_pose
from ...util.se3_average import average_se3
from . import pick_place_bowl_hook as _bowl_exec


# ---------------------------------------------------------------------------
# Paths
# ---------------------------------------------------------------------------

_HERE           = Path(__file__).parent
_REPO_ROOT      = Path(__file__).parents[3]        # …/212_UR5
_PERCEPTION_DIR = Path(__file__).parents[4] / "212_Perception" / "build"
_PERCEPTION_BIN = _HERE / "build" / "perception_bowl"
_CALIB_JSON     = _REPO_ROOT / "calibration" / "build" / "T_ee_camera.json"
_OFFSETS_JSON   = _REPO_ROOT / "calibration" / "build" / "perception_offsets.json"
_YOLO_MODEL     = _PERCEPTION_DIR / "yolo11n.onnx"
_COCO_CLASSES   = _PERCEPTION_DIR / "coco-classes.txt"
_PREVIEW_IMAGE  = Path("/tmp/bowl_detection_preview.jpg")
_SAM2_ENCODER_DEFAULT = _REPO_ROOT / "sam2_hiera_tiny" / "sam2_hiera_tiny.encoder.onnx"
_SAM2_DECODER_DEFAULT = _REPO_ROOT / "sam2_hiera_tiny" / "sam2_hiera_tiny.decoder.onnx"


# ---------------------------------------------------------------------------
# Task constants
# ---------------------------------------------------------------------------

BOWL_PLACE_POSE_TASK = Pose(translation=[-0.1, 0.2, 0.025])

# π → hook on −X rim, approaches from −X — matches observation pose orientation.
GRASP_ANGLE_RAD   = float(np.radians(180))
PLACE_ANGLE_RAD   = float(np.radians(180 + 45))
APPROACH_TILT_RAD = float(np.radians(10))

# Observation pose "candidate3" (2026-05-01): hook tip points +X, camera sees table.
OBS_Q_RAD = [
    0.6497170329093933,
    0.2708507019230346,
    -0.8138680458068848,
    0.761660023326538,
    -0.9829977194415491,
    -4.845102612172262
]

ARM = "ur_left"

CONFIG = PickPlaceConfig(
    transit_z=0.3,
    place_use_contact_descent=False,
    transit_speed=0.1,
    transit_accel=0.2,
    approach_speed=0.05,
    approach_accel=0.2,
    retract_speed=0.1,
    retract_accel=0.2,
    release_aperture_mm=None,
    gripper_open_speed_pct=40,
    gripper_close_speed_pct=30,
)

BOWL_XY_WARN_RADIUS_M = 0.20
BOWL_EXPECTED_TASK    = np.array([0.1, 0.0])
PERCEPTION_ATTEMPTS = 1
POSE_SAMPLES_DEFAULT = 1
TABLE_Z_TASK_M = 0.0
CXX_WARMUP_FRAMES = 1
CXX_DETECT_FRAMES = 1


# ---------------------------------------------------------------------------
# Camera configuration
# ---------------------------------------------------------------------------

# Serial numbers — find with: python3 perception/multicamera_view.py
ARM_CAM_SERIAL = ""   # eye-in-hand RealSense (empty = first available)
TOP_CAM_SERIAL = ""   # overhead top-down RealSense (empty = first available)

# Static T_task_camera for the overhead top-down camera.
# Translation hand-measured; rotation assumes camera faces straight down
# with +X aligned to task +X.  Verify with --dry --cam top before running.
_TC_X =  0.0125
_TC_Y = -0.270 - 0.126 + 0.510 + 0.0425   # = 0.1565 m
_TC_Z =  1.235
TOP_CAM_T_TASK_CAM = np.array([
    [ 1.,  0.,  0., _TC_X],
    [ 0., -1.,  0., _TC_Y],
    [ 0.,  0., -1., _TC_Z],
    [ 0.,  0.,  0.,  1.  ],
], dtype=float)


# ---------------------------------------------------------------------------
# Checkpoint helper  (skipped when auto=True)
# ---------------------------------------------------------------------------

def _confirm(prompt: str, auto: bool) -> None:
    """Block until Enter pressed.  'n'/'q' aborts.  No-op when auto=True."""
    if auto:
        return
    print()
    while True:
        ans = input(f"  >>> {prompt}  [Enter = yes  /  n = abort] ").strip().lower()
        if ans in ("", "y", "yes"):
            return
        if ans in ("n", "no", "q", "quit", "abort"):
            print("  Aborted.")
            raise SystemExit(0)


# ---------------------------------------------------------------------------
# Calibration
# ---------------------------------------------------------------------------

def _load_T_ee_camera(path: Path) -> np.ndarray:
    if not path.exists():
        raise FileNotFoundError(f"Calibration file not found: {path}")
    data = json.loads(path.read_text())
    M = np.eye(4, dtype=float)
    M[:3, :3] = np.array(data["R_ee_camera"], dtype=float)
    M[:3,  3] = np.array(data["t_ee_camera_m"], dtype=float)
    return M


def _offset_key(cam: str, serial: str) -> str:
    serial_key = serial if serial else "default"
    return f"{cam}:{serial_key}"


def _load_perception_offsets(path: Path) -> dict:
    if not path.exists():
        return {}
    try:
        return json.loads(path.read_text())
    except json.JSONDecodeError:
        return {}


def _save_perception_offsets(path: Path, data: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, indent=2))


def _get_xy_offset(path: Path, key: str) -> np.ndarray:
    blob = _load_perception_offsets(path)
    val = blob.get(key, {}).get("xy_offset_m", [0.0, 0.0])
    arr = np.array(val, dtype=float).reshape(2)
    return arr


def _set_xy_offset(path: Path, key: str, xy_offset: np.ndarray) -> None:
    blob = _load_perception_offsets(path)
    if key not in blob:
        blob[key] = {}
    blob[key]["xy_offset_m"] = [float(xy_offset[0]), float(xy_offset[1])]
    _save_perception_offsets(path, blob)


# ---------------------------------------------------------------------------
# T_task_camera
# ---------------------------------------------------------------------------

def _compute_T_task_camera(arm: ArmHandle, T_ee_cam: np.ndarray) -> np.ndarray:
    tcp_rtde  = arm.receive.getActualTCPPose()
    X_base_ee = rtde_to_pose(tcp_rtde)
    X_task_ee = arm.to_task(X_base_ee)
    T = np.eye(4, dtype=float)
    T[:3, :3] = X_task_ee.rotation.as_matrix()
    T[:3,  3] = X_task_ee.translation
    return T @ T_ee_cam


def _compute_T_task_camera_stable(
    arm: ArmHandle,
    T_ee_cam: np.ndarray,
    samples: int = POSE_SAMPLES_DEFAULT,
    sample_dt_s: float = 0.05,
) -> np.ndarray:
    if samples <= 1:
        return _compute_T_task_camera(arm, T_ee_cam)
    Ts = []
    for _ in range(samples):
        Ts.append(_compute_T_task_camera(arm, T_ee_cam))
        time.sleep(sample_dt_s)
    return average_se3(Ts)


def _sam_cpp_args(no_sam: bool, enc: Path, dec: Path) -> list[str]:
    """Pass SAM ONNX paths to C++ whenever both exist and --no-sam was not set."""
    if no_sam:
        return []
    if enc.exists() and dec.exists():
        return ["--sam-encoder", str(enc), "--sam-decoder", str(dec)]
    print(f"[perception] SAM2 ONNX not found (encoder={enc.exists()}, "
          f"decoder={dec.exists()}) — C++ will use YOLO-only geometry.")
    return []


def _run_perception_best(
    arm: ArmHandle,
    T_ee_cam: np.ndarray,
    auto: bool,
    serial: str,
    attempts: int = PERCEPTION_ATTEMPTS,
    pose_samples: int = POSE_SAMPLES_DEFAULT,
    warmup_frames: int = CXX_WARMUP_FRAMES,
    detect_frames: int = CXX_DETECT_FRAMES,
    no_sam: bool = False,
    sam_encoder: Path = _SAM2_ENCODER_DEFAULT,
    sam_decoder: Path = _SAM2_DECODER_DEFAULT,
) -> tuple[dict, np.ndarray]:
    best = None
    for i in range(attempts):
        T_task_cam = _compute_T_task_camera_stable(arm, T_ee_cam, samples=pose_samples)
        print(f"  [attempt {i+1}/{attempts}] camera origin = {np.round(T_task_cam[:3,3], 4)} m")
        result = _run_perception(
            T_task_cam,
            auto=auto,
            serial=serial,
            warmup_frames=warmup_frames,
            detect_frames=detect_frames,
            no_sam=no_sam,
            sam_encoder=sam_encoder,
            sam_decoder=sam_decoder,
        )
        if not result.get("detected", False):
            continue
        conf = float(result.get("confidence", 0.0))
        if best is None or conf > float(best[0].get("confidence", 0.0)):
            best = (result, T_task_cam)
    if best is None:
        raise RuntimeError(f"Bowl not detected after {attempts} attempt(s).")
    return best


# ---------------------------------------------------------------------------
# C++ perception
# ---------------------------------------------------------------------------

def _check_paths() -> None:
    for p, label in [
        (_PERCEPTION_BIN, "Perception binary — run with --build first"),
        (_YOLO_MODEL,     "YOLOv11 ONNX (212_Perception/build/yolo11n.onnx)"),
        (_COCO_CLASSES,   "COCO classes (212_Perception/build/coco-classes.txt)"),
    ]:
        if not p.exists():
            raise FileNotFoundError(f"{label}\n  Not found: {p}")


def _run_perception(T_task_cam: np.ndarray,
                    conf: float = 0.40,
                    auto: bool = False,
                    serial: str = "",
                    warmup_frames: int = CXX_WARMUP_FRAMES,
                    detect_frames: int = CXX_DETECT_FRAMES,
                    no_sam: bool = False,
                    sam_encoder: Path = _SAM2_ENCODER_DEFAULT,
                    sam_decoder: Path = _SAM2_DECODER_DEFAULT,
                    preview_image: Path | None = None) -> dict:
    """Run C++ binary.  In interactive mode saves annotated JPEG and opens it."""
    _check_paths()

    save_preview = preview_image if preview_image is not None else _PREVIEW_IMAGE

    T_str = " ".join(f"{v:.10f}" for v in T_task_cam.flatten())
    cmd = [
        "sudo", "-n",
        str(_PERCEPTION_BIN),
        "--model",          str(_YOLO_MODEL),
        "--classes",        str(_COCO_CLASSES),
        "--conf",           str(conf),
        "--T-task-camera",  T_str,
        "--save-image",     str(save_preview),
        "--grasp-angle-deg", f"{np.degrees(GRASP_ANGLE_RAD):.3f}",
        "--approach-tilt-deg", f"{np.degrees(APPROACH_TILT_RAD):.3f}",
    ]
    if serial:
        cmd.extend(["--serial", serial])
    cmd.extend(_sam_cpp_args(no_sam, sam_encoder, sam_decoder))
    if no_sam:
        cmd.append("--no-sam")
    cmd.extend(["--warmup", str(max(0, warmup_frames))])
    cmd.extend(["--frames", str(max(1, detect_frames))])
    if auto:
        cmd.append("--headless")

    print(f"[perception] {_PERCEPTION_BIN.name}"
          + (" (headless)" if auto else " — saving preview image"))

    result = subprocess.run(cmd, capture_output=True, text=True)

    # Always print C++ stderr so the user sees warmup / detection logs
    if result.stderr:
        for line in result.stderr.strip().splitlines():
            print(f"  [C++] {line}")

    if result.returncode != 0:
        raise RuntimeError(f"Perception binary exited {result.returncode}")

    # Open the annotated image in Preview.app so the user can inspect it
    if not auto and save_preview.exists():
        print(f"\n  [preview] Opening annotated frame: {save_preview}")
        subprocess.Popen(["open", str(save_preview)])   # non-blocking

    try:
        return json.loads(result.stdout)
    except json.JSONDecodeError as e:
        raise RuntimeError(
            f"JSON parse error: {e}\nstdout was:\n{result.stdout[:500]}"
        )


# ---------------------------------------------------------------------------
# Grasp helpers
# ---------------------------------------------------------------------------

def _sanity_check(bowl_base_task_m: list[float]) -> None:
    xy   = np.array(bowl_base_task_m[:2])
    dist = float(np.linalg.norm(xy - BOWL_EXPECTED_TASK))
    if dist > BOWL_XY_WARN_RADIUS_M:
        print(f"  [WARN] Bowl XY {np.round(xy,3)} is {dist*100:.0f} cm from "
              f"expected {BOWL_EXPECTED_TASK} — check calibration / bowl placement.")
    else:
        print(f"  [OK]   Bowl XY {np.round(xy,3)} m — {dist*100:.1f} cm from expected.")


def _print_plan(bowl_base_pose: Pose, grasp, place_pose: Pose) -> None:
    print("=" * 62)
    print("  Arm                :", ARM, "(hook gripper)")
    print("  Detected bowl base :", np.round(bowl_base_pose.translation, 4), "m (task)")
    print("  Grasp angle        :", f"{np.degrees(GRASP_ANGLE_RAD):+.0f}°  (hook on −X rim)")
    print("  Approach tilt      :", f"{np.degrees(APPROACH_TILT_RAD):+.1f}°")
    print("  Grasp pose (task)  :", np.round(grasp.grasp_pose.translation, 4), "m")
    print("  Pregrasp offset    :", f"{grasp.pregrasp_offset*100:.1f} cm")
    print("  Place pose (task)  :", np.round(place_pose.translation, 4), "m")
    print("  Transit Z          :", CONFIG.transit_z, "m")
    print("=" * 62)


# ---------------------------------------------------------------------------
# Dry run
# ---------------------------------------------------------------------------

def _dry_run() -> int:
    T_ee_cam = _load_T_ee_camera(_CALIB_JSON)
    print(f"[calib] t_ee_camera = {np.round(T_ee_cam[:3,3], 4)}")
    static_bowl = Pose(translation=[0.1, 0.0, 0.01])
    grasp = bowl_hook_grasp(static_bowl,
                            angle_rad=GRASP_ANGLE_RAD,
                            approach_tilt_rad=APPROACH_TILT_RAD)
    place_pose = bowl_hook_grasp(BOWL_PLACE_POSE_TASK,
                                 angle_rad=PLACE_ANGLE_RAD,
                                 approach_tilt_rad=APPROACH_TILT_RAD).grasp_pose
    print("\n[dry run] Static bowl pose — no camera / RTDE.")
    _print_plan(static_bowl, grasp, place_pose)
    print("[dry run] No motion commanded.")
    return 0


# ---------------------------------------------------------------------------
# Live run
# ---------------------------------------------------------------------------

def run(
    auto: bool,
    cam: str = "arm",
    attempts: int = PERCEPTION_ATTEMPTS,
    pose_samples: int = POSE_SAMPLES_DEFAULT,
    warmup_frames: int = CXX_WARMUP_FRAMES,
    detect_frames: int = CXX_DETECT_FRAMES,
    zero_camera: bool = False,
    reference_center_task: np.ndarray | None = None,
    motion_planning: bool = True,
    no_sam: bool = False,
    sam_encoder: Path = _SAM2_ENCODER_DEFAULT,
    sam_decoder: Path = _SAM2_DECODER_DEFAULT,
) -> int:
    if not motion_planning:
        _bowl_exec.USE_MOTION_PLANNING = False
        print("[CLI] motion planning DISABLED — bowl transits use sequential moveL.")
    print(f"[debug] executing module: {__file__}")
    print(
        "[debug] config "
        f"cam={cam} attempts={attempts} pose_samples={pose_samples} "
        f"warmup_frames={warmup_frames} detect_frames={detect_frames}"
    )
    with default_session(left=True, right=False) as session:
        arm = session.arms[ARM]

        # ── Phase 1-2: camera setup ────────────────────────────────────────
        print("\n" + "─" * 62)
        if cam == "arm":
            T_ee_cam = _load_T_ee_camera(_CALIB_JSON)
            print(f"[calib] t_ee_camera = {np.round(T_ee_cam[:3,3], 4)}")
            print("[1/5]  Moving to observation pose (candidate3) ...")
            arm.control.moveJ(OBS_Q_RAD, speed=0.3, acceleration=0.2)
            time.sleep(0.3)
            print("  Reached observation pose.  Hook tip → +X, camera faces table.")
            _confirm("Arm at obs pose. Run perception and open detection image?", auto)
            print(f"[2/5]  Reading TCP → T_task_camera (pose samples: {pose_samples}) ...")
            T_task_cam = _compute_T_task_camera_stable(arm, T_ee_cam, samples=pose_samples)
            cam_serial = ARM_CAM_SERIAL
        else:
            print("[1-2/5]  Top-down camera: static transform (no arm movement).")
            T_task_cam = TOP_CAM_T_TASK_CAM
            cam_serial = TOP_CAM_SERIAL
        key = _offset_key(cam, cam_serial)
        xy_offset = _get_xy_offset(_OFFSETS_JSON, key)
        print(f"[calib] loaded XY perception offset for {key}: {np.round(xy_offset, 4)} m")
        print(f"  Camera origin (task frame): {np.round(T_task_cam[:3,3], 4)} m")

        # ── Phase 3: perception ────────────────────────────────────────────
        print("\n" + "─" * 62)
        print("[3/5]  Running C++ bowl perception (YOLO + SAM2 when available) ...")
        if cam == "arm":
            try:
                result, T_task_cam = _run_perception_best(
                    arm, T_ee_cam, auto=auto, serial=cam_serial, attempts=attempts,
                    pose_samples=pose_samples, warmup_frames=warmup_frames, detect_frames=detect_frames,
                    no_sam=no_sam, sam_encoder=sam_encoder, sam_decoder=sam_decoder,
                )
            except RuntimeError as exc:
                print(f"\n[FAIL] {exc}")
                return 1
        else:
            result = _run_perception(
                T_task_cam,
                auto=auto,
                serial=cam_serial,
                warmup_frames=warmup_frames,
                detect_frames=detect_frames,
                no_sam=no_sam,
                sam_encoder=sam_encoder,
                sam_decoder=sam_decoder,
            )

        if not result.get("detected", False):
            print("\n[FAIL] Bowl not detected.  Suggestions:")
            print("  • Is the bowl visible from this pose?")
            print("  • Lower conf: edit _run_perception(conf=0.30)")
            return 1

        conf             = result["confidence"]
        bowl_base_task_m = result["bowl_base_task_m"]
        bowl_rim_task_m  = result["bowl_rim_task_m"]

        print(f"\n  conf           = {conf:.2f}")
        print(f"  bowl_base_task = {np.round(bowl_base_task_m, 4)} m")
        print(f"  bowl_rim_task  = {np.round(bowl_rim_task_m,  4)} m")
        _sanity_check(bowl_base_task_m)

        if not auto:
            print(f"\n  Preview image: {_PREVIEW_IMAGE}  (should have opened in Preview.app)")

        bowl_xy_detected = np.array(bowl_base_task_m[:2], dtype=float)
        if zero_camera:
            if reference_center_task is None:
                raise RuntimeError("zero_camera=True requires reference_center_task.")
            target_xy = np.array(reference_center_task[:2], dtype=float)
            xy_offset = target_xy - bowl_xy_detected
            _set_xy_offset(_OFFSETS_JSON, key, xy_offset)
            print(
                f"[calib] updated XY perception offset for {key}: {np.round(xy_offset, 4)} m "
                f"(detected {np.round(bowl_xy_detected, 4)} -> target {np.round(target_xy, 4)})"
            )

        _confirm("Detection look correct? Build grasp?", auto)

        # ── Phase 4: grasp plan ────────────────────────────────────────────
        print("\n" + "─" * 62)
        print("[4/5]  Building grasp ...")
        bowl_base = np.array(bowl_base_task_m, dtype=float)
        bowl_base[:2] += xy_offset
        bowl_base[2] = TABLE_Z_TASK_M
        bowl_base_pose = Pose(translation=bowl_base)
        print(f"  offset-corrected bowl base xy = {np.round(bowl_base[:2], 4)} m")
        print(f"  table-anchored bowl base z = {TABLE_Z_TASK_M:.3f} m")
        grasp = bowl_hook_grasp(bowl_base_pose,
                                angle_rad=GRASP_ANGLE_RAD,
                                approach_tilt_rad=APPROACH_TILT_RAD)
        place_pose = bowl_hook_grasp(BOWL_PLACE_POSE_TASK,
                                     angle_rad=PLACE_ANGLE_RAD,
                                     approach_tilt_rad=APPROACH_TILT_RAD).grasp_pose
        _print_plan(bowl_base_pose, grasp, place_pose)
        _confirm("Grasp plan looks correct? Execute PICK?", auto)

        # ── Phase 5: pick + place (optional Drake-planned transits) ───────
        print("\n" + "─" * 62)
        _confirm(
            f"Execute PICK and PLACE ({grasp.description}) "
            f"→ place @ {np.round(BOWL_PLACE_POSE_TASK.translation, 3)} m?",
            auto,
        )
        print(f"[5/5]  Pick & place ({grasp.description}) ...")
        if not _bowl_exec.run_on_arm(session, arm, grasp, place_pose, CONFIG):
            return 1

    print("\n✓ Bowl pick-and-place complete.")
    return 0


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main() -> None:
    ap = argparse.ArgumentParser(
        description="Autonomous bowl pick-and-place — YOLO + SAM2 + RealSense + UR5."
    )
    ap.add_argument("--auto", action="store_true",
                    help="Skip all interactive checkpoints (fully autonomous).")
    ap.add_argument("--dry", action="store_true",
                    help="Print plan with static bowl pose; no robot or camera.")
    ap.add_argument("--build", action="store_true",
                    help="cmake build the perception binary then exit.")
    ap.add_argument("--cam", choices=["arm", "top"], default="arm",
                    help="Camera: 'arm' = eye-in-hand (default), 'top' = overhead fixed rig.")
    ap.add_argument("--attempts", type=int, default=PERCEPTION_ATTEMPTS,
                    help="Perception attempts (default 1 for mac stability).")
    ap.add_argument("--pose-samples", type=int, default=POSE_SAMPLES_DEFAULT,
                    help="TCP samples for T_task_camera averaging (default 1).")
    ap.add_argument("--warmup-frames", type=int, default=CXX_WARMUP_FRAMES,
                    help="C++ RealSense warmup frames (default 1 for mac stability).")
    ap.add_argument("--detect-frames", type=int, default=CXX_DETECT_FRAMES,
                    help="C++ frames to aggregate per detection run (default 1 for mac stability).")
    ap.add_argument("--zero-camera", action="store_true",
                    help="Calibrate and save XY perception offset from a known reference center.")
    ap.add_argument("--ref-center", type=float, nargs=3, default=[0.0, 0.0, 0.0],
                    metavar=("X", "Y", "Z"),
                    help="Known task-frame reference center used with --zero-camera.")
    ap.add_argument("--no-sam", action="store_true",
                    help="Disable SAM2 in C++ (YOLO bbox only for depth sampling).")
    ap.add_argument("--sam-encoder", type=Path, default=_SAM2_ENCODER_DEFAULT,
                    help="Path to SAM2 encoder ONNX (passed to C++ when file exists).")
    ap.add_argument("--sam-decoder", type=Path, default=_SAM2_DECODER_DEFAULT,
                    help="Path to SAM2 decoder ONNX.")
    ap.add_argument("--onnxruntime-root", type=Path, default=None,
                    help="When used with --build, pass -DONNXRUNTIME_ROOT to cmake.")
    ap.add_argument(
        "--no-motion-planning",
        action="store_true",
        help="Disable Drake free-space transits; use moveL only (see pick_place_bowl_hook).",
    )
    args = ap.parse_args()

    if args.build:
        import subprocess as sp
        cmake_cfg = ["cmake", "-B", "build", "-DCMAKE_BUILD_TYPE=Release"]
        if args.onnxruntime_root is not None:
            cmake_cfg.append(f"-DONNXRUNTIME_ROOT={args.onnxruntime_root}")
        sp.run(cmake_cfg,
               cwd=str(_HERE), check=True)
        sp.run(["cmake", "--build", "build"], cwd=str(_HERE), check=True)
        print("[build] done →", _PERCEPTION_BIN)
        return

    if args.dry:
        raise SystemExit(_dry_run())

    raise SystemExit(
        run(
            auto=args.auto,
            cam=args.cam,
            attempts=max(1, args.attempts),
            pose_samples=max(1, args.pose_samples),
            warmup_frames=max(0, args.warmup_frames),
            detect_frames=max(1, args.detect_frames),
            zero_camera=args.zero_camera,
            reference_center_task=np.array(args.ref_center, dtype=float),
            motion_planning=not args.no_motion_planning,
            no_sam=args.no_sam,
            sam_encoder=args.sam_encoder,
            sam_decoder=args.sam_decoder,
        )
    )


if __name__ == "__main__":
    main()
