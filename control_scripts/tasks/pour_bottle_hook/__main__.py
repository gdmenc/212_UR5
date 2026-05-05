"""pour_bottle_hook - perception-driven autonomous bottle pick, pour, place.

This follows the same autonomy pattern as pick_place_bowl_hook_full:

1. Move the left hook arm to the fixed observation pose.
2. Read live TCP and compute T_task_camera from the saved hand-eye calibration.
3. Run the C++ YOLO + RealSense detector and save an annotated preview image.
4. Build a task-frame hook-rim grasp from the detected bottle base.
5. Execute pick, pour, and place with interactive checkpoints unless --auto.

Bottle-specific values live in pour_bottle_hook.py; the autonomous wrapper
keeps the same task-frame and checkpoint tactics as the bowl task.
"""

from __future__ import annotations

import argparse
import json
import subprocess
import time
from pathlib import Path

import numpy as np

from ...arm import ArmHandle
from ...grasps.bottle import bottle_hook_grasp, bottle_hook_pour_tcp_pose
from ...pick import pick
from ...place import place
from ...session import default_session
from ...util.poses import Pose
from ...util.rtde_convert import rtde_to_pose
from .pour_bottle_hook import (
    ARM,
    BOTTLE_PICK_POSE_TASK,
    BOTTLE_PLACE_POSE_TASK,
    CONFIG,
    GRASP_ANGLE_RAD,
    POUR_DURATION_S,
    POUR_HEIGHT_ABOVE_RIM_M,
    POUR_MAX_TILT_STEP_RAD,
    POUR_RECEIVER_RIM_Z_TASK,
    POUR_TARGET_TASK,
    POUR_TILT_RAD,
    _pour,
)


# ---------------------------------------------------------------------------
# Paths
# ---------------------------------------------------------------------------

_HERE = Path(__file__).parent
_REPO_ROOT = Path(__file__).parents[3]
_PERCEPTION_DIR = Path(__file__).parents[4] / "212_Perception" / "build"
_PERCEPTION_BIN = _HERE / "build" / "perception_bottle_hook"
_CALIB_JSON = _REPO_ROOT / "calibration" / "build" / "T_ee_camera.json"
_OFFSETS_JSON = _REPO_ROOT / "calibration" / "build" / "perception_offsets.json"
_YOLO_MODEL = _PERCEPTION_DIR / "yolo11n.onnx"
_COCO_CLASSES = _PERCEPTION_DIR / "coco-classes.txt"
_PREVIEW_IMAGE = Path("/tmp/bottle_hook_detection_preview.jpg")


# ---------------------------------------------------------------------------
# Task constants
# ---------------------------------------------------------------------------

# Observation pose "candidate3" (2026-05-01): hook tip points +X, camera sees table.
OBS_Q_RAD = [
    0.6895843744277954,
    -0.8284762662700196,
    1.2472832838641565,
    -0.17453940332446294,
    -1.0418575445758265,
    -4.834977690373556,
]

BOTTLE_XY_WARN_RADIUS_M = 0.20
BOTTLE_EXPECTED_TASK = np.array(BOTTLE_PICK_POSE_TASK.translation[:2], dtype=float)
PERCEPTION_ATTEMPTS = 1
POSE_SAMPLES_DEFAULT = 1
TABLE_Z_TASK_M = 0.0
CXX_WARMUP_FRAMES = 1
CXX_DETECT_FRAMES = 1

# Correction applied to the detected bottle base position before planning.
# Z = -0.01 matches the hand-tuned static pose (deeper hook engagement).
# Tune XY if detection has a residual systematic offset after calibration.
PERCEPTION_OFFSET_M = np.array([0.0, 0.0, -0.01])


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
# Checkpoint helper
# ---------------------------------------------------------------------------

def _confirm(prompt: str, auto: bool) -> None:
    """Block until Enter pressed. 'n'/'q' aborts. No-op when auto=True."""
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
# Calibration and task-camera transform
# ---------------------------------------------------------------------------

def _load_T_ee_camera(path: Path) -> np.ndarray:
    if not path.exists():
        raise FileNotFoundError(f"Calibration file not found: {path}")
    data = json.loads(path.read_text())
    M = np.eye(4, dtype=float)
    M[:3, :3] = np.array(data["R_ee_camera"], dtype=float)
    M[:3, 3] = np.array(data["t_ee_camera_m"], dtype=float)
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


def _compute_T_task_camera(arm: ArmHandle, T_ee_cam: np.ndarray) -> np.ndarray:
    tcp_rtde = arm.receive.getActualTCPPose()
    X_base_ee = rtde_to_pose(tcp_rtde)
    X_task_ee = arm.to_task(X_base_ee)
    T = np.eye(4, dtype=float)
    T[:3, :3] = X_task_ee.rotation.as_matrix()
    T[:3, 3] = X_task_ee.translation
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
    out = Ts[-1].copy()
    out[:3, 3] = np.mean([T[:3, 3] for T in Ts], axis=0)
    return out


def _run_perception_best(
    arm: ArmHandle,
    T_ee_cam: np.ndarray,
    auto: bool,
    serial: str,
    attempts: int = PERCEPTION_ATTEMPTS,
    pose_samples: int = POSE_SAMPLES_DEFAULT,
    warmup_frames: int = CXX_WARMUP_FRAMES,
    detect_frames: int = CXX_DETECT_FRAMES,
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
        )
        if not result.get("detected", False):
            continue
        conf = float(result.get("confidence", 0.0))
        if best is None or conf > float(best[0].get("confidence", 0.0)):
            best = (result, T_task_cam)
    if best is None:
        raise RuntimeError(f"Bottle not detected after {attempts} attempt(s).")
    return best


# ---------------------------------------------------------------------------
# C++ perception
# ---------------------------------------------------------------------------

def _check_paths() -> None:
    for path, label in [
        (_PERCEPTION_BIN, "Perception binary - run with --build first"),
        (_YOLO_MODEL, "YOLOv11 ONNX (212_Perception/build/yolo11n.onnx)"),
        (_COCO_CLASSES, "COCO classes (212_Perception/build/coco-classes.txt)"),
    ]:
        if not path.exists():
            raise FileNotFoundError(f"{label}\n  Not found: {path}")


def _run_perception(
    T_task_cam: np.ndarray,
    conf: float = 0.40,
    auto: bool = False,
    serial: str = "",
    warmup_frames: int = CXX_WARMUP_FRAMES,
    detect_frames: int = CXX_DETECT_FRAMES,
) -> dict:
    """Run C++ binary. In interactive mode saves annotated JPEG and opens it."""
    _check_paths()

    T_str = " ".join(f"{v:.10f}" for v in T_task_cam.flatten())
    cmd = [
        "sudo", "-n",
        str(_PERCEPTION_BIN),
        "--model", str(_YOLO_MODEL),
        "--classes", str(_COCO_CLASSES),
        "--conf", str(conf),
        "--T-task-camera", T_str,
        "--save-image", str(_PREVIEW_IMAGE),
    ]
    if serial:
        cmd.extend(["--serial", serial])
    cmd.extend(["--warmup", str(max(0, warmup_frames))])
    cmd.extend(["--frames", str(max(1, detect_frames))])
    if auto:
        cmd.append("--headless")

    print(f"[perception] {_PERCEPTION_BIN.name}"
          + (" (headless)" if auto else " - saving preview image"))
    result = subprocess.run(cmd, capture_output=True, text=True)

    if result.stderr:
        for line in result.stderr.strip().splitlines():
            print(f"  [C++] {line}")

    if result.returncode != 0:
        raise RuntimeError(f"Perception binary exited {result.returncode}")

    if not auto and _PREVIEW_IMAGE.exists():
        print(f"\n  [preview] Opening annotated frame: {_PREVIEW_IMAGE}")
        subprocess.Popen(["open", str(_PREVIEW_IMAGE)])

    try:
        return json.loads(result.stdout)
    except json.JSONDecodeError as exc:
        raise RuntimeError(
            f"JSON parse error: {exc}\nstdout was:\n{result.stdout[:500]}"
        ) from exc


# ---------------------------------------------------------------------------
# Plan helpers
# ---------------------------------------------------------------------------

def _sanity_check(bottle_base_task_m: list[float]) -> None:
    xy = np.array(bottle_base_task_m[:2], dtype=float)
    dist = float(np.linalg.norm(xy - BOTTLE_EXPECTED_TASK))
    if dist > BOTTLE_XY_WARN_RADIUS_M:
        print(f"  [WARN] Bottle XY {np.round(xy, 3)} is {dist*100:.0f} cm from "
              f"expected {BOTTLE_EXPECTED_TASK} - check calibration / placement.")
    else:
        print(f"  [OK]   Bottle XY {np.round(xy, 3)} m - "
              f"{dist*100:.1f} cm from expected.")


def _build_plan(bottle_base_pose: Pose):
    grasp = bottle_hook_grasp(bottle_base_pose, angle_rad=GRASP_ANGLE_RAD)
    upright_at_pour = bottle_hook_pour_tcp_pose(
        bottle_base_pose,
        POUR_TARGET_TASK,
        angle_rad=GRASP_ANGLE_RAD,
        tilt_rad=0.0,
    )
    pour_pose = bottle_hook_pour_tcp_pose(
        bottle_base_pose,
        POUR_TARGET_TASK,
        angle_rad=GRASP_ANGLE_RAD,
        tilt_rad=POUR_TILT_RAD,
    )
    place_pose = bottle_hook_grasp(
        BOTTLE_PLACE_POSE_TASK,
        angle_rad=GRASP_ANGLE_RAD,
    ).grasp_pose
    return grasp, upright_at_pour, pour_pose, place_pose


def _print_plan(
    bottle_base_pose: Pose,
    grasp,
    upright_at_pour: Pose,
    pour_pose: Pose,
    place_pose: Pose,
) -> None:
    print("=" * 62)
    print("  Arm                   :", ARM, "(hook gripper)")
    print("  Detected bottle base  :", np.round(bottle_base_pose.translation, 4), "m (task)")
    print("  Grasp angle           :", f"{np.degrees(GRASP_ANGLE_RAD):+.0f} deg")
    print("  Grasp pose (task)     :", np.round(grasp.grasp_pose.translation, 4), "m")
    print("  Pregrasp offset       :", f"{grasp.pregrasp_offset * 100:.1f} cm")
    print("  Pour target (opening) :", np.round(POUR_TARGET_TASK, 4), "m")
    print("  Receiver rim z        :", f"{POUR_RECEIVER_RIM_Z_TASK:.3f} m")
    print("  Pour height           :", f"{POUR_HEIGHT_ABOVE_RIM_M * 100:.1f} cm above rim")
    print("  Upright at pour       :", np.round(upright_at_pour.translation, 4), "m")
    print("  Final pour TCP        :", np.round(pour_pose.translation, 4), "m")
    print("  Pour tilt / hold      :", f"{np.degrees(POUR_TILT_RAD):.0f} deg / {POUR_DURATION_S:.1f} s")
    print("  Place pose (task)     :", np.round(place_pose.translation, 4), "m")
    print("  Transit Z             :", CONFIG.transit_z, "m")
    print("=" * 62)


# ---------------------------------------------------------------------------
# Dry run
# ---------------------------------------------------------------------------

def _dry_run() -> int:
    T_ee_cam = _load_T_ee_camera(_CALIB_JSON)
    print(f"[calib] t_ee_camera = {np.round(T_ee_cam[:3, 3], 4)}")
    grasp, upright_at_pour, pour_pose, place_pose = _build_plan(BOTTLE_PICK_POSE_TASK)
    print("\n[dry run] Static bottle pose - no camera / RTDE.")
    _print_plan(BOTTLE_PICK_POSE_TASK, grasp, upright_at_pour, pour_pose, place_pose)
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
) -> int:
    with default_session(left=True, right=False) as session:
        arm = session.arms[ARM]

        # ── Phase 1-2: camera setup ────────────────────────────────────────
        print("\n" + "-" * 62)
        if cam == "arm":
            T_ee_cam = _load_T_ee_camera(_CALIB_JSON)
            print(f"[calib] t_ee_camera = {np.round(T_ee_cam[:3, 3], 4)}")
            print("[1/6]  Moving to observation pose (candidate3) ...")
            arm.control.moveJ(OBS_Q_RAD, speed=0.3, acceleration=0.2)
            time.sleep(0.3)
            print("  Reached observation pose.  Hook tip -> +X, camera faces table.")
            _confirm("Arm at obs pose. Run perception and open detection image?", auto)
            print(f"[2/6]  Reading TCP -> T_task_camera (pose samples: {pose_samples}) ...")
            T_task_cam = _compute_T_task_camera_stable(arm, T_ee_cam, samples=pose_samples)
            cam_serial = ARM_CAM_SERIAL
        else:
            print("[1-2/6]  Top-down camera: static transform (no arm movement).")
            T_task_cam = TOP_CAM_T_TASK_CAM
            cam_serial = TOP_CAM_SERIAL
        key = _offset_key(cam, cam_serial)
        xy_offset = _get_xy_offset(_OFFSETS_JSON, key)
        print(f"[calib] loaded XY perception offset for {key}: {np.round(xy_offset, 4)} m")
        print(f"  Camera origin (task frame): {np.round(T_task_cam[:3, 3], 4)} m")

        print("\n" + "-" * 62)
        print("[3/6]  Running C++ bottle detection ...")
        if cam == "arm":
            try:
                result, T_task_cam = _run_perception_best(
                    arm, T_ee_cam, auto=auto, serial=cam_serial, attempts=attempts,
                    pose_samples=pose_samples, warmup_frames=warmup_frames, detect_frames=detect_frames
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
            )
        if not result.get("detected", False):
            print("\n[FAIL] Bottle not detected. Suggestions:")
            print("  - Is the bottle visible from this pose?")
            print("  - Lower conf: edit _run_perception(conf=0.30)")
            return 1

        conf = result["confidence"]
        bottle_base_task_m = result["bottle_base_task_m"]
        bottle_neck_task_m = result.get("bottle_neck_task_m")
        print(f"\n  conf             = {conf:.2f}")
        print(f"  bottle_base_task = {np.round(bottle_base_task_m, 4)} m")
        if bottle_neck_task_m is not None:
            print(f"  bottle_neck_task = {np.round(bottle_neck_task_m, 4)} m")
        _sanity_check(bottle_base_task_m)

        if not auto:
            print(f"\n  Preview image: {_PREVIEW_IMAGE}  (should have opened in Preview.app)")

        bottle_xy_detected = np.array(bottle_base_task_m[:2], dtype=float)
        if zero_camera:
            if reference_center_task is None:
                raise RuntimeError("zero_camera=True requires reference_center_task.")
            target_xy = np.array(reference_center_task[:2], dtype=float)
            xy_offset = target_xy - bottle_xy_detected
            _set_xy_offset(_OFFSETS_JSON, key, xy_offset)
            print(
                f"[calib] updated XY perception offset for {key}: {np.round(xy_offset, 4)} m "
                f"(detected {np.round(bottle_xy_detected, 4)} -> target {np.round(target_xy, 4)})"
            )

        _confirm("Detection look correct? Build grasp and pour plan?", auto)

        print("\n" + "-" * 62)
        print("[4/6]  Building grasp + pour plan ...")
        bottle_base = np.array(bottle_base_task_m, dtype=float)
        bottle_base[:2] += xy_offset
        bottle_base[2] = TABLE_Z_TASK_M
        bottle_base_pose = Pose(
            translation=bottle_base + PERCEPTION_OFFSET_M
        )
        print(f"  offset-corrected bottle base xy = {np.round(bottle_base[:2], 4)} m")
        print(f"  table-anchored bottle base z = {TABLE_Z_TASK_M:.3f} m")
        print(f"  Corrected base   : {np.round(bottle_base_pose.translation, 4)} m"
              f"  (offset {PERCEPTION_OFFSET_M})")
        grasp, upright_at_pour, pour_pose, place_pose = _build_plan(bottle_base_pose)
        _print_plan(bottle_base_pose, grasp, upright_at_pour, pour_pose, place_pose)
        _confirm("Plan looks correct? Execute PICK?", auto)

        print("\n" + "-" * 62)
        print(f"[5/6]  Picking ({grasp.description}) ...")
        arm.gripper.open()
        pick_result = pick(arm, grasp, CONFIG)
        if not pick_result.success:
            print(f"  pick FAILED: {pick_result.reason}")
            return 1
        print("  pick succeeded.")

        _confirm("Pick done. Execute POUR and PLACE?", auto)
        print("\n" + "-" * 62)
        print(f"[6/6]  Pouring at {np.round(POUR_TARGET_TASK, 4)} then placing ...")
        _pour(
            arm,
            upright_at_pour,
            bottle_base_pose,
            POUR_TARGET_TASK,
            POUR_TILT_RAD,
            POUR_MAX_TILT_STEP_RAD,
            GRASP_ANGLE_RAD,
            CONFIG,
        )
        print("  pour complete.")

        print(f"  Placing at {BOTTLE_PLACE_POSE_TASK.translation} ...")
        place_result = place(arm, place_pose, CONFIG)
        if not place_result.success:
            print(f"  place FAILED: {place_result.reason}")
            return 1
        print("  place succeeded.")

    print("\nBottle hook pick-pour-place complete.")
    return 0


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main() -> None:
    parser = argparse.ArgumentParser(
        description="Autonomous bottle hook pick/pour/place - YOLOv11 + RealSense + UR5."
    )
    parser.add_argument("--auto", action="store_true",
                        help="Skip all interactive checkpoints.")
    parser.add_argument("--dry", action="store_true",
                        help="Print plan with static bottle pose; no robot or camera.")
    parser.add_argument("--build", action="store_true",
                        help="Build the perception_bottle_hook binary then exit.")
    parser.add_argument("--cam", choices=["arm", "top"], default="arm",
                        help="Camera: 'arm' = eye-in-hand (default), 'top' = overhead fixed rig.")
    parser.add_argument("--attempts", type=int, default=PERCEPTION_ATTEMPTS,
                        help="Perception attempts (default 1 for mac stability).")
    parser.add_argument("--pose-samples", type=int, default=POSE_SAMPLES_DEFAULT,
                        help="TCP samples for T_task_camera averaging (default 1).")
    parser.add_argument("--warmup-frames", type=int, default=CXX_WARMUP_FRAMES,
                        help="C++ RealSense warmup frames (default 1 for mac stability).")
    parser.add_argument("--detect-frames", type=int, default=CXX_DETECT_FRAMES,
                        help="C++ frames to aggregate per detection run (default 1 for mac stability).")
    parser.add_argument("--zero-camera", action="store_true",
                        help="Calibrate and save XY perception offset from a known reference center.")
    parser.add_argument("--ref-center", type=float, nargs=3, default=[0.0, 0.0, 0.0],
                        metavar=("X", "Y", "Z"),
                        help="Known task-frame reference center used with --zero-camera.")
    args = parser.parse_args()

    if args.build:
        subprocess.run(["cmake", "-B", "build", "-DCMAKE_BUILD_TYPE=Release"],
                       cwd=str(_HERE), check=True)
        subprocess.run(["cmake", "--build", "build"], cwd=str(_HERE), check=True)
        print("[build] done ->", _PERCEPTION_BIN)
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
        )
    )


if __name__ == "__main__":
    main()
