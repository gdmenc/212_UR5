"""pick_place_bowl_hook_full — perception-driven autonomous bowl pick-and-place.

Interactive checkpoints are ON by default.  Pass --auto to skip all of them.

Pipeline
--------
1. Move left arm to observation pose (candidate3).
2. Read live TCP → compute T_task_camera.
3. Run C++ perception (YOLOv11 + RealSense) → bowl position in task frame.
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
from ...pick import pick
from ...place import place
from ...session import default_session
from ...util.poses import Pose
from ...util.rotations import Rotation
from ...util.rtde_convert import rtde_to_pose


# ---------------------------------------------------------------------------
# Paths
# ---------------------------------------------------------------------------

_HERE           = Path(__file__).parent
_REPO_ROOT      = Path(__file__).parents[3]        # …/212_UR5
_PERCEPTION_DIR = Path(__file__).parents[4] / "212_Perception" / "build"
_PERCEPTION_BIN = _HERE / "build" / "perception_bowl"
_CALIB_JSON     = _REPO_ROOT / "calibration" / "build" / "T_ee_camera.json"
_YOLO_MODEL     = _PERCEPTION_DIR / "yolo11n.onnx"
_COCO_CLASSES   = _PERCEPTION_DIR / "coco-classes.txt"
_PREVIEW_IMAGE  = Path("/tmp/bowl_detection_preview.jpg")


# ---------------------------------------------------------------------------
# Task constants
# ---------------------------------------------------------------------------

BOWL_PLACE_POSE_TASK = Pose(translation=[-0.1, 0.2, 0.01])

# π → hook on −X rim, approaches from −X — matches observation pose orientation.
GRASP_ANGLE_RAD   = float(np.radians(180))
PLACE_ANGLE_RAD   = float(np.radians(180 + 45))
APPROACH_TILT_RAD = float(np.radians(10))

# Observation pose "candidate3" (2026-05-01): hook tip points +X, camera sees table.
OBS_Q_RAD = [
    0.6895843744277954,
    -0.8284762662700196,
    1.2472832838641565,
    -0.17453940332446294,
    -1.0418575445758265,
    -4.834977690373556
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
                    auto: bool = False) -> dict:
    """Run C++ binary.  In interactive mode saves annotated JPEG and opens it."""
    _check_paths()

    T_str = " ".join(f"{v:.10f}" for v in T_task_cam.flatten())
    # On macOS, USB/IOKit device access requires going through the full sudo
    # path even when the parent Python process is already root.  Forking a
    # subprocess from a root parent is NOT sufficient — the child must be
    # launched via sudo so macOS sets up the correct IOKit session.
    # sudo -n = non-interactive (no password prompt); works because the caller
    # already has an active sudo credential (sudo python -m ...).
    cmd = [
        "sudo", "-n",
        str(_PERCEPTION_BIN),
        "--model",          str(_YOLO_MODEL),
        "--classes",        str(_COCO_CLASSES),
        "--conf",           str(conf),
        "--T-task-camera",  T_str,
        "--save-image",     str(_PREVIEW_IMAGE),
    ]
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
    if not auto and _PREVIEW_IMAGE.exists():
        print(f"\n  [preview] Opening annotated frame: {_PREVIEW_IMAGE}")
        subprocess.Popen(["open", str(_PREVIEW_IMAGE)])   # non-blocking

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

def run(auto: bool) -> int:
    T_ee_cam = _load_T_ee_camera(_CALIB_JSON)
    print(f"[calib] t_ee_camera = {np.round(T_ee_cam[:3,3], 4)}")

    with default_session(left=True, right=False) as session:
        arm = session.arms[ARM]

        # ── Phase 1: observation pose ──────────────────────────────────────
        print("\n" + "─" * 62)
        print("[1/5]  Moving to observation pose (candidate3) ...")
        arm.control.moveJ(OBS_Q_RAD, speed=0.3, acceleration=0.2)
        time.sleep(0.3)
        print("  Reached observation pose.")
        print("  Hook tip → +X,  camera faces table/bowl scene.")
        _confirm("Arm at obs pose. Run perception and open detection image?", auto)

        # ── Phase 2: T_task_camera ─────────────────────────────────────────
        print("\n" + "─" * 62)
        print("[2/5]  Reading TCP → T_task_camera ...")
        T_task_cam = _compute_T_task_camera(arm, T_ee_cam)
        print(f"  Camera origin (task frame): {np.round(T_task_cam[:3,3], 4)} m")

        # ── Phase 3: perception ────────────────────────────────────────────
        print("\n" + "─" * 62)
        print("[3/5]  Running C++ bowl detection ...")
        result = _run_perception(T_task_cam, auto=auto)

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
        _confirm("Detection look correct? Build grasp?", auto)

        # ── Phase 4: grasp plan ────────────────────────────────────────────
        print("\n" + "─" * 62)
        print("[4/5]  Building grasp ...")
        bowl_base_pose = Pose(translation=np.array(bowl_base_task_m, dtype=float))
        grasp = bowl_hook_grasp(bowl_base_pose,
                                angle_rad=GRASP_ANGLE_RAD,
                                approach_tilt_rad=APPROACH_TILT_RAD)
        place_pose = bowl_hook_grasp(BOWL_PLACE_POSE_TASK,
                                     angle_rad=PLACE_ANGLE_RAD,
                                     approach_tilt_rad=APPROACH_TILT_RAD).grasp_pose
        _print_plan(bowl_base_pose, grasp, place_pose)
        _confirm("Grasp plan looks correct? Execute PICK?", auto)

        # ── Phase 5a: pick ─────────────────────────────────────────────────
        print("\n" + "─" * 62)
        print(f"[5/5]  Picking ({grasp.description}) ...")
        pick_result = pick(arm, grasp, CONFIG)
        if not pick_result.success:
            print(f"  ✗ pick FAILED: {pick_result.reason}")
            return 1
        print("  ✓ pick succeeded.")

        # ── Phase 5b: place ────────────────────────────────────────────────
        _confirm(f"Pick done. Execute PLACE at "
                 f"{np.round(BOWL_PLACE_POSE_TASK.translation, 3)} m?", auto)

        print(f"  Placing at {BOWL_PLACE_POSE_TASK.translation} ...")
        place_result = place(arm, place_pose, CONFIG)
        if not place_result.success:
            print(f"  ✗ place FAILED: {place_result.reason}")
            return 1
        print("  ✓ place succeeded.")

    print("\n✓ Bowl pick-and-place complete.")
    return 0


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main() -> None:
    ap = argparse.ArgumentParser(
        description="Autonomous bowl pick-and-place — YOLOv11 + RealSense + UR5."
    )
    ap.add_argument("--auto", action="store_true",
                    help="Skip all interactive checkpoints (fully autonomous).")
    ap.add_argument("--dry", action="store_true",
                    help="Print plan with static bowl pose; no robot or camera.")
    ap.add_argument("--build", action="store_true",
                    help="cmake build the perception binary then exit.")
    args = ap.parse_args()

    if args.build:
        import subprocess as sp
        sp.run(["cmake", "-B", "build", "-DCMAKE_BUILD_TYPE=Release"],
               cwd=str(_HERE), check=True)
        sp.run(["cmake", "--build", "build"], cwd=str(_HERE), check=True)
        print("[build] done →", _PERCEPTION_BIN)
        return

    if args.dry:
        raise SystemExit(_dry_run())

    raise SystemExit(run(auto=args.auto))


if __name__ == "__main__":
    main()
