"""Perception-driven bottle rim pick, pour, and place with the left hook arm."""

from __future__ import annotations

import argparse
import json
import subprocess
import time
from pathlib import Path

import numpy as np

from ...arm import ArmHandle
from ...config import PickPlaceConfig
from ...grasps.bottle import bottle_hook_grasp, bottle_hook_pour_tcp_pose
from ...pick import pick
from ...place import place
from ...session import default_session
from ...util.poses import Pose
from ...util.rtde_convert import rtde_to_pose
from .pour_bottle_hook import (
    _pour,
    POUR_MAX_TILT_STEP_RAD,
    POUR_DURATION_S,
)


_HERE = Path(__file__).parent
_REPO_ROOT = Path(__file__).parents[3]
_PERCEPTION_DIR = Path(__file__).parents[4] / "212_Perception" / "build"
_PERCEPTION_BIN = _HERE / "build" / "perception_bottle_hook"
_CALIB_JSON = _REPO_ROOT / "calibration" / "build" / "T_ee_camera.json"
_YOLO_MODEL = _PERCEPTION_DIR / "yolo11n.onnx"
_COCO_CLASSES = _PERCEPTION_DIR / "coco-classes.txt"
_PREVIEW_IMAGE = Path("/tmp/bottle_hook_detection_preview.jpg")

OBS_Q_RAD = [
    0.6895843744277954,
    -0.8284762662700196,
    1.2472832838641565,
    -0.17453940332446294,
    -1.0418575445758265,
    -4.834977690373556,
]

ARM = "ur_left"
GRASP_ANGLE_RAD = float(np.pi)
POUR_TARGET_XY_TASK = np.array([0.0, 0.0])
POUR_RECEIVER_RIM_Z_TASK = 0.154
POUR_HEIGHT_ABOVE_RIM_M = 0.05
POUR_TILT_RAD = float(np.radians(140))
POUR_DURATION_S = 3.0
BOTTLE_PLACE_POSE_TASK = Pose(translation=[-0.1, 0.0, -0.01])

POUR_TARGET_TASK = np.array([
    POUR_TARGET_XY_TASK[0],
    POUR_TARGET_XY_TASK[1],
    POUR_RECEIVER_RIM_Z_TASK + POUR_HEIGHT_ABOVE_RIM_M,
])

CONFIG = PickPlaceConfig(
    transit_z=0.4,
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


def _confirm(prompt: str, auto: bool) -> None:
    if auto:
        return
    print()
    while True:
        ans = input(f"  >>> {prompt}  [Enter = yes / n = abort] ").strip().lower()
        if ans in ("", "y", "yes"):
            return
        if ans in ("n", "no", "q", "quit", "abort"):
            print("  Aborted.")
            raise SystemExit(0)


def _load_T_ee_camera(path: Path) -> np.ndarray:
    if not path.exists():
        raise FileNotFoundError(f"Calibration file not found: {path}")
    data = json.loads(path.read_text())
    M = np.eye(4, dtype=float)
    M[:3, :3] = np.array(data["R_ee_camera"], dtype=float)
    M[:3, 3] = np.array(data["t_ee_camera_m"], dtype=float)
    return M


def _compute_T_task_camera(arm: ArmHandle, T_ee_cam: np.ndarray) -> np.ndarray:
    tcp_rtde = arm.receive.getActualTCPPose()
    X_base_ee = rtde_to_pose(tcp_rtde)
    X_task_ee = arm.to_task(X_base_ee)
    T = np.eye(4, dtype=float)
    T[:3, :3] = X_task_ee.rotation.as_matrix()
    T[:3, 3] = X_task_ee.translation
    return T @ T_ee_cam


def _check_paths() -> None:
    for path, label in [
        (_PERCEPTION_BIN, "Perception binary - run with --build first"),
        (_YOLO_MODEL, "YOLO ONNX model"),
        (_COCO_CLASSES, "COCO classes file"),
    ]:
        if not path.exists():
            raise FileNotFoundError(f"{label}\n  Not found: {path}")


def _run_perception(T_task_cam: np.ndarray, conf: float = 0.40, auto: bool = False) -> dict:
    _check_paths()

    T_str = " ".join(f"{v:.10f}" for v in T_task_cam.flatten())
    cmd = [
        "sudo",
        "-n",
        str(_PERCEPTION_BIN),
        "--model",
        str(_YOLO_MODEL),
        "--classes",
        str(_COCO_CLASSES),
        "--conf",
        str(conf),
        "--T-task-camera",
        T_str,
        "--save-image",
        str(_PREVIEW_IMAGE),
    ]
    if auto:
        cmd.append("--headless")

    print(f"[perception] {_PERCEPTION_BIN.name}" + (" (headless)" if auto else " - saving preview image"))
    result = subprocess.run(cmd, capture_output=True, text=True)

    if result.stderr:
        for line in result.stderr.strip().splitlines():
            print(f"  [C++] {line}")

    if result.returncode != 0:
        raise RuntimeError(f"Perception binary exited {result.returncode}")

    if not auto and _PREVIEW_IMAGE.exists():
        print(f"\n  [preview] Annotated frame: {_PREVIEW_IMAGE}")
        subprocess.Popen(["open", str(_PREVIEW_IMAGE)])

    try:
        return json.loads(result.stdout)
    except json.JSONDecodeError as exc:
        raise RuntimeError(f"JSON parse error: {exc}\nstdout was:\n{result.stdout[:500]}") from exc


def _print_plan(
    bottle_base_pose: Pose,
    grasp,
    upright_at_pour: Pose,
    pour_pose: Pose,
    place_pose: Pose,
) -> None:
    print("=" * 62)
    print("  Arm                   :", ARM, "(hook gripper)")
    print("  Bottle base (task)    :", np.round(bottle_base_pose.translation, 4), "m")
    print("  Grasp angle           :", f"{np.degrees(GRASP_ANGLE_RAD):+.0f} deg")
    print("  Grasp pose (task)     :", np.round(grasp.grasp_pose.translation, 4), "m")
    print("  Pour target (opening) :", np.round(POUR_TARGET_TASK, 4), "m")
    print("  Receiver rim z        :", f"{POUR_RECEIVER_RIM_Z_TASK:.3f} m")
    print("  Pour height           :", f"{POUR_HEIGHT_ABOVE_RIM_M * 100:.1f} cm above rim")
    print("  Upright at pour       :", np.round(upright_at_pour.translation, 4), "m")
    print("  Final pour TCP        :", np.round(pour_pose.translation, 4), "m")
    print("  Pour tilt / hold      :", f"{np.degrees(POUR_TILT_RAD):.0f} deg / {POUR_DURATION_S:.1f} s")
    print("  Place TCP pose        :", np.round(place_pose.translation, 4), "m")
    print("  Pregrasp offset       :", f"{grasp.pregrasp_offset * 100:.1f} cm")
    print("  Transit Z             :", CONFIG.transit_z, "m")
    print("=" * 62)


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


def _dry_run() -> int:
    bottle_base_pose = Pose(translation=[-0.1, 0.0, -0.01])
    grasp, upright_at_pour, pour_pose, place_pose = _build_plan(bottle_base_pose)
    print("[dry run] Static bottle pose - no camera / RTDE.")
    _print_plan(bottle_base_pose, grasp, upright_at_pour, pour_pose, place_pose)
    print("[dry run] No motion commanded.")
    return 0


def run(auto: bool) -> int:
    T_ee_cam = _load_T_ee_camera(_CALIB_JSON)
    print(f"[calib] t_ee_camera = {np.round(T_ee_cam[:3, 3], 4)}")

    with default_session(left=True, right=False) as session:
        arm = session.arms[ARM]

        print("\n" + "-" * 62)
        print("[1/6] Moving left arm to observation pose ...")
        arm.control.moveJ(OBS_Q_RAD, speed=0.3, acceleration=0.2)
        time.sleep(0.3)
        print("  Reached observation pose.")
        _confirm("Run bottle perception?", auto)

        print("\n" + "-" * 62)
        print("[2/6] Reading TCP -> T_task_camera ...")
        T_task_cam = _compute_T_task_camera(arm, T_ee_cam)
        print(f"  Camera origin (task frame): {np.round(T_task_cam[:3, 3], 4)} m")

        print("\n" + "-" * 62)
        print("[3/6] Running C++ bottle detection ...")
        result = _run_perception(T_task_cam, auto=auto)
        if not result.get("detected", False):
            print("\n[FAIL] Bottle not detected.")
            return 1

        bottle_base_task_m = result["bottle_base_task_m"]
        bottle_neck_task_m = result.get("bottle_neck_task_m")
        print(f"\n  conf             = {result['confidence']:.2f}")
        print(f"  bottle_base_task = {np.round(bottle_base_task_m, 4)} m")
        if bottle_neck_task_m is not None:
            print(f"  bottle_neck_task = {np.round(bottle_neck_task_m, 4)} m")
        if not auto:
            print(f"\n  Preview image: {_PREVIEW_IMAGE}")
        _confirm("Detection look correct? Build grasp and pour plan?", auto)

        print("\n" + "-" * 62)
        print("[4/6] Building grasp + pour plan ...")
        bottle_base_pose = Pose(translation=np.array(bottle_base_task_m, dtype=float))
        grasp, upright_at_pour, pour_pose, place_pose = _build_plan(bottle_base_pose)
        _print_plan(bottle_base_pose, grasp, upright_at_pour, pour_pose, place_pose)
        _confirm("Plan looks correct? Execute PICK?", auto)

        print("\n" + "-" * 62)
        print(f"[5/6] Picking ({grasp.description}) ...")
        pick_result = pick(arm, grasp, CONFIG)
        if not pick_result.success:
            print(f"  pick FAILED: {pick_result.reason}")
            return 1
        print("  pick succeeded.")

        _confirm("Pick done. Execute POUR and PLACE?", auto)
        print("\n" + "-" * 62)
        print(f"[6/6] Pouring at {np.round(POUR_TARGET_TASK, 4)} then placing ...")
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

        place_result = place(arm, place_pose, CONFIG)
        if not place_result.success:
            print(f"  place FAILED: {place_result.reason}")
            return 1
        print("  place succeeded.")

    print("\nBottle hook pick-pour-place complete.")
    return 0


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Autonomous bottle hook pick/pour/place - YOLO + RealSense + UR5."
    )
    parser.add_argument("--auto", action="store_true", help="Skip interactive checkpoints.")
    parser.add_argument("--dry", action="store_true", help="Print plan with static bottle pose; no robot or camera.")
    parser.add_argument("--build", action="store_true", help="Build the perception_bottle_hook binary then exit.")
    args = parser.parse_args()

    if args.build:
        subprocess.run(["cmake", "-B", "build", "-DCMAKE_BUILD_TYPE=Release"], cwd=str(_HERE), check=True)
        subprocess.run(["cmake", "--build", "build"], cwd=str(_HERE), check=True)
        print("[build] done ->", _PERCEPTION_BIN)
        return

    if args.dry:
        raise SystemExit(_dry_run())

    raise SystemExit(run(auto=args.auto))


if __name__ == "__main__":
    main()
