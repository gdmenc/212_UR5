"""hook_grasp_from_vision — perception at the *current* arm pose, then **pick only**.

This module is the **only** task entry point wired for the **bench/table-mounted**
hook arm (single UR IP, table-mount task frame, no lab ``X_LEFT_BASE_TASK``).
Other scripts keep the bimanual / Vention lab calibration paths unchanged.

Unlike ``pick_place_bowl_hook_full``, this does **not** move to a fixed observation
configuration: it assumes you jogged the arm to a reasonable view for testing.

Network / end-effector
----------------------
Single UR only: RTDE to ``--robot-ip`` (default ``169.254.9.43``). No ``left``/``right``
session flags. **Tool is always the hook gripper** — ``HookGripper`` + ``TCP_OFFSET_HOOK``.

**Task frame vs lab:** this rig uses a **table-mounted base** (not the Vention left-arm
geometry). We do **not** use ``calibration.X_LEFT_BASE_TASK``. Default is tabletop
coordinates with **task Z up**, origin at table height consistent with the base mount,
zero yaw, and optional ``--task-origin-offset`` / ``--task-yaw-deg`` when you measure
offsets on your bench. Internally the planner arm id stays ``ur_left`` only for Drake.

Pipeline
--------
1. Load ``T_ee_camera`` from **``--hand-eye-json``** (default:
   ``calibration/build/T_ee_camera.json``). **This file must be solved for this robot,
   this camera mount, and this task/base setup** — it is *not* tied to the automatic
   lab ``X_LEFT_BASE_TASK``, but if you never replaced it after moving off the Vention
   rig, you are still using whoever last wrote that JSON (often XY wrong, Z may look OK).
2. Average ``T_task_camera`` over many TCP samples (stable hand-eye at this pose).

This script **does not** load ``calibration/.../perception_offsets.json`` (lab XY tweaks)
or ``calibration.X_LEFT_BASE_TASK`` — task pose uses ``--task-origin-offset`` /
``--task-yaw-deg`` only.

Systematic many-cm error (e.g. ~5 cm, same axis on two rigs) — **not** “normal” RGB–depth
------------------------------------------------------------------
A few pixels of depth–color residual at 0.5 m is usually **millimetres** in 3D, not 5 cm.
Repeated **−Y** (or any fixed task axis) across environments almost always means **frames or
TCP**, not the RealSense align step.

Transform chain (conceptual)::

  p_task = X_base_task⁻¹ · X_base_tcp · T_tcp_cam · p_cam

where ``X_base_tcp`` is what RTDE reports for the **current TCP** (after ``setTcp``),
``T_tcp_cam`` is ``T_ee_camera`` from JSON (camera expressed in **that same TCP frame**),
and ``X_base_task`` is your table-mount task (``--task-origin-offset`` / ``--task-yaw-deg``).
C++ does ``p_task = T_task_cam · p_cam`` with ``T_task_cam = (X_task_tcp) · T_tcp_cam`` and
``X_task_tcp = X_base_task⁻¹ · X_base_tcp``.

**Highest-signal causes (audit in this order):**

1. **TCP used during hand-eye ≠ TCP at runtime.** ``calibration/calibrate_camera.cpp`` collects
   poses via **manual** RTDE pose entry; the JSON is explicitly “camera in **TCP** frame”.
   If the pendant / script used a different ``setTcp`` than ``TCP_OFFSET_HOOK`` (or zero TCP)
   when those numbers were typed, the solved transform bakes in **decimetre-scale** lateral
   error that **repeats on every rig** you run with the same mistake. Fix: recalibrate with
   TCP locked to production values, or verify pose source matches ``Session`` setup.

2. **Task origin / yaw never measured.** Defaults ``(0,0,0)`` and ``0°`` are almost never the
   physical table vs UR base. A pure **5 cm** offset in **task Y** is exactly an unmeasured
   ``--task-origin-offset`` second component or a few degrees of ``--task-yaw-deg`` at arm reach.

3. **Wrong or shared ``T_ee_camera.json``.** Same file copied lab→bench without a new solve, or
   wrong arm / wrong tag run — shows up as **stable** XY bias; Z can still look “fine” depending
   on view.

4. **Hand-eye math / pose entry typos.** Manual six-number entry per sample; one bad sample or
   confused base frame pollutes the whole solve.

5. **RGB–depth / pixel tweaks** — use only after 1–4 are ruled out; they correct **small**
   on-image nudges, not 5 cm in task space unless something else is wrong.

**Sanity check:** jog TCP to a **physical** point you can measure (table corner, tag center),
read task-frame position from the same ``X_base_task`` + TCP as perception; if that is already
~5 cm off, fix task frame / TCP before tuning vision offsets.

3. Run this package's C++ binary ``perception_hook_testing`` (YOLO + SAM + extra
   depth metrics in JSON — not ``pick_place_bowl_hook_full``/``perception_bowl``).
   With SAM, it fits **circle** and **ellipse** in task XY on the mask contour (oblique views
   → oval); ellipse eccentricity flags ``planar_oblique_view_hint``. Circle center is used when
   it matches nominal rim R; otherwise ellipse center may be used when eccentricity is high.
4. Build ``bowl_hook_grasp`` from detected ``bowl_base_task_m``. **Rim azimuth** defaults to
   the point on the rim **toward the current TCP** in task XY (“in front” / shortest planar
   direction from bowl center to hook). Use ``--grasp-flip-180`` if that direction drives
   into a wall; ``--grasp-angle-deg`` overrides manually.
5. Optionally execute **pick** only (no place).

RGB–depth: the C++ pipeline uses ``align_to_color`` + color intrinsics; a few pixels of
residual misalignment still happen on some D4xx units — grasp dots can sit offset on the JPEG
vs the bowl. Tune with Intel RealSense Viewer (depth on RGB), then try ``--depth-color-offset-pix``.

Ray vs rim: perception also reports where the centroid viewing ray meets task plane ``z=0``
(same depth patch as the initial bowl sample). Depth fixes range along the ray; that intersection
fixes one table XY on that ray. Nominal rim radius instead constrains the full rim (circle/ellipse
fit). Compare ``bowl_center_chosen_minus_ray_xy_m`` for the residual.

**Bowl pose:** perception estimates ``bowl_base_task_m`` with **Identity rotation** — the bowl
frame is always aligned to task axes (physically symmetric bowl has no unique yaw from RGB-D).
**XY center** from one depth sample biases outward along the viewing ray when the camera is
oblique; when SAM2 runs, the C++ binary may refine XY using **many mask pixels** (planar median +
PCA diagnostics). PCA eccentricity ≈ 0 means the mask projects as a circle in task XY (axis angle
not meaningful); large eccentricity suggests ellipse-like footprint (camera/view or calibration).

If the hook **overshoots** the rim (misses past the bowl in XY), first check **rim geometry**:
``bowl_hook_grasp`` places contact at ``rim_outer_radius`` from bowl center in the azimuth plane.
Defaults match ``grasps/bowl.py`` (small lab bowl). If your bench bowl differs, or perception C++
uses a different nominal diameter for overlays, set ``--bowl-rim-outer-radius-m`` or
``--rim-radius-scale`` (then ``--grasp-radial-inset-m``, ``--bowl-xy-bias-m``, ``--grasp-z-offset-m``).

Depth diagnostics (stdout JSON) include corner/table median depth vs shrunk-bbox
object median and a bbox depth-gradient cue (rim / discontinuity).

Motion
------
This task **never** uses Drake motion planning — free-space transits are sequential
``moveL``/``transit_xy`` only (temporary bench setup; see ``pick_place_bowl_hook``
but planner stays off here).

Running
-------
    sudo python -m control_scripts.tasks.hook_grasp_from_vision
    sudo python -m control_scripts.tasks.hook_grasp_from_vision --dry
    sudo python -m control_scripts.tasks.hook_grasp_from_vision --auto --no-execute

Build the dedicated binary with ``--build`` (CMake in this directory).

Exit / Ctrl+C
---------------
Handlers install at startup (live runs only): **SIGINT** / **SIGTERM** → **``os._exit``**
so you can always bail during hung IK/transit/RTDE waits (**Ctrl+C** otherwise may do
nothing inside blocking C calls until the handler runs). This **skips** ``Session``
teardown (``stopScript``). Perception failure / timeout also **``os._exit``**; bowl
not detected **``os._exit(1)``** so teardown cannot hang. Use the pendant if needed.
"""

from __future__ import annotations

import argparse
import json
import os
import signal
import subprocess
import sys
from dataclasses import replace
from pathlib import Path

import numpy as np

from ...calibration import TCP_OFFSET_HOOK
from ...grasps import Grasp
from ...grasps.bowl import BOWL_RIM_OUTER_RADIUS_M, BOWL_RIM_Z_OFFSET_M, bowl_hook_grasp
from ...grippers import HookGripper
from ...session import DEFAULT_CONNECT_TRIES, ArmSpec, Session
from ...util.poses import Pose
from ...util.rotations import Rotation
from ...util.rtde_convert import rtde_to_pose
from ..pick_place_bowl_hook_full import pick_place_bowl_hook as _bowl_exec
from ..pick_place_bowl_hook_full.__main__ import (
    APPROACH_TILT_RAD,
    GRASP_ANGLE_RAD,
    _COCO_CLASSES,
    _SAM2_DECODER_DEFAULT,
    _SAM2_ENCODER_DEFAULT,
    _YOLO_MODEL,
    _compute_T_task_camera_stable,
    _load_T_ee_camera,
    _sam_cpp_args,
)


_REPO_ROOT = Path(__file__).resolve().parents[3]
# Same default path as ``pick_place_bowl_hook_full`` — override with ``--hand-eye-json``.
_DEFAULT_HAND_EYE_JSON = _REPO_ROOT / "calibration" / "build" / "T_ee_camera.json"

# --- Defaults: heavier sampling than ``pick_place_bowl_hook_full`` -------------
POSE_SAMPLES_DEFAULT = 15
CXX_WARMUP_FRAMES = 15
CXX_DETECT_FRAMES = 10

_DEFAULT_ROBOT_IP = "169.254.9.43"

# Must match the hook arm name inside Drake WORLD / pick_place_bowl_hook motion code.
_SESSION_HOOK_ARM = "ur_left"

_HERE = Path(__file__).resolve().parent
_PERCEPTION_BIN = _HERE / "build" / "perception_hook_testing"
_PREVIEW_IMAGE = Path("/tmp/hook_grasp_from_vision_preview.jpg")

# ``subprocess.run`` must not block forever if librealsense / C++ hangs.
_PERCEPTION_SUBPROCESS_TIMEOUT_S = float(
    os.environ.get("HOOK_VISION_PERCEPTION_TIMEOUT", "300")
)


def _install_bench_interrupt_handlers() -> None:
    """Exit immediately on SIGINT/SIGTERM (no context-manager unwind — RTDE stop can block)."""

    def _stop(signum: int, _frame) -> None:
        code = 130 if signum == signal.SIGINT else 143
        sys.stderr.write(
            "\n[hook_grasp_from_vision] signal exit — RTDE/stopScript cleanup skipped.\n"
        )
        os._exit(code)

    signal.signal(signal.SIGINT, _stop)
    signal.signal(signal.SIGTERM, _stop)


def _shrink_grasp_xy_toward_bowl_center(grasp: Grasp, bowl_xyz_task: np.ndarray, inset_m: float) -> Grasp:
    """Pull grasp TCP XY toward bowl origin if perception overshoots past the rim."""
    if inset_m <= 0.0:
        return grasp
    g = grasp.grasp_pose.translation.copy()
    c = np.asarray(bowl_xyz_task, dtype=float).reshape(3)
    v_xy = g[:2] - c[:2]
    dist = float(np.linalg.norm(v_xy))
    if dist < 1e-9:
        return grasp
    step = min(float(inset_m), dist - 1e-6)
    u = v_xy / dist
    g[0] -= u[0] * step
    g[1] -= u[1] * step
    new_pose = Pose(translation=g, rotation=grasp.grasp_pose.rotation)
    desc = grasp.description or ""
    return replace(
        grasp,
        grasp_pose=new_pose,
        description=f"{desc} [bench XY inset {inset_m:.3f} m toward bowl]",
    )


def _offset_grasp_z(grasp: Grasp, dz_m: float) -> Grasp:
    if abs(dz_m) < 1e-12:
        return grasp
    t = grasp.grasp_pose.translation.copy()
    t[2] += float(dz_m)
    new_pose = Pose(translation=t, rotation=grasp.grasp_pose.rotation)
    return replace(
        grasp,
        grasp_pose=new_pose,
        description=(grasp.description or "") + f" [bench Δz {dz_m:+.4f} m]",
    )


def _rim_azimuth_rad_toward_tcp(
    bowl_xy_task: np.ndarray,
    tcp_xy_task: np.ndarray,
    *,
    flip_180: bool,
) -> float:
    """Rim contact angle (bowl frame) on the side closest to the hook (planar).

    ``bowl_hook_grasp`` uses ``angle_rad`` such that rim_xyz ∝ (cos θ, sin θ); θ =
    atan2 of vector **bowl → TCP** in task XY puts contact on the near side."""
    b = np.asarray(bowl_xy_task, dtype=float).reshape(2)
    t = np.asarray(tcp_xy_task, dtype=float).reshape(2)
    v = t - b
    ang = float(np.arctan2(v[1], v[0]))
    if flip_180:
        ang += np.pi
    return float(np.arctan2(np.sin(ang), np.cos(ang)))


def _x_base_task_table_mount(
    dx_t: float,
    dy_t: float,
    dz_t: float,
    yaw_deg: float,
) -> Pose:
    """Pose of task origin in base frame — same construction as ``calibration._build_X_base_task``.

    Table-mounted UR: ``R_task_to_base`` is a yaw about **vertical** (task Z ∥ base Z,
    both up). ``(dx_t, dy_t, dz_t)`` is the vector from base origin to task origin
    expressed in **task** coordinates (measure on your table when defaults are wrong).
    """
    yaw = float(np.radians(yaw_deg))
    c, s = np.cos(yaw), np.sin(yaw)
    r_tb = np.array(
        [[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]],
        dtype=float,
    )
    v_task = np.array([dx_t, dy_t, dz_t], dtype=float)
    v_base = r_tb @ v_task
    return Pose(
        translation=v_base,
        rotation=Rotation.from_matrix(r_tb),
    )


def _single_arm_hook_session(
    robot_ip: str,
    *,
    X_base_task: Pose,
    connect_tries: int = DEFAULT_CONNECT_TRIES,
) -> Session:
    """One UR at ``robot_ip`` with hook gripper only (not Robotiq)."""
    return Session(
        {
            _SESSION_HOOK_ARM: ArmSpec(
                name=_SESSION_HOOK_ARM,
                ip=robot_ip,
                X_base_task=X_base_task,
                tcp_offset=TCP_OFFSET_HOOK,
                gripper_factory=lambda rtde_c, rtde_r, rtde_io=None: HookGripper(rtde_c),
                home_q_rad=None,
            )
        },
        connect_tries=connect_tries,
    )


def _check_hook_perception_paths() -> None:
    for p, label in [
        (_PERCEPTION_BIN, "perception_hook_testing — run with --build in this package"),
        (_YOLO_MODEL, "YOLOv11 ONNX (212_Perception/build/yolo11n.onnx)"),
        (_COCO_CLASSES, "COCO classes (212_Perception/build/coco-classes.txt)"),
    ]:
        if not p.exists():
            raise FileNotFoundError(f"{label}\n  Not found: {p}")


def _run_hook_perception(
    T_task_cam: np.ndarray,
    *,
    conf: float = 0.40,
    auto: bool = False,
    serial: str = "",
    warmup_frames: int = CXX_WARMUP_FRAMES,
    detect_frames: int = CXX_DETECT_FRAMES,
    no_sam: bool = False,
    sam_encoder: Path = _SAM2_ENCODER_DEFAULT,
    sam_decoder: Path = _SAM2_DECODER_DEFAULT,
    preview_image: Path | None = None,
    grasp_angle_deg_preview: float | None = None,
    nominal_rim_radius_m: float | None = None,
    depth_color_offset_pix: tuple[int, int] = (0, 0),
) -> dict:
    """Run ``perception_hook_testing`` (same CLI as ``perception_bowl``, extra depth JSON).

    ``grasp_angle_deg_preview`` overrides annotated-preview grasp angle in C++ (defaults to
    lab ``GRASP_ANGLE_RAD``). Actual executed grasp is chosen later in ``run()``."""
    _check_hook_perception_paths()

    save_preview = preview_image if preview_image is not None else _PREVIEW_IMAGE

    g_preview = (
        float(grasp_angle_deg_preview)
        if grasp_angle_deg_preview is not None
        else float(np.degrees(GRASP_ANGLE_RAD))
    )

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
        str(save_preview),
        "--grasp-angle-deg",
        f"{g_preview:.3f}",
        "--approach-tilt-deg",
        f"{np.degrees(APPROACH_TILT_RAD):.3f}",
    ]
    if serial:
        cmd.extend(["--serial", serial])
    cmd.extend(_sam_cpp_args(no_sam, sam_encoder, sam_decoder))
    if no_sam:
        cmd.append("--no-sam")
    cmd.extend(["--warmup", str(max(0, warmup_frames))])
    cmd.extend(["--frames", str(max(1, detect_frames))])
    if nominal_rim_radius_m is not None:
        cmd.extend(["--nominal-rim-radius-m", f"{float(nominal_rim_radius_m):.8f}"])
    du, dv = depth_color_offset_pix
    if du != 0 or dv != 0:
        cmd.extend(["--depth-color-offset-pix", str(int(du)), str(int(dv))])
    if auto:
        cmd.append("--headless")

    print(f"[perception] {_PERCEPTION_BIN.name}" + (" (headless)" if auto else " — saving preview"))

    try:
        result = subprocess.run(
            cmd,
            capture_output=True,
            text=True,
            timeout=_PERCEPTION_SUBPROCESS_TIMEOUT_S,
        )
    except subprocess.TimeoutExpired as exc:
        print(
            f"\n[FAIL] perception_hook_testing timed out after "
            f"{_PERCEPTION_SUBPROCESS_TIMEOUT_S}s (set HOOK_VISION_PERCEPTION_TIMEOUT to adjust).",
            file=sys.stderr,
        )
        if exc.stderr:
            print(exc.stderr[-2000:], file=sys.stderr)
        # os._exit: do not unwind Session — stopScript can hang indefinitely.
        os._exit(124)

    if result.stderr:
        for line in result.stderr.strip().splitlines():
            print(f"  [C++] {line}")

    if result.returncode != 0:
        print(
            f"\n[FAIL] perception_hook_testing failed (exit {result.returncode}). "
            "Fix camera/USB/sudo, then retry.",
            file=sys.stderr,
        )
        os._exit(result.returncode if result.returncode is not None else 1)

    if not auto and save_preview.exists():
        print(f"\n  [preview] Opening annotated frame: {save_preview}")
        subprocess.Popen(["open", str(save_preview)])

    try:
        return json.loads(result.stdout)
    except json.JSONDecodeError as e:
        print(
            f"[FAIL] perception returned invalid JSON: {e}\n"
            f"stdout (first 500 chars):\n{result.stdout[:500]}",
            file=sys.stderr,
        )
        os._exit(1)


def _confirm(prompt: str, auto: bool) -> None:
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


def _cmake_build(onnxruntime_root: Path | None) -> None:
    cmake_cfg = ["cmake", "-B", "build", "-DCMAKE_BUILD_TYPE=Release"]
    if onnxruntime_root is not None:
        cmake_cfg.append(f"-DONNXRUNTIME_ROOT={onnxruntime_root}")
    subprocess.run(cmake_cfg, cwd=str(_HERE), check=True)
    subprocess.run(["cmake", "--build", "build"], cwd=str(_HERE), check=True)
    print("[build] done →", _PERCEPTION_BIN)


def run(
    *,
    auto: bool,
    dry: bool,
    execute: bool,
    robot_ip: str,
    task_origin_offset_m: tuple[float, float, float],
    task_yaw_deg: float,
    pose_samples: int,
    warmup_frames: int,
    detect_frames: int,
    anchor_table_z: bool,
    table_z_m: float,
    no_sam: bool,
    sam_encoder: Path,
    sam_decoder: Path,
    grasp_angle_deg: float | None,
    grasp_flip_180: bool,
    bowl_xy_bias_m: tuple[float, float],
    grasp_radial_inset_m: float,
    grasp_z_offset_m: float,
    bowl_rim_outer_radius_m: float | None,
    bowl_rim_z_offset_m: float | None,
    rim_radius_scale: float,
    hand_eye_json: Path,
    depth_color_offset_pix: tuple[int, int],
) -> int:
    if dry:
        print("[dry] No robot / camera — printing grasp constants only.")
        print(f"  GRASP_ANGLE_RAD = {GRASP_ANGLE_RAD:.4f}  APPROACH_TILT_RAD = {APPROACH_TILT_RAD:.4f}")
        print(f"  --hand-eye-json would load: {hand_eye_json.expanduser().resolve()}")
        print(
            "  [calib scope] Task frame: table-mount flags only (no X_LEFT_BASE_TASK). "
            "No perception_offsets.json. Hand-eye file above maps eye-in-hand camera → TCP."
        )
        return 0

    _install_bench_interrupt_handlers()

    _bowl_exec.USE_MOTION_PLANNING = False
    print("[hook_grasp_from_vision] Motion planning OFF — transits use moveL only (no Drake).")

    ox, oy, oz = task_origin_offset_m
    X_base_task = _x_base_task_table_mount(ox, oy, oz, task_yaw_deg)
    print(
        "[task frame] Table-mount model (not lab X_LEFT): "
        f"offset(task) m = ({ox}, {oy}, {oz}), yaw = {task_yaw_deg}° — "
        f"task origin in base [m] = {np.round(X_base_task.translation, 4)}"
    )

    print(
        f"[config] robot_ip={robot_ip}  pose_samples={pose_samples}  "
        f"warmup={warmup_frames}  detect_frames={detect_frames}  "
        f"anchor_table_z={anchor_table_z}"
    )
    he_path = hand_eye_json.expanduser().resolve()
    print(
        "[calib scope] hand-eye (camera↔gripper): "
        f"{he_path} — must match this bench; a stale/lab file often skews **XY** only.\n"
        "             Not loaded: perception_offsets.json | X_LEFT_BASE_TASK "
        "(task pose from --task-origin-offset / --task-yaw-deg only)."
    )

    with _single_arm_hook_session(robot_ip, X_base_task=X_base_task) as session:
        arm = session.arms[_SESSION_HOOK_ARM]
        q_live = np.asarray(arm.receive.getActualQ(), dtype=float)
        tcp_rtde = arm.receive.getActualTCPPose()
        tcp_task = arm.to_task(rtde_to_pose(tcp_rtde))
        print(f"\n[robot] Connected RTDE @ {robot_ip}  (hook gripper — HookGripper / TCP_OFFSET_HOOK)")
        print(f"[robot] ActualQ [rad] = {np.round(q_live, 4)}")
        print(f"[robot] TCP pose (base frame, rtde vec6) = {np.round(np.asarray(tcp_rtde), 4)}")
        print(f"[robot] TCP position (task frame) [m] = {np.round(tcp_task.translation, 4)}")

        T_ee_cam = _load_T_ee_camera(he_path)
        print(f"[calib] hand-eye t_ee_camera [m] = {np.round(T_ee_cam[:3, 3], 4)}")

        print(f"\n[1/3] Averaging T_task_camera ({pose_samples} samples) at current pose …")
        T_task_cam = _compute_T_task_camera_stable(
            arm, T_ee_cam, samples=max(1, pose_samples)
        )
        print(f"  camera origin (task) = {np.round(T_task_cam[:3, 3], 4)} m")

        nominal_r_for_geom = (
            float(bowl_rim_outer_radius_m)
            if bowl_rim_outer_radius_m is not None
            else float(BOWL_RIM_OUTER_RADIUS_M * rim_radius_scale)
        )

        print("\n[2/3] Running C++ perception …")
        result = _run_hook_perception(
            T_task_cam,
            auto=auto,
            warmup_frames=max(0, warmup_frames),
            detect_frames=max(1, detect_frames),
            no_sam=no_sam,
            sam_encoder=sam_encoder,
            sam_decoder=sam_decoder,
            preview_image=_PREVIEW_IMAGE,
            nominal_rim_radius_m=nominal_r_for_geom,
            depth_color_offset_pix=depth_color_offset_pix,
        )

        if not result.get("detected", False):
            print("\n[FAIL] Bowl not detected.", file=sys.stderr)
            os._exit(1)

        conf = float(result.get("confidence", 0.0))
        bowl_base_task_m = result["bowl_base_task_m"]
        print(f"\n  conf             = {conf:.3f}")
        print(f"  bowl_base_task_m = {np.round(bowl_base_task_m, 4)}")
        cm = result.get("bowl_center_method")
        if cm:
            print(f"  bowl_center      = {cm}")
        if result.get("bowl_center_ray_table_xy_m") is not None:
            ray_xy = np.array(result["bowl_center_ray_table_xy_m"], dtype=float)
            print(
                f"  ray∩table XY       = [{ray_xy[0]*1000:.2f}, {ray_xy[1]*1000:.2f}] mm "
                "(depth→distance along centroid ray; ∩ z=0 fixes XY **on that ray**)"
            )
            if result.get("bowl_center_chosen_minus_ray_xy_m") is not None:
                dcr = np.array(result["bowl_center_chosen_minus_ray_xy_m"], dtype=float)
                print(
                    f"  chosen − ray XY    = [{dcr[0]*1000:.2f}, {dcr[1]*1000:.2f}] mm "
                    "(rim/circle/ellipse vs ray — lateral bias / hand-eye)"
                )
        if result.get("bowl_base_before_mask_refine_m") is not None:
            raw = np.array(result["bowl_base_before_mask_refine_m"], dtype=float)
            ref = np.array(bowl_base_task_m, dtype=float)
            dxy = ref[:2] - raw[:2]
            print(f"  XY refine Δ      = [{dxy[0]*1000:.1f}, {dxy[1]*1000:.1f}] mm (mask median − single patch)")
            print(f"  mask samples n   = {result.get('mask_planar_samples_n', '?')}")
        if result.get("mask_planar_eccentricity") is not None:
            ecc = float(result["mask_planar_eccentricity"])
            maj_deg = float(np.degrees(float(result.get("mask_planar_pca_major_rad_task", 0.0))))
            print(
                f"  mask planar PCA  = ecc≈{ecc:.3f}  major_axis_task≈{maj_deg:+.1f}° "
                "(diag: round footprint→ecc≈0; axis angle arbitrary)"
            )
        if result.get("bowl_circle_fit_radius_m") is not None:
            r_fit = float(result["bowl_circle_fit_radius_m"])
            rmse = float(result.get("bowl_circle_fit_rmse_m", 0.0))
            dr = float(result.get("bowl_circle_fit_radius_minus_nominal_m", 0.0))
            print(
                f"  contour circle fit  r={r_fit * 1000:.2f} mm  RMSE={rmse * 1000:.2f} mm  "
                f"(r−nominal)={dr * 1000:+.2f} mm [nominal={nominal_r_for_geom * 1000:.2f} mm]"
            )
        if result.get("bowl_ellipse_semi_major_m") is not None:
            smaj = float(result["bowl_ellipse_semi_major_m"])
            smin = float(result.get("bowl_ellipse_semi_minor_m", 0.0))
            ecc = float(result.get("bowl_ellipse_eccentricity", 0.0))
            ermse = float(result.get("bowl_ellipse_rmse_m", 0.0))
            ang = float(result.get("bowl_ellipse_angle_deg_opencv", 0.0))
            obl = result.get("planar_oblique_view_hint", False)
            print(
                f"  contour ellipse fit  semi-major={smaj * 1000:.2f} mm  semi-min={smin * 1000:.2f} mm  "
                f"ecc={ecc:.3f}  RMSE={ermse * 1000:.2f} mm  angle(OpenCV)={ang:.1f}°"
            )
            if obl:
                print(
                    "    → planar_oblique_view_hint: oval footprint (camera angle); "
                    "circle/nominal-R QA is weak — ellipse describes the rim projection better."
                )
        if "table_median_depth_m" in result:
            print(
                "  depth metrics (camera frame, m): "
                f"table_med={result.get('table_median_depth_m')}  "
                f"object_med={result.get('object_median_depth_m')}  "
                f"Δ(table−obj)={result.get('depth_delta_table_minus_object_m')}  "
                f"edge={result.get('depth_edge_metric_mean')}"
            )

        bowl_base = np.array(bowl_base_task_m, dtype=float)
        if anchor_table_z:
            bowl_base[2] = table_z_m
            print(f"  [anchor_table_z] forced bowl base z = {table_z_m:.4f} m")

        bx, by = bowl_xy_bias_m
        if abs(bx) > 1e-12 or abs(by) > 1e-12:
            bowl_base[0] += bx
            bowl_base[1] += by
            print(f"  [bench] bowl XY bias [m] = ({bx}, {by})")

        tcp_rtde_plan = arm.receive.getActualTCPPose()
        tcp_task_plan = arm.to_task(rtde_to_pose(tcp_rtde_plan))
        tcp_xy = tcp_task_plan.translation[:2]
        bowl_xy = bowl_base[:2]

        if grasp_angle_deg is not None:
            angle_rad = float(np.radians(grasp_angle_deg))
            grasp_src = f"override --grasp-angle-deg={grasp_angle_deg}"
        else:
            angle_rad = _rim_azimuth_rad_toward_tcp(bowl_xy, tcp_xy, flip_180=grasp_flip_180)
            grasp_src = (
                "rim toward TCP (task XY)"
                + (" + 180° (--grasp-flip-180)" if grasp_flip_180 else "")
            )

        bowl_pose = Pose(translation=bowl_base)
        rim_kw: dict[str, float] = {}
        if bowl_rim_outer_radius_m is not None:
            rim_kw["rim_outer_radius_m"] = float(bowl_rim_outer_radius_m)
        elif abs(rim_radius_scale - 1.0) > 1e-12:
            rim_kw["rim_outer_radius_m"] = float(BOWL_RIM_OUTER_RADIUS_M * rim_radius_scale)
        if bowl_rim_z_offset_m is not None:
            rim_kw["rim_z_offset_m"] = float(bowl_rim_z_offset_m)
        grasp = bowl_hook_grasp(
            bowl_pose,
            angle_rad=angle_rad,
            approach_tilt_rad=APPROACH_TILT_RAD,
            **rim_kw,
        )
        grasp = _shrink_grasp_xy_toward_bowl_center(
            grasp, bowl_base, grasp_radial_inset_m
        )
        grasp = _offset_grasp_z(grasp, grasp_z_offset_m)

        print("\n[plan]")
        print(f"  grasp azimuth    = {np.degrees(angle_rad):+.1f}° ({grasp_src})")
        eff_r = (
            float(bowl_rim_outer_radius_m)
            if bowl_rim_outer_radius_m is not None
            else BOWL_RIM_OUTER_RADIUS_M * rim_radius_scale
        )
        eff_z = (
            float(bowl_rim_z_offset_m)
            if bowl_rim_z_offset_m is not None
            else BOWL_RIM_Z_OFFSET_M
        )
        print(
            f"  rim model        = R={eff_r * 1000:.1f} mm, z_rim={eff_z * 1000:.1f} mm "
            f"(defaults {BOWL_RIM_OUTER_RADIUS_M * 1000:.1f} / {BOWL_RIM_Z_OFFSET_M * 1000:.1f})"
        )
        if grasp_radial_inset_m > 0:
            print(f"  radial inset     = {grasp_radial_inset_m * 100:.1f} cm (toward bowl center in XY)")
        if abs(grasp_z_offset_m) > 1e-9:
            print(f"  grasp Δz         = {grasp_z_offset_m * 1000:.1f} mm (task frame)")
        print(f"  grasp (task)     = {np.round(grasp.grasp_pose.translation, 4)} m")
        print(f"  pregrasp offset  = {grasp.pregrasp_offset * 100:.1f} cm")
        print(
            "  [note] JPEG preview grasp overlay used lab default angle during perception; "
            "executed grasp matches the azimuth above."
        )

        if not execute:
            print("\n[no-execute] Grasp planned; not commanding motion.")
            return 0

        _confirm("Execute pick?", auto)

        print("\n[3/3] Executing pick …")
        ok = _bowl_exec.run_on_arm(
            session,
            arm,
            grasp,
            Pose(),
            pick_only=True,
        )
        return 0 if ok else 1


def main() -> None:
    ap = argparse.ArgumentParser(
        description=(
            "Bench hook arm only: table-mount task frame, single UR IP, perceive → pick "
            "(no place)."
        )
    )
    ap.add_argument("--auto", action="store_true", help="Skip confirmation prompts.")
    ap.add_argument("--dry", action="store_true", help="Print constants; no hardware.")
    ap.add_argument(
        "--robot-ip",
        type=str,
        default=_DEFAULT_ROBOT_IP,
        metavar="ADDR",
        help=f"UR controller IP for RTDE (default {_DEFAULT_ROBOT_IP}).",
    )
    ap.add_argument(
        "--hand-eye-json",
        type=Path,
        default=_DEFAULT_HAND_EYE_JSON,
        metavar="PATH",
        help=(
            "Eye-in-hand calibration JSON (R_ee_camera, t_ee_camera_m). Default: "
            "calibration/build/T_ee_camera.json — replace with a solve from **this** rig."
        ),
    )
    ap.add_argument(
        "--task-origin-offset",
        type=float,
        nargs=3,
        default=[0.0, 0.0, 0.0],
        metavar=("DX", "DY", "DZ"),
        help=(
            "Vector from UR base origin to task origin [m], in task axes "
            "(table Z up; default 0 0 0 = origins coincident when yaw=0). "
            "Same convention as calibration._build_X_base_task."
        ),
    )
    ap.add_argument(
        "--task-yaw-deg",
        type=float,
        default=0.0,
        help="Yaw [deg] about vertical between task frame and base frame (table-mount default 0).",
    )
    ap.add_argument(
        "--no-execute",
        action="store_true",
        help="Run perception and print plan only (no pick).",
    )
    ap.add_argument(
        "--pose-samples",
        type=int,
        default=POSE_SAMPLES_DEFAULT,
        help=f"TCP samples for T_task_cam median (default {POSE_SAMPLES_DEFAULT}).",
    )
    ap.add_argument(
        "--warmup-frames",
        type=int,
        default=CXX_WARMUP_FRAMES,
        help=f"C++ RealSense warmup frames (default {CXX_WARMUP_FRAMES}).",
    )
    ap.add_argument(
        "--detect-frames",
        type=int,
        default=CXX_DETECT_FRAMES,
        help=f"C++ detection aggregation frames (default {CXX_DETECT_FRAMES}).",
    )
    ap.add_argument(
        "--anchor-table-z",
        action="store_true",
        help="Override detected bowl Z with --table-z (task frame).",
    )
    ap.add_argument(
        "--table-z",
        type=float,
        default=0.0,
        metavar="M",
        help="Task-frame Z for bowl base when --anchor-table-z (default 0).",
    )
    ap.add_argument(
        "--depth-color-offset-pix",
        type=int,
        nargs=2,
        default=[0, 0],
        metavar=("DU", "DV"),
        help=(
            "Shift (u,v) for aligned-depth reads vs RGB (pixels). If grasp overlay sits right of "
            "the bowl on the JPEG, try negative DU (e.g. -2 -4). Tune using RealSense Viewer depth-on-RGB."
        ),
    )
    ap.add_argument(
        "--grasp-angle-deg",
        type=float,
        default=None,
        metavar="DEG",
        help=(
            "Fixed rim azimuth [deg] in bowl frame (overrides toward-TCP default). "
            "If omitted, uses rim point toward current TCP in task XY."
        ),
    )
    ap.add_argument(
        "--grasp-flip-180",
        action="store_true",
        help="Add 180° to toward-TCP azimuth (e.g. default heads into a wall). Ignored if --grasp-angle-deg set.",
    )
    ap.add_argument(
        "--bowl-xy-bias-m",
        type=float,
        nargs=2,
        default=[0.0, 0.0],
        metavar=("BX", "BY"),
        help="Add to perceived bowl base XY [m] before grasp (systematic perception/task-frame bias).",
    )
    ap.add_argument(
        "--grasp-radial-inset-m",
        type=float,
        default=0.0,
        metavar="M",
        help=(
            "Pull grasp contact toward bowl center in XY [m] after planning — use when the hook "
            "overshoots past the rim / misses inward."
        ),
    )
    ap.add_argument(
        "--grasp-z-offset-m",
        type=float,
        default=0.0,
        metavar="M",
        help="Add to grasp pose Z [m] in task frame after planning (e.g. touch table early).",
    )
    ap.add_argument(
        "--bowl-rim-outer-radius-m",
        type=float,
        default=None,
        metavar="M",
        help=(
            "Override outer rim radius [m] for hook contact (default: grasps.bowl). "
            "Takes precedence over --rim-radius-scale. Smaller R pulls the grasp toward center."
        ),
    )
    ap.add_argument(
        "--bowl-rim-z-offset-m",
        type=float,
        default=None,
        metavar="M",
        help="Override rim height above bowl base [m] (default: grasps.bowl BOWL_RIM_Z_OFFSET_M).",
    )
    ap.add_argument(
        "--rim-radius-scale",
        type=float,
        default=1.0,
        metavar="S",
        help=(
            "Multiply default lab rim radius by S when --bowl-rim-outer-radius-m is unset "
            f"(default lab R={BOWL_RIM_OUTER_RADIUS_M * 1000:.1f} mm)."
        ),
    )
    ap.add_argument("--no-sam", action="store_true")
    ap.add_argument("--sam-encoder", type=Path, default=_SAM2_ENCODER_DEFAULT)
    ap.add_argument("--sam-decoder", type=Path, default=_SAM2_DECODER_DEFAULT)
    ap.add_argument(
        "--build",
        action="store_true",
        help="cmake build this package → build/perception_hook_testing then exit.",
    )
    ap.add_argument(
        "--onnxruntime-root",
        type=Path,
        default=None,
        help="With --build: pass -DONNXRUNTIME_ROOT to CMake.",
    )
    args = ap.parse_args()

    if args.build:
        _cmake_build(args.onnxruntime_root)
        return

    if not args.dry:
        _install_bench_interrupt_handlers()

    execute = not args.no_execute
    try:
        rc = run(
            auto=args.auto,
            dry=args.dry,
            execute=execute,
            robot_ip=args.robot_ip.strip(),
            task_origin_offset_m=tuple(args.task_origin_offset),
            task_yaw_deg=float(args.task_yaw_deg),
            pose_samples=max(1, args.pose_samples),
            warmup_frames=max(0, args.warmup_frames),
            detect_frames=max(1, args.detect_frames),
            anchor_table_z=args.anchor_table_z,
            table_z_m=args.table_z,
            no_sam=args.no_sam,
            sam_encoder=args.sam_encoder,
            sam_decoder=args.sam_decoder,
            grasp_angle_deg=args.grasp_angle_deg,
            grasp_flip_180=args.grasp_flip_180,
            bowl_xy_bias_m=tuple(args.bowl_xy_bias_m),
            grasp_radial_inset_m=float(args.grasp_radial_inset_m),
            grasp_z_offset_m=float(args.grasp_z_offset_m),
            bowl_rim_outer_radius_m=args.bowl_rim_outer_radius_m,
            bowl_rim_z_offset_m=args.bowl_rim_z_offset_m,
            rim_radius_scale=float(args.rim_radius_scale),
            hand_eye_json=args.hand_eye_json,
            depth_color_offset_pix=(int(args.depth_color_offset_pix[0]), int(args.depth_color_offset_pix[1])),
        )
    except FileNotFoundError as exc:
        print(exc, file=sys.stderr)
        rc = 1
    raise SystemExit(rc)


if __name__ == "__main__":
    main()
