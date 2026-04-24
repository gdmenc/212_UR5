"""Open the microwave door with the hook gripper (left arm).

Four-phase strategy:

    1. Approach   (moveL)      Hook tip to pre-engagement pose below handle.
    2. Engage     (moveL + gripper)
                               Short lift to slide hook under the handle,
                               then extend hook (close).
    3. Pull open  (forceMode)  Commanded pull force along the door's tangent
                               direction. Arm compliantly follows the door
                               arc. Exit when TCP has moved past a position
                               threshold OR time cap.
    4. Release    (moveL + gripper)
                               Retract hook, back out along the same axis.

forceMode is the right choice for phase 3 because the door's exact arc
(hinge pose, hinge stiffness, door spring) is uncertain — a pre-computed
arc of ``moveL`` waypoints would drag the hook off the handle if the
arc is off by >1-2 cm. Force mode lets the arm find the real arc.

Safety:
  - ``rtde_c.forceModeStop()`` MUST fire before returning, even on
    exception — dangling force mode leaves the next RTDE call in an
    unpredictable state. This module wraps phase 3 in try/finally.
  - Force magnitude kept low (~20 N) so a hook slip doesn't pull the
    arm hard. Tune based on the door's resistance.
  - Position threshold + time cap catch both "door reached stop" and
    "hook slipped / door got stuck."
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import List, Optional

import numpy as np
from scipy.spatial.transform import Rotation

from ..arm import ArmHandle
from ..config import DEFAULT, PickPlaceConfig
from ..moves import approach_to, lift_to_transit, retract_to, transit_xy
from ..reachability import is_task_pose_reachable
from ..util.poses import Pose
from ..util.rtde_convert import pose_to_rtde, rtde_to_pose


@dataclass
class MicrowaveDoorSpec:
    """Hand-measured door / handle geometry.

    All poses in task frame."""

    handle_engage_pose_task: Pose
    """Task-frame pose the TCP adopts when the hook is engaged under the
    door handle, before pulling. Hook pointing into the handle from
    below; tool orientation chosen so the hook latches when extended.
    TODO: measure at the lab."""

    pre_engage_offset: np.ndarray = field(
        default_factory=lambda: np.array([0.0, 0.0, -0.03])
    )
    """Task-frame translation from engage pose back to the pre-engage
    pose (where the hook tip is positioned BEFORE the final lift that
    engages the handle). Default: 3 cm below engage in task Z. Adjust
    if the engage motion is other than pure vertical."""

    pull_direction_task: np.ndarray = field(
        default_factory=lambda: np.array([0.0, -1.0, 0.0])
    )
    """Unit vector in task frame pointing along the INITIAL door-pull
    direction (tangent to the door's arc at the start of the swing).
    Default: -y (away from microwave, toward operator). TODO: measure."""

    pull_force_n: float = 20.0
    """Force magnitude (N) commanded during the pull. Tune so the door
    opens without slipping the hook, and stops within the time cap."""

    pull_distance_task: float = 0.25
    """Target TCP displacement (m) along ``pull_direction_task`` before
    we consider the door 'open'. Approximately the arc length from
    closed to fully open — adjust to your door."""

    pull_speed_limit: float = 0.05
    """Max TCP speed (m/s) during force mode. Low to avoid yanking."""

    pull_timeout_s: float = 5.0
    """Hard cap on phase 3. Stops the force mode even if the position
    threshold wasn't met (e.g., hook slipped, door stuck)."""


@dataclass
class OpenMicrowaveResult:
    success: bool
    reason: Optional[str] = None
    door_opened_distance: float = 0.0
    """How far (m) the TCP moved along the pull direction — useful to
    diagnose partial opens."""


# -----------------------------------------------------------------------
#  Force-mode helpers (extracted so phase 3 reads clearly)
# -----------------------------------------------------------------------

def _build_task_frame_for_pull(
    engage_pose_task: Pose,
    pull_direction_task: np.ndarray,
    arm: ArmHandle,
) -> List[float]:
    """Build the rtde_c.forceMode ``task_frame`` argument.

    The task frame is a 6-vector [x,y,z, rx,ry,rz] in base frame coords
    (axis-angle rotation). We want its +x axis aligned with the desired
    pull direction, origin at the current TCP. Position doesn't matter
    mechanically (force mode uses the frame's orientation) but we set
    it to the TCP position for clarity in logs.

    Constructs a rotation that maps +x_task_frame → pull_direction
    (expressed in base frame). Rotation around +x, y, z axes is
    arbitrary for the remaining two — we pick one that keeps the frame
    right-handed and its z roughly upward.
    """
    # Pull direction in base frame
    pull_dir_base = arm.X_base_task.rotation.apply(pull_direction_task)
    pull_dir_base = pull_dir_base / np.linalg.norm(pull_dir_base)

    # Pick task_frame z as "as close to base +z as possible, orthogonal
    # to pull". Then task_frame y = z × x.
    world_up_base = np.array([0.0, 0.0, 1.0])
    if abs(np.dot(pull_dir_base, world_up_base)) > 0.95:
        world_up_base = np.array([0.0, 1.0, 0.0])  # fallback
    z_axis = world_up_base - pull_dir_base * np.dot(pull_dir_base, world_up_base)
    z_axis = z_axis / np.linalg.norm(z_axis)
    y_axis = np.cross(z_axis, pull_dir_base)

    R = np.column_stack([pull_dir_base, y_axis, z_axis])
    rotvec = Rotation.from_matrix(R).as_rotvec()

    # Frame origin: current TCP position in base frame.
    tcp_base = rtde_to_pose(arm.receive.getActualTCPPose())

    return [
        tcp_base.translation[0],
        tcp_base.translation[1],
        tcp_base.translation[2],
        float(rotvec[0]),
        float(rotvec[1]),
        float(rotvec[2]),
    ]


# -----------------------------------------------------------------------
#  Primitive
# -----------------------------------------------------------------------

def open_microwave_door(
    arm: ArmHandle,
    door: MicrowaveDoorSpec,
    config: PickPlaceConfig = DEFAULT,
) -> OpenMicrowaveResult:
    """Open the microwave door end-to-end. See module docstring for the
    four-phase breakdown.

    Assumes ``arm`` has a ``HookGripper`` attached (this primitive does
    not check gripper type — passing a Robotiq 2F-85 here will silently
    close fingers instead of extending a hook, which probably won't
    engage the handle).
    """
    if config.transit_z is None:
        raise ValueError("config.transit_z is unset.")
    if arm.gripper is None:
        raise ValueError(f"arm {arm.name!r} has no gripper attached.")

    engage_pose = door.handle_engage_pose_task
    pre_engage_pose = Pose(
        translation=engage_pose.translation + door.pre_engage_offset,
        rotation=engage_pose.rotation,
    )

    # Pre-flight reachability.
    for label, pose in [("pre_engage", pre_engage_pose),
                        ("engage", engage_pose)]:
        if is_task_pose_reachable(arm, pose) is None:
            return OpenMicrowaveResult(
                success=False,
                reason=f"waypoint '{label}' is kinematically infeasible.",
            )

    # --- Phase 1: approach ---
    lift_to_transit(arm, config.transit_z, config.transit_speed, config.transit_accel)
    transit_xy(arm, pre_engage_pose, config.transit_z,
               config.transit_speed, config.transit_accel)
    approach_to(arm, pre_engage_pose, config.approach_speed, config.approach_accel)

    # --- Phase 2: engage (slide hook under handle, then latch) ---
    approach_to(arm, engage_pose, config.approach_speed, config.approach_accel)
    # Extend hook. On HookGripper, close() = extend. ArmHandle's gripper
    # could be any Gripper; rely on the ABC here.
    arm.gripper.close()

    # --- Phase 3: pull open (force mode) ---
    start_tcp_base = rtde_to_pose(arm.receive.getActualTCPPose())
    task_frame = _build_task_frame_for_pull(engage_pose, door.pull_direction_task, arm)

    selection_vector = [1, 0, 0, 0, 0, 0]        # compliant in +task_x only
    wrench           = [door.pull_force_n, 0.0, 0.0, 0.0, 0.0, 0.0]
    limits           = [door.pull_speed_limit] * 3 + [0.5] * 3   # linear m/s, angular rad/s
    FORCE_MODE_TYPE  = 2   # 2 = task frame given in base coords, not tool

    distance_moved = 0.0
    try:
        arm.control.forceMode(task_frame, selection_vector, wrench,
                              FORCE_MODE_TYPE, limits)
        t_start = time.time()
        while (time.time() - t_start) < door.pull_timeout_s:
            now_tcp = rtde_to_pose(arm.receive.getActualTCPPose())
            delta_base = now_tcp.translation - start_tcp_base.translation
            # Project onto pull direction (in base frame)
            pull_base = arm.X_base_task.rotation.apply(door.pull_direction_task)
            pull_base = pull_base / np.linalg.norm(pull_base)
            distance_moved = float(np.dot(delta_base, pull_base))
            if distance_moved >= door.pull_distance_task:
                break
            time.sleep(0.01)   # 100 Hz poll; don't busy-wait
    finally:
        arm.control.forceModeStop()

    # --- Phase 4: release and retract ---
    arm.gripper.open()   # retract hook
    retract_to(arm, pre_engage_pose, config.retract_speed, config.retract_accel)
    retract_to(
        arm,
        Pose(translation=np.array([pre_engage_pose.translation[0],
                                   pre_engage_pose.translation[1],
                                   config.transit_z]),
             rotation=pre_engage_pose.rotation),
        config.retract_speed, config.retract_accel,
    )

    opened_ok = distance_moved >= 0.8 * door.pull_distance_task
    return OpenMicrowaveResult(
        success=opened_ok,
        reason=None if opened_ok else "door did not reach target pull distance",
        door_opened_distance=distance_moved,
    )
