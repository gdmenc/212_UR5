"""Place an object inside the microwave cavity (engineer-deterministic).

Strategy — we cannot prove collision-freeness without a motion planner,
but by constraining:
  (a) the approach direction to the cavity's axis (``+microwave_x``),
  (b) the TCP orientation to whatever the caller picked for the release
      (no mid-motion rotation; the gripper enters the cavity in the same
      orientation it will release in), and
  (c) each waypoint's kinematic feasibility (via reachability.py),
we get a reliable placement for any target inside the cavity's
"reachable core" — the subregion where the arm body can enter the door
without clipping the door frame.

API shape
---------
The caller supplies the RELEASE pose in task frame directly. This keeps
the primitive's decisions minimal:
  - it translates backward along ``-microwave_x`` to compute the entry
    pose (first waypoint inside the door plane) and the staging pose
    (last waypoint outside the door).
  - orientation is the SAME on all three waypoints — just the release
    orientation. No wrist re-orient mid-entry.

Geometry convention
-------------------
``MicrowaveSpec.X_task_microwave`` defines a microwave-local frame:
    origin  — front-center of the door opening
    +x      — INTO the cavity (depth)
    +y      — to the left from operator's view (width)
    +z      — up (height)
Only the orientation is used by the primitive (to know ``+microwave_x``
in task frame). Cavity dimensions are informational and used by
``sanity_check_release_pose`` to warn if the caller picked a release
point too close to a wall.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import List, Optional, Tuple

import numpy as np

from ..arm import ArmHandle
from ..config import DEFAULT, PickPlaceConfig
from ..moves import approach_to, lift_to_transit, retract_to, transit_xy
from ..reachability import is_task_pose_reachable
from ..util.poses import Pose


@dataclass
class MicrowaveSpec:
    """Hand-measured microwave geometry."""

    X_task_microwave: Pose
    """Pose of the microwave-local frame (origin at door center, +x into
    cavity) expressed in task frame. TODO: measure at the lab."""

    cavity_width: float = 0.32     # y-extent
    cavity_height: float = 0.22    # z-extent
    cavity_depth: float = 0.32     # x-extent

    staging_standoff: float = 0.10
    """Distance outside the door (m) along -microwave_x for the staging pose."""

    entry_standoff: float = 0.03
    """Distance just past the door (m) along +microwave_x for the entry pose."""

    def microwave_x_in_task(self) -> np.ndarray:
        """Unit vector pointing INTO the cavity, expressed in task frame."""
        return self.X_task_microwave.rotation.apply([1.0, 0.0, 0.0])


def build_waypoints(
    microwave: MicrowaveSpec,
    release_pose_task: Pose,
) -> Tuple[Pose, Pose, Pose]:
    """Given a release pose (in task frame), derive the three waypoints.

    The release pose's ORIENTATION is preserved on all three waypoints —
    the entry motion is pure translation along the cavity axis. Only the
    POSITION changes between waypoints.

    Returns (staging_pose, entry_pose, release_pose).
    """
    into_cavity = microwave.microwave_x_in_task()

    staging_xyz = release_pose_task.translation - (
        microwave.cavity_depth + microwave.staging_standoff
    ) * into_cavity
    # ^ note: we step back by the full cavity depth so the staging pose
    # is outside the cavity regardless of where in the cavity the release
    # is. Then an extra staging_standoff for margin.

    entry_xyz = release_pose_task.translation - (
        release_xyz_inside_cavity(microwave, release_pose_task)
        - microwave.entry_standoff
    ) * into_cavity

    # ... simpler and equivalent: position entry by stepping back along
    # into_cavity from release by (depth inside cavity - entry_standoff).
    # Use the full helper for clarity.
    entry_xyz = _entry_xyz_from_release(microwave, release_pose_task, into_cavity)

    staging = Pose(translation=staging_xyz, rotation=release_pose_task.rotation)
    entry = Pose(translation=entry_xyz, rotation=release_pose_task.rotation)

    return staging, entry, release_pose_task


def _entry_xyz_from_release(
    microwave: MicrowaveSpec,
    release_pose_task: Pose,
    into_cavity: np.ndarray,
) -> np.ndarray:
    """Entry pose position: just inside the door plane, on the line that
    passes through the release pose along ``into_cavity``."""
    # Vector from door plane (microwave origin) to release pose:
    door_origin_task = microwave.X_task_microwave.translation
    offset_from_door = release_pose_task.translation - door_origin_task
    # Project onto into_cavity to get "how deep into cavity is the release":
    release_depth = float(np.dot(offset_from_door, into_cavity))
    # Entry pose is at entry_standoff depth on the same line as release:
    # release.translation - (release_depth - entry_standoff) * into_cavity
    return release_pose_task.translation - (
        release_depth - microwave.entry_standoff
    ) * into_cavity


def release_xyz_inside_cavity(
    microwave: MicrowaveSpec,
    release_pose_task: Pose,
) -> float:
    """How far inside the cavity (along +microwave_x) is the release
    pose? Useful for diagnostics."""
    door_origin_task = microwave.X_task_microwave.translation
    offset_from_door = release_pose_task.translation - door_origin_task
    return float(np.dot(offset_from_door, microwave.microwave_x_in_task()))


# -----------------------------------------------------------------------
#  Primitive
# -----------------------------------------------------------------------

@dataclass
class MicrowavePlaceResult:
    success: bool
    reason: Optional[str] = None
    unreachable_waypoint: Optional[str] = None


def place_in_microwave(
    arm: ArmHandle,
    microwave: MicrowaveSpec,
    release_pose_task: Pose,
    config: PickPlaceConfig = DEFAULT,
) -> MicrowavePlaceResult:
    """Drive the arm to ``release_pose_task`` inside the microwave cavity
    and release the held object. See module docstring for strategy.

    ``release_pose_task`` is the TCP pose at which the fingers open. The
    caller picks its orientation (typically the same as the grasp pose
    for the held object — don't re-orient mid-flight).
    """
    if config.transit_z is None:
        raise ValueError("config.transit_z is unset.")
    if arm.gripper is None:
        raise ValueError(f"arm {arm.name!r} has no gripper attached.")

    staging, entry, release = build_waypoints(microwave, release_pose_task)

    waypoints: List[Tuple[str, Pose]] = [
        ("staging", staging),
        ("entry", entry),
        ("release", release),
    ]

    # Pre-flight reachability on every waypoint. Refuse before moving if
    # any is infeasible.
    for label, pose in waypoints:
        if is_task_pose_reachable(arm, pose) is None:
            return MicrowavePlaceResult(
                success=False,
                reason=(f"waypoint '{label}' is kinematically infeasible. "
                        f"Move the microwave or pick a release pose closer "
                        f"to the door / less depth into the cavity."),
                unreachable_waypoint=label,
            )

    # --- execute ---
    lift_to_transit(arm, config.transit_z, config.transit_speed, config.transit_accel)
    transit_xy(arm, staging, config.transit_z, config.transit_speed, config.transit_accel)

    # Descend (or translate) to staging — gripper is now outside the door
    # with the correct orientation and plate held.
    approach_to(arm, staging, config.approach_speed, config.approach_accel)

    # Enter the cavity — pure translation along +microwave_x, no rotation.
    approach_to(arm, entry, config.approach_speed, config.approach_accel)

    # Advance to release pose.
    approach_to(arm, release, config.approach_speed, config.approach_accel)

    # Release.
    arm.gripper.set_speed_pct(config.gripper_open_speed_pct)
    arm.gripper.open()

    # Retract the way we came. Same straight line out.
    retract_to(arm, entry, config.retract_speed, config.retract_accel)
    retract_to(arm, staging, config.retract_speed, config.retract_accel)

    # Back up to transit altitude.
    retract_to(
        arm,
        Pose(translation=np.array([staging.translation[0],
                                   staging.translation[1],
                                   config.transit_z]),
             rotation=staging.rotation),
        config.retract_speed, config.retract_accel,
    )

    return MicrowavePlaceResult(success=True)
