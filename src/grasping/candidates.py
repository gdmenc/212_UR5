"""Per-object grasp candidate poses, in the OBJECT's local frame.

A *grasp candidate* is the pose of the gripper (its TCP frame) relative to
the object being grasped, ``X_OG``. The world-frame grasp is then
``X_WG = X_WO @ X_OG``.

Grasp candidates are **keyed on (object_name, gripper_type)** because a
hook-latch pose and a two-finger pinch are different grasp geometries —
same object, totally different approach direction. ``gripper_type`` is the
short string defined on each ``Gripper`` subclass (``"hook"``,
``"robotiq_2f85"``); primitives read it from ``backend.gripper(arm).type_name``
and pass it in here.

Conventions
-----------
- Coordinates are in the object's SDF frame. Inspect the SDF to confirm
  the frame origin (usually the object's geometric center, sometimes a
  base corner).
- Gripper frame convention used here: +Z points "out of the palm" — the
  approach direction. A top-down grasp therefore has the gripper rotated
  180 deg about X so +Z points down into the object.
- Positive yaw rotates the gripper about the object's +Z axis; used for
  symmetric objects to enumerate rotated variants.

Why one file, not inlined in ``Scene``
--------------------------------------
Primitives need grasps; Scene does not (Scene is just diagram + plant
handles). Keeping grasps in their own module also makes the migration to
perception-driven generation a one-file swap: replace the static dispatch
in ``get_grasp_candidates`` with a call into a grasp detector, same
signature, same callers.
"""

from typing import Dict, List, Optional, Tuple

import numpy as np
from pydrake.math import RigidTransform, RollPitchYaw


# ----- pose helpers (two-finger / Robotiq style) -------------------------

def _top_down(z_offset: float = 0.0, yaw: float = 0.0) -> RigidTransform:
    """Gripper pointing straight down at an object's local origin."""
    return RigidTransform(
        RollPitchYaw(np.pi, 0.0, yaw),
        np.array([0.0, 0.0, z_offset]),
    )


def _side(yaw: float, z: float = 0.0, radius: float = 0.0) -> RigidTransform:
    """Horizontal gripper approaching from the side at angle ``yaw``."""
    approach_dir = np.array([np.cos(yaw), np.sin(yaw), 0.0])
    tcp_pos = approach_dir * radius + np.array([0.0, 0.0, z])
    return RigidTransform(
        RollPitchYaw(np.pi / 2, 0.0, yaw + np.pi / 2),
        tcp_pos,
    )


# ----- pose helpers (hook style) -----------------------------------------

def _hook_latch(yaw: float, z: float, radius: float = 0.0) -> RigidTransform:
    """Approach pose for the hook-gripper latching a rim or edge.

    The hook tip slides in horizontally at angle ``yaw`` (about the
    object's +Z) at height ``z`` and then extends to engage. This helper
    places the TCP just outside the rim; the primitive adds its own
    approach standoff.

    TODO: verify hook-tip frame convention against ``hook_gripper.sdf``.
    The hook's extend-direction in the TCP frame defines the correct
    orientation; the values here are a best-guess starting point.
    """
    approach_dir = np.array([np.cos(yaw), np.sin(yaw), 0.0])
    tcp_pos = approach_dir * radius + np.array([0.0, 0.0, z])
    return RigidTransform(
        RollPitchYaw(0.0, np.pi / 2, yaw),  # TODO tune
        tcp_pos,
    )


# ----- per-object, per-gripper grasp lists -------------------------------
#
# Key: (object_name, gripper_type). Miss -> KeyError (loud, not silent).
# Missing combinations that may be needed later:
#   - ("tray", "hook")            : drag/nudge the tray with the hook?
#   - ("plate_8in", "hook")       : drag plate out of microwave
#   - ("bowl", "robotiq_2f85")    : if we ever want to hand the bowl to the
#                                    two-finger arm.

def _plate_grasps_2f85() -> List[RigidTransform]:
    """Plate, two-finger: top-down rim pinch."""
    return [_top_down(z_offset=0.01)]


def _plate_drag_hook() -> List[RigidTransform]:
    """Plate, hook: latch under rim for drag-out-of-microwave step."""
    # TODO: approach from front (microwave-mouth side) to slide under rim.
    return [_hook_latch(yaw=0.0, z=0.01, radius=0.10)]


def _bowl_grasps_hook() -> List[RigidTransform]:
    """Bowl, hook: latch under rim. Symmetric about Z, try multiple yaws."""
    return [_hook_latch(yaw=y, z=0.04, radius=0.08)
            for y in np.linspace(0, 2 * np.pi, 8, endpoint=False)]


def _cup_grasps_2f85() -> List[RigidTransform]:
    """Cup, two-finger: side grasp around body. Symmetric -> yaw ring."""
    return [_side(yaw=y, z=0.05, radius=0.0)
            for y in np.linspace(0, 2 * np.pi, 12, endpoint=False)]


def _bottle_grasps_2f85() -> List[RigidTransform]:
    """Bottle, two-finger: side grasp at mid-height. 16 yaw candidates."""
    return [_side(yaw=y, z=0.10, radius=0.0)
            for y in np.linspace(0, 2 * np.pi, 16, endpoint=False)]


def _stirrer_grasps_2f85() -> List[RigidTransform]:
    """Stirrer (wooden rod): pinch near the top, long axis vertical.

    Rod is rotationally symmetric, so we enumerate approach yaws. Grasp
    height (z) is near the top of the rod so the bottom end is free to
    enter the cup."""
    return [_side(yaw=y, z=0.08, radius=0.0)
            for y in np.linspace(0, 2 * np.pi, 12, endpoint=False)]


def _tray_handle_grasps_2f85() -> List[RigidTransform]:
    """Tray, two-finger: one grasp per handle (left/right), for bimanual
    carry. List order corresponds to the handle index the primitive
    requests (0 = left handle, 1 = right handle).

    TODO: replace the placeholder offsets with the tray SDF's handle
    positions once the asset is finalized.
    """
    left_handle = _side(yaw=np.pi, z=0.02, radius=-0.15)   # -x side
    right_handle = _side(yaw=0.0, z=0.02, radius=0.15)     # +x side
    return [left_handle, right_handle]


_GRASP_TABLE: Dict[Tuple[str, str], List[RigidTransform]] = {
    ("plate_8in", "robotiq_2f85"): _plate_grasps_2f85(),
    ("plate_8in2", "robotiq_2f85"): _plate_grasps_2f85(),
    ("plate_8in", "hook"): _plate_drag_hook(),

    ("bowl", "hook"): _bowl_grasps_hook(),

    ("cup", "robotiq_2f85"): _cup_grasps_2f85(),
    ("bottle", "robotiq_2f85"): _bottle_grasps_2f85(),
    ("stirrer", "robotiq_2f85"): _stirrer_grasps_2f85(),

    ("tray", "robotiq_2f85"): _tray_handle_grasps_2f85(),
}


def get_grasp_candidates(
    object_name: str,
    gripper_type: str,
    scene: Optional[object] = None,
) -> List[RigidTransform]:
    """Return the list of ``X_OG`` candidates to try, in preferred order.

    Parameters
    ----------
    object_name : logical object key (e.g. ``"plate_8in"``, ``"bowl"``).
    gripper_type : short gripper-kind string from ``Gripper.type_name``
        (``"hook"`` or ``"robotiq_2f85"``). Same object may have different
        grasps for different grippers.
    scene : unused today. Reserved for the future perception-driven path
        where the candidate set depends on live state (clutter on top,
        deformation, occlusion). Keeping the parameter in the signature
        now means callers don't change when we swap implementations.

    Raises
    ------
    KeyError if no candidates are registered for the (object, gripper)
    pair. Add an entry to ``_GRASP_TABLE`` and a helper above.
    """
    key = (object_name, gripper_type)
    if key not in _GRASP_TABLE:
        raise KeyError(
            f"No grasp candidates for object={object_name!r}, "
            f"gripper={gripper_type!r}. Register one in "
            f"src/grasping/candidates.py::_GRASP_TABLE."
        )
    return list(_GRASP_TABLE[key])
