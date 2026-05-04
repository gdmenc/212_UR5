"""Attach the hook gripper (left arm) and Robotiq 2F-85 (right arm).

Both grippers are loaded as static rigid bodies welded to the wrist
flange. We don't simulate open/close in Drake — the Robotiq comes in
two variants (``closed`` and ``open``) so the planner can pick whichever
matches the gripper state in the world for collision checks.

A single named ``FixedOffsetFrame`` per arm is added to ``wrist_3_link``
at the calibrated TCP_OFFSET. Downstream code (``transit.py``,
``execute.py``) keys off these frame names instead of ``wrist_3_link``,
so the IK target is the actual tool tip rather than the wrist flange.

Frame conventions (decoded from the existing pipeline):

  Hook gripper
    - mesh authored with origin at the TCP (rim engagement point)
    - +x of the hook frame points back toward the wrist flange
    - welded with TCP_OFFSET_HOOK = (0, 0, 0.10275, 0, π/2, 0) so its
      origin lands at the calibrated TCP, identical to the historical
      manipulation-station ``X_PC`` weld

  Robotiq 2F-85 (this package's static SDFs)
    - origin at the wrist flange face (xacro convention)
    - +z toward the fingers
    - welded with identity transform; TCP frame sits 184 mm out along
      wrist +z separately
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Literal, Optional

import numpy as np
from pydrake.math import RigidTransform, RollPitchYaw, RotationMatrix
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import MultibodyPlant
from pydrake.multibody.tree import (
    FixedOffsetFrame,
    ModelInstanceIndex,
)

from ...calibration import TCP_OFFSET_HOOK, TCP_OFFSET_ROBOTIQ_2F85
from .arms import ArmHandles


_REPO_ROOT = Path(__file__).resolve().parents[3]
_HOOK_SDF = _REPO_ROOT / "src" / "assets" / "hook_gripper" / "hook_gripper.sdf"
_ROBOTIQ_DIR = _REPO_ROOT / "src" / "assets" / "robotiq_2f_85"


@dataclass
class GripperHandles:
    arm: str
    gripper_model_instance: ModelInstanceIndex
    tcp_frame_name: str


def _tcp_offset_to_rigid_transform(tcp_offset: list) -> RigidTransform:
    """Convert calibration's [x, y, z, rx, ry, rz] (axis-angle) to RigidTransform."""
    x, y, z, rx, ry, rz = tcp_offset
    rotvec = np.array([rx, ry, rz])
    angle = float(np.linalg.norm(rotvec))
    if angle < 1e-9:
        R = RotationMatrix()
    else:
        axis = rotvec / angle
        # Build rotation from axis-angle
        K = np.array([
            [0, -axis[2], axis[1]],
            [axis[2], 0, -axis[0]],
            [-axis[1], axis[0], 0],
        ])
        R_mat = np.eye(3) + np.sin(angle) * K + (1 - np.cos(angle)) * (K @ K)
        R = RotationMatrix(R_mat)
    return RigidTransform(R, [x, y, z])


def _add_tcp_frame(
    plant: MultibodyPlant,
    arm: str,
    arm_instance: ModelInstanceIndex,
    tcp_offset: list,
) -> str:
    """Add a FixedOffsetFrame named ``tcp_<arm>`` on wrist_3_link at the
    calibrated TCP_OFFSET. Returns the frame name for downstream lookup."""
    wrist = plant.GetFrameByName("wrist_3_link", arm_instance)
    X_wrist_tcp = _tcp_offset_to_rigid_transform(tcp_offset)
    frame_name = f"tcp_{arm.removeprefix('ur_')}"   # tcp_left / tcp_right
    plant.AddFrame(FixedOffsetFrame(
        name=frame_name,
        P=wrist,
        X_PF=X_wrist_tcp,
    ))
    return frame_name


def add_hook_gripper(
    plant: MultibodyPlant,
    arm_handle: ArmHandles,
) -> GripperHandles:
    """Weld the hook gripper to ``arm_handle``'s wrist_3_link.

    The hook SDF is authored with origin at the TCP and +x pointing
    back toward the flange. We weld at TCP_OFFSET_HOOK so the
    gripper origin (= TCP) lands at the calibrated TCP location.
    """
    if not _HOOK_SDF.exists():
        raise FileNotFoundError(f"Hook gripper SDF missing: {_HOOK_SDF}")

    parser = Parser(plant, f"hook_gripper_{arm_handle.name}")
    parser.SetAutoRenaming(True)
    (gripper_instance,) = parser.AddModels(str(_HOOK_SDF))

    wrist = plant.GetFrameByName("wrist_3_link", arm_handle.model_instance)
    gripper_frame = plant.GetFrameByName("hook_gripper", gripper_instance)
    X_wrist_gripper = _tcp_offset_to_rigid_transform(TCP_OFFSET_HOOK)
    plant.WeldFrames(wrist, gripper_frame, X_wrist_gripper)

    tcp_frame_name = _add_tcp_frame(
        plant, arm_handle.name, arm_handle.model_instance, TCP_OFFSET_HOOK,
    )

    return GripperHandles(
        arm=arm_handle.name,
        gripper_model_instance=gripper_instance,
        tcp_frame_name=tcp_frame_name,
    )


def add_robotiq_2f85(
    plant: MultibodyPlant,
    arm_handle: ArmHandles,
    *,
    mode: Literal["closed", "open"] = "closed",
) -> GripperHandles:
    """Weld a static Robotiq 2F-85 (with custom 3-in fingers) to
    ``arm_handle``'s wrist_3_link.

    The base mesh is authored with origin at the flange face and +z
    toward the fingers. We weld at identity, then add the TCP frame
    separately at TCP_OFFSET_ROBOTIQ_2F85 (184 mm along wrist +z).
    """
    sdf_path = _ROBOTIQ_DIR / f"robotiq_2f_85_{mode}.sdf"
    if not sdf_path.exists():
        raise FileNotFoundError(
            f"Robotiq SDF missing: {sdf_path} "
            f"(supported modes: closed, open)"
        )

    parser = Parser(plant, f"robotiq_2f_85_{arm_handle.name}")
    parser.SetAutoRenaming(True)
    (gripper_instance,) = parser.AddModels(str(sdf_path))

    wrist = plant.GetFrameByName("wrist_3_link", arm_handle.model_instance)
    gripper_frame = plant.GetFrameByName("robotiq_2f_85", gripper_instance)
    plant.WeldFrames(wrist, gripper_frame, RigidTransform())

    tcp_frame_name = _add_tcp_frame(
        plant, arm_handle.name, arm_handle.model_instance,
        TCP_OFFSET_ROBOTIQ_2F85,
    )

    return GripperHandles(
        arm=arm_handle.name,
        gripper_model_instance=gripper_instance,
        tcp_frame_name=tcp_frame_name,
    )


def add_grippers(
    plant: MultibodyPlant,
    arms: Dict[str, ArmHandles],
    *,
    robotiq_mode: Literal["closed", "open"] = "closed",
) -> Dict[str, GripperHandles]:
    """Attach the hook to ur_left and the Robotiq 2F-85 to ur_right.

    Returns a dict keyed by arm name. ``robotiq_mode`` picks which
    static finger configuration to load.
    """
    out: Dict[str, GripperHandles] = {}
    if "ur_left" in arms:
        out["ur_left"] = add_hook_gripper(plant, arms["ur_left"])
    if "ur_right" in arms:
        out["ur_right"] = add_robotiq_2f85(
            plant, arms["ur_right"], mode=robotiq_mode,
        )
    return out
