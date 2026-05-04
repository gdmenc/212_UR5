"""Load both UR5e arms and weld them at the calibrated bases.

We load ``ur_description/urdf/ur5e.urdf`` (NOT ``ur5e_left.urdf`` /
``ur5e_right.urdf``, which carry hardcoded ``base_joint`` poses we'd
have to compensate for).  The URDF declares its own ``<link name="world"/>``
plus a fixed ``base_joint`` from world → base_link.  Drake treats that
link as the global world body and the joint as auto-applied at parse
time, which means ``base_link`` is already welded to Drake's world by
the time ``Parser.AddModels`` returns — so a second WeldFrames call
fails.

Workaround: read the URDF text, strip the ``<link name="world"/>`` and
``<joint name="base_joint">…</joint>`` elements, parse the modified
URDF.  Now ``base_link`` is a free root link and we weld it ourselves
to ``X_world_armbase``.

Drake world == task frame, so:
    X_world_armbase = X_task_armbase = inverse(X_LEFT_BASE_TASK)

That's the *only* coordinate hop needed.  The arm's six joint angles
(``SetPositions``) line up directly with what RTDE reports as
``getActualQ``.

The URDF references mesh files at relative paths
(``../meshes/ur5e/visual/*.obj``).  Per the project ``MEMORY.md``,
those are stored as git-LFS pointers; if you see Meshcat render the
arms as wireframe boxes or empty, run ``git lfs pull`` and reload.
"""

from __future__ import annotations

import re
import tempfile
from dataclasses import dataclass
from pathlib import Path

from pydrake.math import RigidTransform, RotationMatrix
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import MultibodyPlant
from pydrake.multibody.tree import ModelInstanceIndex

from ...calibration import X_LEFT_BASE_TASK, X_RIGHT_BASE_TASK
from ...util.poses import Pose


# Repo-relative path so this works on any user's checkout.
_REPO_ROOT = Path(__file__).resolve().parents[3]
_UR5E_URDF = _REPO_ROOT / "ur_description" / "urdf" / "ur5e.urdf"


def _read_ur5e_urdf_without_world_weld() -> str:
    """Return the URDF text with ``<link name="world"/>`` and the
    ``base_joint`` element removed, so the parsed model has
    ``base_link`` as a free root link we can weld ourselves."""
    text = _UR5E_URDF.read_text()
    # Match a self-closing or paired world link tag.
    text = re.sub(
        r'<link\s+name="world"\s*/>', "", text, count=1,
    )
    text = re.sub(
        r'<link\s+name="world"\s*>.*?</link>', "", text, count=1, flags=re.DOTALL,
    )
    # Remove the base_joint element entirely.
    text = re.sub(
        r'<joint\s+name="base_joint"[\s\S]*?</joint>', "", text, count=1,
    )
    return text


@dataclass
class ArmHandles:
    """References to the parts of the plant a planner needs to drive an arm."""

    name: str
    """``'ur_left'`` or ``'ur_right'`` — matches the RTDE-side ArmHandle name."""

    model_instance: ModelInstanceIndex


def _pose_to_rigid_transform(pose: Pose) -> RigidTransform:
    """Convert this package's ``Pose`` to a Drake ``RigidTransform``."""
    return RigidTransform(
        RotationMatrix(pose.rotation.as_matrix()),
        pose.translation,
    )


def _add_arm(
    plant: MultibodyPlant,
    name: str,
    X_base_task: Pose,
) -> ArmHandles:
    if not _UR5E_URDF.exists():
        raise FileNotFoundError(
            f"UR5e URDF missing at {_UR5E_URDF}. "
            f"Did you clone the ur_description submodule?"
        )

    urdf_text = _read_ur5e_urdf_without_world_weld()

    # AddModelsFromString can't resolve relative mesh URIs (no anchor
    # path). Write to a temp file *inside the URDF directory* so the
    # parser resolves '../meshes/...' the same way it would for the
    # original ur5e.urdf.
    with tempfile.NamedTemporaryFile(
        mode="w", suffix=".urdf",
        dir=str(_UR5E_URDF.parent), delete=False,
    ) as tf:
        tf.write(urdf_text)
        tmp_path = Path(tf.name)
    try:
        parser = Parser(plant, name)
        parser.SetAutoRenaming(True)
        (model_instance,) = parser.AddModels(str(tmp_path))
    finally:
        tmp_path.unlink(missing_ok=True)

    # Drake world == task frame, so X_world_base = X_task_base = inv(X_base_task).
    #
    # Critical: weld the URDF's ``base`` frame, NOT ``base_link``. The
    # ur_description URDF defines two base frames rotated π around z:
    #
    #   - ``base_link``           — REP-103 aligned (X+ forward)
    #   - ``base`` (= ``base_link_inertia``) — UR controller convention (X+ backward)
    #
    # ``X_LEFT_BASE_TASK`` / ``X_RIGHT_BASE_TASK`` in calibration.py were
    # measured against the controller's ``base`` (which is what RTDE's
    # ``getActualTCPPose`` returns poses in). Welding ``base_link`` here
    # silently put every joint config 180° off in shoulder_pan, so FK
    # disagreed with the recorded waypoints by ~1.5 m at the wrist.
    X_task_base = X_base_task.inverse()
    X_world_base = _pose_to_rigid_transform(X_task_base)

    plant.WeldFrames(
        plant.world_frame(),
        plant.GetFrameByName("base", model_instance),
        X_world_base,
    )

    return ArmHandles(name=name, model_instance=model_instance)


def add_left_arm(plant: MultibodyPlant) -> ArmHandles:
    return _add_arm(plant, "ur_left", X_LEFT_BASE_TASK)


def add_right_arm(plant: MultibodyPlant) -> ArmHandles:
    return _add_arm(plant, "ur_right", X_RIGHT_BASE_TASK)


def add_both_arms(plant: MultibodyPlant) -> dict:
    """Add both arms. Returns ``{'ur_left': handles, 'ur_right': handles}``."""
    return {
        "ur_left": add_left_arm(plant),
        "ur_right": add_right_arm(plant),
    }
