"""Water bottle grasps (the pour-FROM container).

Object-frame convention (matches plate / bowl / cup): origin at the
bottle's resting point on the table, +z up out of the opening, rotational
symmetry about +z.

Bottle shape (measured at the 212 lab):
    - Body diameter:           7.3 cm  (body radius 3.65 cm)
    - Top opening diameter:    ≈ 4 cm  (opening radius ≈ 2 cm)
    - Graspable body height:   13 cm   (from base up to z = 0.13)
    - Total height:            17.5 cm
    - Neck region:             z = 0.13 m to z = 0.175 m, tapering from
                               the body radius down to the opening radius.

Treat the lower 13 cm as a roughly uniform cylinder of radius 3.65 cm —
this is the graspable region. Above 13 cm the bottle narrows toward the
cap; do not plan body grasps there because the radius is changing and
the cap geometry is not modeled.

Grasp strategy: side body pinch with the 2F-85 across the cylindrical
body (radius 3.65 cm → ~7.3 cm finger spread, well inside the 2F-85's
~85 mm range). The grasp z-height is a free parameter inside
``[pad, BOTTLE_GRASPABLE_TOP_Z_M − pad]``; default is mid-body, which
keeps the bottle's CoM near the grasp axis (low wrist torque when
tilted) and leaves clearance both above (cap region) and below (table).

Pouring: see ``bottle_pour_tcp_pose``. Given the same grasp parameters
plus a pour TILT angle (radians) and an opening TARGET (xyz in task
frame), it returns the TCP pose at which the bottle is held tilted with
its opening sitting exactly at TARGET. The math is closed-form because
the bottle is rigid in the gripper — the opening's pose in tool-local
coords is fixed by the grasp choice and never changes thereafter.
"""

from __future__ import annotations

import numpy as np

from ..util.poses import Pose
from ..util.rotations import Rotation
from .base import Grasp


BOTTLE_BODY_RADIUS_M = 0.0365
"""Radial distance from bottle center to the outer wall of the cylindrical
body. Measured 7.3 cm body diameter."""

BOTTLE_OPENING_RADIUS_M = 0.020
"""Radial distance from bottle center to the rim of the top opening
(approximate — measured 4 cm opening diameter, ±a few mm)."""

BOTTLE_GRASPABLE_TOP_Z_M = 0.130
"""Upper z-bound of the graspable cylindrical body (measured from base).
Above this the bottle narrows toward the cap and the radius is no longer
BOTTLE_BODY_RADIUS_M."""

BOTTLE_TOTAL_HEIGHT_M = 0.175
"""Total bottle height including the neck/cap region. The neck spans
z ∈ [BOTTLE_GRASPABLE_TOP_Z_M, BOTTLE_TOTAL_HEIGHT_M] and tapers from
BOTTLE_BODY_RADIUS_M down to BOTTLE_OPENING_RADIUS_M."""

BOTTLE_DEFAULT_GRASP_Z_M = 0.1
"""Default grasp height in bottle frame (mid-body). Roughly half the
graspable region — close to a half-full bottle's CoM, which keeps wrist
torque low while the bottle is tilted during a pour."""

BOTTLE_PREGRASP_OFFSET = 0.05
"""Distance (m) along tool -Z from grasp to pregrasp. 5 cm gives the
fingers room to swing in horizontally without clipping the cylindrical
body during the final approach."""

BOTTLE_GRASP_FORCE = 20.0
"""Newtons. Higher than plate/bowl: the bottle is heavier (especially
filled) and gets tipped past horizontal during a pour, so the friction
budget needs more headroom against gravity."""


def _side_body_rotation(angle_rad: float) -> Rotation:
    """TCP rotation for a SIDE pinch on the bottle body at ``angle_rad``
    (in bottle frame). Tool +Z points radially INWARD (gripper sits
    outside the bottle and approaches inward), Tool Y is the tangent
    direction along which the real 2F-85 fingers close, and Tool X is
    world-down (perpendicular to both —
    completes a right-handed frame).

    Construction before the final tool-roll: at angle 0,
    R = R_y(-π/2) * R_z(π/2) gives Tool axes
    (Tool_X, Tool_Y, Tool_Z) = ((0,1,0), (0,0,-1), (-1,0,0)) — verified
    by direct computation. The lab 2F-85 closes along Tool Y, so we compose
    an extra Tool-Z 90° rotation to make Tool Y horizontal/tangent instead
    of vertical. Yawing the whole frame by ``angle_rad`` about world Z
    places the gripper at any side of the cylinder.
    """
    yaw = Rotation.from_rotvec([0.0, 0.0, angle_rad])
    base = (
        Rotation.from_rotvec([0.0, -np.pi / 2, 0.0])
        * Rotation.from_rotvec([0.0, 0.0, np.pi / 2])
        * Rotation.from_rotvec([0.0, 0.0, np.pi / 2])
    )
    return yaw * base


def bottle_body_grasp(
    X_task_bottle: Pose,
    angle_rad: float = 0.0,
    grasp_z: float = BOTTLE_DEFAULT_GRASP_Z_M,
) -> Grasp:
    """Side body pinch at ``angle_rad`` (bottle frame), at height
    ``grasp_z`` along the bottle's vertical axis.

    Raises ``ValueError`` if ``grasp_z`` is outside the cylindrical
    body region (above the cylinder the radius is changing — not a
    valid pinch target)."""
    if not (0.0 < grasp_z < BOTTLE_GRASPABLE_TOP_Z_M):
        raise ValueError(
            f"grasp_z={grasp_z} must be in (0, {BOTTLE_GRASPABLE_TOP_Z_M}) m "
            f"— above this the bottle tapers into the neck and the body "
            f"radius is no longer constant."
        )
    grasp_xyz = np.array([
        BOTTLE_BODY_RADIUS_M * np.cos(angle_rad),
        BOTTLE_BODY_RADIUS_M * np.sin(angle_rad),
        grasp_z,
    ])
    X_bottle_grasp = Pose(
        translation=grasp_xyz,
        rotation=_side_body_rotation(angle_rad),
    )
    return Grasp(
        grasp_pose=X_task_bottle @ X_bottle_grasp,
        pregrasp_offset=BOTTLE_PREGRASP_OFFSET,
        grasp_force=BOTTLE_GRASP_FORCE,
        description=(
            f"bottle body grasp @ {np.degrees(angle_rad):+.0f}°, "
            f"z={grasp_z*100:.1f} cm"
        ),
    )


HOOK_RIM_PREGRASP_OFFSET = 0.05
"""Vertical pregrasp standoff (m) for the hook engaging the bottle's
opening rim. The hook's throat opening sits 1.6 cm below the TCP in task
z (per HOOK_FINGER_TIP_LATERAL_M); 5 cm leaves ~3.4 cm of clear vertical
space between the finger tip and the rim plane during pregrasp."""


def _hook_rim_rotation(angle_rad: float) -> Rotation:
    """TCP rotation (object frame) for a hook rim grasp at ``angle_rad``
    around the bottle's +z axis. Combined with ``TCP_OFFSET_HOOK = R_y(+π/2)``
    this produces a TCP frame at the grasp pose with:
        - Tool +Z = task -Z (vertical descent direction)
        - Tool +X = radial OUTWARD at angle_rad (away from bottle center)
        - Tool +Y = horizontal tangent
    Matches the rig's natural hook-rim wrist orientation (verified against
    a recorded grasp pose at angle π).
    """
    yaw = Rotation.from_rotvec([0.0, 0.0, angle_rad])
    flip = Rotation.from_rotvec([np.pi, 0.0, 0.0])
    return yaw * flip


def bottle_hook_grasp(
    X_task_bottle: Pose,
    angle_rad: float = 0.0,
) -> Grasp:
    """Hook-gripper rim grasp at the bottle's top opening. Hook descends
    vertically; finger threads through the opening into the neck cavity,
    fixed jaw lands radially outside the rim. Bottle hangs by its rim wall
    clamped between the finger and the fixed jaw.

    Cap must be off — the rim must be exposed for the hook to engage."""
    rim_xyz = np.array([
        BOTTLE_OPENING_RADIUS_M * np.cos(angle_rad),
        BOTTLE_OPENING_RADIUS_M * np.sin(angle_rad),
        BOTTLE_TOTAL_HEIGHT_M,
    ])
    X_bottle_grasp = Pose(
        translation=rim_xyz,
        rotation=_hook_rim_rotation(angle_rad),
    )
    return Grasp(
        grasp_pose=X_task_bottle @ X_bottle_grasp,
        pregrasp_offset=HOOK_RIM_PREGRASP_OFFSET,
        description=f"bottle hook rim grasp @ {np.degrees(angle_rad):+.0f}°",
    )


def _opening_in_hook_tool_frame() -> np.ndarray:
    """Vector from TCP origin to the bottle's opening center, expressed in
    HOOK tool-local coords. INVARIANT under any TCP motion (rigid grasp),
    and ALSO invariant under the grasp angle θ — that's the rotational
    symmetry of the rim circle paying off.

    Derivation: TCP sits on the rim outer edge at bottle (R_rim cos θ,
    R_rim sin θ, H); opening center at (0, 0, H). Vector in bottle frame
    is (-R_rim cos θ, -R_rim sin θ, 0). Tool +X = radial outward at angle
    θ = (cos θ, sin θ, 0) in bottle frame, so the bottle vector projects
    onto tool axes as (x_tool, y_tool, z_tool) = (-R_rim, 0, 0) — θ
    cancels out.
    """
    return np.array([-BOTTLE_OPENING_RADIUS_M, 0.0, 0.0])


def bottle_hook_pour_tcp_pose(
    X_task_bottle: Pose,
    target_task,
    angle_rad: float = 0.0,
    tilt_rad: float = 0.0,
) -> Pose:
    """TCP pose (task frame) at which the bottle — held by a HOOK rim grasp
    at ``angle_rad`` — is tilted by ``tilt_rad`` and its opening center
    sits at ``target_task``.

    Same conventions as ``bottle_pour_tcp_pose`` (the 2F-85 body grasp
    version): positive ``tilt_rad`` tips the bottle's +z away from the
    gripper, around the tool +Y axis. The math is simpler than the body
    grasp because the opening's offset from the TCP is just R_rim (~2 cm)
    along tool -X, independent of any grasp_z parameter — there's no
    grasp_z for a rim grasp.
    """
    target_task = np.asarray(target_task, dtype=float).reshape(3)
    R_tcp = (
        X_task_bottle.rotation
        * _hook_rim_rotation(angle_rad)
        * Rotation.from_rotvec([0.0, tilt_rad, 0.0])
    )
    opening_offset_task = R_tcp.apply(_opening_in_hook_tool_frame())
    return Pose(
        translation=target_task - opening_offset_task,
        rotation=R_tcp,
    )


def _opening_in_tool_frame(grasp_z: float) -> np.ndarray:
    """Vector from the TCP origin to the bottle's opening, expressed in
    tool-local coordinates. INVARIANT under any TCP motion (the bottle
    is rigidly held), so we compute it once from grasp parameters and
    use it for every pour-pose calculation.

    Derivation (at angle_rad = 0): grasp point in bottle frame is
    (R, 0, grasp_z); opening is at (0, 0, H). Vector from grasp to
    opening in bottle frame = (-R, 0, H - grasp_z). Rotated into the
    tool frame after the 90° tool-roll (Tool_X = -z_bottle,
    Tool_Y = -y_bottle, Tool_Z = -x_bottle) gives
    (-(H - grasp_z), 0, R)."""
    return np.array([
        -(BOTTLE_TOTAL_HEIGHT_M - grasp_z),
        0.0,
        BOTTLE_BODY_RADIUS_M,
    ])


def bottle_pour_tcp_pose(
    X_task_bottle: Pose,
    target_task,
    angle_rad: float = 0.0,
    grasp_z: float = BOTTLE_DEFAULT_GRASP_Z_M,
    tilt_rad: float = 0.0,
) -> Pose:
    """TCP pose (task frame) at which the bottle — held by a side body
    grasp at (``angle_rad``, ``grasp_z``) — is tilted by ``tilt_rad`` and
    its opening sits at ``target_task``.

    Parameters
    ----------
    X_task_bottle : Pose
        Bottle pose in task frame at the time of the grasp. Only the
        ROTATION matters for the pour pose (the translation is irrelevant
        — once held, the bottle's task-frame position is determined by
        the gripper, not by where it was originally sitting). For a
        bottle standing freely on the table, this is typically identity
        rotation.
    target_task : array-like (3,)
        Desired xyz of the bottle's OPENING (geometric center of the rim)
        in task frame. Pick this 3-5 cm above the receiver's rim so
        liquid falls cleanly without the opening contacting the rim.
    angle_rad, grasp_z : float
        Same parameters used in ``bottle_body_grasp``. Must match the
        actual pickup grasp — this function assumes the bottle is held
        the way that grasp specifies.
    tilt_rad : float
        Tilt angle from upright. ``> 0`` tips the opening AWAY from the
        gripper (the natural pour direction — the cap end swings down
        and away). ``0`` = upright (no pour); ``π/2`` = horizontal;
        ``~2π/3`` (120°) = typical pour tilt; ``π`` = fully inverted.

    Returns
    -------
    Pose
        TCP pose in task frame. Feed this to ``approach_to`` after a
        transit_xy waypoint over the target.

    Math
    ----
    Let ``v_tool = (0, -(H - grasp_z), R)`` be the opening's position in
    tool-local coords (invariant — see ``_opening_in_tool_frame``). Let
    ``R_tcp`` be the desired TCP rotation in task frame:
    ``R_tcp = X_task_bottle.rotation * R_grasp(angle_rad) * R_y(tilt_rad)``,
    where the final ``R_y(tilt_rad)`` is applied in tool-local frame
    (rotation about the horizontal Tool_Y / finger-close axis, which tips
    the bottle's +z away from the gripper by the direct positive angle).
    Then the
    opening's position in task frame is
    ``opening_task = TCP_translation + R_tcp.apply(v_tool)``, so
    ``TCP_translation = target_task - R_tcp.apply(v_tool)``.
    """
    target_task = np.asarray(target_task, dtype=float).reshape(3)
    R_tcp = (
        X_task_bottle.rotation
        * _side_body_rotation(angle_rad)
        * Rotation.from_rotvec([0.0, tilt_rad, 0.0])
    )
    opening_offset_task = R_tcp.apply(_opening_in_tool_frame(grasp_z))
    return Pose(
        translation=target_task - opening_offset_task,
        rotation=R_tcp,
    )
