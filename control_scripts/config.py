"""Tunable parameters for pick-and-place control.

Every value here is a knob you will turn during the first real-robot dry
run. Left as ``None`` where a sensible default can't be guessed from geometry
alone — those must be set before calling pick/place or you'll get a clear
``ValueError`` at the top of the primitive instead of a silent bad move.

All distances in meters, speeds in m/s (Cartesian) or rad/s (joint),
accelerations in m/s^2 or rad/s^2. Forces in Newtons.
"""

from dataclasses import dataclass, field
from typing import Optional


@dataclass
class PickPlaceConfig:
    """Pass one of these into pick()/place() so overrides are explicit and
    the defaults here stay read-only."""

    # --- Geometry (set from measurements after first dry run) ---
    transit_z: Optional[float] = None
    """World/base-frame altitude for the 'safe transit' plane. Must be above
    every object in the workspace. Estimate: table top + tallest object + 5 cm
    margin. Start conservative (high), lower once motion is known-safe."""

    preplace_offset: float = 0.10
    """Preplace standoff: distance (m) along tool -Z from the place pose to
    the preplace waypoint. Adjust per-object if needed — larger for tall
    objects so the held object clears the surface during the descend."""

    # pregrasp_offset and grasp_force are now PER-GRASP (see grasps.Grasp),
    # not global. Each object module in grasps/ picks the right standoff
    # (plate: 3 cm, bowl: 5 cm, ...) so this config stays workspace-level.

    # --- Speeds (RTDE defaults; override per-move when a primitive needs
    # a slower final approach). ---
    transit_speed: float = 0.2    # m/s, transit MoveL
    transit_accel: float = 0.4     # m/s^2
    approach_speed: float = 0.05   # m/s, final approach to grasp/place
    approach_accel: float = 0.2   # m/s^2
    retract_speed: float = 0.2   # m/s
    retract_accel: float = 0.4     # m/s^2

    # --- Force / contact thresholds ---
    place_contact_threshold: float = 15.0
    """Newtons. MoveUntilContact stop condition for place descent."""

    place_use_contact_descent: bool = True
    """If True (default), place() uses ``move_until_contact`` for the
    final descent — robust to surface-height uncertainty (tables, trays,
    microwave shelves all at different known heights).

    Set False to make the final descent a deterministic ``approach_to
    (place_pose)`` — the TCP lands exactly at the pose the caller
    supplied and the gripper opens there, with no force-seeking. Use
    this for dry-run trials or whenever you want reproducible timing /
    no force-mode RPCs fired."""

    # --- Partial aperture / staged release ---
    release_aperture_mm: Optional[int] = None
    """Robotiq aperture (mm, 0 = closed, ~85 = fully open) used symmetrically
    on both sides of a pick-and-place:

      - place(): partial open at the release pose to let the object settle
        before the fingers go fully wide. Avoids the 85 mm finger swing
        knocking into cavity walls (microwave, shelves).
      - pick():  pre-set aperture just before the final approach so the
        fingers are already near the object envelope rather than fully open.

    Only honoured for grippers that implement ``move_mm`` (Robotiq 2F-85).
    Other grippers fall back to full open()/close(). ``None`` keeps the
    legacy behaviour (full open at release, no preset before grasp)."""

    release_clearance: float = 0.03
    """Distance (m) along tool +Z for a small intermediate hop, applied
    symmetrically:

      - place(): between the partial-open and the full ``open()``, so the
        fingers break contact with the released object before swinging wide.
      - pick():  between ``grasp()`` and the retract to pregrasp, so the
        held object lifts off the surface gently before larger retracts.

    Set 0.0 to disable. Typical values: 1–3 cm."""

    # --- Gripper: per-operation speed/force (applied via the Gripper
    # ABC's set_speed_pct / set_force_pct right before each call; grippers
    # without those methods ignore silently). Per-grasp force from
    # Grasp.grasp_force (Newtons) still overrides gripper_close_force_pct
    # for the actual grasp() call. ---

    gripper_open_speed_pct: int = 100
    """0-100. Speed for ``gripper.open()`` — typically fast, no contact.
    The UR2026 reference example uses 100."""

    gripper_close_speed_pct: int = 100
    """0-100. Speed for ``gripper.close()`` and ``gripper.grasp()`` —
    slower than open by default so we engage fragile objects gently.
    Bump up to 100 for rigid objects where speed matters."""

    gripper_close_force_pct: int = 80
    """0-100. Baseline force envelope for ``gripper.close()``. NOT used
    by ``grasp()`` — that call takes a per-object Newton value via
    ``Grasp.grasp_force`` and overrides. This field only matters if
    someone calls ``close()`` directly (outside pick)."""

    # --- Safety bounds (not yet enforced — collect for later). ---
    workspace_z_min: Optional[float] = None
    """Lowest z the TCP is allowed to reach. Prevents commanding into the table."""


DEFAULT = PickPlaceConfig()
