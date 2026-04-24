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
    transit_speed: float = 0.25    # m/s, transit MoveL
    transit_accel: float = 0.5     # m/s^2
    approach_speed: float = 0.05   # m/s, final approach to grasp/place
    approach_accel: float = 0.25   # m/s^2
    retract_speed: float = 0.15    # m/s
    retract_accel: float = 0.4     # m/s^2

    # --- Force / contact thresholds ---
    place_contact_threshold: float = 15.0
    """Newtons. MoveUntilContact stop condition for place descent."""

    # --- Gripper defaults (applied once per pick/place via gripper's
    # set_speed_pct / set_force_pct — grippers without these methods
    # ignore silently). Per-grasp force still overrides via Grasp.grasp_force. ---
    gripper_speed_pct: int = 100
    """0-100. Robotiq close/open speed. 100 is the UR2026 example default."""

    gripper_force_pct: int = 50
    """0-100. Default force envelope applied at pick/place entry. Grasp
    force from Grasp.grasp_force (in Newtons) overrides for the actual
    grasp call; this is the open()/close()/move() baseline."""

    # --- Safety bounds (not yet enforced — collect for later). ---
    workspace_z_min: Optional[float] = None
    """Lowest z the TCP is allowed to reach. Prevents commanding into the table."""


DEFAULT = PickPlaceConfig()
