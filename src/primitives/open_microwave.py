"""Open / close microwave: hook the handle and sweep along the door arc.

Specialized for the microwave task:
  - The door rotates about a fixed hinge axis. The handle therefore moves
    on a known circular arc — radius = distance from hinge to handle,
    center on the hinge axis.
  - We parameterize the TCP target as a function of door angle theta. At
    each theta we have a desired handle position and approach orientation,
    which together define X_WTcp(theta).
  - The custom ``HookGripper`` latches onto the handle; we then stream the
    swept Cartesian path via DiffIK + ServoStream.
  - The hook stays latched throughout; we only unlatch after the door has
    reached the target angle and the arm has retracted.

Closing is the same operation with a reversed sweep, so both directions
live in one module — ``open_microwave`` and ``close_microwave`` wrap the
shared sweep with appropriate start/end angles.

High-level flow (open):

    1. MoveJ / ServoStream to approach pose just outside the handle.
    2. MoveL onto the handle.
    3. GripperCommand("close")  # latch the hook
    4. diff_ik sweep from theta=0 to theta=theta_open; ServoStream.
    5. GripperCommand("open")   # unlatch
    6. MoveL retract.

The hinge axis and handle radius come from scene state — expected on the
scene's microwave object or as entries in ``scene.named_poses`` (e.g.
``"microwave_hinge_axis"``, ``"microwave_handle_closed"``).
"""

from typing import Optional

import numpy as np

from .base import PrimitiveResult


def _sweep_door(
    backend,
    planner,
    scene,
    arm: str,
    theta_start: float,
    theta_end: float,
    num_samples: int,
) -> PrimitiveResult:
    """Shared implementation. Latch handle at theta_start, DiffIK sweep to
    theta_end, unlatch. Both open and close route through here."""
    # TODO
    raise NotImplementedError


def open_microwave(
    backend,
    planner,
    scene,
    arm: str = "ur_left",
    theta_open: float = np.pi / 2,  # radians; 90deg door open
    num_samples: int = 100,
) -> PrimitiveResult:
    """Latch the hook on the door handle and sweep from closed (0) to
    ``theta_open``.

    TODO (design choice — do NOT implement before deciding):
      Option A — pure-position sweep (DiffIK + ServoStream) along the
      full arc from 0 to ``theta_open``. Simple, reuses the
      ``_sweep_door`` helper as-is. Risk: if the door latch resists for
      the first few degrees, the arm tracks the commanded pose and
      blows through a high transient load.
      Option B — hybrid: force-mode tangent-to-door-arc for the first
      ~10 deg to pop the latch under a bounded wrench, then switch to
      position sweep for the rest. Safer on the mechanism, but needs
      the ForceMode segment path working end-to-end in sim.

      Pick after we see how the real door latches (does the magnet
      resist meaningfully, or does it release almost immediately under
      a small torque?). Until then, keep this unimplemented so we do
      not ship a strategy that breaks the door on first contact.
    """
    raise NotImplementedError(
        "open_microwave: choose pure-position sweep (A) vs. force-mode "
        "latch-pop followed by position sweep (B); see docstring."
    )


def close_microwave(
    backend,
    planner,
    scene,
    arm: str = "ur_left",
    theta_start: float = np.pi / 2,
    num_samples: int = 100,
) -> PrimitiveResult:
    """Close the microwave door.

    TODO (design choice — do NOT implement before deciding):
      Option A — reverse-sweep the hook along the door arc, same as
      ``open_microwave`` but with start/end swapped. Latch-pull close,
      gentle, symmetric with the open routine.
      Option B — hook retracts, arm swings around and shoves the door
      closed with a force-mode push on the outer face. Faster, no
      latching needed, but unbounded contact dynamics on the slam.

      Pick one once we see how the door behaves on real hardware
      (does the latch engage cleanly at low speed? Does the magnet
      require a minimum closing velocity?). Until then, leave this
      primitive unimplemented so the recipe fails loudly at the step
      instead of silently using the wrong strategy.
    """
    raise NotImplementedError(
        "close_microwave: choose reverse-sweep (A) vs. force push (B) "
        "based on hardware behavior; see module docstring."
    )
