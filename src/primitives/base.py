"""Primitive contract.

A primitive is an IMPERATIVE function with signature roughly:

    primitive(backend, planner, scene, *args, **kwargs) -> PrimitiveResult

It:
  1. Reads the current scene (object poses, joint state) to resolve goal
     poses (grasp pose, drop pose, contact pose, ...).
  2. Calls planners to turn goal poses into joint trajectories, or computes
     Cartesian segments directly for short contact-adjacent moves.
  3. Issues ``Segment`` commands through ``backend.execute(arm, seg)`` to
     drive the arm.
  4. Checks status between segments (e.g. ``backend.gripper(arm).status()``
     after a grasp, or the wrench after a contact move) and branches on
     failure — retry, abort, or re-plan.
  5. Returns a ``PrimitiveResult`` the sequencer can inspect.

Why imperative (not "return a list of segments to be executed by someone
else")? Contact-rich primitives like tray-push need a live loop — "push
until torque exceeds threshold, then release, then retract." Grasp
primitives need retry logic on slip. Static segment lists cannot express
those. Keeping primitives imperative + ``Segment``-typed at the backend
boundary gets us the best of both: testable command flow, reactive control.
"""

from dataclasses import dataclass, field
from typing import Any, Optional


@dataclass
class PrimitiveResult:
    """Return value every primitive must produce.

    Attributes
    ----------
    success : bool
        True if the primitive achieved its goal (grasp confirmed, pour
        complete, door fully open, etc.).
    reason : str | None
        Human-readable failure reason on ``success=False``. Used for
        logging and for the sequencer's branching logic.
    data : dict
        Primitive-specific payload — e.g. ``pick`` can return the final
        grasp pose actually used, so the sequencer can re-plan a place
        around that.
    """

    success: bool
    reason: Optional[str] = None
    data: dict = field(default_factory=dict)
