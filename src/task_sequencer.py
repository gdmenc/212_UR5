"""Task sequencer: composes primitives into full tasks.

A *task* is a finite-state sequence over primitives. For class scope we keep
each task as a straight Python function with early-return on failure — no
behavior trees, no symbolic planning, no explicit FSM class. If a task grows
non-trivial retry / replanning logic, lift that specific task into an FSM
at that point; do not pre-build the machinery.

Available tasks (see individual functions for the expected flow):
  - ``plate_into_microwave`` : open door, pick plate, place inside, close.
  - ``plate_out_of_microwave`` : inverse of the above.
  - ``pour_bottle`` : pick bottle, pour at target, place back.
  - ``rotate_tray`` : bimanual non-prehensile rotation via corner pushes.

Every task takes ``(backend, planner, scene)`` and returns a
``PrimitiveResult`` (success/failure from the last primitive that ran, or
aggregate success if all passed). Tasks compose primitives with
short-circuit-on-failure — the moment any primitive returns
``success=False``, the task stops and bubbles that result up.
"""

from .primitives.base import PrimitiveResult
from .primitives.open_microwave import open_microwave
from .primitives.pick import pick
from .primitives.place import place
from .primitives.pour import pour
from .primitives.push import push


def plate_into_microwave(backend, planner, scene) -> PrimitiveResult:
    """Open microwave -> pick plate -> place inside -> close microwave.

    Arms:
      - ``ur_left`` operates the microwave door (hook gripper).
      - ``ur_right`` moves the plate (two-finger gripper).
    """
    # TODO: real implementation. Sketch:
    #   r = open_microwave(backend, planner, scene, arm="ur_left")
    #   if not r.success: return r
    #   r = pick(backend, planner, scene, object_name="plate_right", arm="ur_right")
    #   if not r.success: return r
    #   r = place(backend, planner, scene,
    #             target_pose=scene.microwave_interior_pose,
    #             arm="ur_right")
    #   if not r.success: return r
    #   r = push(backend, planner, scene, arm="ur_left", ...)   # close door
    #   return r
    raise NotImplementedError


def plate_out_of_microwave(backend, planner, scene) -> PrimitiveResult:
    """Open microwave -> pick plate from inside -> place on table -> close."""
    # TODO
    raise NotImplementedError


def pour_bottle(backend, planner, scene) -> PrimitiveResult:
    """Pick bottle -> move above target vessel -> pour -> place bottle back."""
    # TODO
    raise NotImplementedError


def rotate_tray(backend, planner, scene) -> PrimitiveResult:
    """Bimanual non-prehensile tray rotation.

    Both arms push opposite corners of the tray simultaneously. The net
    asymmetric force produces a torque about the tray's vertical axis and
    rotates it to the goal orientation. Neither arm grasps — this is pure
    force-mode pushing.

    Implementation note: the push calls may need to be coordinated across
    arms (start together, terminate on shared angle condition). That is
    task-level logic, not primitive-level — if the shared condition grows
    beyond a few lines, lift it into a small helper here.
    """
    # TODO
    raise NotImplementedError
