"""Task sequencer: composes primitives into full tasks.

A *task* is a finite-state sequence over primitives. For class scope we
keep each task as a straight Python function with early-return on failure
— no behavior trees, no symbolic planning, no explicit FSM class. If a
task grows non-trivial retry / replanning logic, lift that specific task
into an FSM at that point; do not pre-build the machinery.

The primary deliverable is ``full_recipe`` — the 8-step demo that moves
items onto the microwave, heats a bowl, cooks a plate, pours + stirs a
drink, and transports the tray. Each step is factored out as its own
sub-task so they can be unit-tested in isolation on the rig before
stitching together.

Every task takes ``(backend, planner, scene)`` and returns a
``PrimitiveResult``. Tasks compose primitives with short-circuit-on-
failure — the moment any primitive returns ``success=False``, the task
stops and bubbles that result up. Primitives that return useful data
(e.g. ``pick`` returns the X_WG actually used) stash it in the result's
``data`` dict for downstream steps to read.
"""

from typing import Optional

from .primitives.base import PrimitiveResult
from .primitives.bimanual.carry_tray import carry_tray
from .primitives.bimanual.pour_stabilized import pour_stabilized
from .primitives.drag import drag
from .primitives.open_microwave import close_microwave, open_microwave
from .primitives.pick import pick
from .primitives.place import place
from .primitives.press_button import press_button
from .primitives.push import push
from .primitives.stir import stir


# ----- helpers ------------------------------------------------------------

def _chain(*steps) -> PrimitiveResult:
    """Run primitives in order, short-circuiting on the first failure.

    Each ``step`` is a zero-arg lambda returning a ``PrimitiveResult``.
    Keeps the recipe functions readable by factoring out the
    ``if not r.success: return r`` boilerplate."""
    for step in steps:
        r = step()
        if not r.success:
            return r
    return PrimitiveResult(success=True)


# ----- individual sub-tasks ----------------------------------------------

def stage_drink_items_on_microwave(backend, planner, scene) -> PrimitiveResult:
    """Step 1: move cup, bottle, stirrer from their initial positions to
    the top of the microwave (clears counter for the heating steps)."""
    return _chain(
        lambda: pick(backend, planner, scene, "cup", arm="ur_right"),
        lambda: place(backend, planner, scene,
                      scene.named_poses["microwave_top_cup"], arm="ur_right"),
        lambda: pick(backend, planner, scene, "bottle", arm="ur_right"),
        lambda: place(backend, planner, scene,
                      scene.named_poses["microwave_top_bottle"], arm="ur_right"),
        lambda: pick(backend, planner, scene, "stirrer", arm="ur_right"),
        lambda: place(backend, planner, scene,
                      scene.named_poses["microwave_top_stirrer"], arm="ur_right"),
    )


def heat_bowl(backend, planner, scene) -> PrimitiveResult:
    """Steps 2-3: open microwave with hook, place bowl (hook), close door,
    press heat + start, open, retrieve bowl, place on tray."""
    return _chain(
        lambda: open_microwave(backend, planner, scene, arm="ur_left"),
        lambda: pick(backend, planner, scene, "bowl", arm="ur_left"),  # hook
        lambda: place(backend, planner, scene,
                      scene.named_poses["microwave_interior"], arm="ur_left"),
        lambda: close_microwave(backend, planner, scene, arm="ur_left"),
        lambda: press_button(backend, planner, scene, arm="ur_right",
                             button_pose=scene.named_poses["microwave_button_heat"]),
        lambda: press_button(backend, planner, scene, arm="ur_right",
                             button_pose=scene.named_poses["microwave_button_start"]),
        # TODO: Wait(heat_duration) — need a "wait seconds" sequencer step.
        lambda: open_microwave(backend, planner, scene, arm="ur_left"),
        lambda: pick(backend, planner, scene, "bowl", arm="ur_left"),
        lambda: place(backend, planner, scene,
                      scene.named_poses["tray_bowl_slot"], arm="ur_left"),
    )


def cook_plate(backend, planner, scene) -> PrimitiveResult:
    """Steps 4-5: hook opens microwave, two-finger places plate in, hook
    closes door. Then run (press heat/start + cook). Then hook opens
    again; if the plate sits too far back for the two-finger arm to grasp
    cleanly, hook drags it forward a few cm. Two-finger picks plate and
    places on tray."""
    return _chain(
        lambda: open_microwave(backend, planner, scene, arm="ur_left"),
        lambda: pick(backend, planner, scene, "plate_8in", arm="ur_right"),
        lambda: place(backend, planner, scene,
                      scene.named_poses["microwave_interior"], arm="ur_right"),
        lambda: close_microwave(backend, planner, scene, arm="ur_left"),
        lambda: press_button(backend, planner, scene, arm="ur_right",
                             button_pose=scene.named_poses["microwave_button_heat"]),
        lambda: press_button(backend, planner, scene, arm="ur_right",
                             button_pose=scene.named_poses["microwave_button_start"]),
        # TODO: Wait(cook_duration)
        lambda: open_microwave(backend, planner, scene, arm="ur_left"),
        # Optional drag — conditionally executed in the body below instead
        # of in _chain (needs a runtime reachability check).
        lambda: _maybe_drag_plate_forward(backend, planner, scene),
        lambda: pick(backend, planner, scene, "plate_8in", arm="ur_right"),
        lambda: place(backend, planner, scene,
                      scene.named_poses["tray_plate_slot"], arm="ur_right"),
    )


def _maybe_drag_plate_forward(backend, planner, scene) -> PrimitiveResult:
    """Step 5 conditional: check if the two-finger arm can reach the plate
    at its current in-microwave pose. If not, use the hook to drag it
    forward ~5 cm. If it can reach already, skip and return success.

    Implementation: IK-check a candidate 2-finger grasp against the live
    plant state; if infeasible, run ``drag``."""
    # TODO: use solve_ik(plant, plate_grasp_pose, q_seed=right arm q) to
    # test reachability. Only call drag() if IK fails.
    return PrimitiveResult(success=True, reason="drag check not implemented")


def pour_and_stir_drink(backend, planner, scene) -> PrimitiveResult:
    """Steps 6-7: move cup/bottle/stirrer down from microwave top, pour
    (bimanual: hook holds cup, two-finger pours bottle), stir with
    two-finger, place cup on tray."""
    return _chain(
        # Down from microwave top
        lambda: pick(backend, planner, scene, "cup", arm="ur_right"),
        lambda: place(backend, planner, scene,
                      scene.named_poses["counter_cup"], arm="ur_right"),
        lambda: pick(backend, planner, scene, "bottle", arm="ur_right"),
        # Hook latches the cup for the pour
        lambda: pick(backend, planner, scene, "cup", arm="ur_left"),
        # Bimanual pour
        lambda: pour_stabilized(backend, planner, scene,
                                pour_arm="ur_right",
                                holder_arm="ur_left",
                                pour_origin=scene.named_poses["cup_pour_origin"].translation(),
                                pour_axis=(0, 1, 0)),
        # Bottle back to counter (or to microwave_top), hook releases cup
        lambda: place(backend, planner, scene,
                      scene.named_poses["counter_bottle"], arm="ur_right"),
        # Stir
        lambda: pick(backend, planner, scene, "stirrer", arm="ur_right"),
        lambda: stir(backend, planner, scene, arm="ur_right",
                     cup_origin=scene.named_poses["cup_inside_bottom"].translation(),
                     cup_axis=(0, 0, 1)),
        lambda: place(backend, planner, scene,
                      scene.named_poses["counter_stirrer"], arm="ur_right"),
        # Cup onto tray
        lambda: pick(backend, planner, scene, "cup", arm="ur_right"),
        lambda: place(backend, planner, scene,
                      scene.named_poses["tray_cup_slot"], arm="ur_right"),
    )


def transport_tray(backend, planner, scene) -> PrimitiveResult:
    """Step 8: bimanual rigid-grasp + carry to the tray's final location
    with bounded tilt to avoid spillage.

    Alternative, kept around because the user flagged "maybe both":
    corner-push in-place rotation via ``push`` on each arm. Which one the
    task needs depends on whether the final location requires translation
    (carry) or just a rotation (push). Default here is carry; flip to
    push-rotate by swapping the body of this function."""
    return _chain(
        # Both arms grasp tray handles (one per arm).
        lambda: pick(backend, planner, scene, "tray", arm="ur_left"),
        lambda: pick(backend, planner, scene, "tray", arm="ur_right"),
        # Coordinated carry.
        lambda: carry_tray(backend, planner, scene,
                            target_pose=scene.named_poses["tray_final"],
                            left_arm="ur_left",
                            right_arm="ur_right"),
    )


# ----- top-level recipe ---------------------------------------------------

def full_recipe(backend, planner, scene) -> PrimitiveResult:
    """The 8-step demo end-to-end. Short-circuits on first sub-task failure
    so a partial run tells you exactly where it stopped."""
    return _chain(
        lambda: stage_drink_items_on_microwave(backend, planner, scene),
        lambda: heat_bowl(backend, planner, scene),
        lambda: cook_plate(backend, planner, scene),
        lambda: pour_and_stir_drink(backend, planner, scene),
        lambda: transport_tray(backend, planner, scene),
    )


# ----- standalone tasks (kept for testing / isolation) -------------------

def rotate_tray_corner_push(backend, planner, scene) -> PrimitiveResult:
    """Alternative to ``transport_tray`` — in-place rotation using
    asymmetric corner pushes, non-prehensile. Kept because step 8 may
    actually need this instead of (or in addition to) the rigid carry."""
    # TODO: parallel ``push`` on left_arm + right_arm with opposite
    # directions. Simultaneity is wall-clock only for now; if we need
    # shared tray-angle sensing, promote to primitives/bimanual/.
    raise NotImplementedError
