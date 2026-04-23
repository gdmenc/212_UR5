"""Press-button primitive: push along a target's outward normal.

Used for the microwave control panel: approach the button along its
outward normal, contact the surface, apply a brief force to register the
press, and retract. A single-shot primitive — if the task needs a sequence
of presses (set time, set power, start), the sequencer calls this once
per button.

Position control would slam against the panel under the slightest
calibration error. Using ``MoveUntilContact`` to find the surface, then a
short ``ForceMode`` hold, puts a bound on the peak force regardless of
how wrong the nominal button pose is.

High-level flow:

    X_WApproach = X_WButton * [0, 0, -standoff]     # back off along -press_axis
    q_approach  = solve_ik(plant, X_WApproach)
    backend.execute(arm, ServoStream(sample(plan_trajopt(...))))
    backend.execute(arm, MoveUntilContact(v_base=along_press_axis, threshold=low))
    backend.execute(arm, ForceMode(button_frame, sel=[0,0,1,0,0,0],
                                    wrench=[0,0,press_force,0,0,0], ...))
    backend.execute(arm, Wait(0.2))                  # hold, let click register
    backend.execute(arm, MoveL(X_WApproach))         # retract
"""

from typing import Optional

from pydrake.math import RigidTransform

from .base import PrimitiveResult


def press_button(
    backend,
    planner,
    scene,
    arm: str,
    button_pose: RigidTransform,  # +Z points OUT of the panel at the button
    press_force: float = 8.0,      # N; microwave membrane buttons need ~3-10 N
    press_depth: float = 0.005,    # m; hard limit on penetration
    approach_standoff: float = 0.05,
    hold_time: float = 0.2,
) -> PrimitiveResult:
    """Approach along -Z of ``button_pose``, contact, press, hold, retract."""
    # TODO
    raise NotImplementedError
