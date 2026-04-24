"""Surface-seeking descent for place() (task-frame velocity).

Commands a Cartesian velocity (expressed in task frame) and relies on the
UR controller to stop when TCP force crosses ``threshold``. Replaces a
hand-tuned final-descent Z with something robust to surface-height error.

``v_task`` is a 6-vector: linear (x, y, z in task frame, m/s) + angular
(rx, ry, rz, rad/s). Typical use for place():
    v_task = [0, 0, -0.02, 0, 0, 0]   # 2 cm/s downward in task frame

The linear and angular parts are rotated into the arm's base frame before
the RTDE call (velocities are rotated only — no translation component).
"""

from typing import Sequence

from ..arm import ArmHandle


def move_until_contact(
    arm: ArmHandle,
    v_task: Sequence[float],
    accel: float,
    threshold: float,
) -> None:
    v_base = arm.task_velocity_to_base(v_task)
    arm.control.moveUntilContact(v_base, accel, threshold)
