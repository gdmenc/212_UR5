"""Surface-seeking descent for place() / press / etc. (task-frame velocity).

Commands a Cartesian velocity (expressed in task frame) via ``speedL``
and polls the TCP force vector. Stops the motion when |F_xyz| crosses
``threshold`` Newtons.

Why not ``rtde_c.moveUntilContact``: the Python binding's signature is
``moveUntilContact(xd, direction=[0]*6, acceleration=1.2)`` — there is
no force-threshold parameter exposed. Its threshold is the controller's
internal default (typically conservative, ~10 N), which is too high for
low-force tasks like button presses where contact happens at 2-5 N. The
polling version below keeps the user-supplied threshold meaningful.

``v_task`` is a 6-vector: linear (x, y, z in task frame, m/s) + angular
(rx, ry, rz, rad/s). Typical use for place():
    v_task = [0, 0, -0.02, 0, 0, 0]   # 2 cm/s downward in task frame

The linear and angular parts are rotated into the arm's base frame
before the RTDE call (velocities are rotated only — no translation
component).

Force baseline: this primitive does NOT zero the F/T sensor before
moving. It assumes the controller's payload calibration is accurate
enough that the TCP wrench reads near zero at standstill. If the
held tool weighs enough that gravity bias triggers the threshold
prematurely, call ``arm.control.zeroFtSensor()`` once with the arm
at the standoff orientation BEFORE ``move_until_contact``.
"""

import time
from typing import Sequence

from ..arm import ArmHandle


def move_until_contact(
    arm: ArmHandle,
    v_task: Sequence[float],
    accel: float,
    threshold: float,
    poll_interval_s: float = 0.01,
    timeout_s: float = 30.0,
) -> None:
    """Drive at ``v_task`` (task frame) until |TCP force| crosses
    ``threshold`` Newtons. Raises RuntimeError on timeout (no contact
    in ``timeout_s`` seconds — surfaces configuration errors like the
    velocity pointing away from the surface).
    """
    v_base = arm.task_velocity_to_base(v_task)
    threshold_sq = float(threshold) ** 2

    arm.control.speedL(v_base, accel)
    t_start = time.time()
    try:
        while True:
            if time.time() - t_start > timeout_s:
                raise RuntimeError(
                    f"move_until_contact: no contact within {timeout_s:.1f} s "
                    f"(threshold={threshold} N). Check that v_task points "
                    f"TOWARD the surface and that the F/T baseline is reasonable."
                )
            f = arm.receive.getActualTCPForce()
            # |F_xyz|^2 — compare squared so we avoid the sqrt per loop.
            f_mag_sq = f[0] * f[0] + f[1] * f[1] + f[2] * f[2]
            if f_mag_sq >= threshold_sq:
                return
            time.sleep(poll_interval_s)
    finally:
        # Always stop the speed command on exit (success, timeout, or
        # exception). speedStop with the same accel decelerates cleanly.
        try:
            arm.control.speedStop(accel)
        except Exception:
            # Fallback if the binding version doesn't accept the accel arg.
            arm.control.speedStop()
