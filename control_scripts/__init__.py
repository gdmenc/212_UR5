"""RTDE-only pick-and-place control scripts.

Deliberately separate from ``src/``: this package has NO pydrake dependency
and runs natively on any machine that can install ``ur_rtde`` + numpy + scipy.
Intended as the real-robot execution path while the full Drake sim path in
``src/`` remains in development.

Layout
------
control_scripts/
    config.py              Tunables (TRANSIT_Z, PREGRASP_OFFSET, speeds).
    arm.py                 ArmHandle — rtde_c + rtde_r + gripper + calibration.
    calibration.py         X_LEFT_BASE_TASK, X_RIGHT_BASE_TASK, TCP offsets.
    util/
        poses.py           Pose (translation + scipy Rotation), composition.
        frames.py          task_to_base / base_to_task conversion helpers.
        rtde_convert.py    Pose <-> UR axis-angle list.
    moves/                 One atomic move per file; each a thin RTDE wrapper
                           that converts task -> base before the controller call.
        lift_to_transit.py
        transit_xy.py
        approach_to.py     (formerly descend_to — 'approach' is direction-agnostic)
        retract_to.py
        move_until_contact.py
    pick.py                Composes the modular moves into a full pick.
    place.py               Composes the modular moves into a full place.

Frame convention
----------------
All primitive/move Pose arguments are in **task frame**. Each move converts
to the arm's base frame at the RTDE boundary via ``arm.to_base()``. This
means the caller reasons in one shared, human-friendly frame (table-top
center, z up) regardless of which arm is driving the motion.
"""
