"""Edit this file to define tomorrow's autonomous path trials.

Start with `RIGHT_TASK_FRAME_PROBE` below. The main things to edit are:
    1. `joint_targets["home"]`       safe initial joint configuration
    2. `waypoints[...]`               task-frame XYZ points (and optional rotvec)
    3. `workspace_limits`             allowed XYZ box in task frame
    4. `keepout_boxes`                obstacles to stay out of
    5. `sequence`                     the exact order of steps to run

Frame convention
----------------
All waypoints live in the shared TASK frame from `control_scripts/calibration.py`:
    - origin: top center of table
    - +x: toward the right arm
    - +y: toward the microwave
    - +z: upward from the tabletop

That means a path can be defined as human-readable tabletop points instead of
per-robot base-frame coordinates.
"""

from __future__ import annotations

import math

from ..calibration import TCP_OFFSET_ROBOTIQ_2F85, X_RIGHT_BASE_TASK
from .models import (
    BoxRegion,
    JointTarget,
    TaskWaypoint,
    TrialDefinition,
    move_home,
    move_l,
    report_state,
    wait_seconds,
)


# Safe joint seed copied directly from `ur_2026/forcemode_dual_arm_mount_example.py`.
# Keep the wrap as-written until you have a measured alternative on the real arm.
RIGHT_HOME_2026_FORCE_MODE_DEG = [-312.46, -69.69, 101.25, -87.30, -239.69, -135.00]

# Straight-down tool orientation in TASK frame, also from the 2026 dual-arm example.
TOOL_DOWN_ROT_TASK = [0.0, math.pi, 0.0]


RIGHT_TASK_FRAME_PROBE = TrialDefinition(
    name="right_task_frame_probe",
    description=(
        "Right-arm dry-run/live probe path for validating task-frame waypoints, "
        "home pose, and simple keep-out logic before attempting grasps."
    ),
    arm="ur_right",
    ip="192.168.1.102",
    x_base_task=X_RIGHT_BASE_TASK,
    tcp_offset=TCP_OFFSET_ROBOTIQ_2F85,
    gripper=None,
    default_tool_rotvec_task=TOOL_DOWN_ROT_TASK,
    joint_targets={
        "home": JointTarget(
            name="home",
            joints_deg=RIGHT_HOME_2026_FORCE_MODE_DEG,
            speed=1.05,
            accel=1.4,
        ),
    },
    waypoints={
        # First point should usually include an explicit orientation so the rest
        # of the path can omit rotvec and inherit it.
        "origin_hover": TaskWaypoint(
            name="origin_hover",
            xyz=[0.00, 0.00, 0.18],
            rotvec=TOOL_DOWN_ROT_TASK,
        ),
        "grasp_pre_align": TaskWaypoint(
            name="grasp_pre_align",
            xyz=[0.10, 0.02, 0.18],
        ),
        "grasp_probe": TaskWaypoint(
            name="grasp_probe",
            xyz=[0.10, 0.02, 0.10],
        ),
        "front_right_hover": TaskWaypoint(
            name="front_right_hover",
            xyz=[0.14, 0.18, 0.18],
        ),
        "front_left_hover": TaskWaypoint(
            name="front_left_hover",
            xyz=[-0.14, 0.18, 0.18],
        ),
        "retreat_high": TaskWaypoint(
            name="retreat_high",
            xyz=[0.00, 0.00, 0.24],
        ),
    },
    workspace_limits=BoxRegion(
        name="right_probe_workspace",
        xyz_min=[-0.22, -0.05, 0.08],
        xyz_max=[0.22, 0.26, 0.28],
    ),
    keepout_boxes=[
        # Add obstacles here in task-frame coordinates.
        # Example:
        # BoxRegion(
        #     name="camera_post",
        #     xyz_min=[-0.03, 0.12, 0.00],
        #     xyz_max=[0.03, 0.18, 0.35],
        # ),
    ],
    sequence=[
        report_state("startup"),
        move_home(),
        report_state("after_home"),
        move_l("origin_hover"),
        move_l("grasp_pre_align"),
        move_l("grasp_probe", speed=0.05, accel=0.15),
        wait_seconds(0.5, label="pause_at_probe"),
        move_l("grasp_pre_align"),
        move_l("front_right_hover"),
        move_l("front_left_hover"),
        move_l("retreat_high"),
        move_l("origin_hover"),
        report_state("final"),
    ],
)


TRIALS = {
    RIGHT_TASK_FRAME_PROBE.name: RIGHT_TASK_FRAME_PROBE,
}


DEFAULT_TRIAL = RIGHT_TASK_FRAME_PROBE.name
