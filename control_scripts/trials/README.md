# Autonomous Trials

Edit [`control_scripts/trials/definitions.py`](/Users/gdmen/MIT/sp26/2.12/212_UR5/control_scripts/trials/definitions.py) to define:

- safe `home` joint targets
- task-frame waypoints
- workspace limits
- keep-out boxes
- the step sequence to execute

Run commands from the repo root:

```bash
python3 run.py list-trials
python3 run.py trial --trial right_task_frame_probe
python3 run.py trial --trial right_task_frame_probe --live --state-only
python3 run.py trial --trial right_task_frame_probe --live --arms ur_right
```

Notes:

- Dry-run mode does not import RTDE or connect to hardware.
- Waypoints are in the shared task frame, not robot-base coordinates.
- The runner collision-checks straight `moveL` segments against the task-frame workspace box and any keep-out boxes you define.
- Joint moves (`moveJ`) are not collision-checked. Treat the configured `home` joint target as a trusted safe seed.
- If you want to promote a grasp pose from `control_scripts/grasps/*` into this runner, convert that `Pose` with `TaskWaypoint.from_pose(...)` and add it to the `waypoints` table.
- Manual teleop can reuse the same workspace/keep-out geometry via `python3 run.py manual --safety-trial right_task_frame_probe`.
