---
name: Planning stack architecture
description: build_scene → plan_transit → execute_plan flow, key parameters, and scene configuration options
type: reference
---

**Scene building**: `build_scene(meshcat=None, microwave_door_open_angle_rad=..., skip_static_objects=(...), attached_objects=(...))` in `planning/build_scene.py`. Door angle is static — welded at build time. `attached_objects` welds collision geometry to gripper for held-object planning.

**Planning**: `plan_transit(plant, arm, waypoints, current_q={...}, plant_context=...)` in `planning/transit.py`. Waypoints are task-frame TCP Poses. `current_q` accepts a dict keyed by arm name (natural from RTDE). Routes to simple cubic spline first; falls back to KTO only when spline fails safety checks or shape constraints are requested.

**Execution**: `execute_plan(plan, session, method='moveJ_path')` in `planning/execute.py`. Sparse-samples trajectory, sends as blended `moveJ(path=...)`. Needs `Session` object (not just ArmHandle) because it looks up `session.arms[plan.arm]`.

**Sim HOME**: `planning/__init__.py` defines `SIM_HOME_Q_LEFT/RIGHT` — decoupled from `calibration.HOME_Q_RAD_*`. Use sim HOME for the partner arm when it's not in the RTDE session.
