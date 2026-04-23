# 2.12 Final Project — Bimanual UR5e

Autonomous manipulation on a two-arm UR5e rig. The same Python code drives
both a Drake simulator and the real robots over RTDE, so we can prototype
in sim and run on hardware without rewriting the task logic.

**Target tasks (end-to-end drink + meal recipe)**

1. Move cup, bottle, and stirrer from their initial positions onto the
   top of the microwave (clears counter for heating).
2. Open microwave with the hook; prepare the hook arm to pick up the bowl.
3. Heat bowl — two-finger presses button on microwave panel. Place bowl
   on the tray. Prepare two-finger arm to pick up plate.
4. Hook opens microwave again; two-finger places the plate in; hook
   closes the door.
5. If the plate sits too far back for the two-finger arm to grasp, the
   hook drags it forward. Two-finger picks plate and places on the tray.
6. Move cup, bottle, stirrer down from microwave top. **Bimanual pour**:
   hook steadies the cup while the two-finger arm pours the bottle.
7. Two-finger stirs the drink, then places the cup on the tray.
8. **Bimanual tray transport** to the final location, keeping the tray
   level so nothing spills. (Corner-push in-place rotation is a backup
   if we need pure rotation, not translation.)

The `task_sequencer.py` deliverable is `full_recipe()` which stitches
these together; each step is also its own sub-task function so they are
testable in isolation on the rig.

**Hardware**

- Two UR5e's: left arm IP `192.168.1.101`, right arm `192.168.1.102`.
- Left arm carries a custom hook end-effector (for the microwave handle
  and tray edges). Right arm carries a Robotiq 2F-85 two-finger.
- Top-down and front-facing cameras (RealSense) for perception.

## Architecture

```
src/
├── scene.py                  Build the Drake diagram (plant, controllers,
│                             trajectory source, meshcat). Runnable as
│                             `python -m src.scene` for a visual smoke test.
├── segments.py               Dataclasses (MoveJ, MoveL, ServoStream,
│                             ForceMode, ...) — the command type primitives
│                             send to backends.
├── task_sequencer.py         One function per high-level task; composes
│                             primitives with short-circuit on failure.
│
├── primitives/               Task-level motion primitives.
│   ├── pick.py               approach, grasp, lift
│   ├── place.py              approach, descend until contact, release
│   ├── push.py               compliant Cartesian push (corner-push tray)
│   ├── pour.py               rotate TCP about a fixed axis
│   ├── stir.py               circular sweep inside the cup
│   ├── drag.py               hook variant of push (pull a rim toward the arm)
│   ├── press_button.py       contact + brief force hold (microwave panel)
│   ├── open_microwave.py     hook handle, sweep door arc (open & close)
│   └── bimanual/             two-arm coordinated primitives
│       ├── carry_tray.py     both arms grasp handles, carry level
│       └── pour_stabilized.py  hook holds cup while two-finger pours
│
├── planners/                 Trajectory planning + IK.
│   ├── trajopt.py            primary: B-spline trajectory optimization
│   ├── rrt_connect.py        fallback: RRT-Connect for narrow passages
│   ├── ik.py                 pose -> joint config
│   ├── diff_ik.py            Cartesian-velocity streaming
│   └── warmstart.py          LRU cache for trajopt initial guesses
│
├── grasping/                 Per-object grasp candidates.
│   └── candidates.py         static X_OG table; swappable for perception
│                             later behind the same entry point.
│
├── grippers/                 End-effector drivers.
│   ├── robotiq_2f85.py       real: TCP socket port 63352; sim: SDF + joints
│   └── hook_gripper.py       custom active hook; real actuation TBD
│
├── execution/                The sim / real seam.
│   ├── backend.py            Protocol: execute(segment), state(), gripper()
│   ├── sim_backend.py        drives a Drake Simulator
│   ├── real_backend.py       drives two UR5e's over RTDE
│   └── sim_controllers.py    sim-only joint controller factories
│                             (consumed by scene.py at diagram build)
│
└── assets/                   URDFs, SDFs, meshes.
```

Each subfolder has its own `README.md` with interface details.

## Data flow

```
task_sequencer.task()
    -> primitive(backend, planner, scene, ...)
        -> solve_ik / plan_trajopt         (planners/)
        -> backend.execute(arm, Segment)   (execution/)
            -> sim:  Drake Simulator step(s)
            -> real: RTDE call(s)
    -> Gripper.grasp() / .open() via backend.gripper(arm)  (grippers/)
```

Primitives are **imperative**: they call the backend directly and branch on
runtime state (grasp success, contact force). They don't return a
precomputed list of motions — force-mode tasks like tray rotation need a
live loop that static segment lists can't express.

`Segment`s remain the command type at the primitive/backend boundary, so
the sim-vs-real swap is still a one-line config flip.

## Running

Visual scene check (no primitives, no backend):
```
python -m src.scene
```
opens Meshcat in the browser. Edit the YAML or any asset, rerun, refresh
the browser tab.

Single-asset iteration (faster than rebuilding the full scene):
```python
from pydrake.visualization import ModelVisualizer
viz = ModelVisualizer()
viz.parser().AddModels("src/assets/hook_gripper/hook_gripper.sdf")
viz.Run()
```

## Conventions

- **Arm names**: `"ur_left"`, `"ur_right"` throughout. Defined from the
  *robot's* perspective, not the viewer's (matches `ur_2026/` examples).
- **Joint order for each arm**: `[shoulder_pan, shoulder_lift, elbow,
  wrist_1, wrist_2, wrist_3]`. 6 DOF per arm.
- **TCP offset**: 0.174 m along the flange Z for the Robotiq 2F-85.
- **Poses**: `pydrake.math.RigidTransform` in code; convert to RTDE
  `[x, y, z, rx, ry, rz]` axis-angle at the real-backend dispatch boundary.

## References

- Reference real-robot code: `ur_2026/object_grasp_example.py` (pick and
  place with RTDE + Robotiq socket). Keep its patterns in mind when
  filling in `RealBackend`.
- Drake docs: <https://drake.mit.edu/pydrake/>
- RTDE Python docs: <https://sdurobotics.gitlab.io/ur_rtde/>
