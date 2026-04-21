# grippers/

End-effector drivers. Each subclass of `Gripper` wraps both a sim interface
(Drake plant + joint hooks) and a real interface (driver socket / GPIO), so
primitives can stay agnostic — they just call `backend.gripper(arm).grasp()`.

## Files

| File | What it is |
|---|---|
| `base.py` | `Gripper` ABC. Four methods: `open`, `close`, `grasp(force)`, `status()`. |
| `robotiq_2f85.py` | Robotiq 2F-85 two-finger. Real-side talks to the UR controller over TCP port 63352; sim-side not yet wired (scene currently has a WSG-50 on the right arm). |
| `hook_gripper.py` | Custom team-built active hook. Welded to the left arm in the current YAML. Real-side actuation is TBD — plug in under `open`/`close`. |

## Conventions

- One gripper instance per arm. The `backend` looks it up on each
  `GripperCommand` segment.
- Constructor takes **either** a real driver (`rtde_control=...`) **or** a
  sim plant (`sim_plant=..., sim_arm_name=...`), never both. Mixing is a
  configuration error.
- `grasp(force)` is the semantic "hold with N Newtons of force" command and
  returns `True` if an object was detected. `close()` just drives to the
  minimum aperture with no force target — useful for pre-homing.

## Adding a new gripper

1. Add a subclass of `Gripper` in a new file.
2. Implement the four methods for whichever side(s) you need.
3. Add the class to `__init__.py`'s `__all__`.
4. If sim, add the SDF/URDF under `src/assets/<name>/` and extend
   `scene.py` to weld it to the appropriate wrist frame.
