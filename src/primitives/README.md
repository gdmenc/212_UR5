# primitives/

Task-level motion primitives. Each is an imperative function that reads the
scene, plans the motion, and drives the backend.

## Contract

```
primitive(backend, planner, scene, *args, **kwargs) -> PrimitiveResult
```

- `backend` — a `Backend` protocol instance (sim or real). Used to execute
  `Segment`s and read live state (joint positions, TCP wrench, gripper).
- `planner` — trajectory planner instance (typically `plan_trajopt` with
  `plan_rrt_connect` fallback).
- `scene` — the `Scene` dataclass from `src/scene.py`. Provides the plant,
  current joint positions, and object-pose lookups.

All primitives return a `PrimitiveResult(success, reason, data)`. The
sequencer branches on `success`; `data` carries primitive-specific payload
(e.g. `pick` returns the actual grasp pose used so `place` can plan a
consistent release).

## Why imperative, not segment-returning

- Grasp primitives need retry logic across multiple grasp candidates.
- Force-mode primitives (push, open_microwave) need a live loop that
  monitors wrench/travel and terminates on condition, not on a fixed time.
- Static `list[Segment]` can't express either. Imperative primitives can.

`Segment`s are still the command type sent to the backend — the primitive
chooses which to emit each step.

## Files

Single-arm:

| File | Primitive | Used by tasks |
|---|---|---|
| `base.py` | `PrimitiveResult` dataclass | all |
| `pick.py` | `pick(object_name, arm, ...)` | every pick-bearing step |
| `place.py` | `place(target_pose, arm, ...)` | every placement (microwave, tray, counter) |
| `push.py` | `push(arm, push_frame, direction, ...)` | corner-push tray rotation |
| `pour.py` | `pour(arm, pour_origin, pour_axis, ...)` | single-arm pour (fallback; bimanual version in `bimanual/`) |
| `stir.py` | `stir(arm, cup_origin, cup_axis, ...)` | drink-stir |
| `drag.py` | `drag(arm, start_pose, direction, travel)` | hook drags plate forward out of microwave |
| `press_button.py` | `press_button(arm, button_pose, force, ...)` | microwave panel (heat / start) |
| `open_microwave.py` | `open_microwave(arm)`, `close_microwave(arm)` | both door operations share a swept-arc core |

Bimanual (see `bimanual/README.md`):

| File | Primitive | Used by tasks |
|---|---|---|
| `bimanual/carry_tray.py` | `carry_tray(target_pose, left_arm, right_arm, tilt_bound, ...)` | final tray transport |
| `bimanual/pour_stabilized.py` | `pour_stabilized(pour_arm, holder_arm, ...)` | drink pour where hook steadies cup |

## Adding a new primitive

1. Write a new `src/primitives/<name>.py` with a single function following
   the contract.
2. Add a module docstring describing the flow in pseudocode (don't
   pseudocode in the body — just in the docstring, like the existing files).
3. Register in `__init__.py`'s `__all__`.
4. If the primitive needs scene state the scene doesn't expose yet,
   extend the `Scene` dataclass (don't reach into the plant directly from
   primitives).

## Related

- **Grasp candidates** used by `pick` (and optionally `place` for pose
  consistency) live in `src/grasping/`, not in `scene`. Keeps the pick
  primitive agnostic to whether grasps come from a hand table today or a
  perception module tomorrow.
