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

| File | Primitive | Used by tasks |
|---|---|---|
| `base.py` | `PrimitiveResult` dataclass | all |
| `pick.py` | `pick(object_name, arm, ...)` | plate to/from microwave, bottle for pour |
| `place.py` | `place(target_pose, arm, ...)` | plate into microwave, bottle back on table |
| `push.py` | `push(arm, push_frame, direction, ...)` | tray rotation (both arms), microwave door close |
| `pour.py` | `pour(arm, pour_origin, pour_axis, ...)` | bottle-pour task |
| `open_microwave.py` | `open_microwave(arm)` | plate-to-microwave task |

## Adding a new primitive

1. Write a new `src/primitives/<name>.py` with a single function following
   the contract.
2. Add a module docstring describing the flow in pseudocode (don't
   pseudocode in the body — just in the docstring, like the existing files).
3. Register in `__init__.py`'s `__all__`.
4. If the primitive needs scene state the scene doesn't expose yet,
   extend the `Scene` dataclass (don't reach into the plant directly from
   primitives).
