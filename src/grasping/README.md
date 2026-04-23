# grasping/

Per-object grasp poses. Today: hand-edited table in `candidates.py`.
Tomorrow (if we need it): perception-driven grasp generation lives here
too, behind the same `get_grasp_candidates` entry point.

## Contract

```python
from src.grasping import get_grasp_candidates

gtype = backend.gripper(arm).type_name  # e.g. "hook" or "robotiq_2f85"
candidates = get_grasp_candidates("bowl", gripper_type=gtype, scene=scene)
# -> list[RigidTransform], each X_OG (gripper pose in the object frame),
#    in preferred order for the pick primitive to try.
```

Gripper-awareness matters because a hook-latch pose and a two-finger
pinch are different grasp geometries on the same object — the bowl is
hooked under the rim (`"hook"`), while the plate is pinched at the rim
(`"robotiq_2f85"`). The table's key is `(object_name, gripper_type)`.

The `pick` primitive iterates the returned list, runs IK + collision
check per candidate against the live plant, and commits to the first
feasible one. That means the static table only needs to enumerate
*nominal* valid grasps per (object, gripper) pair — scene-specific
filtering happens for free at pick time.

## Files

| File | What it does |
|---|---|
| `candidates.py` | Hand-edited `X_OG` table per object, helper functions for common grasp types (top-down, side-ring), and the `get_grasp_candidates` dispatch. |

## Adding a new graspable object

1. Write a helper (or reuse `_top_down` / `_side` / `_hook_latch`) that
   returns `list[RigidTransform]` for the object, for ONE gripper type.
   If the object is grasped by both gripper kinds, write two helpers.
2. Register in `_GRASP_TABLE` with a `(object_name, gripper_type)` key.
   `object_name` must match the scenario YAML; `gripper_type` must match
   the string set on the corresponding `Gripper` subclass.
3. Visually sanity-check in Meshcat: draw each candidate pose on the
   object as an axis triad and confirm the approach direction is sane.

## Migration path to perception-driven grasping

When the static table stops covering the scenes we care about (heavy
clutter on the plate, deformed bottles, novel objects), swap the
implementation of `get_grasp_candidates` without touching callers:

- **Step 1 (primitive fitting)**: segment the object from a depth image,
  fit a cylinder/box/plane, choose grasps from a small library based on
  the fit. Moderate complexity, no ML.
- **Step 2 (learned detection)**: run something like Contact-GraspNet or
  AnyGrasp on the depth image and return its top-K ranked grasps. Handles
  novelty/clutter/deformation, but wiring the model + perception pipeline
  is non-trivial.

Both implementations keep the same signature:
`get_grasp_candidates(object_name, scene) -> list[RigidTransform]`.
