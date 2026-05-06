---
name: Drake scene context wiring
description: Critical pattern for getting collision-checking-capable plant context from build_scene
type: reference
---

`build_scene()` returns `SceneHandles` with `.plant`, `.diagram`, `.scene_graph`.

For collision queries to work, the plant context MUST come from the diagram context:
```python
diagram_ctx = scene.diagram.CreateDefaultContext()
plant_ctx = scene.plant.GetMyMutableContextFromRoot(diagram_ctx)
```

`plant.CreateDefaultContext()` produces a standalone context with no SceneGraph port wiring — `plant.get_geometry_query_input_port().Eval(ctx)` throws RuntimeError, and `_check_collision_along` in transit.py returns `nan` clearance silently.

`plan_transit()` accepts `plant_context=` kwarg. If None, it calls `plant.CreateDefaultContext()` which hits this bug. Always pass the diagram-routed context.
