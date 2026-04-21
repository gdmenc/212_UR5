# planners/

Trajectory planners and inverse-kinematics helpers. Primitives call into
this folder to turn goal poses into joint trajectories the backend can
execute.

## Files

| File | What it does | When to use |
|---|---|---|
| `base.py` | `Planner` protocol. One method: `plan(plant, q_start, q_goal, duration) -> BsplineTrajectory \| None`. | — |
| `trajopt.py` | `plan_trajopt(...)`. Kinematic trajectory optimization; smooth, fast with warm start. | Default for free-space transits. |
| `rrt_connect.py` | `plan_rrt_connect(...)`. Sampling-based, probabilistically complete. | Fallback when trajopt fails (narrow passages — microwave reach-through). |
| `ik.py` | `solve_ik(...)`. Pose-to-config solver with optional collision avoidance. | Any time a primitive has a Cartesian goal pose and needs a joint config. |
| `diff_ik.py` | `DiffIK(plant, arm).step(V_WTcp, dt)`. Cartesian-velocity streaming. | Pre-grasp approach, pour-arc tracking, Cartesian push. |
| `warmstart.py` | Tiny LRU cache of solved B-spline control points keyed on problem inputs. | Enable once trajopt solve time is a bottleneck; skip for class scope. |

## Typical primitive call pattern

```python
# 1. Resolve Cartesian goal -> joint config.
q_goal = solve_ik(plant, X_WPreGrasp, q_seed=scene.q(arm))

# 2. Try trajopt first.
cps = warmstart.lookup(q_start, q_goal, scene.hash())
traj = plan_trajopt(plant, q_start, q_goal, warmstart_cps=cps)

# 3. Fall back to RRT if trajopt fails.
if traj is None:
    traj = plan_rrt_connect(plant, q_start, q_goal)

# 4. Stream the joint trajectory via ServoStream segment.
backend.execute(arm, ServoStream(q_traj=sample(traj, dt=0.008)))
```

## Not included (by design)

- **GCS (Graphs of Convex Sets)**. Overkill for class scope, would be the
  next upgrade past trajopt+RRT if we ever need it.
- **Task-and-motion planning**. The `task_sequencer.py` is a flat Python
  script per task; we do not do symbolic task planning.
