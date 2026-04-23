# primitives/bimanual/

Primitives that drive both arms simultaneously with shared state or a
shared reference frame. Live in a subfolder (not alongside single-arm
primitives) because the signature shape is different — two arm names
instead of one, and the caller is expected to have brought both arms
into a consistent precondition state before calling.

## When something needs to be a bimanual primitive

Three tests. If any apply, single-arm primitives called in sequence
won't cut it:

1. **Both arms must be active during the same time window.** E.g., hook
   holds the cup *while* the pour is in progress.
2. **Coordinated world-frame target.** E.g., carry-tray — both arms move
   so the tray reaches a specified pose with bounded tilt; each arm's
   trajectory is derived from the shared tray pose, not planned
   independently.
3. **Force / torque coupling between the arms.** E.g., the tray-rotation
   corner-push task (lives in single-arm `push.py` today but is actually
   a coordination problem if it ever grows timing constraints).

## Files

| File | What it does |
|---|---|
| `carry_tray.py` | Rigid bimanual carry. Both arms grasp handles; tray travels to `target_pose` with `tilt_bound` radians of world-up. |
| `pour_stabilized.py` | Hook holds cup, two-finger pours bottle. Bimanual because the hold must run concurrently with the pour. |

## Not here (and why)

- **Corner-push tray rotation** stays in `primitives/push.py` for now
  because each arm runs an independent `push` with symmetric force; the
  only "coordination" is starting both at the same wall-clock time.
  Promote if we add a termination condition that depends on shared
  tray-angle sensing.
- **Dual pick** — calling `pick` on each arm in sequence is fine;
  there's nothing shared between them.
