# execution/

The sim/real seam. Primitives emit `Segment`s; backends in this folder turn
them into actual motion.

## Files

| File | What it is |
|---|---|
| `backend.py` | `Backend` Protocol. Three methods: `execute(arm, segment)`, `state(arm)`, `gripper(arm)`. |
| `sim_backend.py` | Drives a Drake `Simulator` built by `scene.build_scene`. |
| `real_backend.py` | Drives two UR5e's over RTDE. Stub until we're on hardware. |
| `sim_controllers.py` | **Sim-only** joint controller factories (`make_inverse_dynamics_controller`, `make_joint_stiffness_controller`). Consumed by `scene.py` during diagram construction — not by `sim_backend` directly. |

Anything prefixed `sim_*.py` is a Drake-only building block; real hardware
never touches it. (Real UR5e's have their own cascade controller burned
into the UR control box; RTDE gives us joint targets in and nothing else.)

## Segment -> backend call mapping

Both backends dispatch on `isinstance` over the `Segment` union.

| Segment | Sim | Real |
|---|---|---|
| `MoveJ` | write `q_target` to controller's `desired_state` port, `AdvanceTo` until converged | `rtde_c.moveJ(q, speed, accel)` |
| `MoveL` | DiffIK integrate Cartesian goal -> joint targets | `rtde_c.moveL(pose, speed, accel)` |
| `ServoStream` | stream waypoints, `AdvanceTo(dt)` per step | tight 125 Hz loop of `rtde_c.servoJ(q, 0, 0, dt, lookahead, gain)` |
| `ForceMode` | Cartesian admittance sub-controller (not yet built) | `rtde_c.forceMode(frame, selection, wrench, type, limits)` |
| `MoveUntilContact` | sim loop reading `plant.get_contact_results_output_port()` | `rtde_c.moveUntilContact(v_base)` |
| `GripperCommand` | delegate to the `Gripper` attached to the arm | same |
| `Wait` | `simulator.AdvanceTo(t + duration)` | `time.sleep(duration)` |

## Per-arm gripper binding

Each backend owns a `{arm_name: Gripper}` dict. On construction:

- Sim: `HookGripper(sim_plant=...)` on `ur_left`, `Robotiq2F85(sim_plant=...)` on `ur_right`.
  Matches `scenario_files/bimanual.yaml`.
- Real: `HookGripper(rtde_control=...)` on `ur_left`, `Robotiq2F85(rtde_control=...)` on `ur_right`.

`backend.gripper(arm)` returns the instance; `GripperCommand` segments
dispatch through it.

## Real-time caveats

- `servoJ` and `forceMode` on real require refresh every ~8 ms. Do not
  block between ticks — keep I/O tight in the streaming loop.
- Always call `forceModeStop()` before the next `moveJ` / `moveL`, or the
  lingering force command fights the new position command.
- Pose encoding: RTDE uses `[x, y, z, rx, ry, rz]` axis-angle. Our
  `RigidTransform` needs conversion at the segment dispatch boundary.
