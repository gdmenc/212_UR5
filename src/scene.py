"""Scene construction: builds the Drake diagram for the bimanual UR5e rig.

Responsibilities:
  1. Parse ``scenario_files/bimanual.yaml`` (arms, grippers, tables, objects)
     via Drake's ``LoadScenario`` / ``MakeHardwareStation`` pipeline.
  2. Instantiate a per-arm controller (from ``execution/sim_controllers.py``)
     and wire its ``desired_state`` input port to a trajectory source.
  3. Attach MeshcatVisualizer (visual + optional collision role) for debug.
  4. Return a ``Scene`` dataclass that primitives and backends consume.

Used two ways:
  - As a module: ``build_scene()`` -> Scene object passed to SimBackend.
  - As a script: ``python -m src.scene`` opens Meshcat and runs a brief
    simulation with the arms at a neutral configuration so you can visually
    sanity-check any scene change. This is the tight edit-run-see loop for
    YAML edits; combine with Drake's ``ModelVisualizer`` for individual
    asset iteration.
"""

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, Optional

from pydrake.math import RigidTransform
from pydrake.multibody.plant import MultibodyPlant


_REPO_ROOT = Path(__file__).resolve().parent.parent
SCENARIO_PATH = str(_REPO_ROOT / "scenario_files" / "bimanual.yaml")


@dataclass
class Scene:
    """Everything the rest of the system needs to talk to the diagram.

    Attributes
    ----------
    diagram : pydrake.systems.framework.Diagram
        The built diagram. Pass to ``Simulator(diagram)`` in SimBackend.
    plant : MultibodyPlant
        The multibody plant containing both arms, both grippers, and the
        scene objects. Used by planners (IK, collision queries) and by
        the sim backend (reading state).
    command_ports : dict[str, InputPort]
        Per-arm ``desired_state`` input ports (keys: ``"ur_left"``,
        ``"ur_right"``) that SimBackend writes each segment's target to.
    object_body_names : dict[str, str]
        Logical name -> plant body name. Lets primitives do
        ``scene.object_pose("plate_left")`` without knowing the raw SDF
        body names.
    named_poses : dict[str, RigidTransform]
        Task-relevant world-frame poses that are *not* object poses:
        ``"microwave_top"`` (staging surface for cup/bottle/stirrer),
        ``"microwave_interior"`` (inside the cavity, for plate/bowl),
        ``"microwave_button_heat"`` / ``"microwave_button_start"``,
        ``"tray_final"``, etc. Populated at build time from a hand-edited
        table; primitives and the sequencer reference by name so the
        recipe reads like prose.
    meshcat : optional Meshcat handle for interactive debug.

    Grasp candidates (``X_OG`` per object) live in ``src/grasping/`` so
    they are easy to swap for a perception-driven generator later without
    touching diagram construction.
    """

    diagram: Any
    plant: MultibodyPlant
    command_ports: Dict[str, Any] = field(default_factory=dict)
    object_body_names: Dict[str, str] = field(default_factory=dict)
    named_poses: Dict[str, Any] = field(default_factory=dict)
    meshcat: Optional[Any] = None


def build_scene(meshcat=None, show_collision: bool = False) -> Scene:
    """Build and return the Scene.

    Parameters
    ----------
    meshcat : optional Meshcat handle. If provided, attach a visualizer.
        Leave None for headless (unit tests, batch runs).
    show_collision : if True, attach a second visualizer showing collision
        geometry (Drake's ``Role.kProximity``) alongside the visual mesh.
        Essential when debugging planner failures — the planner sees
        collision, not visual.

    Steps (TODO: implement):
      1. builder = DiagramBuilder()
      2. scenario = LoadScenario(filename=SCENARIO_PATH)
      3. station = builder.AddSystem(MakeHardwareStation(scenario))
      4. plant  = station.GetSubsystemByName("plant")
      5. For each arm in ("ur_left", "ur_right"):
           controller, ctrl_plant = <CHOICE>(...)
           builder.AddSystem(controller)
           Connect plant state -> controller estimated_state
           Connect controller output -> plant actuation
           Expose controller.desired_state as a diagram input port or
           stash the InputPort in command_ports.

         TODO (design choice — do NOT pick arbitrarily before deciding):
           Option A — ``make_inverse_dynamics_controller``. Tight
             tracking, cancels the arm's dynamics + gravity. Good
             default for free-space pick-and-place; less forgiving when
             the TCP is in contact (will fight external wrenches).
           Option B — ``make_joint_stiffness_controller``. Compliant
             PD-with-gravity-comp. Natural softness under contact
             (relevant for push, tray rotation, drag). Position
             tracking is looser.
           Option C — mix: different default per arm, or switch at
             runtime based on the current primitive (e.g. push/tray
             -> stiffness, pick/place -> inverse dynamics).

           Decision should be data-driven — run both on a pick task in
           sim and pick whichever is cleaner, then see if tray/push
           tasks force a switch. Leaving unimplemented to force the
           experiment.
      6. If meshcat:
           MeshcatVisualizer.AddToBuilder(..., Role.kIllustration)
           if show_collision:
               MeshcatVisualizer.AddToBuilder(..., Role.kProximity, prefix="coll")
      7. diagram = builder.Build()
      8. Populate ``object_body_names`` from a small mapping (logical
         name -> plant body name). Grasp candidates are NOT populated
         here — they live in ``src/grasping/candidates.py`` and are
         looked up by the primitives at pick time.
      9. Populate ``named_poses`` with the task-level reference poses
         (microwave top, button locations, tray final). Keep this table
         at the bottom of this module; primitives dereference by name.
    """
    # TODO
    raise NotImplementedError


if __name__ == "__main__":
    # Visual smoke test. Run after any YAML / controller change to verify
    # the diagram builds and looks right. Does not exercise primitives.
    from pydrake.geometry import StartMeshcat
    from pydrake.systems.analysis import Simulator

    meshcat = StartMeshcat()
    scene = build_scene(meshcat=meshcat, show_collision=True)

    simulator = Simulator(scene.diagram)
    simulator.set_target_realtime_rate(1.0)

    meshcat.AddButton("Stop Simulation")
    while meshcat.GetButtonClicks("Stop Simulation") < 1:
        simulator.AdvanceTo(simulator.get_context().get_time() + 1.0)
