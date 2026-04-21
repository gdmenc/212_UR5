"""Sim backend: executes ``Segment``s against a Drake Simulator.

Maps each Segment type to the right way of driving a Drake diagram built
by ``src/scene.py``:

  MoveJ            -> write q_target to the arm's InverseDynamicsController
                      ``desired_state`` input; AdvanceTo long enough for the
                      controller to converge.
  MoveL            -> diff-IK integrate from X_WTcp_now to X_WTcp_goal in
                      Cartesian space; stream the resulting joint targets.
  ServoStream      -> stream pre-sampled q_traj waypoints at ``dt``.
  ForceMode        -> apply commanded wrench through a Cartesian admittance
                      sub-controller (TODO: this sub-controller does not
                      exist yet; build it only when the tray task is real).
  MoveUntilContact -> loop: advance sim, read plant contact results, stop
                      when total force on arm exceeds threshold.
  GripperCommand   -> dispatch to the Gripper attached to the arm.
  Wait             -> simulator.AdvanceTo(t + duration).

Non-realtime: we advance the Simulator manually, so "streaming" is just a
loop that pushes one waypoint then advances by dt.
"""

from typing import Any

from ..grippers.base import Gripper
from ..segments import (
    ForceMode,
    GripperCommand,
    MoveJ,
    MoveL,
    MoveUntilContact,
    Segment,
    ServoStream,
    Wait,
)


class SimBackend:
    """Drives a Drake Simulator built by ``scene.build_scene``."""

    def __init__(self, scene) -> None:
        self._scene = scene
        # TODO: stash references we will poke every segment:
        #   self._simulator = Simulator(scene.diagram)
        #   self._context = self._simulator.get_mutable_context()
        #   self._plant_ctx = scene.plant.GetMyContextFromRoot(self._context)
        #   self._cmd_ports = {"ur_left": scene.left_command_port, ...}
        #   self._grippers = {"ur_left": HookGripper(sim_plant=scene.plant, ...),
        #                     "ur_right": Robotiq2F85(sim_plant=scene.plant, ...)}

    def execute(self, arm: str, segment: Segment) -> None:
        # TODO: dispatch on segment type.
        if isinstance(segment, MoveJ):
            raise NotImplementedError("MoveJ sim dispatch")
        if isinstance(segment, MoveL):
            raise NotImplementedError("MoveL sim dispatch")
        if isinstance(segment, ServoStream):
            raise NotImplementedError("ServoStream sim dispatch")
        if isinstance(segment, ForceMode):
            raise NotImplementedError("ForceMode needs admittance sub-controller")
        if isinstance(segment, MoveUntilContact):
            raise NotImplementedError("MoveUntilContact sim dispatch")
        if isinstance(segment, GripperCommand):
            raise NotImplementedError("dispatch to self._grippers[arm]")
        if isinstance(segment, Wait):
            raise NotImplementedError("simulator.AdvanceTo(t + segment.duration)")
        raise TypeError(f"Unknown segment type: {type(segment).__name__}")

    def state(self, arm: str) -> dict:
        # TODO: read from plant context.
        #   q = plant.GetPositions(plant_ctx, model_instance)
        #   v = plant.GetVelocities(plant_ctx, model_instance)
        #   X_WTcp = plant.EvalBodyPoseInWorld(plant_ctx, tcp_body)
        #   wrench = <compute from contact results or external load>
        raise NotImplementedError

    def gripper(self, arm: str) -> Gripper:
        raise NotImplementedError
