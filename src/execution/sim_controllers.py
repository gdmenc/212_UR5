"""Sim-only joint controllers for the UR5e arms.

Two factory functions, one per scheme. Each returns ``(controller, plant)``
— the plant is returned alongside because Drake's controller systems hold a
reference to it, so its lifetime must match the controller's.

Why this lives in ``execution/``: controllers are the piece of the sim
backend that turns a ``desired_state`` into joint torques on the Drake
plant. Real hardware has its own cascade controller inside the UR5e
control box; ``real_backend`` sends joint targets, it does not tune a
controller. So anything under ``execution/sim_*`` is a Drake-only building
block consumed by ``scene.build_scene`` at diagram construction time.

Gripper note: the WSG-50 is welded to wrist_3 in the current scenario but
is NOT included in the controller plant below. That introduces a ~1 kg
gravity bias at the wrist; the P terms absorb it in practice. If tracking
needs to be tighter, extend ``_build_ur5e_controller_plant`` to weld the
gripper SDF at the same offset used in the YAML.
"""

from pathlib import Path

import numpy as np
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import MultibodyPlant
from pydrake.systems.controllers import (
    InverseDynamicsController,
    JointStiffnessController,
)

_REPO_ROOT = Path(__file__).resolve().parents[2]
UR5E_URDF = str(_REPO_ROOT / "ur_description" / "urdf" / "ur5e.urdf")

# THESE ARE DEFAULT GAINS, SUBJECT TO CHANGE
# Joint order: [shoulder_pan, shoulder_lift, elbow, wrist_1, wrist_2, wrist_3].
# Distal joints carry less inertia, so gains taper toward the wrist.
STIFFNESS_KP = np.array([800.0, 800.0, 600.0, 400.0, 400.0, 200.0])
STIFFNESS_KD = 2.0 * np.sqrt(STIFFNESS_KP)  # ~critical at unit effective mass

# Inverse dynamics pre-multiplies by M^-1, so the PID gains act in
# acceleration units and are numerically smaller than the stiffness gains.
ID_KP = np.array([200.0, 200.0, 200.0, 100.0, 100.0, 100.0])
ID_KI = np.zeros(6)
ID_KD = 2.0 * np.sqrt(ID_KP)


def _build_ur5e_controller_plant(urdf_path: str = UR5E_URDF) -> MultibodyPlant:
    plant = MultibodyPlant(time_step=0.0)
    Parser(plant).AddModels(urdf_path)
    plant.Finalize()
    return plant


def make_joint_stiffness_controller(
    urdf_path: str = UR5E_URDF,
    kp: np.ndarray = STIFFNESS_KP,
    kd: np.ndarray = STIFFNESS_KD,
):
    """tau = kp*(q_d - q) - kd*v + g(q). Compliant; forgiving in contact."""
    plant = _build_ur5e_controller_plant(urdf_path)
    controller = JointStiffnessController(plant=plant, kp=kp, kd=kd)
    return controller, plant


def make_inverse_dynamics_controller(
    urdf_path: str = UR5E_URDF,
    kp: np.ndarray = ID_KP,
    ki: np.ndarray = ID_KI,
    kd: np.ndarray = ID_KD,
    has_reference_acceleration: bool = False,
):
    """tau = M*(qdd_d + kp*e + ki*int(e) + kd*edot) + C*v + g. Stiff tracking."""
    plant = _build_ur5e_controller_plant(urdf_path)
    controller = InverseDynamicsController(
        robot=plant,
        kp=kp,
        ki=ki,
        kd=kd,
        has_reference_acceleration=has_reference_acceleration,
    )
    return controller, plant


if __name__ == "__main__":
    for factory, name in [
        (make_joint_stiffness_controller, "JointStiffnessController"),
        (make_inverse_dynamics_controller, "InverseDynamicsController"),
    ]:
        ctrl, plant = factory()
        port_names = [
            ctrl.get_input_port(i).get_name() for i in range(ctrl.num_input_ports())
        ]
        print(
            f"{name}: dof={plant.num_positions()}, "
            f"inputs={port_names}, outputs={ctrl.num_output_ports()}"
        )
