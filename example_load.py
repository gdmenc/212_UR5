"""
Loader + smoke-test visualizer for the hooker gripper SDF.

The SDF declares 3 revolute joints in a tree:
    joint_A       : body  -> arm_A        (the ONE actuated joint)
    joint_B       : body  -> arm_B        (passive)
    joint_end_A   : arm_A -> end_link     (passive)

The parallelogram loop (arm_B <-> end_link at B2) is NOT in the SDF tree.
Instead we close the mechanism with two coupler constraints:

    q_B       = +1 * q_A          (both arms stay parallel)
    q_end_A   = -1 * q_A          (end_link orientation stays fixed)

With those in place, commanding q_A is equivalent to commanding the whole
gripper — end_link translates along an arc without rotating, and arm_B's
distal pivot stays glued to end_link's matching hole.

Usage:
    python example_load.py                # just prints model info
    python example_load.py --meshcat      # open meshcat + sweep q_A
"""
from __future__ import annotations

import argparse
import time
from pathlib import Path

import numpy as np
from pydrake.geometry import Meshcat, StartMeshcat
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import AddMultibodyPlantSceneGraph
from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import DiagramBuilder
from pydrake.visualization import AddDefaultVisualization


SDF_PATH = Path(__file__).parent / "src" / "assets" / "hook_gripper" / "hooker.sdf"


def add_hooker(plant, parser=None, weld_to_world: bool = True):
    """Parse the hooker SDF, add the parallelogram coupler constraints,
    and (optionally) weld `body` to the world so it floats in place.

    Returns
    -------
    model_instance : ModelInstanceIndex
    joint_A        : the driving joint (use this to command motion)
    """
    if parser is None:
        parser = Parser(plant)
    (model_instance,) = parser.AddModels(str(SDF_PATH))

    # Named-joint handles
    joint_A     = plant.GetJointByName("joint_A",     model_instance)
    joint_B     = plant.GetJointByName("joint_B",     model_instance)
    joint_end_A = plant.GetJointByName("joint_end_A", model_instance)

    # --- Close the parallelogram loop ---
    # Drake's AddCouplerConstraint(j0, j1, gear_ratio, offset) enforces
    #     q0 = gear_ratio * q1 + offset.
    # Parallelogram: both arms rotate together.
    plant.AddCouplerConstraint(joint_B, joint_A, gear_ratio=1.0, offset=0.0)
    # End-effector stays parallel to body: q_end = -q_A.
    plant.AddCouplerConstraint(joint_end_A, joint_A, gear_ratio=-1.0, offset=0.0)

    if weld_to_world:
        plant.WeldFrames(
            plant.world_frame(),
            plant.GetFrameByName("body", model_instance),
        )

    return model_instance, joint_A


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--meshcat", action="store_true",
                    help="open Meshcat and sweep the gripper through its range")
    ap.add_argument("--duration", type=float, default=6.0)
    args = ap.parse_args()

    builder = DiagramBuilder()
    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=0.001)
    model_instance, joint_A = add_hooker(plant)
    plant.Finalize()

    # Report what we built
    print(f"\nLoaded hooker gripper from {SDF_PATH}")
    print(f"  num positions : {plant.num_positions(model_instance)}")
    print(f"  num actuators : {plant.num_actuators()}")
    print(f"  joints        :")
    for idx in plant.GetJointIndices(model_instance):
        j = plant.get_joint(idx)
        if j.num_positions() > 0:
            print(f"    - {j.name():<14s} type={j.type_name():<12s} "
                  f"limits=[{j.position_lower_limits()[0]:+.2f}, "
                  f"{j.position_upper_limits()[0]:+.2f}] rad")

    if not args.meshcat:
        print("\n(pass --meshcat to open the viewer and sweep the joint)")
        return

    meshcat = StartMeshcat()
    AddDefaultVisualization(builder, meshcat=meshcat)
    diagram = builder.Build()

    simulator = Simulator(diagram)
    diagram_ctx = simulator.get_mutable_context()
    plant_ctx = plant.GetMyMutableContextFromRoot(diagram_ctx)

    # Sweep q_A from open to closed and back so you can visually confirm
    # the parallelogram kinematics.
    q_lo, q_hi = joint_A.position_lower_limits()[0], joint_A.position_upper_limits()[0]

    t0 = time.time()
    while time.time() - t0 < args.duration:
        t = time.time() - t0
        # ping-pong between limits
        s = 0.5 * (1 - np.cos(2 * np.pi * t / 3.0))  # 3 s period
        q = q_lo + s * (q_hi - q_lo)
        joint_A.set_angle(plant_ctx, q)
        diagram.ForcedPublish(diagram_ctx)
        time.sleep(0.02)

    print("done.")


if __name__ == "__main__":
    main()
