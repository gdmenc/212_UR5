"""
Loader + smoke-test visualizer for the hooker gripper SDF.

The SDF declares 3 revolute joints in a tree:
    joint_A       : body  -> arm_A        (passive)
    joint_B       : body  -> arm_B        (the ONE actuated joint — servo)
    joint_end_A   : arm_A -> end_link     (passive)

The parallelogram loop (arm_B <-> end_link at B2) is NOT in the SDF tree.
Instead we close the mechanism with two coupler constraints:

    q_A       = +1 * q_B          (both arms stay parallel)
    q_end_A   = -1 * q_B          (end_link orientation stays fixed)

With those in place, commanding q_B (the servo on link3) is equivalent to
commanding the whole gripper — end_link translates along an arc without
rotating, and arm_A's distal pivot stays glued to end_link's matching hole.

Usage:
    python example_load.py                # just prints model info
    python example_load.py --meshcat      # open meshcat + sweep q_B
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
    joint_B        : the driving joint (servo on link3 — use this to command motion)
    """
    if parser is None:
        parser = Parser(plant)
    (model_instance,) = parser.AddModels(str(SDF_PATH))
 
    joint_A     = plant.GetJointByName("joint_A",     model_instance)
    joint_B     = plant.GetJointByName("joint_B",     model_instance)
    joint_end_A = plant.GetJointByName("joint_end_A", model_instance)
 
    # Close the parallelogram loop (solved every simulator step).
    # joint_B is the servo-driven joint; arm_A and end_link follow.
    plant.AddCouplerConstraint(joint_A,     joint_B,  1.0, 0.0)
    plant.AddCouplerConstraint(joint_end_A, joint_B, -1.0, 0.0)
 
    if weld_to_world:
        plant.WeldFrames(
            plant.world_frame(),
            plant.GetFrameByName("body", model_instance),
        )
    return model_instance, joint_B
 
 
def set_gripper_opening(plant, context, model_instance, q_B):
    """Kinematic setter that respects the parallelogram.
 
    Use this whenever you touch the gripper's positions outside of
    dynamic simulation (IK, initial conditions, teleop pose snaps, ...).
    Under simulation the coupler constraints handle this for you.
    
    q_B is the servo angle (joint_B on link3/arm_B).
    """
    plant.GetJointByName("joint_A",     model_instance).set_angle(context,  q_B)
    plant.GetJointByName("joint_B",     model_instance).set_angle(context,  q_B)
    plant.GetJointByName("joint_end_A", model_instance).set_angle(context, -q_B)
 
 
# ---------------------------------------------------------------------------
# Visual smoke tests
# ---------------------------------------------------------------------------
 
def _kinematic_sweep(args):
    """Pure-kinematic ping-pong: sets all three joints directly (via q_B).
    Use this to eyeball the mesh geometry; no physics is being simulated."""
    from pydrake.geometry import StartMeshcat
    builder = DiagramBuilder()
    plant, _ = AddMultibodyPlantSceneGraph(builder, time_step=0.001)
    model_instance, joint_B = add_hooker(plant)
    plant.Finalize()
 
    meshcat = StartMeshcat()
    AddDefaultVisualization(builder, meshcat=meshcat)
    diagram = builder.Build()
    simulator = Simulator(diagram)
    diag_ctx  = simulator.get_mutable_context()
    plant_ctx = plant.GetMyMutableContextFromRoot(diag_ctx)
 
    q_lo, q_hi = joint_B.position_lower_limits()[0], joint_B.position_upper_limits()[0]
    t0 = time.time()
    while time.time() - t0 < args.duration:
        t = time.time() - t0
        s = 0.5 * (1 - np.cos(2 * np.pi * t / 3.0))          # 3 s period
        q = q_lo + s * (q_hi - q_lo)
        set_gripper_opening(plant, plant_ctx, model_instance, q)   # <-- all three
        diagram.ForcedPublish(diag_ctx)
        time.sleep(0.02)
 
 
def _dynamic_sweep(args):
    """Actually simulate the gripper: feed a constant torque to joint_A and
    let Drake's solver enforce the coupler constraints. This is the path
    you'll use in pick-and-place — swap the ConstantVectorSource for a
    controller and you're done."""
    from pydrake.geometry import StartMeshcat
    builder = DiagramBuilder()
    plant, _ = AddMultibodyPlantSceneGraph(builder, time_step=0.001)
    model_instance, joint_B = add_hooker(plant)
    plant.Finalize()
 
    # One actuator in the whole model -> one-element input vector.
    assert plant.num_actuators() == 1
    # Small constant torque to close the gripper; sign picks direction.
    torque = builder.AddSystem(ConstantVectorSource([-0.2]))
    builder.Connect(torque.get_output_port(),
                    plant.get_actuation_input_port(model_instance))
 
    meshcat = StartMeshcat()
    AddDefaultVisualization(builder, meshcat=meshcat)
    diagram = builder.Build()
    simulator = Simulator(diagram)
    simulator.set_target_realtime_rate(1.0)
 
    # Start in a consistent initial configuration.
    plant_ctx = plant.GetMyMutableContextFromRoot(simulator.get_mutable_context())
    set_gripper_opening(plant, plant_ctx, model_instance, 0.0)
 
    simulator.AdvanceTo(args.duration)
 
 
# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------
 
def _describe(plant, model_instance):
    print(f"\nLoaded hooker gripper from {SDF_PATH}")
    print(f"  num_positions  : {plant.num_positions()}")
    print(f"  num_actuators  : {plant.num_actuators()}")
    print(f"  num_constraints: {plant.num_constraints()}")
    print(f"  joints:")
    for idx in plant.GetJointIndices(model_instance):
        j = plant.get_joint(idx)
        if j.num_positions() > 0:
            lim = f"[{j.position_lower_limits()[0]:+.2f},{j.position_upper_limits()[0]:+.2f}]"
            print(f"    - {j.name():<14s} type={j.type_name():<10s} limits={lim} rad")
 
 
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--meshcat", choices=("kin", "sim"), default=None,
                    help="open Meshcat: 'kin' = pure kinematic sweep, "
                         "'sim' = dynamic sim driven by a torque source")
    ap.add_argument("--duration", type=float, default=6.0)
    args = ap.parse_args()
 
    if args.meshcat is None:
        builder = DiagramBuilder()
        plant, _ = AddMultibodyPlantSceneGraph(builder, time_step=0.001)
        model_instance, _ = add_hooker(plant)
        plant.Finalize()
        _describe(plant, model_instance)
        print("\n(pass --meshcat kin  to see a kinematic sweep)")
        print("(pass --meshcat sim  to see a dynamic sim)")
        return
 
    if args.meshcat == "kin":
        _kinematic_sweep(args)
    else:
        _dynamic_sweep(args)
 
 
if __name__ == "__main__":
    main()