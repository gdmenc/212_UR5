from pydrake.all import ModelVisualizer, DiagramBuilder, MeshcatVisualizer, Simulator, StartMeshcat

from manipulation.station import LoadScenario, MakeHardwareStation
from manipulation.utils import RenderDiagram

def main():
    # Start the visualizer.
    meshcat = StartMeshcat()

    scenario_data = """
    directives:
    - add_model:
        name: ur_left
        file: ur_description/urdf/ur5e.urdf
    - add_weld:
        parent: world
        child: ur_left::base_link
    - add_model:
        name: wsg_left
        file: package://drake_models/wsg_50_description/sdf/schunk_wsg_50_with_tip.sdf
    - add_weld:
        parent: ur_left::ur_ee_fixed_joint_parent
        child: wsg_left::body
        X_PC:
            translation: [0, 0, 0.09]
            rotation: !Rpy { deg: [90, 0, 90]}
    - add_model:
        name: ur_right
        file: ur_description/urdf/ur5e.urdf
    - add_weld:
        parent: world
        child: ur_right::base_link
        X_PC:
            translation: [.6, 0, 0]
            rotation: !Rpy { deg: [0, 0, 0]}
    - add_model:
        name: wsg_right
        file: package://drake_models/wsg_50_description/sdf/schunk_wsg_50_with_tip.sdf
    - add_weld:
        parent: ur_right::wrist_3_link
        child: wsg_right::body
        X_PC:
            translation: [0, 0, 0.09]
            rotation: !Rpy { deg: [90, 0, 90]}
    """

    scenario = LoadScenario(data=scenario_data)
    builder = DiagramBuilder()

    station = builder.AddSystem(MakeHardwareStation(scenario))
    plant = station.GetSubsystemByName("plant")

    visualizer = MeshcatVisualizer.AddToBuilder(
        builder, station.GetOutputPort("query_object"), meshcat
    )
    visualizer = MeshcatVisualizer.AddToBuilder(
        builder, station.GetOutputPort("query_object"), meshcat
    )

    diagram = builder.Build()
    simulator = Simulator(diagram)
    simulator.set_target_realtime_rate(1.0)

    meshcat.AddButton("Stop Simulation")
    while meshcat.GetButtonClicks("Stop Simulation") < 1:
        simulator.AdvanceTo(simulator.get_context().get_time() + 1.0)

if __name__ == "__main__":
    main()