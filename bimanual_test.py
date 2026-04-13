from pydrake.all import ModelVisualizer, DiagramBuilder, MeshcatVisualizer, Simulator, StartMeshcat
from pydrake.multibody.parsing import PackageMap

from manipulation.station import LoadScenario, MakeHardwareStation
from manipulation.utils import RenderDiagram

def main():
    package_map = PackageMap()
    package_map.Add("ur_description", "/Users/gdmen/MIT/sp26/2.12/212_UR5/ur_description")

    # Start the visualizer.
    meshcat = StartMeshcat()

    scenario = LoadScenario(filename="scenario_files/bimanual.yaml")

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