from pydrake.all import ModelVisualizer, DiagramBuilder, MeshcatVisualizer, Simulator, StartMeshcat
from pydrake.multibody.parsing import PackageMap

from pydrake.systems.sensors import CameraConfig, ApplyCameraConfig
from pydrake.systems.framework import DiagramBuilder
from pydrake.multibody.plant import MultibodyPlant
from pydrake.geometry import SceneGraph
from pydrake.visualization import AddFrameTriadIllustration

from manipulation.station import LoadScenario, MakeHardwareStation
from manipulation.utils import RenderDiagram

def main():
    package_map = PackageMap()
    package_map.Add("ur_description", "/Users/gdmen/MIT/sp26/2.12/212_UR5/ur_description")

    # Start the visualizer.
    meshcat = StartMeshcat()

    # Load Scenario file to get all the models loaded into the scene
    scenario = LoadScenario(filename="scenario_files/bimanual.yaml")

    builder = DiagramBuilder()

    station = builder.AddSystem(MakeHardwareStation(scenario))
    plant = station.GetSubsystemByName("plant")
    scene_graph = station.GetSubsystemByName("scene_graph")

    gripper_body = plant.GetBodyByName("hook_gripper")
    AddFrameTriadIllustration(
        scene_graph=scene_graph,
        body=gripper_body,
        length=0.1,   
        radius=0.004, 
    )

    # wsg_right = plant.GetBodyByName("wsg_right::body")
    # AddFrameTriadIllustration(
    #     scene_graph=scene_graph,
    #     body=wsg_right,
    #     length=0.1,   
    #     radius=0.004, 
    # )

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