from pydrake.all import ModelVisualizer, PackageMap, StartMeshcat

from manipulation import running_as_notebook

def main():
    # Start the visualizer.
    meshcat = StartMeshcat()
    visualizer = ModelVisualizer(meshcat=meshcat)
    visualizer.AddModels("ur_description/urdf/ur5e.urdf")
    visualizer.Run()

if __name__ == "__main__":
    main()