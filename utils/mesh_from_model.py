from pydrake.multibody.mesh_to_model import MakeModelFromMesh

MakeModelFromMesh(
    input_filename="../src/assets/microwave/microwave.obj",
    output_filename="../src/assets/microwave/microwave.sdf"
)