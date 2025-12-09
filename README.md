# urdf-simplifier

Collision geometries represented in URDF files are almost always meshes. Unfortunately, meshes are usually composed of a large number of triangles and when motion planning, more specifically checking against collisions, each of the mesh triangles needs to be checked which dramatically slows down VIAM's motion planning.

The repository provides code which reads in an URDF file, simplifies the meshes to be a bouding rectangular prisms and returns another URDF file ready for use.
This way each collision geometry has at most 6 faces which need to be checked.
