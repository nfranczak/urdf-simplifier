# urdf-simplifier

Collision geometries represented in URDF files are almost always meshes. Unfortunately, meshes are usually composed of a large number of triangles and when motion planning, more specifically checking against collisions, each of the mesh triangles needs to be checked which dramatically slows down VIAM's motion planning.

The repository provides code which reads in an URDF file, simplifies the meshes to be a bounding rectangular prisms and returns another URDF file ready for use.
This way each collision geometry has at most 6 faces which need to be checked.

## Usage

### Prerequisites
- Go 1.23.5 or later
- The `stl-bounding-box` package (automatically resolved via go.mod)

### Running the Tool

```bash
go run main.go <input.urdf> <output.urdf>
```

**Arguments:**
- `input.urdf` - Path to the input URDF file
- `output.urdf` - Path where the simplified URDF will be written

### Example

```bash
# Simplify a Universal Robots UR20 URDF
go run main.go /path/to/ur20.urdf /path/to/ur20_simplified.urdf

# Simplify a UFactory UF850 URDF
go run main.go ufactory/uf850.urdf ufactory/uf850_simplified.urdf
```

### What the Tool Does

The tool performs the following transformations:

1. **Removes visual elements** - All `<visual>` tags are removed
2. **Removes inertial properties** - The entire `<inertial>` section is removed
3. **Moves origin to link level** - The `<origin>` from within `<inertial>` is moved to be a direct child of `<link>`
4. **Replaces collision meshes with bounding boxes** - Each collision `<mesh>` is replaced with a `<box>` element with dimensions calculated from the mesh's bounding box

The tool automatically resolves `package://` URIs to find STL mesh files and calculates their bounding boxes using the `stl-bounding-box` package.

## Output Format

The simplified URDF is compatible with VIAM's RDK and contains only the essential information needed for motion planning:
- Link names and structure
- Joint definitions and constraints
- Collision geometries as simple boxes
- Link origins (extracted from inertial data)
