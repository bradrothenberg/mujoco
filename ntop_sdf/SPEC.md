# Native implicit geometry physics specification

The goal is to evaluate nTop implicit geometry in interactive physics. The
external integration boundary accepts field values, optional gradients, and
conservative bounds. Use documented nTop SDK APIs and do not invent APIs. Collision
detection must query implicit fields directly. Display meshes are allowed.

The requested milestones are: reproduce the unmodified Giles and Andrews demo;
implement and test reusable SDF collision and rigid-body dynamics; investigate
elastic deformation using an invertible volumetric map; compare complete CPU
steps with MuJoCo; and provide an interactive diagnostic viewer. Preserve source
commits, exact build commands, raw benchmark measurements, and limitations.

Use Release builds, deterministic inputs, small tests before large benchmarks,
and measured results. Never report collision-only time as full-step time. If
deformation is not robust, prioritize validated rigid-body results. Do not
claim elastic deformation without a working elastic model and contact feedback.

## Current implementation

- The reference checkout remains separate at `D:/nTop/sdf-dcd-reference`.
- This fork adds a standalone cached implicit collider and rigid-body solver.
- A MuJoCo plugin accepts external field callbacks. An opt-in collision path
  uses cached Nelder-Mead manifolds for supported primitive and SDF pairs.
- The fork uses provider bounds for plugin SDF broad phase. The attached
  collider rejects mesh, plane, height-field, ellipsoid, other SDF-plugin, and
  flex pairs that are not implemented in the mesh-free path.
- An optional nTop Core adapter imports real `.implicit` files. It exposes
  values, derivatives, conservative bounds, and SDK batch queries. The SDK
  and nTop sample assets remain outside source control.
- Tests cover nTop Core 5.50.2 sphere and gyroid files, including 5 mm MuJoCo
  drops and a standalone sphere drop. Test other SDK versions and models.
- The viewer imports `.implicit` files and normalizes their size for the scene.
  Its surface is display-only. Collision queries use the nTop field.
- Elastic deformation, GPU evaluation, and continuous collision detection are
  not implemented. Do not describe them as complete.
