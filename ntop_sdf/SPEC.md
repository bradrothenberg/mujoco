# Native implicit geometry physics specification

The goal is to evaluate nTop-style implicit geometry in interactive physics. The
external integration boundary accepts field values, optional gradients, and
conservative bounds. It must not assume a proprietary nTop API. Collision
detection must query implicit fields directly. Display meshes are allowed.

The requested milestones are: reproduce the unmodified Giles and Andrews demo;
implement and test reusable SDF collision and rigid-body dynamics; investigate
elastic deformation using an invertible volumetric map; compare complete CPU
steps with MuJoCo; and provide an interactive diagnostic viewer. Preserve source
commits, exact build commands, raw benchmark measurements, and limitations.

Use Release builds, deterministic inputs, small tests before large benchmarks,
and measured results. Never report collision-only time as full-step time. If
deformation is not robust, prioritize validated rigid-body results. Do not
claim an nTop connection until a real evaluator has been integrated.

## Current implementation

- The reference checkout remains separate at `D:/nTop/sdf-dcd-reference`.
- This fork adds a standalone cached implicit collider and rigid-body solver.
- A MuJoCo plugin accepts external field callbacks. An opt-in collision path
  uses cached Nelder-Mead manifolds for supported primitive and SDF pairs.
- The fork uses provider bounds for plugin SDF broad phase. The attached
  collider rejects mesh, plane, height-field, ellipsoid, other SDF-plugin, and
  flex pairs that are not implemented in the mesh-free path.
- Elastic deformation, GPU evaluation, continuous collision detection, and a
  real nTop evaluator are not implemented. Do not describe them as complete.
- The diagnostic viewer uses the standalone solver and analytic primitives.
  Its render meshes are display-only. The viewer does not visualize callbacks
  or elastic deformation.
