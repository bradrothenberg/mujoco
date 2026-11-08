# Work on the nTop SDF integration

Keep the upstream MuJoCo tree and the independent reference checkout intact.
Put integration code, tests, benchmarks, and reports in this directory unless a
documented engine change is necessary. Record upstream commits and retain all
third-party copyright notices. Use C++20 for new integration code.

Collision geometry must remain implicit. A visualization mesh is permitted but
must never become the collision representation. External evaluators must expose
values, optional gradients, conservative local bounds, and a batch interface.
Do not invent nTop APIs. Validate numerical edge cases, conservative bounds,
field lifetime, reentrancy, and failures. Report all known limitations.

Compile Release builds and run tests before committing. Benchmark complete
steps and collision stages separately. Store raw results with the commands,
compiler, scene, seed, hardware, and source commits. Compare identical scenes
and settings, and do not claim performance parity without measurements.

Use uv for Python. Do not overwrite unrelated repositories. Use short, direct
technical writing without emojis or em dashes.

## Architecture

- `include/ntop_sdf/field.hpp` defines the evaluator and batch interface.
  `src/field.cpp` implements primitives and implicit operators.
- `src/ntop_core.cpp` and `include/ntop_sdf/ntop_core.hpp` connect exported
  `.implicit` bodies to the documented nTop Core C SDK. The optional SDK
  dependency is supplied by the user through `NTOP_CORE_ROOT`. Do not commit
  the SDK, its sample assets, or private nTop models. The adapter serializes
  SDK calls because the SDK thread-safety contract is not documented here.
- `world.hpp` and `src/world.cpp` adapt the Giles and Andrews solver, expose
  rigid bodies and persistent pair queries, and validate callback results.
- `vendor/giles` retains the upstream license and adapted collision/solver
  code. `reference` measures the separate, unmodified checkout.
- `mujoco_plugin.cpp` registers process-local field handles.
  `mujoco_collision.cpp` installs the opt-in cached collision callback. An
  attached `mjData` must be detached before deletion. Review thread and
  lifetime rules before changing this integration.
- MuJoCo's `src/user/user_mesh.cc`, `user_objects.cc`, and `user_objects.h`
  carry the provider's conservative SDF bounds into collision geometry.
- `demo` displays analytic shapes and, when the SDK is available, nTop
  implicits. Display meshes never enter collision queries. OBJ import and
  voxel SDF collision are disabled. It is not an elastic-body viewer.
- `tests`, `bench`, `scripts`, and `results` contain regressions, executable
  measurements, a reproduction script, and recorded CSV files.
- `tools/ntop_core_info.cpp` inspects an exported field. The optional
  `tools/ntop_mujoco_run.cpp` drops one exported body in MuJoCo. It estimates
  mass properties by coarse field sampling and accepts a caller-set density.

Build and test commands and known limitations are in `README.md` and
`REPORT.md`. The current MuJoCo integration requires explicit field bounds,
does not support flex contact, and does not implement elastic deformation.
