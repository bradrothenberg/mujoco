# Implicit collision and MuJoCo integration

This prototype evaluates signed implicit fields directly during collision
detection. It includes a standalone C++20 rigid-body solver, a MuJoCo SDF
plugin, a cached collision path for that plugin, regression tests, benchmarks,
and an interactive rigid-body viewer. An optional adapter loads real nTop
`.implicit` exports through the documented nTop Core C SDK. The generic field
interface also accepts caller-supplied evaluators.

**Status:** elastic deformation is not implemented. Continuous collision
detection and general guarantees for thin or deeply intersecting non-convex
features are also missing. See [REPORT.md](REPORT.md) before using the results.

## Load an nTop implicit

Export an implicit body from nTop with the **Export Implicit Body** block. The
SDK reads the resulting `.implicit` file directly. It does not read a notebook
file. Install or unpack nTop Core separately; the SDK and its sample files are
not part of this repository. These commands use the local SDK and MuJoCo build
on this machine:

```powershell
cd "D:\nTop\mujoco"
cmake -S ntop_sdf -B build-ntop-sdf -G Ninja -DCMAKE_BUILD_TYPE=Release -DMUJOCO_BUILD_DIR=D:/nTop/mujoco/build-ntop -DNTOP_CORE_ROOT=D:/nTop/mujoco/build-ntop-core-sdk
cmake --build build-ntop-sdf --parallel 6
ctest --test-dir build-ntop-sdf --output-on-failure
./build-ntop-sdf/ntop_core_info.exe ./build-ntop-core-sdk/examples/assets/gyroid_sphere_5mm_radius.implicit
./build-ntop-sdf/ntop_mujoco_run.exe ./build-ntop-core-sdk/examples/assets/gyroid_sphere_5mm_radius.implicit 1000 1 240
```

Set `NTOP_CORE_ROOT` to the SDK directory containing `include/ntop_core` and
`lib`. Set `MUJOCO_BUILD_DIR` to this fork's built MuJoCo directory. To run the
sample tests, the SDK must also contain its `examples/assets` folder. Otherwise
set `NTOP_CORE_TEST_IMPLICIT` and `NTOP_CORE_GYROID_IMPLICIT` to equivalent SDK
fixtures. The sphere test expects a 5 mm radius sphere at the origin.

`ntop_mujoco_run` loads the requested file and drops one body onto a box floor.
Its arguments are file path, uniform density in kg/m³, duration in seconds,
and display rate in Hz. It uses four physics substeps per displayed step. Its
mass and inertia come from coarse volume sampling; choose a suitable material
density and supply better mass properties for engineering use. The reported
elapsed time includes the first collision's setup and is not the steady-state
benchmark. The C++ API is [ntop_core.hpp](include/ntop_sdf/ntop_core.hpp):

```cpp
auto field = ntop_sdf::load_ntop_core_implicit("part.implicit");
ntop_sdf::register_mujoco_plugin();
auto id = ntop_sdf::register_mujoco_field(field);
// Use id in MJCF, then call attach_mujoco_collision after mj_makeData.
```

The default scale preserves the export's dimensions in metres. The optional
second argument applies a uniform scale about the field origin. Keep the field
registered until all MuJoCo models and data using it are destroyed. The SDK
must support the file version. An unsupported file requires a compatible SDK.
The current tests use nTop Core 5.50.2. Collision uses direct field and
derivative queries, never a surface mesh. SDK calls are serialized because the
SDK's cross-thread query contract has not been established. Array queries use
the SDK's batch API. The collision optimizer currently queries single points.

The optional standalone solver has default tolerances intended for unit-sized
objects. Set `World::collision_settings` in world units before simulating small
parts. For example, the 5 mm sphere test uses a 0.1 mm margin, 0.05 mm allowed
penetration, and 0.001 mm optimizer tolerance. These are test settings, not
universal tolerances for every model.

## Build the standalone library and tests

The following commands use the MSYS2 UCRT64 compiler on the test machine. Run
them from the root of this MuJoCo checkout in PowerShell:

```powershell
cmake -S ntop_sdf -B build-ntop-sdf -G Ninja -DCMAKE_BUILD_TYPE=Release -DCMAKE_C_COMPILER=C:/msys64/ucrt64/bin/gcc.exe -DCMAKE_CXX_COMPILER=C:/msys64/ucrt64/bin/g++.exe
cmake --build build-ntop-sdf --parallel 6
ctest --test-dir build-ntop-sdf --output-on-failure
./build-ntop-sdf/ntop_sdf_bench.exe 10 10 20 60
./build-ntop-sdf/ntop_sdf_accuracy.exe
```

To make a fresh checkout of the integration branch:

```powershell
cd "D:\nTop"
git clone --branch ntop-sdf https://github.com/bradrothenberg/mujoco.git
cd "D:\nTop\mujoco"
```

Use a different destination if `D:\nTop\mujoco` already exists.

On another platform, select an installed C++20 compiler and generator. A
multi-configuration generator also needs `--config Release` for build and test.

## Build the diagnostic viewer

The viewer uses SDL2 and Dear ImGui at the commits listed in its CMake file.
CMake downloads them when no reference checkout is specified. It uses render
meshes for display. Its collision queries use analytic fields or nTop Core;
OBJ import and voxel collision are disabled.

```powershell
cmake -S ntop_sdf -B build-demo -G Ninja -DCMAKE_BUILD_TYPE=Release -DCMAKE_C_COMPILER=C:/msys64/ucrt64/bin/gcc.exe -DCMAKE_CXX_COMPILER=C:/msys64/ucrt64/bin/g++.exe -DNTOP_SDF_BUILD_DEMO=ON -DNTOP_CORE_ROOT=D:/nTop/mujoco/build-ntop-core-sdk
cmake --build build-demo --target ntop_sdf_demo --parallel 6
./build-demo/ntop_sdf_demo.exe
```

To start with the SDK gyroid sample already selected:

```powershell
./build-demo/ntop_sdf_demo.exe ./build-ntop-core-sdk/examples/assets/gyroid_sphere_5mm_radius.implicit
```

To reuse the independent reference checkout instead of downloading the viewer
dependencies, add `-DNTOP_SDF_REFERENCE_SOURCE=D:/nTop/sdf-dcd-reference` to
the configure command. The viewer shows preset analytic scenes. Select a shape,
adjust the scale, and drop it. Left drag selects and moves a body; right drag
orbits, and the middle button pans. The selected-body panel changes position
and rotation. Enable contact points and normals under Visualization. Collision
and full-step timings appear under Statistics. Choose **Open .implicit** to load
an nTop export. The viewer centers the part and scales its longest bounding-box
dimension to one scene unit. It displays the scale factor. This is a diagnostic
scale, not a simulation in the exported physical units. The viewer uses the
adapted standalone solver; it does not show elastic deformation. Without
`NTOP_CORE_ROOT`, the analytic scenes still work and the file button is absent.

## Build the MuJoCo adapter

Build the fork first. The following Windows GCC configuration suppresses two
upstream Windows format warnings that otherwise fail under `-Werror`. MuJoCo
downloads its pinned dependencies during configuration.

```powershell
cmake -S . -B build-ntop -G Ninja -DCMAKE_BUILD_TYPE=Release -DCMAKE_C_COMPILER=C:/msys64/ucrt64/bin/gcc.exe -DCMAKE_CXX_COMPILER=C:/msys64/ucrt64/bin/g++.exe -DCMAKE_CXX_FLAGS="-Wno-error=format -Wno-error=format-extra-args" -DMUJOCO_BUILD_EXAMPLES=OFF -DMUJOCO_BUILD_SIMULATE=OFF -DMUJOCO_BUILD_TESTS=OFF
cmake --build build-ntop --parallel 6
cmake -S ntop_sdf -B build-ntop-sdf -G Ninja -DCMAKE_BUILD_TYPE=Release -DMUJOCO_BUILD_DIR=D:/nTop/mujoco/build-ntop
cmake --build build-ntop-sdf --parallel 6
ctest --test-dir build-ntop-sdf --output-on-failure
./build-ntop-sdf/mujoco_bench.exe 10 10 20 60 cached 40 20 0.005
```

Change `MUJOCO_BUILD_DIR` to the absolute path of your MuJoCo build. On
Windows, reconfigure the adapter after rebuilding MuJoCo so CMake recopies the
DLL. The adapter is linked into the host program as `ntop_mujoco`; it is not a
standalone DLL that can resolve arbitrary field callbacks in another process.

Register the plugin and a `FieldPtr` before compiling MJCF. Use plugin name
`ntop.sdf.callback` and the returned numeric `field_id` as its config value.
After `mj_makeData`, call `attach_mujoco_collision(model, data)`. Call
`detach_mujoco_collision(data)` before freeing that data, then unregister the
field after all models and data that use it are gone. The executable example
is [mujoco_plugin_test.cpp](tests/mujoco_plugin_test.cpp).

Attachment replaces collision-table entries process-wide, but dispatches other
models to the original functions. Install it during setup, before other
threads step MuJoCo. Each `mjData` needs its own attachment. Do not modify the
model or detach data while it is being stepped. The attached path rejects
potential SDF collisions against planes, triangle meshes, height fields,
ellipsoids, other SDF plugins, and flex bodies. Use a box floor. Check
`mujoco_collision_statistics(data).failures` after stepping.

MuJoCo compiles an SDF display mesh once. This fork uses the evaluator's
conservative bounds for SDF broad-phase collision. The attached collider uses
field queries for its supported pairs. Supply explicit body inertia when you
do not want inertia inferred from the display mesh.

## Field interface

See [field.hpp](include/ntop_sdf/field.hpp) and
[world.hpp](include/ntop_sdf/world.hpp). A field has a negative interior, a
conservative local bounding box, a value, and an optional gradient. It must
accept points outside its bounds. `callback` accepts batched values and an
optional batched gradient. Without a gradient, it uses central differences.
Callers own captured resources and must make callbacks reentrant when needed.
Bounds must remain valid for the life of a compiled model.
The implementation requires a finite conservative envelope. If an external
evaluator does not provide bounds, the caller must provide an envelope; there
is no automatic conservative fallback for an arbitrary unbounded function.
The current optimizer and MuJoCo plugin query one point at a time. The
finite-difference fallback batches its six samples. The interface permits a
future batch-oriented search or GPU backend; it does not provide one today.

For fields that are not exact signed distances, the collision adapter uses
`f / |grad f|` as a local distance approximation. It is a first-order local
correction, not an exact distance transform. The standalone solver counts
near-zero gradients. The cached MuJoCo path substitutes a fallback normal for
them but does not count them. The standalone solver throws on non-finite values
or gradients. The cached MuJoCo path records a failure and drops that collision
pair. The stock MuJoCo fallback can map a bad value to a distant field value.
Validate callbacks before simulation and check the failure counter. The
standalone solver uses single-precision collision and rigid-body state. Field
callbacks use doubles.

The standalone `World` exposes transforms, velocities, density, mass, inertia,
contacts, and timing statistics. If mass and inertia are not supplied, it
samples the enclosed volume to estimate them. `PairQuery` retains a contact
manifold across calls. Boolean operations and bend, twist, and displacement
operators work on fields. Displacement requires user-provided conservative
bounds.

## Reproduce measurements

The benchmark arguments are body count, warmup steps, measured steps, and
target rate. MuJoCo also takes `native`, `field`, `cached`, or `core`, followed by the
SDF initialization-point and iteration counts and the per-geom contact margin.
Results are single-threaded CPU measurements. They are not GPU results.

For the nTop Core comparison, use the SDK's 5 mm sphere fixture. The benchmark
scales it by 100 to match the 0.5 m radius analytic sphere. It runs native
MuJoCo spheres, cached analytic SDFs, and cached nTop Core SDFs in the same
scene. This command records 10, 100, and 1,000 bodies at all three rates:

```powershell
./ntop_sdf/scripts/run_core_benchmarks.ps1 -SphereImplicit ./build-ntop-core-sdk/examples/assets/sphere_5mm_radius.implicit
```

The recorded results are [10 to 1,000 bodies](results/ntop_core_sphere_benchmarks.csv)
and [10,000 bodies at 60 Hz](results/ntop_core_10000_benchmarks.csv). These
short runs compare full simulation steps. Different contact algorithms generate
different numbers of contacts. They do not measure rendering or setup time.

```powershell
./build-ntop-sdf/ntop_sdf_bench.exe 100 10 20 120
./build-ntop-sdf/mujoco_bench.exe 100 10 20 120 native 40 20 0.005
./build-ntop-sdf/mujoco_bench.exe 100 10 20 120 field 40 20 0.005
./build-ntop-sdf/mujoco_bench.exe 100 10 20 120 cached 40 20 0.005
./build-ntop-sdf/ntop_sdf_accuracy.exe
```

To run the full Windows sweep into a new timestamped directory:

```powershell
./ntop_sdf/scripts/run_benchmarks.ps1
```

Add `-IncludeStockSdf` to measure the slower stock MuJoCo SDF collider too.
Use the script's `-Bodies`, `-Rates`, `-Warmup`, and `-Steps` parameters for
smaller runs.

The raw CSVs and the baseline reproduction instructions are in
[results](results/) and [reference/README.md](reference/README.md). The
collision-only and full-step columns measure different work and must not be
interchanged.
