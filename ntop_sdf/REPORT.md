# Native implicit contact: engineering report

## Assessment

The prototype supports direct implicit-field contact for the tested rigid-body
scenes. It combines a cached Nelder-Mead manifold with a sequential-impulse
solver. It can also send those contacts to MuJoCo. The tests and measurements
do not establish performance parity with MuJoCo's analytic primitive collider.
They do not establish robust contact for arbitrary non-convex fields.

An optional adapter now reads nTop `.implicit` exports through nTop Core. The
5 mm sphere and gyroid samples passed rigid contact tests in MuJoCo. The
diagnostic viewer also loads the gyroid and displays its contact points.

The elastic-deformation stage is not implemented. There is no inverse
deformation map, elastic material model, deformable contact feedback, or soft
body demo. The results cannot answer whether elastic implicit solids run at
interactive rates.

## nTop Core validation and performance

The test runtime was nTop Core 5.50.2. The adapter calls its documented C API
for import, bounds, values, derivatives, and array queries. It preserves nTop's
metre units by default. It releases each SDK handle when the field is destroyed.
Calls are serialized while the SDK's thread-safety contract is unconfirmed.
The standalone solver's collision settings can be set for small parts. The
MuJoCo cached collider now scales its optimizer tolerance to the smaller body.

The SDK sphere fixture has a 5 mm radius. Its SHA-256 is
`36d6b75fc8e1b6faaa2f5549d6d56a4b0cfcbacaad8ce079f07d858c152e25c8`.
The tests check its field, gradient, bounds, scale, batch results, sampled mass,
and 1 mm overlap with another sphere. They also run a standalone floor drop.
Other tests translate the sphere away from its field origin and rotate it
before the MuJoCo drop. Both tests check the world-space center of mass.
In MuJoCo the sphere starts 1 mm above a box floor. After 1 second, its center
is at 5.007 mm and its speed is below the test limit. A 0.5 mm initial
penetration is checked against the expected normal and separation.

The SDK gyroid sphere fixture has SHA-256
`6224d8d2140706f016579bc30eaaa0675cc8e7a0794da33cedf6160a8f59eec1`.
It settled at a center height of 4.895 mm in the same 1 second MuJoCo drop.
Its non-unit gradients exercise the local distance correction. The 1.5 mm
settling tolerance is a smoke-test limit, not a proof of accurate gyroid
contact. The viewer displays a separately generated surface. Its collider
queries nTop Core directly. Twelve other local exports also opened and were
queried, but those checks did not simulate them.

The following CPU measurements use the SDK sphere scaled 100 times to match
MuJoCo's 0.5 m analytic sphere. The box floor, initial positions, mass,
friction, gravity, timestep, four substeps, 20 solver iterations, 10 warmup
steps, and 20 measured steps match. Both geoms have a 0.005 m contact margin.
The native collider produces one contact per sphere; the SDF collider produces
about three in the 60 Hz cases. Thus, the results include both evaluator and
contact-manifold differences. They measure complete steps, not rendering or
model compilation. The machine was not isolated or CPU-pinned.

| Dynamic spheres | Native MuJoCo, 60 Hz step | nTop Core SDF, 60 Hz step |
| ---: | ---: | ---: |
| 10 | 0.135 ms | 1.548 ms |
| 100 | 1.188 ms | 12.807 ms |
| 1,000 | 10.241 ms | 145.427 ms |
| 10,000 | 163.143 ms | 1,511.565 ms |

At 100 spheres, the nTop Core path averaged 14.576 ms at 120 Hz and 19.130 ms
at 240 Hz. Those values exceed the 8.33 ms and 4.17 ms step budgets. The
10,000-body nTop process peaked at 425,574,400 bytes of working set, including
setup. The native process peaked at 176,205,824 bytes. See the raw
[10 to 1,000 body results](results/ntop_core_sphere_benchmarks.csv) and
[10,000 body results](results/ntop_core_10000_benchmarks.csv).

These results show feasible direct rigid contact for the tested small scenes.
They do not establish parity with MuJoCo, robust arbitrary implicit contact,
deformation, or GPU performance. The optimizer issues scalar SDK calls even
though the adapter exposes the SDK's batch interface. Thin features, deep
penetration, continuous collision, and zero-gradient points still need study.

## Method and integration boundary

The field interface accepts values, optional gradients, and a finite
conservative envelope. It provides a batch interface. Central differences use
one batch of six points when the caller does not supply a gradient. The present
contact optimizers otherwise request one point at a time. If an evaluator has
no bounds, the caller must supply an envelope. The implementation does not
infer a guaranteed bound from arbitrary function samples.

The standalone solver adapts the Giles and Andrews reference. It uses sort and
sweep, curvature-biased candidate points, cached contacts, Nelder-Mead search,
friction, sequential impulses, warm starting, and split position correction.
This fork adds a restitution target and a field adapter. Volume sampling
estimates mass, center of mass, inertia, and curvature seeds during setup.
Callers can instead set mass and diagonal inertia. The simulation and collision
state use single precision.

For a non-distance field, the adapter uses `f / |grad f|` as a local distance
estimate. It normalizes the gradient for the contact direction. This is a
first-order approximation. It need not be accurate for deep overlap or a
strongly distorted field. A near-zero gradient has no well-defined normal. The
standalone solver counts these events and uses a fallback direction. The
MuJoCo cached path uses a fallback direction but does not count them. A
non-finite callback result causes the standalone solver to throw. The MuJoCo
cached path records a failure and drops the affected pair.

MuJoCo's standard SDF model compiler constructs a display mesh. This fork uses
the field provider's conservative bounds for SDF collision bounds. When the
cached path is attached, supported primitive/SDF and SDF/SDF narrow-phase
queries evaluate fields directly. The attached path rejects potentially
colliding SDF/plane, SDF/mesh, SDF/height-field, SDF/ellipsoid, other SDF
plugins, and flex configurations. This avoids MuJoCo paths that can use a mesh
for SDF collision. Users must supply explicit body inertia to avoid inference
from the display mesh.

The collision-table hook is process-wide. An explicit attachment associates a
cache with each `mjData`; detach it before freeing that data. Install it before
other threads start stepping models. The cache serializes collision callbacks
within one data instance. Handles in MJCF are process-local and must be
registered again before the model is reloaded.

## Correctness checks

The Windows Release CTest suite has twelve cases when both MuJoCo and nTop Core
are enabled. Two cases check Unicode paths. A standalone-only build has two.
The standalone tests cover
primitive and warped-field gradients, a batched callback with finite
differences, sphere/sphere and sphere/box-floor contacts at zero gap and in
penetration, a deeper sphere overlap, cached pair queries, torus contact and
its hole, two tori, a twisted field, gravity, restitution, friction, mass and
inertia, pose changes, invalid data, and 200 falling spheres. A five-box stack
runs for five simulated seconds. Its checks bound height, lateral drift, and
velocity. These are regression thresholds, not a general stability proof.

The MuJoCo test registers a scaled callback field, checks the conservative
bounds, rejects a plane pair, copies `mjData`, settles four spheres on a box,
and checks an SDF/SDF contact and normal. The test uses explicit mass and
inertia. It checks the cached collision failure counter.

The separate [contact accuracy CSV](results/contact_accuracy.csv) contains 15
analytic sphere/sphere and sphere/floor cases. It includes touching cases and
overlap up to 0.8 length units. In this sample, the largest separation error is
approximately 0.000039 length units. The largest contact-position error is
approximately 0.00621 length units, and the largest normal error is about
0.995 degrees. Each case reports its own result. These numbers do not bound
errors for arbitrary shapes or feature sizes.

## Earlier benchmark method

The following measurements were taken before the nTop Core integration and
the smaller-scale collision tolerance changes. They are historical results
from the previous working milestone. Use the nTop Core results above for the
current comparison.

Measurements used an Intel Core Ultra 7 165H with 22 logical processors on
Windows 10.0.26200. The machine also has an NVIDIA RTX 5000 Ada Generation
Laptop GPU with 16,376 MiB of memory. No GPU computation was measured. Each
simulation used one CPU thread. The compiler was MSYS2 UCRT64 GCC 15.2.0.
Standalone code used `-O3 -DNDEBUG`. The MuJoCo Release build used `-O3`,
link-time optimization, and the Windows GCC warning options in the README.

The reference commit is `08539dd3ad5fa9b41fdd160c730747de0b510b4e`. The
MuJoCo base commit is `371c2b444582552c977bc696d135d552ddcda613`. The scene
places unit-density spheres of radius 0.5 on a static box. Sphere centers
start at height 0.49, with 1.05 grid spacing. Gravity is `(0,-9.81,0)`. The
standalone and MuJoCo comparison uses matching analytic sphere mass and inertia.
The unmodified reference uses its own sampled mass properties.

Each case uses 10 warmup steps and 20 measured steps. It uses four substeps per
step and 20 solver iterations per substep. Tests use nominal rates of 60, 120,
and 240 Hz. The same step counts cover different simulated durations at those
rates. The machine was not isolated or CPU-pinned. These short runs report
means, not tail latency or confidence intervals.

The standalone solver uses a 0.02 contact margin and permits 0.01 penetration
into that margin. MuJoCo uses a 0.005 margin per geom, producing a 0.01 pair
margin. The expected resting sphere center is therefore near 0.51. MuJoCo uses
PGS with pyramidal friction and its own compliant constraint model. The
standalone solver uses a different friction and stabilization implementation.
MuJoCo's native sphere collider usually emits one contact per sphere; the SDF
manifolds may emit more. Thus the scenes share geometry, mass, gravity, and
thread count, but the contact discretizations and solvers are not identical.

`mean_total_step_ms` includes the complete step. `mean_narrow_ms` measures
collision work only. The standalone `mean_solve_ms` includes constraint setup,
gravity, and solve work; it excludes integration. MuJoCo's constraint timer
measures its constraint stage. The timers are not interchangeable. Counts in
the `last_substep` columns are not totals across substeps. Field queries per
generated contact use totals over all substeps. They count field-interface
queries, not the six internal samples used by a finite-difference gradient.
Peak working set includes process and model setup. Model compilation and shape
preparation are excluded from step times.

After publication, a clean checkout of the integration branch built the
standalone library and passed its two tests. The same checkout built the
adapter against the existing local MuJoCo build and passed all three tests.

## Measured results

The table gives mean complete-step time in milliseconds at a 60 Hz nominal
rate. The raw files contain the 120 Hz and 240 Hz runs and stage timings.

| Dynamic spheres | Original reference | Standalone SDF | MuJoCo native sphere | MuJoCo cached SDF | MuJoCo stock SDF |
| ---: | ---: | ---: | ---: | ---: | ---: |
| 10 | 0.342 | 0.470 | 0.146 | 0.796 | 5.067 |
| 100 | 3.577 | 3.717 | 1.026 | 6.772 | 48.626 |
| 1,000 | 38.274 | 47.878 | 13.419 | 66.356 | 450.424 |
| 10,000 | 384.368 | 496.903 | 133.071 | 534.786 | not measured |

The 60 Hz budget is 16.67 ms per step. The standalone and cached MuJoCo paths
exceed that budget at 1,000 bodies in this test. The native sphere case meets
it at 1,000 bodies on the measured mean. The cached path takes about 4.9 times
as long as the native sphere case at that size. This is not a comparison of
equal contact counts or equivalent solvers.

The budgets for 120 Hz and 240 Hz are 8.33 ms and 4.17 ms per step. At 100
bodies, the standalone means are 4.284 ms and 4.595 ms. The cached MuJoCo
means are 5.981 ms and 7.221 ms. At 1,000 bodies, both paths exceed both
budgets. These are means from the short runs described above.

At 1,000 bodies, the cached MuJoCo SDF path takes less time than the stock
MuJoCo SDF search in this case. The stock search uses 40 initialization points
and 20 SDF iterations. The cached search uses the reference's four cached
points, one new sample, and its own optimizer settings. At the end of the
stock run, the lowest sphere center is about 0.425, which places its bottom
about 0.075 below the floor. The lowest cached-SDF sphere center is about
0.5085. The cached path's maximum final speed is about 0.086 length units per
second. This single scene demonstrates a benefit from caching, but it also
shows residual motion and does not prove general contact stability.

At 1,000 bodies and 60 Hz, the standalone run reports about 15 field queries
per generated contact. The cached MuJoCo run reports about 52. The definitions
and contact counts differ, so these are workload indicators rather than a
direct efficiency ranking. The 10,000-body peak working sets were about 33 MiB
for the standalone process, 163 MiB for MuJoCo native spheres, and 391 MiB for
the cached MuJoCo SDF case. These include setup and are not per-body allocation
measurements. No GPU timings are available.

## Limitations

- The sampled collision search is not exhaustive. It can miss small,
  disconnected, or deep intersections, especially with poor bounds or seeds.
- The code has no continuous collision detection. Fast bodies can tunnel.
- Restitution, friction, and stability have only the bounded tests above.
- Contact caches can grow with recently active pairs and serialize callbacks
  for a MuJoCo data instance. They are not a parallel SDF backend.
- The viewer shows analytic and nTop Core rigid-body scenes. It does not show
  arbitrary caller callbacks or elastic deformation. Visual checks covered
  the sphere/torus scene, an nTop gyroid, contact normals, body selection, and
  the transform controls.
- The tests do not cover every MuJoCo model feature, platform, scale, unit
  convention, or arbitrary user callback. Check the failure counter and
  validate each application.

## Sources and reproducibility

The reference implementation and its license are at
<https://github.com/savant117/sdf-dcd-demo>. The method is described at
<https://savant117.github.io/sdf-dcd-demo/>. The MuJoCo source is at
<https://github.com/google-deepmind/mujoco>. The independent reference build,
source revisions, and commands are in [reference/README.md](reference/README.md).
Raw measurements are in [results](results/). Commands and the benchmark sweep
script are in [README.md](README.md).
