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
