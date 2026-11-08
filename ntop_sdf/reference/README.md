# Independent Giles and Andrews baseline

The source checkout is separate from this fork and remains unmodified. The
benchmark driver links the original collision, optimizer, solver, shape, and
render-mesh source files. The primitive SDFs are queried directly for collision.
No OBJ or voxel SDF is used. The original GUI was also built and launched.

- Reference commit: `08539dd3ad5fa9b41fdd160c730747de0b510b4e`
- SDL commit: `98d1f3a45aae568ccd6ed5fec179330f47d4d356`
- ImGui commit: `f5befd2d29e66809cd1110a152e375a7f1981f06`
- Hardware: Intel Core Ultra 7 165H, 22 logical processors; NVIDIA RTX 5000 Ada.
- OS: Windows 10.0.26200. The measured solver is single-threaded on the CPU.
- Compiler: MSYS2 UCRT64 GCC 15.2.0; C++17; `-O3 -DNDEBUG`.
- Baseline GUI: built in Release and stayed running during a four-second launch check.
- Scenario: 10 to 10,000 spheres of radius 0.5 on a static box floor; initial
  centers at height 0.49, grid spacing 1.05; gravity is `(0,-9.81,0)`.
- Original solver defaults: four substeps and 20 constraint iterations per
  substep. The original deterministic RNG seed is `0x853c49e6748fea9b`.
- Each measurement has 10 warmup steps and 20 timed steps. Counts report the
  final substep of each step. Timings are short exploratory runs on a shared
  machine, not confidence intervals. Objective evaluations are not the total
  number of SDF calls. No GPU calculation was measured.

The `mean_narrow_ms` column measures manifold updates only. It excludes the
broad phase and dynamics. `mean_total_step_ms` times the entire solver step.
See [the raw CSV](../results/reference_windows_gcc15.csv).

Example reproduction from `D:\nTop` in PowerShell:

```powershell
git clone --recurse-submodules https://github.com/savant117/sdf-dcd-demo.git sdf-dcd-reference
git -C sdf-dcd-reference checkout 08539dd3ad5fa9b41fdd160c730747de0b510b4e
git -C sdf-dcd-reference submodule update --init --recursive
cmake -S sdf-dcd-reference -B sdf-dcd-reference/build-mingw -G Ninja -DCMAKE_BUILD_TYPE=Release -DCMAKE_C_COMPILER=C:/msys64/ucrt64/bin/gcc.exe -DCMAKE_CXX_COMPILER=C:/msys64/ucrt64/bin/g++.exe
cmake --build sdf-dcd-reference/build-mingw --parallel 6
cmake -S mujoco/ntop_sdf/reference -B mujoco/build-reference-bench -G Ninja -DCMAKE_BUILD_TYPE=Release -DREFERENCE_SOURCE=D:/nTop/sdf-dcd-reference -DCMAKE_CXX_COMPILER=C:/msys64/ucrt64/bin/g++.exe
cmake --build mujoco/build-reference-bench --parallel 4
./mujoco/build-reference-bench/reference_bench.exe 10 10 20 60
```

The last command takes dynamic body count, warmup steps, measured steps, and
nominal timestep rate. Run the GUI from its build directory to load its assets.
