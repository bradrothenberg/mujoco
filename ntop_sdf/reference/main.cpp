// Independent benchmark driver for the unmodified Giles and Andrews solver.
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <memory>

#include "solver.h"

int main(int argc, char** argv) {
  const int count = argc > 1 ? std::atoi(argv[1]) : 10;
  const int warmup = argc > 2 ? std::atoi(argv[2]) : 30;
  const int steps = argc > 3 ? std::atoi(argv[3]) : 120;
  const int hz = argc > 4 ? std::atoi(argv[4]) : 60;
  if (count < 1 || warmup < 0 || steps < 1 || hz < 1) {
    std::cerr << "usage: reference_bench [bodies>0] [warmup>=0] [steps>0] [hz>0]\n";
    return 1;
  }
  Solver solver;
  solver.dt = 1.0f / static_cast<float>(hz);
  auto sphere = std::make_shared<Sphere>(0.5f);
  sphere->finalize();
  const int side = static_cast<int>(std::ceil(std::sqrt(static_cast<float>(count))));
  const float half = side * 0.525f + 2.0f;
  auto floor = std::make_shared<Box>(float3{half, 0.5f, half});
  solver.add(new Body(floor, {0, -0.5f, 0}, {0, 0, 0, 1}, 1, true));
  for (int i = 0; i < count; ++i) {
    const float x = (i % side - (side - 1) * 0.5f) * 1.05f;
    const float z = (i / side - (side - 1) * 0.5f) * 1.05f;
    solver.add(new Body(sphere, {x, 0.49f, z}));
  }
  for (int i = 0; i < warmup; ++i) solver.step();
  double step_ms = 0, narrow_ms = 0, pairs = 0, contacts = 0, queries = 0;
  for (int i = 0; i < steps; ++i) {
    auto start = std::chrono::steady_clock::now();
    solver.step();
    step_ms += std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - start).count();
    narrow_ms += solver.stats.milliseconds;
    pairs += solver.stats.pairs;
    contacts += solver.stats.contacts;
    queries += solver.stats.queries;
  }
  std::cout << "engine,dynamic_bodies,static_bodies,hz,warmup_steps,measured_steps,substeps,solver_iterations,mean_total_step_ms,mean_narrow_ms,mean_last_substep_pairs,mean_last_substep_contacts,mean_objective_evaluations_per_step\n";
  std::cout << std::setprecision(9) << "giles_andrews," << count << ",1," << hz << ','
            << warmup << ',' << steps << ',' << solver.substeps << ',' << solver.iterations << ','
            << step_ms / steps << ',' << narrow_ms / steps << ',' << pairs / steps << ','
            << contacts / steps << ',' << queries / steps << '\n';
}
