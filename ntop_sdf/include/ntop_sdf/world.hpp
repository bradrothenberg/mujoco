#pragma once

#include <cstddef>
#include <memory>
#include <vector>
#include "ntop_sdf/field.hpp"

namespace ntop_sdf {

struct Quaternion { double x{}, y{}, z{}, w{1}; };
struct BodyDesc {
  FieldPtr field;
  Vec3 position{};
  Quaternion rotation{};
  Vec3 velocity{};
  Vec3 angular_velocity{};
  double density{1.0};
  double mass{};  // zero uses sampled volume times density
  Vec3 diagonal_inertia{};  // all zero uses sampled inertia, in body axes
  double friction{0.5};
  double restitution{};
  bool is_static{};
};
struct BodyState { Vec3 position, velocity, angular_velocity; Quaternion rotation; };
struct MassProperties {
  double mass{};
  Vec3 center_of_mass{};  // relative to the field origin
  double inertia[9]{};   // row-major, body axes, about the center of mass
};
struct Contact {
  std::size_t a{}, b{};
  Vec3 position, normal;  // normal points from a to b
  double separation{};   // negative means overlap
  bool cached{};
  double penetration_depth() const { return separation < 0 ? -separation : 0; }
};
struct Statistics {
  double total_ms{}, broad_ms{}, narrow_ms{}, solve_ms{};
  int pairs{}, contacts{}, generated_contacts{}, objective_evaluations{};
  std::size_t field_queries{}, degenerate_gradients{};
};

// Independent pair query with a persistent manifold. Poses are field origins.
class PairQuery {
 public:
  PairQuery(FieldPtr a, FieldPtr b);
  ~PairQuery();
  PairQuery(PairQuery&&) noexcept;
  PairQuery& operator=(PairQuery&&) noexcept;
  PairQuery(const PairQuery&) = delete;
  PairQuery& operator=(const PairQuery&) = delete;
  void set_pose(std::size_t body, Vec3 position, Quaternion rotation = {});
  void set_margin(double margin);
  std::vector<Contact> query();
 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

// The current solver uses single precision and a fixed substep count. Field
// callbacks use doubles. One World is not safe to mutate from several threads.
class World {
 public:
  World();
  ~World();
  World(World&&) noexcept;
  World& operator=(World&&) noexcept;
  World(const World&) = delete;
  World& operator=(const World&) = delete;
  std::size_t add(const BodyDesc& body);
  void gravity(Vec3 g);
  // Distances are in world units. Configure these for small parts before
  // stepping. This also selects the unsmoothed contact objective.
  void collision_settings(double margin, double allowed_penetration, double tolerance);
  void set_pose(std::size_t id, Vec3 position, Quaternion rotation);
  void set_velocity(std::size_t id, Vec3 linear, Vec3 angular);
  void step(double seconds);
  BodyState body(std::size_t id) const;
  MassProperties mass_properties(std::size_t id) const;
  std::vector<Contact> contacts() const;
  Statistics statistics() const;
 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace ntop_sdf
