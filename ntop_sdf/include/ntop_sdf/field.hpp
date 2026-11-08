#pragma once

#include <functional>
#include <memory>
#include <span>

namespace ntop_sdf {

struct Vec3 { double x{}, y{}, z{}; };
struct Bounds { Vec3 min, max; };

// Fields use local coordinates, with negative values inside. Bounds must enclose
// the entire zero set. Callbacks must accept arbitrary points, including points
// outside those bounds during gradient estimation and contact optimization.
class Field {
 public:
  virtual ~Field() = default;
  virtual Bounds bounds() const = 0;
  virtual double value(Vec3 p) const = 0;
  virtual Vec3 gradient(Vec3 p) const;
  virtual bool exact_distance() const { return false; }
  virtual bool convex() const { return false; }
  virtual void values(std::span<const Vec3> points, std::span<double> out) const;
  virtual void gradients(std::span<const Vec3> points, std::span<Vec3> out) const;
};

using FieldPtr = std::shared_ptr<const Field>;
using ValueBatch = std::function<void(std::span<const Vec3>, std::span<double>)>;
using GradientBatch = std::function<void(std::span<const Vec3>, std::span<Vec3>)>;

// The callback owner must keep captured objects alive and make callbacks
// reentrant if the same field is evaluated concurrently. Gradients are optional.
FieldPtr callback(Bounds bounds, ValueBatch value, GradientBatch gradient = {});
FieldPtr sphere(double radius);
FieldPtr box(Vec3 half_extents);
FieldPtr capsule(double radius, double half_length);  // axis: y
FieldPtr cylinder(double radius, double half_height);  // axis: y
FieldPtr torus(double major_radius, double minor_radius);  // axis: y
FieldPtr translated(FieldPtr field, Vec3 offset);
FieldPtr unite(FieldPtr a, FieldPtr b);
FieldPtr intersect(FieldPtr a, FieldPtr b);
FieldPtr subtract(FieldPtr a, FieldPtr b);
FieldPtr bend(FieldPtr field, double radians_per_unit);
FieldPtr twist(FieldPtr field, double radians_per_unit);
// A displacement may create zero sets outside the original bounds. The caller
// must supply conservative bounds for the resulting field.
FieldPtr displace(FieldPtr field, double amplitude, double frequency, Bounds bounds);

}  // namespace ntop_sdf
