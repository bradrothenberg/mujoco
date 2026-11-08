#include "ntop_sdf/field.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

#include "shapes.h"

namespace ntop_sdf {
namespace {

void check(Bounds b) {
  if (!(std::isfinite(b.min.x) && std::isfinite(b.min.y) && std::isfinite(b.min.z) &&
        std::isfinite(b.max.x) && std::isfinite(b.max.y) && std::isfinite(b.max.z) &&
        b.min.x < b.max.x && b.min.y < b.max.y && b.min.z < b.max.z))
    throw std::invalid_argument("field bounds must be finite and have positive extent");
}
void check(FieldPtr f) { if (!f) throw std::invalid_argument("null field"); }
void positive(double x) { if (!(std::isfinite(x) && x > 0)) throw std::invalid_argument("dimension must be finite and positive"); }
float3 to_float(Vec3 p) { return {static_cast<float>(p.x), static_cast<float>(p.y), static_cast<float>(p.z)}; }
Vec3 to_vec(float3 p) { return {p.x, p.y, p.z}; }

class Callback final : public Field {
 public:
  Callback(Bounds b, ValueBatch v, GradientBatch g) : b_(b), v_(std::move(v)), g_(std::move(g)) {
    check(b_); if (!v_) throw std::invalid_argument("value callback is required");
  }
  Bounds bounds() const override { return b_; }
  double value(Vec3 p) const override { double out{}; v_({&p,1}, {&out,1}); return out; }
  Vec3 gradient(Vec3 p) const override {
    if (!g_) return Field::gradient(p);
    Vec3 out{}; g_({&p,1}, {&out,1}); return out;
  }
  void values(std::span<const Vec3> p, std::span<double> out) const override {
    if (p.size() != out.size()) throw std::invalid_argument("batch sizes differ"); v_(p,out);
  }
  void gradients(std::span<const Vec3> p, std::span<Vec3> out) const override {
    if (p.size() != out.size()) throw std::invalid_argument("batch sizes differ");
    if (g_) g_(p,out); else Field::gradients(p,out);
  }
 private:
  Bounds b_; ValueBatch v_; GradientBatch g_;
};

enum class PrimitiveType { sphere, box, capsule, cylinder, torus };
class Primitive final : public Field {
 public:
  Primitive(PrimitiveType t, Vec3 p, Bounds b) : t_(t), p_(p), b_(b) { check(b); }
  Bounds bounds() const override { return b_; }
  double value(Vec3 p) const override { return eval(p, nullptr); }
  Vec3 gradient(Vec3 p) const override { float3 g{}; eval(p, &g); return to_vec(g); }
  bool exact_distance() const override { return true; }
  bool convex() const override { return t_ != PrimitiveType::torus; }
 private:
  double eval(Vec3 p, float3* g) const {
    const auto q = to_float(p);
    switch (t_) {
      case PrimitiveType::sphere: return sdSphere(q, static_cast<float>(p_.x), g);
      case PrimitiveType::box: return sdBox(q, to_float(p_), g);
      case PrimitiveType::capsule: return sdCapsule(q, static_cast<float>(p_.x), static_cast<float>(p_.y), g);
      case PrimitiveType::cylinder: return sdCylinder(q, static_cast<float>(p_.x), static_cast<float>(p_.y), g);
      case PrimitiveType::torus: return sdTorus(q, static_cast<float>(p_.x), static_cast<float>(p_.y), g);
    }
    return 0;
  }
  PrimitiveType t_; Vec3 p_; Bounds b_;
};

enum class Op { translate, combine_union, combine_intersect, combine_subtract, bend, twist, displace };
class Composite final : public Field {
 public:
  Composite(Op op, FieldPtr a, FieldPtr b, Vec3 p, Bounds bounds)
      : op_(op), a_(std::move(a)), b_(std::move(b)), p_(p), bounds_(bounds) { check(a_); check(bounds_); }
  Bounds bounds() const override { return bounds_; }
  bool exact_distance() const override { return op_ == Op::translate && a_->exact_distance(); }
  bool convex() const override { return op_ == Op::translate && a_->convex(); }
  double value(Vec3 p) const override {
    switch(op_) {
      case Op::translate: return a_->value({p.x-p_.x,p.y-p_.y,p.z-p_.z});
      case Op::combine_union: return std::min(a_->value(p),b_->value(p));
      case Op::combine_intersect: return std::max(a_->value(p),b_->value(p));
      case Op::combine_subtract: return std::max(a_->value(p),-b_->value(p));
      case Op::bend: { double c=std::cos(p_.x*p.x), s=std::sin(p_.x*p.x); return a_->value({c*p.x-s*p.y,s*p.x+c*p.y,p.z}); }
      case Op::twist: { double c=std::cos(p_.x*p.y), s=std::sin(p_.x*p.y); return a_->value({c*p.x-s*p.z,p.y,s*p.x+c*p.z}); }
      case Op::displace: return a_->value(p)+p_.x*std::sin(p_.y*p.x)*std::sin(p_.y*p.y)*std::sin(p_.y*p.z);
    }
    return 0;
  }
  Vec3 gradient(Vec3 p) const override {
    if (op_ == Op::translate) return a_->gradient({p.x-p_.x,p.y-p_.y,p.z-p_.z});
    if (op_ == Op::combine_union || op_ == Op::combine_intersect || op_ == Op::combine_subtract) {
      const double av=a_->value(p), bv=b_->value(p)*(op_==Op::combine_subtract?-1:1);
      const bool take_a = op_==Op::combine_union ? av<=bv : av>=bv;
      Vec3 g=take_a?a_->gradient(p):b_->gradient(p);
      if (!take_a && op_==Op::combine_subtract) return {-g.x,-g.y,-g.z};
      return g;
    }
    if (op_ == Op::bend) {
      double c=std::cos(p_.x*p.x),s=std::sin(p_.x*p.x);
      Vec3 q{c*p.x-s*p.y,s*p.x+c*p.y,p.z}; auto g=a_->gradient(q);
      return {(c-p_.x*q.y)*g.x+(s+p_.x*q.x)*g.y,-s*g.x+c*g.y,g.z};
    }
    if (op_ == Op::twist) {
      double c=std::cos(p_.x*p.y),s=std::sin(p_.x*p.y);
      Vec3 q{c*p.x-s*p.z,p.y,s*p.x+c*p.z}; auto g=a_->gradient(q);
      return {c*g.x+s*g.z,-p_.x*q.z*g.x+g.y+p_.x*q.x*g.z,-s*g.x+c*g.z};
    }
    if (op_ == Op::displace) {
      auto g=a_->gradient(p); const double w=p_.y, k=p_.x*w;
      double sx=std::sin(w*p.x),sy=std::sin(w*p.y),sz=std::sin(w*p.z);
      return {g.x+k*std::cos(w*p.x)*sy*sz,g.y+k*sx*std::cos(w*p.y)*sz,g.z+k*sx*sy*std::cos(w*p.z)};
    }
    return Field::gradient(p);
  }
 private:
  Op op_; FieldPtr a_, b_; Vec3 p_; Bounds bounds_;
};

Bounds merged(Bounds a, Bounds b) {
  return {{std::min(a.min.x,b.min.x),std::min(a.min.y,b.min.y),std::min(a.min.z,b.min.z)},
          {std::max(a.max.x,b.max.x),std::max(a.max.y,b.max.y),std::max(a.max.z,b.max.z)}};
}
}  // namespace

Vec3 Field::gradient(Vec3 p) const {
  const Bounds b=bounds();
  const double scale=std::max({b.max.x-b.min.x,b.max.y-b.min.y,b.max.z-b.min.z});
  const double h=std::cbrt(std::numeric_limits<double>::epsilon())*scale;
  const Vec3 q[]={{p.x+h,p.y,p.z},{p.x-h,p.y,p.z},{p.x,p.y+h,p.z},
                  {p.x,p.y-h,p.z},{p.x,p.y,p.z+h},{p.x,p.y,p.z-h}};
  double f[6]{}; values(q,f);
  return {(f[0]-f[1])/(2*h),(f[2]-f[3])/(2*h),(f[4]-f[5])/(2*h)};
}
void Field::values(std::span<const Vec3> p, std::span<double> out) const {
  if (p.size()!=out.size()) throw std::invalid_argument("batch sizes differ");
  for (std::size_t i=0;i<p.size();++i) out[i]=value(p[i]);
}
void Field::gradients(std::span<const Vec3> p, std::span<Vec3> out) const {
  if (p.size()!=out.size()) throw std::invalid_argument("batch sizes differ");
  for (std::size_t i=0;i<p.size();++i) out[i]=gradient(p[i]);
}
FieldPtr callback(Bounds b, ValueBatch v, GradientBatch g) { return std::make_shared<Callback>(b,std::move(v),std::move(g)); }
FieldPtr sphere(double r) { positive(r); return std::make_shared<Primitive>(PrimitiveType::sphere,Vec3{r,0,0},Bounds{{-r,-r,-r},{r,r,r}}); }
FieldPtr box(Vec3 h) { positive(h.x); positive(h.y); positive(h.z); return std::make_shared<Primitive>(PrimitiveType::box,h,Bounds{{-h.x,-h.y,-h.z},h}); }
FieldPtr capsule(double r,double h) { positive(r); positive(h); return std::make_shared<Primitive>(PrimitiveType::capsule,Vec3{r,h,0},Bounds{{-r,-h-r,-r},{r,h+r,r}}); }
FieldPtr cylinder(double r,double h) { positive(r); positive(h); return std::make_shared<Primitive>(PrimitiveType::cylinder,Vec3{r,h,0},Bounds{{-r,-h,-r},{r,h,r}}); }
FieldPtr torus(double R,double r) { positive(R); positive(r); return std::make_shared<Primitive>(PrimitiveType::torus,Vec3{R,r,0},Bounds{{-R-r,-r,-R-r},{R+r,r,R+r}}); }
FieldPtr translated(FieldPtr f,Vec3 p) { check(f); auto b=f->bounds(); return std::make_shared<Composite>(Op::translate,f,nullptr,p,Bounds{{b.min.x+p.x,b.min.y+p.y,b.min.z+p.z},{b.max.x+p.x,b.max.y+p.y,b.max.z+p.z}}); }
FieldPtr unite(FieldPtr a,FieldPtr b) { check(a); check(b); return std::make_shared<Composite>(Op::combine_union,a,b,Vec3{},merged(a->bounds(),b->bounds())); }
FieldPtr intersect(FieldPtr a,FieldPtr b) { check(a); check(b); return std::make_shared<Composite>(Op::combine_intersect,a,b,Vec3{},a->bounds()); }
FieldPtr subtract(FieldPtr a,FieldPtr b) { check(a); check(b); return std::make_shared<Composite>(Op::combine_subtract,a,b,Vec3{},a->bounds()); }
FieldPtr bend(FieldPtr f,double k) { check(f); auto b=f->bounds(); double x=std::max(std::abs(b.min.x),std::abs(b.max.x)), y=std::max(std::abs(b.min.y),std::abs(b.max.y)); double r=std::hypot(x,y); return std::make_shared<Composite>(Op::bend,f,nullptr,Vec3{k,0,0},Bounds{{-r,-r,b.min.z},{r,r,b.max.z}}); }
FieldPtr twist(FieldPtr f,double k) { check(f); auto b=f->bounds(); double x=std::max(std::abs(b.min.x),std::abs(b.max.x)), z=std::max(std::abs(b.min.z),std::abs(b.max.z)); double r=std::hypot(x,z); return std::make_shared<Composite>(Op::twist,f,nullptr,Vec3{k,0,0},Bounds{{-r,b.min.y,-r},{r,b.max.y,r}}); }
FieldPtr displace(FieldPtr f,double a,double w,Bounds b) { check(f); return std::make_shared<Composite>(Op::displace,f,nullptr,Vec3{a,w,0},b); }
}  // namespace ntop_sdf
