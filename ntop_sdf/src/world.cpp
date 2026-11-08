#include "ntop_sdf/world.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <unordered_map>

#include "solver.h"

namespace ntop_sdf {
namespace {
float checked_float(double v) {
  float f=static_cast<float>(v);
  if(!std::isfinite(f)) throw std::domain_error("value exceeds single-precision range");
  return f;
}
float3 as_float(Vec3 v) { return {checked_float(v.x),checked_float(v.y),checked_float(v.z)}; }
Vec3 as_vec(float3 v) { return {v.x,v.y,v.z}; }
quat as_float(Quaternion q) {
  double len=std::sqrt(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w);
  if(!std::isfinite(len)||len<1e-10) throw std::invalid_argument("invalid orientation");
  return {static_cast<float>(q.x/len),static_cast<float>(q.y/len),static_cast<float>(q.z/len),static_cast<float>(q.w/len)};
}
Quaternion as_quat(quat q) { return {q.x,q.y,q.z,q.w}; }
bool finite(Vec3 v) { return std::isfinite(v.x)&&std::isfinite(v.y)&&std::isfinite(v.z); }

struct FieldShape final : Shape {
  FieldPtr field;
  mutable std::size_t queries{}, degeneracies{};
  explicit FieldShape(FieldPtr f) : field(std::move(f)) {
    auto b=field->bounds();
    if (!finite(b.min)||!finite(b.max)||b.min.x>=b.max.x||b.min.y>=b.max.y||b.min.z>=b.max.z)
      throw std::invalid_argument("invalid field bounds");
    bounds={as_float(b.min),as_float(b.max)};
    if(!(bounds.min.x<bounds.max.x && bounds.min.y<bounds.max.y && bounds.min.z<bounds.max.z))
      throw std::invalid_argument("field bounds collapse at single precision");
    convex=field->convex();
    // Shape::finalize samples the field for volume, inertia and seed points.
    // It does not use or generate a surface mesh for this adapter.
    finalize();
    if (!(volume>0) || !std::isfinite(volume)) throw std::invalid_argument("field has no measurable interior");
  }
  float sdf(float3 p, float3* grad=nullptr) const override {
    ++queries;
    Vec3 x=as_vec(p); Vec3 g{}; double v;
    if (grad || !field->exact_distance()) {
      v=field->value_gradient(x,g);
      if (!finite(g)) throw std::domain_error("field returned a non-finite gradient");
      if (!field->exact_distance()) {
        double n=std::hypot(g.x,g.y,g.z);
        if(!std::isfinite(n)) throw std::domain_error("field gradient is too large");
        if (n>1e-12) { v/=n; g={g.x/n,g.y/n,g.z/n}; }
        else { ++degeneracies; g={0,1,0}; }
      }
      if (grad) *grad=as_float(g);
    } else v=field->value(x);
    if (!std::isfinite(v)) throw std::domain_error("field returned a non-finite value");
    return checked_float(v);
  }
};
}  // namespace

struct World::Impl {
  Solver solver;
  std::unordered_map<const Field*,std::shared_ptr<FieldShape>> shapes;
  Statistics stats;
};
struct PairQuery::Impl {
  std::shared_ptr<FieldShape> a_shape,b_shape;
  std::unique_ptr<Body> a,b;
  Manifold manifold;
  CollisionParams params;
  Random rng;
  Impl(FieldPtr fa,FieldPtr fb) {
    if(!fa||!fb) throw std::invalid_argument("pair fields are required");
    a_shape=std::make_shared<FieldShape>(std::move(fa));
    b_shape=std::make_shared<FieldShape>(std::move(fb));
    a=std::make_unique<Body>(a_shape,float3{0,0,0},quat{0,0,0,1},1,true);
    b=std::make_unique<Body>(b_shape,float3{0,0,0},quat{0,0,0,1},1,true);
    manifold.bodyA=a.get(); manifold.bodyB=b.get();
    const float scale=std::min(maxComponent(size(a_shape->bounds)),maxComponent(size(b_shape->bounds)));
    params.tolerance=std::min(1e-4f,scale*1e-4f); params.epsilon=0; params.maxIterations=150;
  }
};
PairQuery::PairQuery(FieldPtr a,FieldPtr b) : impl_(std::make_unique<Impl>(std::move(a),std::move(b))) {}
PairQuery::~PairQuery()=default;
PairQuery::PairQuery(PairQuery&&) noexcept=default;
PairQuery& PairQuery::operator=(PairQuery&&) noexcept=default;
void PairQuery::set_pose(std::size_t id,Vec3 position,Quaternion q) {
  double q2=q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w;
  if(id>1||!finite(position)||!std::isfinite(q2)||q2<1e-20) throw std::invalid_argument("invalid pair pose");
  auto& b=id?*impl_->b:*impl_->a; b.orientation=normalize(as_float(q));
  b.position=as_float(position)+rotate(b.orientation,b.shape->centerOfMass*b.scale); b.updateBounds();
  // Reusing seeds across a pose change is useful; stale impulses are not.
  for(auto& p:impl_->manifold.points) {p.normalImpulse=0;p.frictionImpulse={0,0,0};}
}
void PairQuery::set_margin(double margin) {
  if(!std::isfinite(margin)||margin<0) throw std::invalid_argument("invalid margin");
  impl_->params.margin=checked_float(margin);
}
std::vector<Contact> PairQuery::query() {
  auto& p=impl_->params;
  if(!overlaps(expand(impl_->a->bounds,p.margin*.5f),expand(impl_->b->bounds,p.margin*.5f))) {
    impl_->manifold.points.clear(); return {};
  }
  CollisionStats s; impl_->manifold.update(p,0,impl_->rng,s);
  std::vector<Contact> out;
  for(const auto& c:impl_->manifold.points) if(c.active)
    out.push_back({0,1,as_vec(c.x),as_vec(c.normal),c.separation,c.cached});
  return out;
}
World::World() : impl_(std::make_unique<Impl>()) {}
World::~World()=default;
World::World(World&&) noexcept=default;
World& World::operator=(World&&) noexcept=default;

std::size_t World::add(const BodyDesc& desc) {
  if (!desc.field || !finite(desc.position) || !finite(desc.velocity) || !finite(desc.angular_velocity) ||
      !std::isfinite(desc.density) || desc.density<=0 || !std::isfinite(desc.friction) || desc.friction<0 ||
      !std::isfinite(desc.restitution) || desc.restitution<0 || desc.restitution>1 ||
      !std::isfinite(desc.mass) || desc.mass<0 || !finite(desc.diagonal_inertia))
    throw std::invalid_argument("invalid body properties");
  auto I=desc.diagonal_inertia;
  bool custom_inertia=I.x!=0 || I.y!=0 || I.z!=0;
  if(custom_inertia && (!(I.x>0&&I.y>0&&I.z>0) || I.x+I.y<I.z || I.x+I.z<I.y || I.y+I.z<I.x))
    throw std::invalid_argument("invalid principal inertia");
  double q2=desc.rotation.x*desc.rotation.x+desc.rotation.y*desc.rotation.y+
            desc.rotation.z*desc.rotation.z+desc.rotation.w*desc.rotation.w;
  if (!std::isfinite(q2)||q2<1e-20) throw std::invalid_argument("invalid body orientation");
  auto it=impl_->shapes.find(desc.field.get());
  if (it==impl_->shapes.end()) it=impl_->shapes.emplace(desc.field.get(),std::make_shared<FieldShape>(desc.field)).first;
  auto body=std::make_unique<Body>(it->second,as_float(desc.position),as_float(desc.rotation),1.0f,desc.is_static);
  if (!desc.is_static) {
    float scale=checked_float(desc.mass>0 ? desc.mass/body->mass : desc.density);
    if(!(scale>0)) throw std::invalid_argument("body mass is below single-precision range");
    body->mass*=scale;
    body->inertia=body->inertia*scale;
    if(custom_inertia) body->inertia=diagonal(as_float(I));
    double a=body->inertia[0][0],b=body->inertia[0][1],c=body->inertia[0][2];
    double d=body->inertia[1][1],e=body->inertia[1][2],f=body->inertia[2][2];
    double det=a*(d*f-e*e)-b*(b*f-e*c)+c*(b*e-d*c);
    if(!std::isfinite(body->mass)||!(body->mass>0)||!std::isfinite(det)||!(a>0&&a*d-b*b>0&&det>0))
      throw std::invalid_argument("body mass or inertia is not finite and positive definite");
    body->updateInertia();
  }
  body->friction=checked_float(desc.friction);
  body->restitution=static_cast<float>(desc.restitution);
  body->velocity=as_float(desc.velocity);
  body->angularVelocity=as_float(desc.angular_velocity);
  impl_->solver.add(body.release());
  return impl_->solver.bodies.size()-1;
}
void World::gravity(Vec3 g) { if(!finite(g)) throw std::invalid_argument("invalid gravity"); impl_->solver.gravity=as_float(g); }
void World::collision_settings(double margin,double allowed,double tolerance) {
  if(!std::isfinite(margin)||!std::isfinite(allowed)||!std::isfinite(tolerance)||
     margin<0||allowed<0||allowed>margin||tolerance<=0)
    throw std::invalid_argument("invalid collision settings");
  const float m=checked_float(margin),a=checked_float(allowed),t=checked_float(tolerance);
  if((margin>0&&!(m>0))||(allowed>0&&!(a>0))||!(t>0))
    throw std::invalid_argument("collision setting is below single-precision range");
  auto& p=impl_->solver.params;
  p.margin=m; p.allowedPenetration=a; p.tolerance=t; p.epsilon=0;
  p.replacementThreshold=std::min(0.01f,std::max(p.tolerance,p.margin*.5f));
}
void World::set_pose(std::size_t id, Vec3 position, Quaternion rotation) {
  double q2=rotation.x*rotation.x+rotation.y*rotation.y+rotation.z*rotation.z+rotation.w*rotation.w;
  if(!finite(position)||!std::isfinite(q2)||q2<1e-20) throw std::invalid_argument("invalid pose");
  auto& b=*impl_->solver.bodies.at(id);
  b.orientation=normalize(as_float(rotation));
  b.position=as_float(position)+rotate(b.orientation,b.shape->centerOfMass*b.scale);
  b.updateInertia(); b.updateBounds();
  for(auto it=impl_->solver.manifolds.begin();it!=impl_->solver.manifolds.end();) {
    if(it->first.first==static_cast<int>(id)||it->first.second==static_cast<int>(id)) it=impl_->solver.manifolds.erase(it);
    else ++it;
  }
}
void World::set_velocity(std::size_t id, Vec3 linear, Vec3 angular) {
  if(!finite(linear)||!finite(angular)) throw std::invalid_argument("invalid velocity");
  auto& b=*impl_->solver.bodies.at(id); b.velocity=as_float(linear); b.angularVelocity=as_float(angular);
}
void World::step(double seconds) {
  if (!std::isfinite(seconds)||seconds<=0) throw std::invalid_argument("invalid timestep");
  impl_->stats = {};
  std::size_t q=0,d=0;
  for (const auto& [_,shape]:impl_->shapes) { q+=shape->queries; d+=shape->degeneracies; }
  impl_->solver.dt=checked_float(seconds);
  if(!(impl_->solver.dt>0)) throw std::invalid_argument("timestep is below single-precision range");
  auto start=std::chrono::steady_clock::now();
  impl_->solver.step();
  impl_->stats.total_ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count();
  for (const auto& [_,shape]:impl_->shapes) { impl_->stats.field_queries+=shape->queries; impl_->stats.degenerate_gradients+=shape->degeneracies; }
  impl_->stats.field_queries-=q;
  impl_->stats.degenerate_gradients-=d;
  const auto& s=impl_->solver.stats;
  impl_->stats.broad_ms=s.broadMilliseconds; impl_->stats.narrow_ms=s.milliseconds; impl_->stats.solve_ms=s.solveMilliseconds;
  impl_->stats.pairs=s.pairs; impl_->stats.contacts=s.contacts; impl_->stats.generated_contacts=s.generatedContacts;
  impl_->stats.objective_evaluations=s.queries;
}
BodyState World::body(std::size_t id) const {
  const auto& b=*impl_->solver.bodies.at(id);
  return {as_vec(b.toWorld({0,0,0})),as_vec(b.velocity),as_vec(b.angularVelocity),as_quat(b.orientation)};
}
MassProperties World::mass_properties(std::size_t id) const {
  const auto& b=*impl_->solver.bodies.at(id);
  MassProperties result; result.mass=b.mass; result.center_of_mass=as_vec(b.shape->centerOfMass);
  if(!b.isStatic) for(int i=0;i<3;i++) for(int j=0;j<3;j++) result.inertia[3*i+j]=b.inertia[i][j];
  return result;
}
std::vector<Contact> World::contacts() const {
  std::vector<Contact> result;
  for (const auto& [pair,manifold]:impl_->solver.manifolds)
    for (const auto& p:manifold.points) if (p.active)
      result.push_back({static_cast<std::size_t>(pair.first),static_cast<std::size_t>(pair.second),as_vec(p.x),as_vec(p.normal),p.separation,p.cached});
  return result;
}
Statistics World::statistics() const { return impl_->stats; }
}  // namespace ntop_sdf
