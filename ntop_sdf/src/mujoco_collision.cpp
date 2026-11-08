#include "ntop_sdf/mujoco_plugin.hpp"
#include "mujoco_internal.hpp"

#include <algorithm>
#include <cmath>
#include <map>
#include <memory>
#include <mutex>
#include <shared_mutex>
#include <stdexcept>
#include <tuple>
#include <unordered_map>
#include <vector>

#include "collide.h"

namespace ntop_sdf {
namespace {
float3 fv(Vec3 v) { return {static_cast<float>(v.x),static_cast<float>(v.y),static_cast<float>(v.z)}; }
Vec3 dv(float3 v) { return {v.x,v.y,v.z}; }

// MuJoCo capsules and cylinders use z as their symmetry axis.
class ZAxis final : public Field {
 public:
  explicit ZAxis(FieldPtr f) : f_(std::move(f)) {}
  Bounds bounds() const override { auto b=f_->bounds(); return {{b.min.x,b.min.z,b.min.y},{b.max.x,b.max.z,b.max.y}}; }
  double value(Vec3 p) const override { return f_->value({p.x,p.z,p.y}); }
  Vec3 gradient(Vec3 p) const override { auto g=f_->gradient({p.x,p.z,p.y}); return {g.x,g.z,g.y}; }
  bool exact_distance() const override { return f_->exact_distance(); }
  bool convex() const override { return f_->convex(); }
 private: FieldPtr f_;
};

struct ProxyShape final : Shape {
  FieldPtr field;
  std::uint64_t* queries;
  ProxyShape(FieldPtr f,std::uint64_t* q) : field(std::move(f)), queries(q) {
    auto b=field->bounds(); bounds={fv(b.min),fv(b.max)}; convex=field->convex();
    if(!(std::isfinite(bounds.min.x)&&std::isfinite(bounds.min.y)&&std::isfinite(bounds.min.z)&&
         std::isfinite(bounds.max.x)&&std::isfinite(bounds.max.y)&&std::isfinite(bounds.max.z)&&
         bounds.min.x<bounds.max.x&&bounds.min.y<bounds.max.y&&bounds.min.z<bounds.max.z))
      throw std::invalid_argument("field bounds are invalid at single precision");
    finalize();
  }
  float sdf(float3 p,float3* grad=nullptr) const override {
    ++*queries;
    auto x=dv(p); double f; Vec3 g{};
    if(grad || !field->exact_distance()) {
      f=field->value_gradient(x,g);
      if(!std::isfinite(g.x)||!std::isfinite(g.y)||!std::isfinite(g.z))
        throw std::domain_error("non-finite SDF gradient");
      if(!field->exact_distance()) {
        const double n=std::hypot(g.x,g.y,g.z);
        if(!std::isfinite(n)) throw std::domain_error("SDF gradient is too large");
        if(n>1e-12) { f/=n;g={g.x/n,g.y/n,g.z/n}; }
        else g={0,1,0};
      }
      if(grad) {
        *grad=fv(g);
        if(!std::isfinite(grad->x)||!std::isfinite(grad->y)||!std::isfinite(grad->z))
          throw std::domain_error("SDF gradient exceeds single-precision range");
      }
    } else f=field->value(x);
    float value=static_cast<float>(f);
    if(!std::isfinite(value)) throw std::domain_error("SDF value exceeds single-precision range");
    return value;
  }
};

struct PairState {
  std::unique_ptr<Body> a,b;
  Manifold manifold;
  double last_time=-1;
};
using ShapeKey=std::tuple<int,int,double,double,double>;
struct Cache {
  explicit Cache(const mjModel* m) : model(m) { params.epsilon=0; }
  const mjModel* model;
  std::mutex mutex;
  std::map<ShapeKey,std::shared_ptr<ProxyShape>> shapes;
  std::map<std::pair<int,int>,PairState> pairs;
  CollisionParams params;
  Random rng;
  double swept_time=-1;
  std::uint64_t failures=0;
  std::uint64_t field_queries=0, generated_contacts=0, pair_queries=0;
};
std::shared_mutex caches_mutex;
std::unordered_map<const mjData*,std::shared_ptr<Cache>> caches;
std::once_flag install_once;
mjfCollision originals[mjNGEOMTYPES][mjNGEOMTYPES]{};

std::shared_ptr<Cache> cache_for(const mjData* d) {
  std::shared_lock lock(caches_mutex); auto it=caches.find(d); return it==caches.end()?nullptr:it->second;
}
std::shared_ptr<ProxyShape> shape_for(Cache& cache,const mjModel* m,const mjData* d,int g) {
  const int type=m->geom_type[g], instance=type==mjGEOM_SDF?m->geom_plugin[g]:-1;
  const mjtNum* s=m->geom_size+3*g;
  ShapeKey key{type,instance,s[0],s[1],s[2]};
  auto it=cache.shapes.find(key); if(it!=cache.shapes.end()) return it->second;
  FieldPtr f;
  switch(type) {
    case mjGEOM_SDF: f=detail::field_for_geom(m,d,g); break;
    case mjGEOM_BOX: f=box({s[0],s[1],s[2]}); break;
    case mjGEOM_SPHERE: f=sphere(s[0]); break;
    case mjGEOM_CAPSULE: f=std::make_shared<ZAxis>(capsule(s[0],s[1])); break;
    case mjGEOM_CYLINDER: f=std::make_shared<ZAxis>(cylinder(s[0],s[1])); break;
    default: throw std::invalid_argument("unsupported geometry for implicit collision");
  }
  if(!f) return {};
  auto result=std::make_shared<ProxyShape>(std::move(f),&cache.field_queries); cache.shapes.emplace(key,result); return result;
}
quat orientation(const mjData* d,int geom) {
  auto q=d->geom_xquat+4*geom;
  return normalize(quat{static_cast<float>(q[1]),static_cast<float>(q[2]),static_cast<float>(q[3]),static_cast<float>(q[0])});
}
void update_body(Body& b,const mjModel* m,const mjData* d,int g) {
  auto p=d->geom_xpos+3*g;
  b.orientation=orientation(d,g);
  float3 origin{static_cast<float>(p[0]),static_cast<float>(p[1]),static_cast<float>(p[2])};
  if(!std::isfinite(origin.x)||!std::isfinite(origin.y)||!std::isfinite(origin.z))
    throw std::domain_error("geom position exceeds single-precision range");
  b.position=origin+rotate(b.orientation,b.shape->centerOfMass);
  const int body=m->geom_bodyid[g];
  b.isStatic=body==0;
  mjtNum velocity[6]{}; mj_objectVelocity(m,d,mjOBJ_GEOM,g,velocity,0);
  b.angularVelocity={static_cast<float>(velocity[0]),static_cast<float>(velocity[1]),static_cast<float>(velocity[2])};
  b.velocity={static_cast<float>(velocity[3]),static_cast<float>(velocity[4]),static_cast<float>(velocity[5])};
  b.velocity+=cross(b.angularVelocity,b.position-origin);
  b.updateBounds();
}
int original(const mjModel* m,mjData* d,mjPreContact* con,int g1,int g2,mjtNum margin) {
  auto fn=originals[m->geom_type[g1]][m->geom_type[g2]];
  return fn?fn(m,d,con,g1,g2,margin):0;
}
int collide(const mjModel* m,mjData* d,mjPreContact* con,int g1,int g2,mjtNum margin) {
  auto cache=cache_for(d);
  if(!cache || cache->model!=m) return original(m,d,con,g1,g2,margin);
  if(m->geom_type[g2]==mjGEOM_SDF && !detail::field_for_geom(m,d,g2)) return original(m,d,con,g1,g2,margin);
  if(m->geom_type[g1]==mjGEOM_SDF && !detail::field_for_geom(m,d,g1)) return original(m,d,con,g1,g2,margin);
  std::lock_guard lock(cache->mutex);
  try {
    ++cache->pair_queries;
    if(cache->swept_time!=d->time) {
      // Keep recently separated pairs briefly, but do not grow the cache forever.
      for(auto it=cache->pairs.begin();it!=cache->pairs.end();) {
        if(it->second.last_time>d->time || d->time-it->second.last_time>10*m->opt.timestep) it=cache->pairs.erase(it);
        else ++it;
      }
      cache->swept_time=d->time;
    }
    auto& pair=cache->pairs[{g1,g2}];
    if(!pair.a) {
      auto sa=shape_for(*cache,m,d,g1),sb=shape_for(*cache,m,d,g2);
      if(!sa||!sb) return original(m,d,con,g1,g2,margin);
      pair.a=std::make_unique<Body>(sa,float3{0,0,0},quat{0,0,0,1},1,true);
      pair.b=std::make_unique<Body>(sb,float3{0,0,0},quat{0,0,0,1},1,true);
      pair.manifold.bodyA=pair.a.get(); pair.manifold.bodyB=pair.b.get();
    }
    update_body(*pair.a,m,d,g1); update_body(*pair.b,m,d,g2);
    float dt=pair.last_time>=0&&d->time>pair.last_time?static_cast<float>(d->time-pair.last_time):0;
    pair.last_time=d->time;
    auto params=cache->params;
    const float scale=std::min(maxComponent(size(pair.a->shape->bounds)),maxComponent(size(pair.b->shape->bounds)));
    params.tolerance=std::min(params.tolerance,scale*1e-4f);
    params.replacementThreshold=std::min(params.replacementThreshold,scale*.01f);
    params.margin=static_cast<float>(std::max(mjtNum(0),margin));
    if(!overlaps(expand(pair.a->bounds,params.margin*.5f),expand(pair.b->bounds,params.margin*.5f))) {
      pair.manifold.points.clear(); return 0;
    }
    CollisionStats stats; pair.manifold.update(params,dt,cache->rng,stats);
    int n=0, limit=std::min(mjMAXCONPAIR,std::max(0,m->opt.sdf_initpoints));
    for(const auto& p:pair.manifold.points) if(p.active && n<limit) {
      con[n].dist=p.separation;
      con[n].pos[0]=p.x.x;con[n].pos[1]=p.x.y;con[n].pos[2]=p.x.z;
      con[n].normal[0]=p.normal.x;con[n].normal[1]=p.normal.y;con[n].normal[2]=p.normal.z;
      con[n].tangent[0]=con[n].tangent[1]=con[n].tangent[2]=0;
      ++n;
    }
    cache->generated_contacts+=n;
    return n;
  } catch(const std::exception& e) {
    ++cache->failures; mju_warning("nTop SDF collision failed: %s",e.what()); return 0;
  } catch(...) { ++cache->failures; mju_warning("nTop SDF collision failed"); return 0; }
}
void install() {
  for(int type:{mjGEOM_SPHERE,mjGEOM_CAPSULE,mjGEOM_CYLINDER,mjGEOM_BOX,mjGEOM_SDF}) {
    originals[type][mjGEOM_SDF]=mjCOLLISIONFUNC[type][mjGEOM_SDF];
    mjCOLLISIONFUNC[type][mjGEOM_SDF]=collide;
  }
}
bool could_collide(const mjModel* m,int a,int b) {
  if(m->geom_bodyid[a]==m->geom_bodyid[b]) return false;
  return (m->geom_contype[a]&m->geom_conaffinity[b]) || (m->geom_contype[b]&m->geom_conaffinity[a]);
}
}  // namespace

void attach_mujoco_collision(const mjModel* model,mjData* d) {
  if(!model||!d) throw std::invalid_argument("model and data are required");
  if(model->nflex) throw std::invalid_argument("mesh-free SDF collision does not support MuJoCo flex");
  std::vector<int> implicit, unsupported;
  for(int g=0;g<model->ngeom;++g) {
    int t=model->geom_type[g];
    if(t==mjGEOM_SDF) {
      if(detail::field_for_geom(model,d,g)) implicit.push_back(g);
      else unsupported.push_back(g);
    } else if(t==mjGEOM_PLANE||t==mjGEOM_MESH||t==mjGEOM_HFIELD||t==mjGEOM_ELLIPSOID) {
      unsupported.push_back(g);
    }
  }
  for(int a:implicit) for(int b:unsupported) if(could_collide(model,a,b))
    throw std::invalid_argument("SDF pair has an unsupported geometry or plugin; use an implicit box floor");
  for(int i=0;i<model->npair;++i) {
    int a=model->pair_geom1[i],b=model->pair_geom2[i];
    bool a_implicit=std::binary_search(implicit.begin(),implicit.end(),a);
    bool b_implicit=std::binary_search(implicit.begin(),implicit.end(),b);
    bool a_unsupported=std::binary_search(unsupported.begin(),unsupported.end(),a);
    bool b_unsupported=std::binary_search(unsupported.begin(),unsupported.end(),b);
    if((a_implicit&&b_unsupported)||(b_implicit&&a_unsupported))
      throw std::invalid_argument("explicit SDF pair has an unsupported geometry or plugin");
  }
  std::call_once(install_once,install);
  std::unique_lock lock(caches_mutex);
  if(!caches.emplace(d,std::make_shared<Cache>(model)).second) throw std::invalid_argument("mjData already attached");
}
void detach_mujoco_collision(mjData* d) { std::unique_lock lock(caches_mutex); caches.erase(d); }
std::uint64_t mujoco_collision_failures(const mjData* d) {
  return mujoco_collision_statistics(d).failures;
}
MujocoCollisionStatistics mujoco_collision_statistics(const mjData* d) {
  auto cache=cache_for(d); if(!cache) return {};
  std::lock_guard lock(cache->mutex);
  return {cache->field_queries,cache->generated_contacts,cache->pair_queries,cache->failures};
}
}  // namespace ntop_sdf
