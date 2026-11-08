#include "ntop_sdf/mujoco_plugin.hpp"

#include <atomic>
#include <cmath>
#include <cstdlib>
#include <limits>
#include <mutex>
#include <shared_mutex>
#include <stdexcept>
#include <string_view>
#include <unordered_map>

#include <mujoco/mjplugin.h>
#include <mujoco/mujoco.h>
#include "mujoco_internal.hpp"

namespace ntop_sdf {
namespace {
std::shared_mutex registry_mutex;
std::unordered_map<std::uint64_t,FieldPtr> registry;
std::atomic<std::uint64_t> next_id{1};
const char* attributes[]={"field_id"};

FieldPtr lookup(std::uint64_t id) {
  std::shared_lock lock(registry_mutex);
  auto it=registry.find(id); return it==registry.end()?nullptr:it->second;
}
std::uint64_t parse(const char* text) {
  if (!text || !*text || *text=='-') return 0;
  char* end=nullptr; auto id=std::strtoull(text,&end,10);
  return end && *end=='\0' && id<=9007199254740991ULL ? id : 0;
}
Vec3 point(const mjtNum p[3]) { return {p[0],p[1],p[2]}; }
FieldPtr* data(const mjData* d,int instance) { return reinterpret_cast<FieldPtr*>(d->plugin_data[instance]); }
double safe_value(const FieldPtr& f,Vec3 x,bool corrected) noexcept {
  try {
    double v=f->value(x);
    if(corrected&&!f->exact_distance()) {
      auto g=f->gradient(x); double n=std::sqrt(g.x*g.x+g.y*g.y+g.z*g.z);
      if(std::isfinite(n)&&n>1e-12) v/=n;
    }
    return std::isfinite(v)?v:mjMAXVAL;
  } catch (...) { return mjMAXVAL; }
}
}  // namespace

namespace detail {
FieldPtr field_for_geom(const mjModel* model,const mjData* d,int geom) {
  int instance=model->geom_plugin[geom];
  if(instance<0 || instance>=model->nplugin) return {};
  const mjpPlugin* plugin=mjp_getPluginAtSlot(model->plugin[instance]);
  if(!plugin || std::string_view(plugin->name)!="ntop.sdf.callback") return {};
  auto* f=data(d,instance); return f?*f:FieldPtr{};
}
}

std::uint64_t register_mujoco_field(FieldPtr field) {
  if(!field) throw std::invalid_argument("null field");
  auto b=field->bounds();
  if(!(std::isfinite(b.min.x)&&std::isfinite(b.min.y)&&std::isfinite(b.min.z)&&
       std::isfinite(b.max.x)&&std::isfinite(b.max.y)&&std::isfinite(b.max.z)&&
       b.min.x<b.max.x&&b.min.y<b.max.y&&b.min.z<b.max.z))
    throw std::invalid_argument("invalid field bounds");
  auto id=next_id.fetch_add(1);
  if(id>9007199254740991ULL) throw std::overflow_error("field handle range exhausted");
  std::unique_lock lock(registry_mutex); registry.emplace(id,std::move(field)); return id;
}
bool unregister_mujoco_field(std::uint64_t id) {
  std::unique_lock lock(registry_mutex); return registry.erase(id)!=0;
}
void register_mujoco_plugin() {
  static std::once_flag once;
  std::call_once(once,[] {
    mjpPlugin plugin; mjp_defaultPlugin(&plugin);
    plugin.name="ntop.sdf.callback"; plugin.capabilityflags=mjPLUGIN_SDF;
    plugin.nattribute=1; plugin.attributes=attributes;
    plugin.nstate=+[](const mjModel*,int){return 0;};
    plugin.init=+[](const mjModel* m,mjData* d,int i) {
      auto f=lookup(parse(mj_getPluginConfig(m,i,"field_id")));
      if(!f) return -1;
      d->plugin_data[i]=reinterpret_cast<uintptr_t>(new FieldPtr(std::move(f))); return 0;
    };
    plugin.destroy=+[](mjData* d,int i){delete data(d,i); d->plugin_data[i]=0;};
    plugin.copy=+[](mjData* dest,const mjModel*,const mjData* src,int i){*data(dest,i)=*data(src,i);};
    plugin.reset=+[](const mjModel*,mjtNum*,void*,int){};
    plugin.compute=+[](const mjModel*,mjData*,int,int){};
    plugin.sdf_distance=+[](const mjtNum p[3],const mjData* d,int i)->mjtNum {
      auto* f=data(d,i); return f&&*f?safe_value(*f,point(p),true):mjMAXVAL;
    };
    plugin.sdf_gradient=+[](mjtNum out[3],const mjtNum p[3],const mjData* d,int i) {
      out[0]=0; out[1]=1; out[2]=0;
      try {
        auto* f=data(d,i); if(!f||!*f) return;
        auto g=(*f)->gradient(point(p));
        double n=std::sqrt(g.x*g.x+g.y*g.y+g.z*g.z);
        if(!std::isfinite(n)||n<=1e-12) return;
        if(!(*f)->exact_distance()) {g.x/=n;g.y/=n;g.z/=n;}
        out[0]=g.x;out[1]=g.y;out[2]=g.z;
      } catch (...) {}
    };
    plugin.sdf_staticdistance=+[](const mjtNum p[3],const mjtNum* a)->mjtNum {
      auto f=lookup(static_cast<std::uint64_t>(a[0])); return f?safe_value(f,point(p),false):mjMAXVAL;
    };
    plugin.sdf_attribute=+[](mjtNum a[],const char*[],const char* v[]) {
      auto id=parse(v[0]); a[0]=id&&lookup(id)?static_cast<mjtNum>(id):std::numeric_limits<mjtNum>::quiet_NaN();
    };
    plugin.sdf_aabb=+[](mjtNum out[6],const mjtNum* a) {
      auto f=lookup(static_cast<std::uint64_t>(a[0]));
      if(!f) { for(int i=0;i<6;i++) out[i]=0; return; }
      auto b=f->bounds(); out[0]=(b.min.x+b.max.x)/2;out[1]=(b.min.y+b.max.y)/2;out[2]=(b.min.z+b.max.z)/2;
      out[3]=(b.max.x-b.min.x)/2;out[4]=(b.max.y-b.min.y)/2;out[5]=(b.max.z-b.min.z)/2;
    };
    if(mjp_registerPlugin(&plugin)<0) throw std::runtime_error("could not register MuJoCo plugin");
  });
}
}  // namespace ntop_sdf
