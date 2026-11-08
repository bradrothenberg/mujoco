#include "scenes.h"
#include "ntop_sdf/ntop_core.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <mutex>
#include <stdexcept>
#include <ntop_core/ntop_core.h>

namespace {
struct ImplicitShape final : Shape {
    ntop_sdf::FieldPtr field;
    explicit ImplicitShape(ntop_sdf::FieldPtr f) : field(std::move(f)) {
        auto b=field->bounds();
        bounds={{static_cast<float>(b.min.x),static_cast<float>(b.min.y),static_cast<float>(b.min.z)},
                {static_cast<float>(b.max.x),static_cast<float>(b.max.y),static_cast<float>(b.max.z)}};
        finalize();
        if(!(volume>0)) throw std::runtime_error("nTop field has no measurable interior");
    }
    float sdf(float3 p,float3* grad=nullptr) const override {
        ntop_sdf::Vec3 g{};
        double value=field->value_gradient({p.x,p.y,p.z},g);
        double length=std::hypot(g.x,g.y,g.z);
        if(!std::isfinite(value)||!std::isfinite(length)) throw std::runtime_error("nTop field returned non-finite values");
        if(length>1e-12) { value/=length; g={g.x/length,g.y/length,g.z/length}; }
        else g={0,1,0};
        if(grad) *grad={static_cast<float>(g.x),static_cast<float>(g.y),static_cast<float>(g.z)};
        return static_cast<float>(value);
    }
};
struct DisplayMesh {
    Mesh mesh;
    double scale;
    ntop_sdf::Vec3 center;
    bool failed{};
    std::mutex mutex;
};
void vertex(void* context,const ntop_core_vec3* v,uint32_t index) {
    auto& d=*static_cast<DisplayMesh*>(context);
    std::lock_guard lock(d.mutex);
    try {
        if(!v) { d.failed=true; return; }
        if(index>=d.mesh.positions.size()) d.mesh.positions.resize(static_cast<std::size_t>(index)+1);
        d.mesh.positions[index]={static_cast<float>(v->x*d.scale-d.center.x),
                                 static_cast<float>(v->y*d.scale-d.center.y),
                                 static_cast<float>(v->z*d.scale-d.center.z)};
    } catch(...) { d.failed=true; }
}
void triangle(void* context,uint32_t a,uint32_t b,uint32_t c) {
    auto& d=*static_cast<DisplayMesh*>(context);
    std::lock_guard lock(d.mutex);
    try { d.mesh.indices.insert(d.mesh.indices.end(),{a,b,c}); } catch(...) { d.failed=true; }
}
}  // namespace

ShapePtr importNtopImplicit(const std::string& path,std::string& message) {
    // Normalize the display scene to one world unit across. This is a demo
    // scale only. The MuJoCo adapter preserves metres by default.
    auto raw=ntop_sdf::load_ntop_core_implicit(path);
    auto b=raw->bounds();
    const double extent=std::max({b.max.x-b.min.x,b.max.y-b.min.y,b.max.z-b.min.z});
    if(!(std::isfinite(extent)&&extent>0)) throw std::runtime_error("invalid nTop bounds");
    const double scale=1/extent;
    auto scaled=ntop_sdf::load_ntop_core_implicit(path,scale);
    auto sb=scaled->bounds();
    ntop_sdf::Vec3 center{(sb.min.x+sb.max.x)*.5,(sb.min.y+sb.max.y)*.5,(sb.min.z+sb.max.z)*.5};
    auto centered=ntop_sdf::translated(scaled,{-center.x,-center.y,-center.z});
    auto shape=std::make_shared<ImplicitShape>(centered);

    // This mesh is used only by the renderer. Collision and mass sampling
    // always call ImplicitShape::sdf and the nTop Core evaluator.
    ntop_core_handle handle{};
    auto result=ntop_core_import_from_file(path.c_str(),&handle);
    if(result!=ntop_core_import_result::SUCCESS || !handle) throw std::runtime_error("cannot open nTop display source");
    DisplayMesh display{{},scale,center};
    ntop_core_generate_mesh_by_DC(handle,extent/64,0.1,&display,&display,nullptr,vertex,triangle,nullptr);
    ntop_core_release(handle);
    if(display.failed||display.mesh.positions.empty()||display.mesh.indices.empty())
        throw std::runtime_error("nTop could not create a display surface");
    for(auto i:display.mesh.indices) if(i>=display.mesh.positions.size()) throw std::runtime_error("invalid display indices");
    display.mesh.normals.resize(display.mesh.positions.size());
    display.mesh.computeNormals();
    shape->mesh=std::move(display.mesh);
    char text[180];
    std::snprintf(text,sizeof(text),"Display scaled %.6g x; collision queries the nTop field.",scale);
    message=text;
    return shape;
}
