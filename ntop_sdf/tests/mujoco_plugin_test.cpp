#include "ntop_sdf/mujoco_plugin.hpp"

#include <cmath>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>

#include <mujoco/mujoco.h>

int main() {
  using namespace ntop_sdf;
  try {
    auto field=callback({{-1,-1,-1},{1,1,1}},[](auto points,auto values){
      for(std::size_t i=0;i<points.size();++i) {
        auto p=points[i]; values[i]=7*(std::sqrt(p.x*p.x+p.y*p.y+p.z*p.z)-.5);
      }
    });
    register_mujoco_plugin(); auto id=register_mujoco_field(field);
    std::ostringstream xml;
    xml << R"(<mujoco><compiler inertiafromgeom="false"/><extension><plugin plugin="ntop.sdf.callback"><instance name="shape"><config key="field_id" value=")"
        << id << R"("/></instance></plugin></extension><asset><mesh name="surface"><plugin instance="shape"/></mesh></asset>
        <option timestep="0.004166666666666667" gravity="0 -9.81 0" sdf_initpoints="20" sdf_iterations="20"/>
        <worldbody><geom name="floor" type="box" size="10 .5 3" pos="0 -.5 0"/>)";
    for(int i=0;i<4;i++) {
      xml << "<body pos=\"" << (i-1.5)*2.5 << " .49 0\"><freejoint/><inertial pos=\"0 0 0\" mass=\"1\" diaginertia=\".1 .1 .1\"/>";
      xml << "<geom name=\"implicit" << i << "\" type=\"sdf\" mesh=\"surface\"><plugin instance=\"shape\"/></geom></body>";
    }
    xml << "</worldbody></mujoco>";
    auto text=xml.str(); mjVFS vfs; mj_defaultVFS(&vfs);
    if(mj_addBufferVFS(&vfs,"test.xml",text.data(),static_cast<int>(text.size()))) throw std::runtime_error("cannot create XML buffer");
    char error[1024]{}; mjModel* model=mj_loadXML("test.xml",&vfs,error,sizeof(error)); mj_deleteVFS(&vfs);
    if(!model) throw std::runtime_error(error);
    const int sdf_geom=mj_name2id(model,mjOBJ_GEOM,"implicit0");
    if(sdf_geom<0 || std::abs(model->geom_aabb[6*sdf_geom+3]-1)>1e-12 ||
       std::abs(model->geom_aabb[6*sdf_geom+4]-1)>1e-12 ||
       std::abs(model->geom_aabb[6*sdf_geom+5]-1)>1e-12)
      throw std::runtime_error("compiled SDF bounds do not match conservative provider bounds");
    mjData* d=mj_makeData(model); if(!d) throw std::runtime_error("mj_makeData failed");
    const int floor=mj_name2id(model,mjOBJ_GEOM,"floor");
    auto type=model->geom_type[floor]; model->geom_type[floor]=mjGEOM_PLANE;
    bool rejected=false;
    try { attach_mujoco_collision(model,d); } catch(const std::invalid_argument&) { rejected=true; }
    model->geom_type[floor]=type;
    if(!rejected) throw std::runtime_error("mesh-based SDF-plane path was not rejected");
    attach_mujoco_collision(model,d);
    mj_forward(model,d);
    if(d->ncon<1) throw std::runtime_error("MuJoCo found no SDF-box contact");
    const auto* plugin=mjp_getPluginAtSlot(model->plugin[0]);
    mjtNum p[]={.6,0,0}, g[3]{};
    const double distance=plugin->sdf_distance(p,d,0); plugin->sdf_gradient(g,p,d,0);
    if(std::abs(distance-.1)>1e-6 || std::abs(g[0]-1)>1e-6) throw std::runtime_error("non-unit field correction failed");
    mjData* copy=mj_copyData(nullptr,model,d); if(!copy) throw std::runtime_error("mj_copyData failed");
    detach_mujoco_collision(d); mj_deleteData(d); d=copy;
    attach_mujoco_collision(model,d);
    for(int i=0;i<240;i++) mj_step(model,d);
    for(int i=0;i<4;i++)
      if(!std::isfinite(d->qpos[7*i+1])||std::abs(d->qpos[7*i+1]-.5)>.04)
        throw std::runtime_error("MuJoCo SDF sphere did not settle");
    // Move two SDF bodies above the floor and overlap them. This exercises the
    // SDF-SDF table entry rather than the box-SDF entry used by the drop test.
    d->qpos[1]=2; d->qpos[7]=d->qpos[0]+.9; d->qpos[8]=2; d->qpos[9]=d->qpos[2];
    mj_forward(model,d);
    int second=mj_name2id(model,mjOBJ_GEOM,"implicit1"); bool sdf_pair=false;
    for(int i=0;i<d->ncon;i++) {
      auto& c=d->contact[i];
      if(((c.geom[0]==sdf_geom&&c.geom[1]==second)||(c.geom[1]==sdf_geom&&c.geom[0]==second))&&c.dist<-.05) {
        double direction=c.geom[0]==sdf_geom?1:-1;
        if(direction*c.frame[0]>.9 && std::abs(c.dist+.1)<.04) sdf_pair=true;
      }
    }
    if(!sdf_pair) throw std::runtime_error("cached SDF-SDF collision was missed");
    std::cout<<"PASS MuJoCo callback SDF, conservative bounds, plane rejection, copy, settling, and SDF-SDF contact\n";
    if(mujoco_collision_failures(d)) throw std::runtime_error("cached collision reported a failure");
    detach_mujoco_collision(d); mj_deleteData(d); mj_deleteModel(model);
    if(!unregister_mujoco_field(id)) throw std::runtime_error("field unregister failed");
    return 0;
  } catch(const std::exception& e) { std::cerr<<"FAIL MuJoCo plugin: "<<e.what()<<'\n'; return 1; }
}
