#include "ntop_sdf/ntop_core.hpp"
#include "ntop_sdf/mujoco_plugin.hpp"
#include "../tools/utf8_argv.hpp"

#include <cmath>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string_view>
#include <mujoco/mujoco.h>

int main(int argc,char** argv) {
  using namespace ntop_sdf;
  try {
    if(argc<2 || argc>3) throw std::invalid_argument("usage: ntop_core_mujoco_test shape.implicit [gyroid|offset|offset_rotated]");
    const bool gyroid=argc==3 && std::string_view(argv[2])=="gyroid";
    const bool rotated=argc==3 && std::string_view(argv[2])=="offset_rotated";
    const bool offset=rotated || (argc==3 && std::string_view(argv[2])=="offset");
    if(argc==3&&!gyroid&&!offset) throw std::invalid_argument("unknown test mode");
    auto field=load_ntop_core_implicit(ntop_path_argument(1,argv[1]));
    if(offset) field=translated(field,{.011,.013,-.017});
    register_mujoco_plugin(); auto id=register_mujoco_field(field);
    std::ostringstream xml;
    xml << R"(<mujoco><compiler inertiafromgeom="false"/><extension><plugin plugin="ntop.sdf.callback"><instance name="shape"><config key="field_id" value=")"
        << id << R"("/></instance></plugin></extension><asset><mesh name="surface"><plugin instance="shape"/></mesh></asset>
        <option timestep="0.001" gravity="0 -9.81 0" sdf_initpoints="20"/><worldbody>
        <geom name="floor" type="box" size=".02 .002 .02" pos="0 -.002 0" margin=".00005"/>
        )";
    if(rotated) xml<<R"(<body name="ball" pos=".013 -.0065 .017" quat=".7071067811865476 0 0 .7071067811865476"><freejoint/><inertial pos=".011 .013 -.017" mass=".001" diaginertia="1e-8 1e-8 1e-8"/>)";
    else if(offset) xml<<R"(<body name="ball" pos="-.011 -.0085 .017"><freejoint/><inertial pos=".011 .013 -.017" mass=".001" diaginertia="1e-8 1e-8 1e-8"/>)";
    else xml<<R"(<body name="ball" pos="0 .0045 0"><freejoint/><inertial pos="0 0 0" mass=".001" diaginertia="1e-8 1e-8 1e-8"/>)";
    xml<<R"(<geom name="implicit" type="sdf" mesh="surface" margin=".00005"><plugin instance="shape"/></geom></body>
        </worldbody></mujoco>)";
    auto text=xml.str(); mjVFS vfs; mj_defaultVFS(&vfs);
    if(mj_addBufferVFS(&vfs,"ntop.xml",text.data(),static_cast<int>(text.size()))) throw std::runtime_error("cannot create XML buffer");
    char error[1024]{}; mjModel* model=mj_loadXML("ntop.xml",&vfs,error,sizeof(error)); mj_deleteVFS(&vfs);
    if(!model) throw std::runtime_error(error);
    mjData* data=mj_makeData(model); if(!data) throw std::runtime_error("mj_makeData failed");
    attach_mujoco_collision(model,data);
    mj_forward(model,data);
    bool hit=false;
    for(int i=0;i<data->ncon;i++) if(data->contact[i].dist<-.00035 && data->contact[i].dist>-.00065 &&
                                       std::abs(data->contact[i].frame[1])>.85) hit=true;
    if(!gyroid&&!hit) throw std::runtime_error("5 mm nTop sphere-floor penetration or normal incorrect");
    // Release from rest 1 mm above the floor, then check settling.
    data->qpos[1]=.006-(rotated?.011:offset?.013:0); mju_zero(data->qvel,model->nv); mj_forward(model,data);
    for(int i=0;i<1000;i++) mj_step(model,data);
    mj_forward(model,data);
    const int ball=mj_name2id(model,mjOBJ_BODY,"ball");
    const double center_y=data->xipos[3*ball+1];
    if(!std::isfinite(center_y) || std::abs(center_y-.005)>(gyroid?.0015:.0006) || std::abs(data->qvel[1])>.05)
      throw std::runtime_error("5 mm nTop shape did not settle on the floor: center y="+std::to_string(center_y)+" velocity="+std::to_string(data->qvel[1]));
    auto stats=mujoco_collision_statistics(data);
    if(stats.failures || !stats.generated_contacts) throw std::runtime_error("nTop MuJoCo collision failed");
    std::cout<<"PASS MuJoCo nTop Core "<<ntop_core_version()<<(gyroid?" gyroid":rotated?" rotated offset sphere":offset?" offset sphere":" sphere")<<" drop; final center y="<<center_y
             <<" contacts="<<stats.generated_contacts<<" field_queries="<<stats.field_queries<<'\n';
    detach_mujoco_collision(data); mj_deleteData(data); mj_deleteModel(model); unregister_mujoco_field(id);
    return 0;
  } catch(const std::exception& e) { std::cerr<<"FAIL MuJoCo nTop Core: "<<e.what()<<'\n'; return 1; }
}
