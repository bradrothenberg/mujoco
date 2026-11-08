#include "ntop_sdf/ntop_core.hpp"
#include "ntop_sdf/mujoco_plugin.hpp"
#include "ntop_sdf/world.hpp"
#include "utf8_argv.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <mujoco/mujoco.h>

int main(int argc,char** argv) {
  using namespace ntop_sdf;
  try {
    if(argc<2||argc>5) throw std::invalid_argument("usage: ntop_mujoco_run file.implicit [density_kg_m3=1000] [seconds=1] [hz=240]");
    const double density=argc>2?std::stod(argv[2]):1000;
    const double seconds=argc>3?std::stod(argv[3]):1;
    const int hz=argc>4?std::stoi(argv[4]):240;
    if(!(std::isfinite(density)&&density>0&&std::isfinite(seconds)&&seconds>0&&seconds<=60&&hz>=30&&hz<=2000))
      throw std::invalid_argument("density, duration, or rate is out of range");

    auto field=load_ntop_core_implicit(ntop_path_argument(1,argv[1]));
    auto b=field->bounds();
    const double dx=b.max.x-b.min.x,dy=b.max.y-b.min.y,dz=b.max.z-b.min.z;
    const double length=std::max({dx,dy,dz});
    World mass_world;
    BodyDesc sampled; sampled.field=field; sampled.density=density;
    const auto mass=mass_world.mass_properties(mass_world.add(sampled));
    const double floor_half=4*length,thickness=.1*length, margin=.005*length;
    const double x=-(b.min.x+b.max.x)*.5, y=.25*length-b.min.y, z=-(b.min.z+b.max.z)*.5;

    register_mujoco_plugin(); const auto id=register_mujoco_field(field);
    std::ostringstream xml; xml<<std::setprecision(17);
    xml<<"<mujoco><compiler inertiafromgeom=\"false\"/><extension><plugin plugin=\"ntop.sdf.callback\"><instance name=\"shape\"><config key=\"field_id\" value=\""
       <<id<<"\"/></instance></plugin></extension><asset><mesh name=\"surface\"><plugin instance=\"shape\"/></mesh></asset>"
       <<"<option timestep=\""<<1.0/(4*hz)<<"\" gravity=\"0 -9.81 0\" sdf_initpoints=\"20\"/><worldbody>"
       <<"<geom type=\"box\" size=\""<<floor_half<<' '<<thickness<<' '<<floor_half<<"\" pos=\"0 "<<-thickness<<" 0\" margin=\""<<margin<<"\"/>"
       <<"<body name=\"part\" pos=\""<<x<<' '<<y<<' '<<z<<"\"><freejoint/><inertial pos=\""
       <<mass.center_of_mass.x<<' '<<mass.center_of_mass.y<<' '<<mass.center_of_mass.z<<"\" mass=\""<<mass.mass
       <<"\" fullinertia=\""<<mass.inertia[0]<<' '<<mass.inertia[4]<<' '<<mass.inertia[8]<<' '
       <<mass.inertia[1]<<' '<<mass.inertia[2]<<' '<<mass.inertia[5]<<"\"/>"
       <<"<geom type=\"sdf\" mesh=\"surface\" margin=\""<<margin<<"\"><plugin instance=\"shape\"/></geom></body></worldbody></mujoco>";
    auto text=xml.str(); mjVFS vfs; mj_defaultVFS(&vfs);
    if(mj_addBufferVFS(&vfs,"ntop.xml",text.data(),static_cast<int>(text.size()))) throw std::runtime_error("cannot create XML buffer");
    char error[1024]{}; mjModel* model=mj_loadXML("ntop.xml",&vfs,error,sizeof(error)); mj_deleteVFS(&vfs);
    if(!model) throw std::runtime_error(error);
    mjData* data=mj_makeData(model); if(!data) throw std::runtime_error("mj_makeData failed");
    attach_mujoco_collision(model,data);
    const int steps=static_cast<int>(std::ceil(seconds*hz))*4;
    int max_contacts=0;
    const auto start=std::chrono::steady_clock::now();
    for(int i=0;i<steps;i++) { mj_step(model,data); max_contacts=std::max(max_contacts,data->ncon); }
    const double elapsed=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count();
    mj_forward(model,data);
    const auto stats=mujoco_collision_statistics(data);
    const int body=mj_name2id(model,mjOBJ_BODY,"part");
    const double final_y=data->xipos[3*body+1];
    if(stats.failures||!std::isfinite(final_y)) throw std::runtime_error("collision or simulation failed");
    std::cout<<std::setprecision(9)<<"nTop Core "<<ntop_core_version()<<", density_kg_m3="<<density
             <<", mass_kg="<<mass.mass<<", simulated_seconds="<<data->time<<", substeps="<<steps
             <<", elapsed_ms="<<elapsed<<", final_center_y_m="<<final_y<<", max_contacts="<<max_contacts
             <<", total_contacts="<<stats.generated_contacts<<", field_queries="<<stats.field_queries<<'\n';
    detach_mujoco_collision(data); mj_deleteData(data); mj_deleteModel(model); unregister_mujoco_field(id);
    return 0;
  } catch(const std::exception& e) { std::cerr<<"nTop MuJoCo run failed: "<<e.what()<<'\n'; return 1; }
}
