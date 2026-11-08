#include "ntop_sdf/mujoco_plugin.hpp"
#ifdef NTOP_SDF_WITH_CORE
#include "ntop_sdf/ntop_core.hpp"
#include "../tools/utf8_argv.hpp"
#endif

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <numbers>
#include <sstream>
#include <string>

#include <mujoco/mujoco.h>
#ifdef _WIN32
#include <windows.h>
#include <psapi.h>
#else
#include <sys/resource.h>
#endif

double clock_ms() {
  return std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now().time_since_epoch()).count();
}
std::size_t peak_memory_bytes() {
#ifdef _WIN32
  PROCESS_MEMORY_COUNTERS p{};
  return GetProcessMemoryInfo(GetCurrentProcess(),&p,sizeof(p)) ? p.PeakWorkingSetSize : 0;
#else
  rusage r{}; return getrusage(RUSAGE_SELF,&r)==0 ? static_cast<std::size_t>(r.ru_maxrss)*1024 : 0;
#endif
}
int main(int argc,char** argv) {
  int count=argc>1?std::atoi(argv[1]):10, warmup=argc>2?std::atoi(argv[2]):10;
  int steps=argc>3?std::atoi(argv[3]):20,hz=argc>4?std::atoi(argv[4]):60;
  std::string mode=argc>5?argv[5]:"native";
  int initpoints=argc>6?std::atoi(argv[6]):40, sdf_iterations=argc>7?std::atoi(argv[7]):20;
  double margin=argc>8?std::strtod(argv[8],nullptr):0;
  if(count<1||warmup<0||steps<1||hz<1||initpoints<1||sdf_iterations<1||!std::isfinite(margin)||margin<0||
     (mode!="native"&&mode!="field"&&mode!="cached"&&mode!="core") || (mode=="core"&&argc<10)) {
    std::cerr<<"usage: mujoco_bench [bodies>0] [warmup>=0] [steps>0] [hz>0] [native|field|cached|core] [sdf_initpoints] [sdf_iterations] [margin] [core.implicit]\n"; return 1;
  }
  std::uint64_t field_id=0;
  if(mode!="native") {
    ntop_sdf::FieldPtr field;
    if(mode=="core") {
#ifdef NTOP_SDF_WITH_CORE
      try { field=ntop_sdf::load_ntop_core_implicit(ntop_path_argument(9,argv[9]),100); }
      catch(const std::exception& e) { std::cerr<<e.what()<<'\n';return 1; }
#else
      std::cerr<<"core mode requires NTOP_CORE_ROOT at build time\n";return 1;
#endif
    } else field=ntop_sdf::sphere(.5);
    ntop_sdf::register_mujoco_plugin(); field_id=ntop_sdf::register_mujoco_field(field);
  }
  const int side=static_cast<int>(std::ceil(std::sqrt(static_cast<float>(count))));
  const float half=side*.525f+2;
  std::ostringstream xml; xml<<std::setprecision(17)<<"<mujoco><size memory=\"512M\"/>";
  if(mode!="native") xml<<"<extension><plugin plugin=\"ntop.sdf.callback\"><instance name=\"shape\"><config key=\"field_id\" value=\""<<field_id<<"\"/></instance></plugin></extension><asset><mesh name=\"surface\"><plugin instance=\"shape\"/></mesh></asset>";
  xml<<"<option timestep=\""<<1.0/(4*hz)<<"\" gravity=\"0 -9.81 0\" solver=\"PGS\" iterations=\"20\" tolerance=\"0\" cone=\"pyramidal\" sdf_initpoints=\""<<initpoints<<"\" sdf_iterations=\""<<sdf_iterations<<"\"/>";
  xml<<"<default><geom friction=\".5 .005 .0001\" density=\"1\" margin=\""<<margin<<"\"/></default><worldbody><geom type=\"box\" pos=\"0 -.5 0\" size=\""<<half<<" .5 "<<half<<"\"/>";
  const double mass=(4.0/3)*std::numbers::pi*.5*.5*.5, inertia=.4*mass*.5*.5;
  for(int i=0;i<count;i++) {
    float x=(i%side-(side-1)*.5f)*1.05f,z=(i/side-(side-1)*.5f)*1.05f;
    xml<<"<body pos=\""<<x<<" .49 "<<z<<"\"><freejoint/>";
    if(mode!="native") xml<<"<inertial pos=\"0 0 0\" mass=\""<<mass<<"\" diaginertia=\""<<inertia<<' '<<inertia<<' '<<inertia<<"\"/><geom type=\"sdf\" mesh=\"surface\"><plugin instance=\"shape\"/></geom>";
    else xml<<"<geom type=\"sphere\" size=\".5\"/>";
    xml<<"</body>";
  }
  xml<<"</worldbody></mujoco>";
  auto input=xml.str(); mjVFS vfs; mj_defaultVFS(&vfs);
  if(mj_addBufferVFS(&vfs,"benchmark.xml",input.data(),static_cast<int>(input.size()))) {std::cerr<<"VFS error\n";return 1;}
  char error[1024]{}; mjModel* model=mj_loadXML("benchmark.xml",&vfs,error,sizeof(error)); mj_deleteVFS(&vfs);
  if(!model) {std::cerr<<error<<'\n';return 1;}
  mjData* data=mj_makeData(model); if(!data) {std::cerr<<"mj_makeData failed\n";return 1;}
  if(mode=="cached"||mode=="core") ntop_sdf::attach_mujoco_collision(model,data);
  mjcb_time=clock_ms;
  for(int i=0;i<warmup;i++) for(int k=0;k<4;k++) mj_step(model,data);
  auto before=ntop_sdf::mujoco_collision_statistics(data);
  std::memset(data->timer,0,sizeof(data->timer));
  double elapsed=0,contacts=0;
  for(int i=0;i<steps;i++) {
    auto start=clock_ms(); for(int k=0;k<4;k++) mj_step(model,data); elapsed+=clock_ms()-start; contacts+=data->ncon;
  }
  double min_y=1e30,max_y=-1e30,max_speed=0;
  for(int i=0;i<count;i++) {
    double y=data->qpos[7*i+1],vx=data->qvel[6*i],vy=data->qvel[6*i+1],vz=data->qvel[6*i+2];
    min_y=std::min(min_y,y); max_y=std::max(max_y,y); max_speed=std::max(max_speed,std::sqrt(vx*vx+vy*vy+vz*vz));
  }
  auto after=ntop_sdf::mujoco_collision_statistics(data);
  auto queries=after.field_queries-before.field_queries, generated=after.generated_contacts-before.generated_contacts;
  std::cout<<"engine,dynamic_bodies,static_bodies,hz,warmup_steps,measured_steps,substeps,solver_iterations,sdf_initpoints,sdf_iterations,margin,mean_total_step_ms,mean_broad_ms,mean_narrow_ms,mean_constraint_ms,mean_last_substep_contacts,mean_generated_contacts_per_step,field_queries_per_generated_contact,min_body_y,max_body_y,max_speed,peak_working_set_bytes\n";
  std::cout<<std::setprecision(9)<<"mujoco_"<<mode<<','<<count<<",1,"<<hz<<','<<warmup<<','<<steps<<",4,20,"<<initpoints<<','<<sdf_iterations<<','<<margin<<','
    <<elapsed/steps<<','<<data->timer[mjTIMER_COL_BROAD].duration/steps<<','<<data->timer[mjTIMER_COL_NARROW].duration/steps<<','
    <<data->timer[mjTIMER_CONSTRAINT].duration/steps<<','<<contacts/steps<<',';
  if(mode=="cached"||mode=="core") std::cout<<static_cast<double>(generated)/steps<<','<<(generated?static_cast<double>(queries)/generated:0);
  else std::cout<<',';
  std::cout<<','<<min_y<<','<<max_y<<','<<max_speed<<','<<peak_memory_bytes()<<'\n';
  if(mode=="cached"||mode=="core") {
    if(ntop_sdf::mujoco_collision_failures(data)) {std::cerr<<"cached collision failed\n";return 1;}
    ntop_sdf::detach_mujoco_collision(data);
  }
  mjcb_time=nullptr; mj_deleteData(data); mj_deleteModel(model);
  if(field_id) ntop_sdf::unregister_mujoco_field(field_id);
}
