#include "ntop_sdf/world.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <numbers>

#ifdef _WIN32
#include <windows.h>
#include <psapi.h>
#else
#include <sys/resource.h>
#endif

using namespace ntop_sdf;
std::size_t peak_memory_bytes() {
#ifdef _WIN32
  PROCESS_MEMORY_COUNTERS p{};
  return GetProcessMemoryInfo(GetCurrentProcess(),&p,sizeof(p)) ? p.PeakWorkingSetSize : 0;
#else
  rusage r{}; return getrusage(RUSAGE_SELF,&r)==0 ? static_cast<std::size_t>(r.ru_maxrss)*1024 : 0;
#endif
}
int main(int argc,char** argv) {
  const int count=argc>1?std::atoi(argv[1]):10;
  const int warmup=argc>2?std::atoi(argv[2]):10;
  const int steps=argc>3?std::atoi(argv[3]):20;
  const int hz=argc>4?std::atoi(argv[4]):60;
  if(count<1||warmup<0||steps<1||hz<1) {
    std::cerr<<"usage: ntop_sdf_bench [bodies>0] [warmup>=0] [steps>0] [hz>0]\n"; return 1;
  }
  World world;
  const int side=static_cast<int>(std::ceil(std::sqrt(static_cast<float>(count))));
  const float half=side*.525f+2;
  BodyDesc floor; floor.field=box({half,.5,half}); floor.position={0,-.5,0}; floor.is_static=true; world.add(floor);
  auto shape=sphere(.5);
  const double mass=(4.0/3)*std::numbers::pi*.5*.5*.5;
  const double inertia=.4*mass*.5*.5;
  for(int i=0;i<count;i++) {
    BodyDesc b; b.field=shape;
    b.mass=mass; b.diagonal_inertia={inertia,inertia,inertia};
    b.position={(i%side-(side-1)*.5f)*1.05f,.49f,(i/side-(side-1)*.5f)*1.05f};
    world.add(b);
  }
  for(int i=0;i<warmup;i++) world.step(1.0/hz);
  double total=0,broad=0,narrow=0,solve=0,pairs=0,contacts=0,generated=0,queries=0,objectives=0,degeneracies=0;
  for(int i=0;i<steps;i++) {
    world.step(1.0/hz); auto s=world.statistics();
    total+=s.total_ms; broad+=s.broad_ms; narrow+=s.narrow_ms; solve+=s.solve_ms;
    pairs+=s.pairs; contacts+=s.contacts; generated+=s.generated_contacts; queries+=s.field_queries; objectives+=s.objective_evaluations;
    degeneracies+=s.degenerate_gradients;
  }
  double min_y=1e30,max_y=-1e30,max_speed=0;
  for(int i=0;i<count;i++) { auto b=world.body(i+1); min_y=std::min(min_y,b.position.y); max_y=std::max(max_y,b.position.y); max_speed=std::max(max_speed,std::sqrt(b.velocity.x*b.velocity.x+b.velocity.y*b.velocity.y+b.velocity.z*b.velocity.z)); }
  std::cout<<"engine,dynamic_bodies,static_bodies,hz,warmup_steps,measured_steps,substeps,solver_iterations,mean_total_step_ms,mean_broad_ms,mean_narrow_ms,mean_solve_ms,mean_last_substep_pairs,mean_last_substep_contacts,mean_generated_contacts_per_step,mean_field_queries_per_step,field_queries_per_generated_contact,mean_objective_evaluations_per_step,degenerate_gradients,min_body_y,max_body_y,max_speed,peak_working_set_bytes\n";
  std::cout<<std::setprecision(9)<<"ntop_sdf,"<<count<<",1,"<<hz<<','<<warmup<<','<<steps<<",4,20,"
    <<total/steps<<','<<broad/steps<<','<<narrow/steps<<','<<solve/steps<<','<<pairs/steps<<','<<contacts/steps<<','
    <<generated/steps<<','<<queries/steps<<','<<(generated?queries/generated:0)<<','<<objectives/steps<<','<<degeneracies<<','<<min_y<<','<<max_y<<','<<max_speed<<','<<peak_memory_bytes()<<'\n';
}
