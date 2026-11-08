#include "ntop_sdf/ntop_core.hpp"
#include "ntop_sdf/world.hpp"
#include "../tools/utf8_argv.hpp"

#include <cmath>
#include <iostream>
#include <stdexcept>
#include <thread>
#include <vector>

using namespace ntop_sdf;
namespace {
void require(bool ok,const char* message) { if(!ok) throw std::runtime_error(message); }
void check_near(double v,double expected,double tolerance,const char* message) {
  if(!std::isfinite(v)||std::abs(v-expected)>tolerance) throw std::runtime_error(message);
}
void run(const char* path) {
  auto field=load_ntop_core_implicit(path);
  auto bounds=field->bounds();
  check_near(bounds.min.x,-.005,1e-8,"sphere min bound"); check_near(bounds.max.x,.005,1e-8,"sphere max bound");
  check_near(field->value({0,0,0}),-.005,1e-8,"sphere center");
  check_near(field->value({.005,0,0}),0,1e-8,"sphere surface");
  check_near(field->value({.007,0,0}),.002,1e-8,"sphere exterior");
  Vec3 gradient{};
  check_near(field->value_gradient({.007,0,0},gradient),.002,1e-8,"combined field");
  check_near(gradient.x,1,1e-8,"gradient x"); check_near(gradient.y,0,1e-8,"gradient y");

  std::vector<Vec3> points(65538,{.007,0,0}), grads(points.size());
  std::vector<double> values(points.size());
  field->values(points,values); field->gradients(points,grads);
  for(std::size_t i=0;i<points.size();++i) {
    check_near(values[i],.002,1e-8,"batch field"); check_near(grads[i].x,1,1e-8,"batch gradient");
  }
  bool rejected=false;
  try { field->values(points,std::span<double>(values).first(1)); } catch(const std::invalid_argument&) { rejected=true; }
  require(rejected,"batch size mismatch accepted");
  rejected=false;
  try { load_ntop_core_implicit("this-file-does-not-exist.implicit"); } catch(const std::runtime_error&) { rejected=true; }
  require(rejected,"missing file accepted");
  rejected=false;
  try { load_ntop_core_implicit(path,-1); } catch(const std::invalid_argument&) { rejected=true; }
  require(rejected,"negative scale accepted");

  auto large=load_ntop_core_implicit(path,100);
  check_near(large->bounds().max.x,.5,1e-6,"scaled bound");
  check_near(large->value({.7,0,0}),.2,1e-6,"scaled field");
  check_near(large->gradient({.7,0,0}).x,1,1e-6,"scaled gradient");
  std::vector<std::thread> threads;
  for(int t=0;t<4;t++) threads.emplace_back([field] {
    for(int i=0;i<200;i++) if(std::abs(field->value({.007,0,0})-.002)>1e-8) std::terminate();
  });
  for(auto& thread:threads) thread.join();

  World world;
  BodyDesc body; body.field=field;
  auto body_id=world.add(body);
  const auto mass=world.mass_properties(body_id).mass;
  check_near(mass,4.0/3.0*3.141592653589793*.005*.005*.005,1e-8,"5 mm sampled mass");

  World drop;
  drop.collision_settings(.0001,.00005,.000001);
  BodyDesc floor; floor.field=box({.02,.002,.02}); floor.position={0,-.002,0}; floor.is_static=true; drop.add(floor);
  BodyDesc ball; ball.field=field; ball.position={0,.006,0}; ball.density=1000; auto drop_id=drop.add(ball);
  for(int i=0;i<1000;i++) drop.step(.001);
  auto state=drop.body(drop_id);
  check_near(state.position.y,.005,.0006,"standalone 5 mm nTop sphere drop");
  require(std::abs(state.velocity.y)<.05,"standalone nTop sphere did not settle");
  require(!drop.contacts().empty(),"standalone nTop sphere has no floor contact");

  PairQuery pair(field,sphere(.005));
  pair.set_margin(0);
  pair.set_pose(1,{.009,0,0});
  auto contacts=pair.query();
  require(!contacts.empty(),"missing 5 mm nTop sphere contact");
  bool valid=false;
  for(const auto& c:contacts) if(c.normal.x>.85 && std::abs(c.separation+.001)<.0002) valid=true;
  require(valid,"5 mm nTop sphere contact is inaccurate");
  pair.set_pose(1,{.011,0,0});
  require(pair.query().empty(),"separated 5 mm spheres reported contact");
  std::cout<<"PASS nTop Core "<<ntop_core_version()<<" values, gradients, batches, scale, lifetime and 5 mm contact\n";
}
}
int main(int argc,char** argv) {
  try { if(argc!=2) throw std::invalid_argument("usage: ntop_core_test sphere.implicit"); auto path=ntop_path_argument(1,argv[1]); run(path.c_str()); return 0; }
  catch(const std::exception& e) { std::cerr<<"FAIL nTop Core: "<<e.what()<<'\n'; return 1; }
}
