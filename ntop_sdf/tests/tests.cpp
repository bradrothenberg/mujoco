#include "ntop_sdf/field.hpp"
#include "ntop_sdf/world.hpp"

#include <cmath>
#include <functional>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

using namespace ntop_sdf;
namespace {
void require(bool c, const std::string& message) { if (!c) throw std::runtime_error(message); }
void near(double a,double b,double e,const std::string& message) {
  if (!std::isfinite(a)||std::abs(a-b)>e)
    throw std::runtime_error(message+": got "+std::to_string(a)+", expected "+std::to_string(b));
}
void gradients(const Field& f,Vec3 p) {
  const double h=1e-3;
  auto g=f.gradient(p);
  near(g.x,(f.value({p.x+h,p.y,p.z})-f.value({p.x-h,p.y,p.z}))/(2*h),3e-3,"gradient x");
  near(g.y,(f.value({p.x,p.y+h,p.z})-f.value({p.x,p.y-h,p.z}))/(2*h),3e-3,"gradient y");
  near(g.z,(f.value({p.x,p.y,p.z+h})-f.value({p.x,p.y,p.z-h}))/(2*h),3e-3,"gradient z");
}
void fields() {
  auto s=sphere(1);
  near(s->value({2,0,0}),1,1e-6,"sphere distance");
  near(s->value({0,0,0}),-1,1e-6,"sphere interior");
  gradients(*s,{1.2,.3,.4});
  gradients(*box({1,2,3}),{1.3,2.4,.5});
  gradients(*capsule(.5,1),{.8,1.3,.2});
  gradients(*cylinder(.5,1),{.8,1.3,.2});
  gradients(*torus(1,.2),{1.3,.2,.2});
  auto c=unite(s,translated(s,{3,0,0}));
  require(c->value({3,0,0})<0,"union should include translated sphere");
  auto cut=subtract(s,sphere(.3));
  require(cut->value({0,0,0})>0 && cut->value({.8,0,0})<0,"subtraction shell");
  auto tw=twist(box({1,.5,.5}),.3);
  require(tw->value({0,0,0})<0,"twist origin");
  gradients(*tw,{.8,.27,.4});
  auto be=bend(box({1,.5,.5}),.3);
  require(be->value({0,0,0})<0,"bend origin");
  gradients(*be,{.8,.27,.4});
  auto di=displace(s,.05,3,{{-2,-2,-2},{2,2,2}});
  require(di->value({0,0,0})<0,"displacement origin");
  gradients(*di,{.8,.27,.4});
}
void callback_test() {
  int batch=0;
  auto f=callback({{-2,-2,-2},{2,2,2}},[&](auto points,auto out) {
    ++batch;
    for (std::size_t i=0;i<points.size();++i)
      out[i]=3*(std::sqrt(points[i].x*points[i].x+points[i].y*points[i].y+points[i].z*points[i].z)-1);
  });
  auto g=f->gradient({1.2,0,0});
  near(g.x,3,1e-7,"finite difference callback");
  require(batch==1,"finite differences should use one batch");
  Vec3 p[]={{1,0,0},{2,0,0}}; double v[2]; f->values(p,v);
  near(v[0],0,1e-12,"batch first"); near(v[1],3,1e-12,"batch second");
}
void sphere_contacts(double offset, double expected) {
  World w; w.gravity({0,0,0}); auto s=sphere(.5);
  BodyDesc a; a.field=s; a.is_static=true; w.add(a);
  BodyDesc b; b.field=s; b.position={offset,0,0}; w.add(b);
  w.step(1.0/240);
  auto c=w.contacts(); require(!c.empty(),"missing sphere-sphere contact");
  auto best=c.front();
  for (const auto& p:c) if (p.separation<best.separation) best=p;
  near(best.separation,expected,.035,"sphere-sphere separation");
  require(best.normal.x>.85,"sphere-sphere normal");
}
void floor_contact(double y,double expected) {
  World w; w.gravity({0,0,0});
  BodyDesc a; a.field=box({5,.5,5}); a.position={0,-.5,0}; a.is_static=true; w.add(a);
  BodyDesc b; b.field=sphere(.5); b.position={0,y,0}; w.add(b);
  w.step(1.0/240);
  auto c=w.contacts(); require(!c.empty(),"missing sphere-floor contact");
  auto best=c.front(); for(const auto& p:c) if(p.separation<best.separation) best=p;
  near(best.separation,expected,.035,"sphere-floor separation");
  require(best.normal.y>.85,"sphere-floor normal");
}
void sphere_drop() {
  World w;
  BodyDesc floor; floor.field=box({3,.5,3}); floor.position={0,-.5,0}; floor.is_static=true; w.add(floor);
  BodyDesc ball; ball.field=sphere(.5); ball.position={0,2,0}; auto id=w.add(ball);
  for(int i=0;i<240;i++) w.step(1.0/120);
  auto b=w.body(id);
  near(b.position.y,.5,.08,"sphere settled height");
  require(std::abs(b.velocity.y)<.2,"sphere should settle");
  require(!w.contacts().empty(),"settled sphere has no contacts");
}
void box_stack() {
  World w;
  BodyDesc floor; floor.field=box({3,.5,3}); floor.position={0,-.5,0}; floor.is_static=true; w.add(floor);
  auto field=box({.5,.5,.5});
  for(int i=0;i<5;i++) { BodyDesc b; b.field=field; b.position={0,.5+i,0}; w.add(b); }
  for(int i=0;i<600;i++) w.step(1.0/120);
  for(int i=0;i<5;i++) {
    auto b=w.body(i+1);
    near(b.position.y,.5+i,.15,"box stack height");
    require(std::abs(b.velocity.y)<.3,"stack should settle");
    require(std::abs(b.position.x)<.2 && std::abs(b.position.z)<.2,"stack drifted laterally");
    require(std::sqrt(b.angular_velocity.x*b.angular_velocity.x+b.angular_velocity.y*b.angular_velocity.y+b.angular_velocity.z*b.angular_velocity.z)<.4,"stack has excessive rotation");
  }
}
void scaled_field_contact() {
  auto scaled=callback({{-.5,-.5,-.5},{.5,.5,.5}},[](auto points,auto out) {
    for(std::size_t i=0;i<points.size();++i) {
      auto p=points[i]; out[i]=7*(std::sqrt(p.x*p.x+p.y*p.y+p.z*p.z)-.5);
    }
  });
  World w; w.gravity({0,0,0});
  BodyDesc floor; floor.field=box({3,.5,3}); floor.position={0,-.5,0}; floor.is_static=true; w.add(floor);
  BodyDesc ball; ball.field=scaled; ball.position={0,.49,0}; w.add(ball);
  w.step(1.0/240);
  auto c=w.contacts(); require(!c.empty(),"missing scaled-field contact");
  auto best=c.front(); for(const auto& p:c) if(p.separation<best.separation) best=p;
  near(best.separation,-.01,.035,"scaled-field penetration");
  require(best.normal.y>.85,"scaled-field normal");
}
void torus_contacts() {
  World w; w.gravity({0,0,0});
  BodyDesc ring; ring.field=torus(1,.25); ring.is_static=true; w.add(ring);
  BodyDesc ball; ball.field=sphere(.25); ball.position={1,.4,0}; w.add(ball);
  w.step(1.0/240);
  require(!w.contacts().empty(),"sphere did not contact torus tube");
  World hole; hole.gravity({0,0,0}); hole.add(ring);
  ball.position={0,0,0}; hole.add(ball); hole.step(1.0/240);
  require(hole.contacts().empty(),"sphere in torus hole has a false contact");
}
void warped_contact() {
  World w; w.gravity({0,0,0});
  BodyDesc floor; floor.field=box({3,.5,3}); floor.position={0,-.5,0}; floor.is_static=true; w.add(floor);
  BodyDesc warped; warped.field=twist(box({1,.5,.4}),.4); warped.position={0,.45,0}; w.add(warped);
  w.step(1.0/240); require(!w.contacts().empty(),"twisted box missed floor contact");
  for(const auto& c:w.contacts()) require(std::isfinite(c.separation)&&std::isfinite(c.normal.y),"invalid twisted contact");
}
double bounce_height(double restitution) {
  World w;
  BodyDesc floor; floor.field=box({3,.5,3}); floor.position={0,-.5,0}; floor.is_static=true; floor.restitution=restitution; w.add(floor);
  BodyDesc ball; ball.field=sphere(.5); ball.position={0,2,0}; ball.restitution=restitution; auto id=w.add(ball);
  bool hit=false; double maximum=0;
  for(int i=0;i<300;i++) {
    w.step(1.0/120); auto state=w.body(id);
    if(state.position.y<.6) hit=true;
    if(hit) maximum=std::max(maximum,state.position.y);
  }
  require(hit,"ball did not hit the floor"); return maximum;
}
void restitution_test() {
  double inelastic=bounce_height(0),elastic=bounce_height(.8);
  require(elastic>inelastic+.25,"restitution did not increase rebound height");
}
double slide_speed(double friction) {
  World w;
  BodyDesc floor; floor.field=box({5,.5,5}); floor.position={0,-.5,0}; floor.is_static=true; floor.friction=friction; w.add(floor);
  BodyDesc b; b.field=box({.5,.5,.5}); b.position={0,.5,0}; b.velocity={2,0,0}; b.friction=friction; auto id=w.add(b);
  for(int i=0;i<120;i++) w.step(1.0/120);
  return std::abs(w.body(id).velocity.x);
}
void friction_test() { require(slide_speed(0)>slide_speed(.8)+.2,"friction did not slow sliding"); }
void pose_test() {
  World w; w.gravity({0,0,0}); BodyDesc b; b.field=translated(sphere(.5),{.4,0,0}); auto id=w.add(b);
  w.set_pose(id,{3,2,1},{0,0,0,1});
  auto p=w.body(id).position; near(p.x,3,1e-5,"shape origin after teleport"); near(p.y,2,1e-5,"shape origin after teleport");
  w.set_velocity(id,{1,0,0},{0,0,0}); w.step(.01);
  require(w.body(id).position.x>3,"set velocity had no effect");
}
void pair_query_test() {
  auto s=sphere(.5); PairQuery pair(s,s);
  pair.set_pose(0,{0,0,0}); pair.set_pose(1,{.9,0,0});
  auto contacts=pair.query(); require(!contacts.empty(),"pair query missed penetrating spheres");
  auto best=contacts.front(); for(const auto& c:contacts) if(c.separation<best.separation) best=c;
  near(best.separation,-.1,.002,"pair query penetration");
  require(best.normal.x>.99,"pair query normal");
  contacts=pair.query(); bool cached=false; for(const auto& c:contacts) cached|=c.cached;
  require(cached,"pair query did not reuse cached contacts");
  pair.set_pose(1,{2,0,0}); require(pair.query().empty(),"pair query found a separated contact");
  pair.set_pose(1,{.2,0,0}); contacts=pair.query(); require(!contacts.empty(),"deep overlap not found");
  best=contacts.front(); for(const auto& c:contacts) if(c.separation<best.separation) best=c;
  near(best.separation,-.8,.005,"deep overlap depth");
}
void nonconvex_pair() {
  PairQuery pair(torus(1,.25),torus(1,.25)); pair.set_pose(1,{0,.3,0});
  auto contacts=pair.query(); require(!contacts.empty(),"two overlapping tori produced no contact");
  for(const auto& c:contacts) require(std::isfinite(c.separation)&&std::isfinite(c.normal.y),"invalid torus pair contact");
}
void mass_test() {
  World w; BodyDesc b; b.field=sphere(.5); b.mass=2; b.diagonal_inertia={.2,.3,.4}; auto id=w.add(b);
  auto p=w.mass_properties(id); near(p.mass,2,1e-6,"explicit mass");
  near(p.inertia[0],.2,1e-6,"explicit inertia x"); near(p.inertia[4],.3,1e-6,"explicit inertia y");
  near(p.inertia[8],.4,1e-6,"explicit inertia z");
  b.diagonal_inertia={1,1,3}; bool rejected=false;
  try { w.add(b); } catch(const std::invalid_argument&) { rejected=true; }
  require(rejected,"unphysical inertia was accepted");
}
void invalid_field_test() {
  auto f=callback({{-1,-1,-1},{1,1,1}},[](auto,auto out) {
    for(auto& value:out) value=std::numeric_limits<double>::quiet_NaN();
  });
  World w; BodyDesc b; b.field=f; bool rejected=false;
  try { w.add(b); } catch(const std::domain_error&) { rejected=true; }
  require(rejected,"non-finite field was not rejected");
  BodyDesc huge; huge.field=sphere(.5); huge.position={1e300,0,0}; rejected=false;
  try { w.add(huge); } catch(const std::exception&) { rejected=true; }
  require(rejected,"out-of-range body position was not rejected");
  huge.position={0,0,0}; huge.rotation={0,0,0,1e20}; auto id=w.add(huge);
  near(w.body(id).rotation.w,1,1e-6,"large quaternion normalization");
  rejected=false; try { w.step(1e-300); } catch(const std::invalid_argument&) { rejected=true; }
  require(rejected,"timestep underflow was not rejected");
}
void falling_bodies() {
  World w; BodyDesc floor; floor.field=box({10,.5,10}); floor.position={0,-.5,0}; floor.is_static=true; w.add(floor);
  auto s=sphere(.25);
  for(int i=0;i<200;i++) { BodyDesc b; b.field=s; b.position={(i%10-4.5)*.55,2+(i/100)*.55,((i/10)%10-4.5)*.55}; w.add(b); }
  for(int i=0;i<300;i++) w.step(1.0/120);
  for(int i=0;i<200;i++) {
    auto b=w.body(i+1); require(std::isfinite(b.position.y)&&b.position.y>.20&&b.position.y<4,"falling body escaped or penetrated floor");
    require(std::isfinite(b.velocity.y)&&std::abs(b.velocity.y)<3,"falling body became unstable");
  }
}
}  // namespace

int main() {
  std::vector<std::pair<const char*,std::function<void()>>> tests={
    {"fields",fields},{"callback batch",callback_test},
    {"sphere sphere zero gap",[]{sphere_contacts(1,0);}},
    {"sphere sphere penetration",[]{sphere_contacts(.9,-.1);}},
    {"sphere floor zero gap",[]{floor_contact(.5,0);}},
    {"sphere floor penetration",[]{floor_contact(.49,-.01);}},
    {"sphere drop",sphere_drop},{"box stack",box_stack},{"scaled callback contact",scaled_field_contact},
    {"torus contacts",torus_contacts},{"warped contact",warped_contact},
    {"restitution",restitution_test},{"friction",friction_test},{"pose",pose_test},{"pair query",pair_query_test},
    {"nonconvex pair",nonconvex_pair},{"mass and inertia",mass_test},{"invalid field",invalid_field_test},
    {"200 falling bodies",falling_bodies}};
  int failed=0;
  for(auto& [name,test]:tests) try { test(); std::cout<<"PASS "<<name<<'\n'; }
  catch(const std::exception& e) { ++failed; std::cerr<<"FAIL "<<name<<": "<<e.what()<<'\n'; }
  return failed?1:0;
}
