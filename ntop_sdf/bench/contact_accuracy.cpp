#include "ntop_sdf/world.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <numbers>
#include <string>

using namespace ntop_sdf;
double norm(Vec3 p) { return std::sqrt(p.x*p.x+p.y*p.y+p.z*p.z); }
Vec3 scale(Vec3 p,double s) { return {p.x*s,p.y*s,p.z*s}; }
bool measure(const std::string& name,PairQuery& pair,double expected,Vec3 normal,Vec3 point) {
  auto contacts=pair.query();
  if(contacts.empty()) {
    std::cout<<name<<','<<expected<<",nan,nan,nan,nan,0\n"; return false;
  }
  auto best=contacts.front(); for(auto c:contacts) if(c.separation<best.separation) best=c;
  double dot=std::clamp(best.normal.x*normal.x+best.normal.y*normal.y+best.normal.z*normal.z,-1.0,1.0);
  double angle=std::acos(dot)*180/std::numbers::pi;
  Vec3 e{best.position.x-point.x,best.position.y-point.y,best.position.z-point.z};
  std::cout<<std::setprecision(10)<<name<<','<<expected<<','<<best.separation<<','
    <<std::abs(best.separation-expected)<<','<<norm(e)<<','<<angle<<','<<contacts.size()<<'\n';
  return std::abs(best.separation-expected)<.001 && norm(e)<.01 && angle<2;
}
int main() {
  std::cout<<"case,expected_separation,actual_separation,absolute_separation_error,contact_position_error,normal_error_degrees,contacts\n";
  bool ok=true;
  for(double distance:{.2,.5,.9,1.0}) for(Vec3 raw: {Vec3{1,0,0},Vec3{0,1,0},Vec3{1,2,3}}) {
    Vec3 n=scale(raw,1/norm(raw)); auto s=sphere(.5); PairQuery pair(s,s);
    pair.set_pose(0,{0,0,0}); pair.set_pose(1,scale(n,distance));
    ok &= measure("sphere_sphere",pair,distance-1,n,scale(n,distance*.5));
  }
  for(double y:{.5,.49,.4}) {
    PairQuery pair(box({5,.5,5}),sphere(.5));
    pair.set_pose(0,{0,-.5,0}); pair.set_pose(1,{0,y,0});
    ok &= measure("sphere_floor",pair,y-.5,{0,1,0},{0,(y-.5)*.5,0});
  }
  return ok?0:1;
}
