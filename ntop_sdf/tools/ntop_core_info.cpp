#include "ntop_sdf/ntop_core.hpp"
#include "utf8_argv.hpp"

#include <iomanip>
#include <iostream>
#include <stdexcept>

int main(int argc,char** argv) {
  try {
    if(argc!=2) throw std::invalid_argument("usage: ntop_core_info file.implicit");
    auto f=ntop_sdf::load_ntop_core_implicit(ntop_path_argument(1,argv[1]));
    auto b=f->bounds();
    ntop_sdf::Vec3 center{(b.min.x+b.max.x)/2,(b.min.y+b.max.y)/2,(b.min.z+b.max.z)/2},gradient{};
    auto value=f->value_gradient(center,gradient);
    std::cout<<std::setprecision(10)<<"nTop Core "<<ntop_sdf::ntop_core_version()<<"\n"
      <<"bounds in metres: ["<<b.min.x<<", "<<b.min.y<<", "<<b.min.z<<"] to ["
      <<b.max.x<<", "<<b.max.y<<", "<<b.max.z<<"]\n"
      <<"field at bounds center: "<<value<<"\ngradient: ["<<gradient.x<<", "<<gradient.y<<", "<<gradient.z<<"]\n";
    return 0;
  } catch(const std::exception& e) { std::cerr<<e.what()<<'\n'; return 1; }
}
