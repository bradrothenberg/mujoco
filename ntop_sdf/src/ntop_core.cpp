#include "ntop_sdf/ntop_core.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

#include <ntop_core/ntop_core.h>

namespace ntop_sdf {
namespace {

// The SDK documentation does not state that separate handles can be queried
// concurrently. Serialize all SDK operations until that contract is known.
std::mutex core_mutex;

struct ValueResult { std::span<double> out; int calls{}; bool valid{true}; };
struct GradientResult { std::span<Vec3> out; int calls{}; bool valid{true}; };

void receive_values(void* context, const double* values, std::uint32_t count) {
  auto& result=*static_cast<ValueResult*>(context);
  ++result.calls;
  if(result.calls!=1 || count!=result.out.size() || (!values && count)) { result.valid=false; return; }
  std::copy_n(values,count,result.out.data());
}
void receive_gradients(void* context, const ntop_core_derivative* values, std::uint32_t count) {
  auto& result=*static_cast<GradientResult*>(context);
  ++result.calls;
  if(result.calls!=1 || count!=result.out.size() || (!values && count)) { result.valid=false; return; }
  for(std::uint32_t i=0;i<count;i++) result.out[i]={values[i].dx,values[i].dy,values[i].dz};
}

class CoreField final : public Field {
 public:
  CoreField(const std::string& path,double scale) : scale_(scale) {
    if(path.empty() || !(std::isfinite(scale) && scale>0))
      throw std::invalid_argument("nTop Core path and a finite positive scale are required");
    std::lock_guard lock(core_mutex);
    auto result=ntop_core_import_from_file(path.c_str(),&handle_);
    if(result!=ntop_core_import_result::SUCCESS || !handle_) {
      if(handle_) { ntop_core_release(handle_); handle_=nullptr; }
      const char* reason="unknown error";
      switch(result) {
        case ntop_core_import_result::FILE_DOESNT_EXIST: reason="file does not exist"; break;
        case ntop_core_import_result::FILE_OPEN_ERROR: reason="cannot open file"; break;
        case ntop_core_import_result::CORRUPT_FILE: reason="corrupt file"; break;
        case ntop_core_import_result::UNSUPPORTED_VERSION: reason="unsupported file version; use a newer nTop Core SDK"; break;
        case ntop_core_import_result::OUT_OF_MEMORY: reason="out of memory"; break;
      }
      throw std::runtime_error("nTop Core could not load implicit: "+std::string(reason)+" (result "+std::to_string(result)+")");
    }
    ntop_core_bounding_box b{};
    ntop_core_query_bounding_box(handle_,&b);
    bounds_={{b.min.x*scale,b.min.y*scale,b.min.z*scale},
             {b.max.x*scale,b.max.y*scale,b.max.z*scale}};
    if(!(std::isfinite(bounds_.min.x)&&std::isfinite(bounds_.min.y)&&std::isfinite(bounds_.min.z)&&
         std::isfinite(bounds_.max.x)&&std::isfinite(bounds_.max.y)&&std::isfinite(bounds_.max.z)&&
         bounds_.min.x<bounds_.max.x&&bounds_.min.y<bounds_.max.y&&bounds_.min.z<bounds_.max.z)) {
      ntop_core_release(handle_); handle_=nullptr;
      throw std::runtime_error("nTop Core returned invalid or unbounded geometry");
    }
  }
  ~CoreField() override { std::lock_guard lock(core_mutex); ntop_core_release(handle_); }
  Bounds bounds() const override { return bounds_; }
  double value(Vec3 p) const override {
    std::lock_guard lock(core_mutex);
    return scale_*ntop_core_query_field(handle_,point(p));
  }
  Vec3 gradient(Vec3 p) const override {
    Vec3 g{}; value_gradient(p,g); return g;
  }
  double value_gradient(Vec3 p,Vec3& g) const override {
    ntop_core_derivative d{};
    std::lock_guard lock(core_mutex);
    ntop_core_query_derivative(handle_,point(p),&d);
    g={d.dx,d.dy,d.dz}; return scale_*d.value;
  }
  void values(std::span<const Vec3> points,std::span<double> out) const override {
    if(points.size()!=out.size()) throw std::invalid_argument("batch sizes differ");
    for(std::size_t start=0;start<points.size();) {
      const auto count=std::min<std::size_t>(points.size()-start,65536);
      std::vector<ntop_core_vec3> input; input.reserve(count);
      for(auto p:points.subspan(start,count)) input.push_back(point(p));
      ValueResult result{out.subspan(start,count)};
      { std::lock_guard lock(core_mutex);
        ntop_core_query_field_array(handle_,input.data(),static_cast<std::uint32_t>(count),&result,receive_values); }
      if(!result.valid || result.calls!=1) throw std::runtime_error("nTop Core returned an invalid value batch");
      if(scale_!=1) for(auto& v:result.out) v*=scale_;
      start+=count;
    }
  }
  void gradients(std::span<const Vec3> points,std::span<Vec3> out) const override {
    if(points.size()!=out.size()) throw std::invalid_argument("batch sizes differ");
    for(std::size_t start=0;start<points.size();) {
      const auto count=std::min<std::size_t>(points.size()-start,65536);
      std::vector<ntop_core_vec3> input; input.reserve(count);
      for(auto p:points.subspan(start,count)) input.push_back(point(p));
      GradientResult result{out.subspan(start,count)};
      { std::lock_guard lock(core_mutex);
        ntop_core_query_derivative_array(handle_,input.data(),static_cast<std::uint32_t>(count),&result,receive_gradients); }
      if(!result.valid || result.calls!=1) throw std::runtime_error("nTop Core returned an invalid derivative batch");
      start+=count;
    }
  }
 private:
  ntop_core_vec3 point(Vec3 p) const { return {p.x/scale_,p.y/scale_,p.z/scale_}; }
  ntop_core_handle handle_{};
  double scale_;
  Bounds bounds_{};
};
}  // namespace

FieldPtr load_ntop_core_implicit(const std::string& path,double scale) {
  return std::make_shared<CoreField>(path,scale);
}
std::string ntop_core_version() {
  int32_t major=0,minor=0,patch=0;
  { std::lock_guard lock(core_mutex); ntop_core_library_version(&major,&minor,&patch); }
  return std::to_string(major)+"."+std::to_string(minor)+"."+std::to_string(patch);
}
}  // namespace ntop_sdf
