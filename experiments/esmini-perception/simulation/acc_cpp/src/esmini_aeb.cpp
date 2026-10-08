// Author: Zhuo Ma
// Calls upstream C++ AEB functions. No TTC/braking algorithm is implemented here.
#include "accsim/control.hpp"
#include "native_acc_bridge.h"
#include <cmath>
#include <stdexcept>
#include <vector>
namespace accsim {
namespace {
template<class T> ACCSIM_Object values(const T& x,int id) {
    return {id,x.object_category,x.pose.x,x.pose.y,x.pose.z,x.pose.h,x.pose.p,x.pose.r,x.speed,
        x.box.length,x.box.width,x.box.height,x.box.cx,x.box.cy,x.box.cz,
        x.velocity.x,x.velocity.y,x.velocity.z,x.acceleration.x,x.acceleration.y,x.acceleration.z};
}
}
EsminiAeb::EsminiAeb(AebConfig config) {
    const ACCSIM_AebConfig native{config.ttc_s,config.max_deceleration_mps2};
    context_=ACCSIM_NativeAebCreate(&native);
    if(!context_) throw std::runtime_error("Cannot construct native esmini AEB context");
}
EsminiAeb::~EsminiAeb() { ACCSIM_NativeAebDestroy(context_); }
ControlRequest EsminiAeb::update(const ControllerInput& input) {
    if(!(input.dt_s>0) || !std::isfinite(input.dt_s) || !std::isfinite(input.ego.speed))
        throw std::invalid_argument("Invalid AEB input");
    const auto ego=values(input.ego,input.ego.id);
    std::vector<ACCSIM_Object> objects;
    for(const auto& object:input.perception.objects) objects.push_back(values(object,object.track_id));
    ACCSIM_AebOutput native{};
    if(ACCSIM_NativeAebStep(context_,&ego,objects.data(),static_cast<int>(objects.size()),input.dt_s,&native)!=0)
        throw std::runtime_error("Native esmini AEB bridge failed");
    ControlRequest result;
    result.acceleration_mps2=native.acceleration_mps2;
    result.desired_speed_mps=native.desired_speed_mps;
    result.lead_id=native.lead_id;
    result.observed_gap_m=native.observed_gap_m;
    result.observed_ttc_s=native.observed_ttc_s;
    result.aeb_active=native.active!=0;
    return result;
}
} // namespace accsim
