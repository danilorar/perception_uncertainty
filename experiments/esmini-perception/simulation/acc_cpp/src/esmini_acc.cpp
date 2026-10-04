// Author: Zhuo Ma
// Calls the original ControllerACC::Step, compiled into our esmini library.
#include "accsim/control.hpp"
#include "native_acc_bridge.h"
#include <cmath>
#include <stdexcept>
#include <vector>
namespace accsim {
namespace {
ACCSIM_Object packet(const ObjectState& x) {
    return {x.id,x.object_category,x.pose.x,x.pose.y,x.pose.z,x.pose.h,x.pose.p,x.pose.r,x.speed,
            x.box.length,x.box.width,x.box.height,x.box.cx,x.box.cy,x.box.cz};
}
ACCSIM_Object packet(const Observation& x) {
    return {x.track_id,x.object_category,x.pose.x,x.pose.y,x.pose.z,x.pose.h,x.pose.p,x.pose.r,x.speed,
            x.box.length,x.box.width,x.box.height,x.box.cx,x.box.cy,x.box.cz};
}
}
ControlRequest EsminiAcc::update(const ControllerInput& input) const {
    if(!(input.dt_s>0) || !std::isfinite(input.ego.speed)) throw std::invalid_argument("Invalid ACC input");
    const auto ego=packet(input.ego);
    std::vector<ACCSIM_Object> observations;
    for(const auto& object:input.perception.objects) observations.push_back(packet(object));
    const ACCSIM_AccConfig config{config_.time_gap_s,config_.set_speed_mps,
                                 config_.max_acceleration,config_.max_deceleration,config_.lateral_distance_m};
    ACCSIM_AccOutput native{};
    if(ACCSIM_NativeAccStep(&ego,observations.data(),static_cast<int>(observations.size()),&config,input.dt_s,&native)!=0)
        throw std::runtime_error("The native esmini ACC bridge failed");
    ControlRequest result;
    result.desired_speed_mps=native.desired_speed_mps;
    result.acceleration_mps2=(native.desired_speed_mps-input.ego.speed)/input.dt_s;
    result.lead_id=native.lead_id; result.observed_gap_m=native.observed_gap_m;
    result.close_gap_stop=native.lead_id>=0 && native.observed_gap_m<1 && native.desired_speed_mps==0 && input.ego.speed>0;
    return result;
}
} // namespace accsim
