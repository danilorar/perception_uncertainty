// Author: Zhuo Ma
// This file is compiled INTO esminiLib, next to the unchanged upstream modules.
// ControllerACC::Step is the original esmini v3.8.1 implementation, not a port.
// Only copied, post-uncertainty observations enter its isolated Entities view.
#define ACCSIM_BRIDGE_BUILD
#include "native_acc_bridge.h"
#include "ControllerACC.hpp"
#include "Entities.hpp"
#include <cmath>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>

namespace {
using namespace scenarioengine;
class ObservationAcc final : public ControllerACC {
public:
    ObservationAcc(InitArgs* args, Entities& observations):ControllerACC(args) {
        entities_=&observations; // Never the running scenario's Entities container.
    }
};
std::string precise(double x) { std::ostringstream s; s<<std::setprecision(17)<<x; return s.str(); }
Vehicle* copy_vehicle(const ACCSIM_Object& data,Entities& entities) {
    auto* vehicle=new Vehicle();
    entities.object_.push_back(vehicle); // Entities owns all copies, including ego.
    vehicle->id_=data.id; vehicle->category_=data.category;
    vehicle->name_="observation_"+std::to_string(data.id);
    vehicle->boundingbox_.dimensions_.length_=data.length;
    vehicle->boundingbox_.dimensions_.width_=data.width;
    vehicle->boundingbox_.dimensions_.height_=data.height;
    vehicle->boundingbox_.center_.x_=data.cx;
    vehicle->boundingbox_.center_.y_=data.cy;
    vehicle->boundingbox_.center_.z_=data.cz;
    // Static OpenDRIVE geometry is shared; no target state is looked up by ID.
    vehicle->pos_.SetInertiaPos(data.x,data.y,data.z,data.h,data.p,data.r);
    vehicle->SetSpeed(data.speed);
    return vehicle;
}
}
extern "C" ACCSIM_API int ACCSIM_NativeAccStep(const ACCSIM_Object* ego,const ACCSIM_Object* observations,
                                              int count,const ACCSIM_AccConfig* config,double dt,
                                              ACCSIM_AccOutput* output) {
    if(!ego || !config || !output || count<0 || (count && !observations) || !(dt>0)) return -1;
    try {
        Entities view;
        auto* host=copy_vehicle(*ego,view);
        host->SetMaxAcceleration(config->max_acceleration);
        host->SetMaxDeceleration(config->max_deceleration);
        for(int i=0;i<count;++i) copy_vehicle(observations[i],view);
        OSCProperties properties;
        properties.property_.push_back({"mode","additive"}); // Step does not move this proxy.
        properties.property_.push_back({"timeGap",precise(config->time_gap_s)});
        properties.property_.push_back({"setSpeed",precise(config->set_speed_mps)});
        properties.property_.push_back({"lateralDist",precise(config->lateral_distance_m)});
        Controller::InitArgs args{};
        args.name="ObservedBuiltinACC"; args.type="ACCController"; args.properties=&properties;
        ObservationAcc controller(&args,view);
        controller.LinkObject(host);
        ControlActivationMode modes[static_cast<unsigned int>(ControlDomains::COUNT)];
        for(auto& mode:modes) mode=ControlActivationMode::OFF;
        modes[static_cast<unsigned int>(ControlDomains::DOMAIN_LONG)]=ControlActivationMode::ON;
        controller.Activate(modes);
        // Activate/additive synchronizes from ego; then restore the configured
        // cruise target. Constructing an observation view per tick ensures exact
        // ego speed rather than retaining a previous unexecuted plant request.
        controller.SetSetSpeed(config->set_speed_mps);
        controller.Step(dt); // THE UPSTREAM C++ CONTROLLER IS EXECUTED HERE.
        output->desired_speed_mps=host->GetSpeed();
        output->lead_id=-1; output->observed_gap_m=std::numeric_limits<double>::infinity();
        for(std::size_t i=1;i<view.object_.size();++i) {
            auto* target=view.object_[i];
            if(std::abs(host->lookahead_sensor_pos_[0]-target->pos_.GetX())<1e-7 &&
               std::abs(host->lookahead_sensor_pos_[1]-target->pos_.GetY())<1e-7) {
                output->lead_id=target->id_;
                double lateral=0,longitudinal=0;
                host->FreeSpaceDistance(target,&lateral,&longitudinal);
                output->observed_gap_m=longitudinal;
                break;
            }
        }
        return 0;
    } catch(...) { return -2; }
}
