// Author: Zhuo Ma
// This file is compiled INTO esminiLib, next to the unchanged upstream modules.
// ControllerACC::Step is the original esmini v3.8.1 implementation, not a port.
// Persistent ACC uses native target copies; the packet API remains for component tests.
#define ACCSIM_BRIDGE_BUILD
#include "native_acc_bridge.h"
#include "ControllerACC.hpp"
#include "Entities.hpp"
#include "ScenarioReader.hpp"
#include "ScenarioEngine.hpp"
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
    vehicle->pos_.SetVel(data.vx,data.vy,data.vz);
    vehicle->pos_.SetAcc(data.ax,data.ay,data.az);
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

namespace {
// Only the target view differs. The real ego and upstream lifecycle stay intact.
ACCSIM_PersistentAccOutput last_output{};
bool output_valid=false;
class PersistentAcc final : public ControllerACC {
    bool copies_;
    Entities observations_;
public:
    PersistentAcc(InitArgs* args, bool copies):ControllerACC(args),copies_(copies) {
        output_valid=false;
    }
    ~PersistentAcc() override { output_valid=false; }
    void Step(double dt) override {
        Entities* truth=entities_;
        for(auto* target:observations_.object_) delete target;
        observations_.object_.clear();
        if(copies_) {
            for(auto* target:truth->object_) {
                if(!target || target==object_) continue;
                // Position assignment copies rel_pos_. Do not retain a link into
                // live scenario state, or treat unresolved relative coordinates as absolute.
                const auto type=target->pos_.GetType();
                if(target->pos_.GetRelativePosition()!=nullptr ||
                   (type!=roadmanager::Position::PositionType::NORMAL &&
                    type!=roadmanager::Position::PositionType::ROUTE))
                    throw std::runtime_error("Persistent ACC cannot snapshot relative position for target " +
                                             std::to_string(target->id_));
                // Copy native Position, including road/lane mapping; no world reprojection.
                auto* copy=new Object(target->type_);
                copy->id_=target->id_;
                copy->category_=target->category_;
                copy->pos_=target->pos_;
                copy->boundingbox_=target->boundingbox_;
                copy->SetSpeed(target->GetSpeed());
                observations_.object_.push_back(copy);
            }
            entities_=&observations_;
        }
        const double speed=object_->GetSpeed();
        try { ControllerACC::Step(dt); }
        catch(...) { entities_=truth; throw; }
        last_output={scenario_engine_->getSimulationTime(),speed,object_->GetSpeed(),
                     std::numeric_limits<double>::infinity(),-1,0};
        // Upstream exposes the selected object's position via its lookahead marker.
        for(auto* target:entities_->object_) {
            if(!target || target==object_) continue;
            ++last_output.target_count;
            if(std::abs(object_->lookahead_sensor_pos_[0]-target->pos_.GetX())<1e-7 &&
               std::abs(object_->lookahead_sensor_pos_[1]-target->pos_.GetY())<1e-7) {
                last_output.lead_id=target->id_;
                double lateral=0;
                object_->FreeSpaceDistance(target,&lateral,&last_output.observed_gap_m);
            }
        }
        entities_=truth;
        output_valid=true;
    }
};
Controller* persistent_acc(void* args) {
    return new PersistentAcc(static_cast<Controller::InitArgs*>(args),true);
}
Controller* reference_acc(void* args) {
    return new PersistentAcc(static_cast<Controller::InitArgs*>(args),false);
}
}
extern "C" ACCSIM_API void ACCSIM_RegisterPersistentAcc() {
    ScenarioReader::RegisterController("PerceptionACC",persistent_acc);
    ScenarioReader::RegisterController("ReferenceACC",reference_acc);
}
extern "C" ACCSIM_API int ACCSIM_GetPersistentAccOutput(ACCSIM_PersistentAccOutput* output) {
    if(!output || !output_valid) return -1;
    *output=last_output;
    return 0;
}
