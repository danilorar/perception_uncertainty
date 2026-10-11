// Author: Zhuo Ma
// Connects independent target observations to esmini's native ACC.
// It does not implement the ACC algorithm. It is compiled into esminiLib.
// ControllerACC::Step is the original esmini v3.8.1 implementation, not a port.
// One native controller stays attached to the real ego and sees independent target copies.
#define ACCSIM_BRIDGE_BUILD
#include "native_acc_bridge.h"
#include "ControllerACC.hpp"
#include "Entities.hpp"
#include "ScenarioReader.hpp"
#include "ScenarioEngine.hpp"
#include <cmath>
#include <limits>
#include <stdexcept>

namespace {
using namespace scenarioengine;
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

extern "C" ACCSIM_API int ACCSIM_HasViewer() {
#ifdef _USE_OSG
    return 1;
#else
    return 0;
#endif
}
