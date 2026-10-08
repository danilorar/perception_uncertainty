// Author: Zhuo Ma
// Compiled INTO esminiLib alongside the unmodified v3.8.1 R157 controller.
// This selects its AEB component, not the combined human-reference-driver model.
// Native Process computes road-relative distance/TTC; UpdateAEB handles trigger
// and frontal overlap; ReactCritical supplies the original 0.6s brake ramp.
#define ACCSIM_BRIDGE_BUILD
#include "native_acc_bridge.h"
#include "ControllerALKS_R157SM.hpp"
#include <cmath>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <stdexcept>
namespace {
using namespace scenarioengine;
using Native=ControllerALKS_R157SM;
std::string precise(double x) { std::ostringstream s; s<<std::setprecision(17)<<x; return s.str(); }
void fill(Vehicle& vehicle,const ACCSIM_Object& d) {
    vehicle.id_=d.id; vehicle.category_=d.category;
    vehicle.name_="perceived_"+std::to_string(d.id);
    vehicle.boundingbox_.dimensions_.length_=d.length;
    vehicle.boundingbox_.dimensions_.width_=d.width;
    vehicle.boundingbox_.dimensions_.height_=d.height;
    vehicle.boundingbox_.center_.x_=d.cx;
    vehicle.boundingbox_.center_.y_=d.cy;
    vehicle.boundingbox_.center_.z_=d.cz;
    vehicle.pos_.SetInertiaPos(d.x,d.y,d.z,d.h,d.p,d.r);
    vehicle.SetSpeed(d.speed);
    vehicle.pos_.SetVel(d.vx,d.vy,d.vz);
    vehicle.pos_.SetAcc(d.ax,d.ay,d.az);
}
class ObservedAeb final: public Native {
public:
    ObservedAeb(InitArgs* args,Entities& view):Native(args) {
        entities_=&view;
        model_->entities_=&view; // Private copies only, never player->entities_.
    }
};
struct Context {
    Entities view; // Owns the permanent ego proxy and transient observed targets.
    OSCProperties properties;
    Vehicle* host=nullptr;
    std::unique_ptr<ObservedAeb> controller; // Destroy before the owned vehicles.
    Native::ReferenceDriver* aeb=nullptr;
    explicit Context(const ACCSIM_AebConfig& config) {
        properties.property_={{"model","ReferenceDriver"},{"cruise","false"},{"logLevel","0"},
            {"aebTTC",precise(config.ttc_s)},{"aebDeceleration",precise(config.max_deceleration_mps2)}};
        Controller::InitArgs args{};
        args.name="ObservedBuiltinAEBComponent";
        args.type="ALKS_R157SM_Controller"; args.properties=&properties;
        host=new Vehicle(); view.object_.push_back(host);
        controller=std::make_unique<ObservedAeb>(&args,view);
        controller->LinkObject(host);
        // Upstream stores privately inherited ReferenceDriver as Model* using
        // reinterpret_cast; use the same pinned-version conversion back.
        aeb=reinterpret_cast<Native::ReferenceDriver*>(controller->model_);
    }
    void clear_targets() {
        controller->model_->ResetObjectInFocus();
        while(view.object_.size()>1) { delete view.object_.back(); view.object_.pop_back(); }
    }
};
}
extern "C" ACCSIM_API void* ACCSIM_NativeAebCreate(const ACCSIM_AebConfig* config) {
    if(!config || !(config->ttc_s>0) || !std::isfinite(config->ttc_s) ||
       !(config->max_deceleration_mps2>0) || !std::isfinite(config->max_deceleration_mps2)) return nullptr;
    try { return new Context(*config); } catch(...) { return nullptr; }
}
extern "C" ACCSIM_API void ACCSIM_NativeAebDestroy(void* context) { delete static_cast<Context*>(context); }
extern "C" ACCSIM_API int ACCSIM_NativeAebStep(void* context,const ACCSIM_Object* ego,
    const ACCSIM_Object* observations,int count,double dt,ACCSIM_AebOutput* output) {
    if(!context || !ego || !output || count<0 || (count && !observations) || !(dt>0) || !std::isfinite(dt)) return -1;
    try {
        auto& ctx=*static_cast<Context*>(context);
        ctx.clear_targets();
        fill(*ctx.host,*ego);
        auto* model=ctx.controller->model_;
        model->dt_=dt;
        *output={ego->speed,0,std::numeric_limits<double>::infinity(),std::numeric_limits<double>::infinity(),-1,0};
        for(int i=0;i<count;++i) {
            auto* target=new Vehicle(); ctx.view.object_.push_back(target); fill(*target,observations[i]);
            Native::Model::ObjectInfo info; info.obj=target;
            if(model->Process(info)!=0) continue; // ORIGINAL esmini geometry/TTC.
            const bool was_active=ctx.aeb->aeb_.active_;
            ctx.aeb->UpdateAEB(ctx.host,&info); // ORIGINAL trigger/overlap test.
            // Prefer the target that actually triggered; otherwise log min TTC.
            if((!was_active && ctx.aeb->aeb_.active_) ||
               (output->lead_id<0 || info.ttc<output->observed_ttc_s)) {
                output->lead_id=target->id_; output->observed_gap_m=info.dist_long;
                output->observed_ttc_s=info.ttc;
            }
        }
        if(ctx.aeb->aeb_.active_) {
            // Brake state is preserved even if the next perception is empty.
            // The upstream AEB component resets its latch when ego stops.
            output->desired_speed_mps=ctx.aeb->ReactCritical(); // ORIGINAL ramp.
            output->acceleration_mps2=model->acc_;
        } else {
            model->acc_=0; // Nominal driver holds current speed before triggering.
        }
        output->active=ctx.aeb->aeb_.active_;
        ctx.clear_targets(); // No stale pointer to a previously perceived object.
        return 0;
    } catch(...) { return -2; }
}
