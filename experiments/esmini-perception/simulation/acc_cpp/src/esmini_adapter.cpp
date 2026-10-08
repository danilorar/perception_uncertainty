// Author: Zhuo Ma
// All native API calls and truth sampling are confined to this adapter.
#include "accsim/engine.hpp"
#include "esminiLib.hpp"
#include <cmath>
#include <stdexcept>
#include <string>
namespace accsim {
namespace {
void checked(int status,const char* operation) { if(status!=0) throw std::runtime_error(operation); }
ObjectState state(int id) {
    SE_ScenarioObjectState s{};
    checked(SE_GetObjectState(id,&s),"SE_GetObjectState failed");
    ObjectState result{s.id,s.objectType,s.objectCategory,{s.x,s.y,s.z,s.h,s.p,s.r},
            {s.length,s.width,s.height,s.centerOffsetX,s.centerOffsetY,s.centerOffsetZ},s.speed};
    checked(SE_GetObjectVelocityGlobalXYZ(id,&result.velocity.x,&result.velocity.y,&result.velocity.z),"Get object velocity failed");
    checked(SE_GetObjectAccelerationGlobalXYZ(id,&result.acceleration.x,&result.acceleration.y,&result.acceleration.z),"Get object acceleration failed");
    return result;
}
}
EsminiAdapter::EsminiAdapter(const PreparedScenario& scenario,const std::filesystem::path& output,
                             const std::filesystem::path& resources,const SensorConfig& sensor,double dt) {
    std::vector<std::string> options={"acc_sim","--osc",scenario.generated.string(),"--path",scenario.source.parent_path().string(),
        "--headless","--disable_stdout","--traj_filter","0","--seed","0","--collision",
        "--fixed_timestep",std::to_string(dt),"--record",(output/"sim.dat").string(),
        "--logfile_path",(output/"run.log").string()};
    if(!resources.empty()) options.insert(options.begin()+5,(resources/"resources").string());
    std::vector<const char*> argv;
    for(const auto& option:options) argv.push_back(option.c_str());
    if(SE_InitWithArgs(static_cast<int>(argv.size()),argv.data())!=0) { SE_Close(); throw std::runtime_error("Cannot initialize esmini; inspect run.log"); }
    active_=true;
    try {
        ego_id_=SE_GetIdByName("object_1");
        if(ego_id_<0) throw std::runtime_error("Missing object_1");
        const int count=SE_GetNumberOfObjects();
        if(count<2 || count>sensor.capacity) throw std::runtime_error("Insufficient ideal-sensor capacity");
        for(int i=0;i<count;++i) { const int id=SE_GetId(i); if(id!=ego_id_) target_ids_.push_back(id); }
        sensor_buffer_.resize(static_cast<std::size_t>(sensor.capacity));
        // v3.8.1's implementation uses radians, despite its header's degree comment.
        sensor_id_=SE_AddObjectSensor(ego_id_,sensor.x_m,sensor.y_m,sensor.z_m,sensor.yaw_rad,
                                     sensor.near_m,sensor.far_m,sensor.fov_rad,sensor.capacity);
        if(sensor_id_<0) throw std::runtime_error("Cannot add ideal sensor");
        checked(SE_StepDT(0.0),"Cannot refresh initial ideal sensor");
        // At time zero esmini has no displacement history for velocity. The
        // adapter already derives initial speeds from the original first segment;
        // seed velocity consistently for all vehicles, without advancing time.
        for(int i=0;i<count;++i) {
            const auto initial=state(SE_GetId(i));
            checked(SE_ReportObjectVel(initial.id,initial.speed*std::cos(initial.pose.h),
                       initial.speed*std::sin(initial.pose.h),0),"Initialize velocity failed");
        }
        if(std::abs(SE_GetSimulationTime())>1e-9) throw std::runtime_error("Initialization advanced physical time");
        SE_ScenarioObjectState ego{};
        checked(SE_GetObjectState(ego_id_,&ego),"Cannot verify external ego bridge");
        if(ego.ctrl_type!=1) throw std::runtime_error("Ego ExternalController is not active");
    } catch(...) { SE_Close(); active_=false; throw; }
}
EsminiAdapter::~EsminiAdapter() { if(active_) SE_Close(); }
WorldTruthFrame EsminiAdapter::truth() const {
    WorldTruthFrame frame; frame.time_s=SE_GetSimulationTime(); frame.ego=state(ego_id_);
    for(int id:target_ids_) frame.targets.push_back(state(id));
    return frame;
}
PerceptionFrame EsminiAdapter::sense(std::uint64_t sequence) const {
    auto buffer=sensor_buffer_;
    const int count=SE_FetchSensorObjectList(sensor_id_,buffer.data());
    if(count<0 || count>static_cast<int>(buffer.size())) throw std::runtime_error("Invalid sensor count");
    PerceptionFrame frame; frame.sequence=sequence;
    frame.measurement_time_s=frame.delivery_time_s=SE_GetSimulationTime();
    for(int i=0;i<count;++i) {
        const auto s=state(buffer[static_cast<std::size_t>(i)]);
        if(s.id==ego_id_) continue;
        frame.objects.push_back({s.id,s.object_type,s.object_category,s.pose,s.box,s.speed,s.velocity,s.acceleration});
    }
    return frame;
}
void EsminiAdapter::advance(const EgoMotion& ego,const AppliedCommand& command,double dt) {
    const auto& s=ego.state;
    checked(SE_ReportObjectPos(ego_id_,s.pose.x,s.pose.y,s.pose.z,s.pose.h,s.pose.p,s.pose.r),"Report ego pose failed");
    checked(SE_ReportObjectSpeed(ego_id_,s.speed),"Report ego speed failed");
    checked(SE_ReportObjectVel(ego_id_,s.speed*std::cos(s.pose.h),s.speed*std::sin(s.pose.h),0),"Report ego velocity failed");
    checked(SE_ReportObjectAcc(ego_id_,command.acceleration_mps2*std::cos(s.pose.h),command.acceleration_mps2*std::sin(s.pose.h),0),"Report ego acceleration failed");
    checked(SE_StepDT(dt),"Advance esmini failed");
    const auto committed=state(ego_id_);
    if(std::hypot(committed.pose.x-s.pose.x,committed.pose.y-s.pose.y)>1e-7 || std::abs(committed.speed-s.speed)>1e-7)
        throw std::runtime_error("Scenario overwrote the externally owned ego state");
}
bool EsminiAdapter::quit() const { return SE_GetQuitFlag()!=0; }
bool EsminiAdapter::collision() const { return SE_GetObjectNumberOfCollisions(ego_id_)>0; }
} // namespace accsim
