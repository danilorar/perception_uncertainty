// Author: Zhuo Ma
#pragma once
#include "scenario.hpp"
#include "native_acc_bridge.h"
#include <filesystem>
#include <vector>
namespace accsim {
// Optional NCAP base-scenario parameters. Unset values preserve XOSC defaults.
struct ScenarioOverrides {
    bool set_ego_speed = false;
    double ego_speed_kph = 0;
    bool set_target_speed = false;
    double target_speed_kph = 0;
    bool set_target_braking = false;
    bool target_braking = false;
};

class EsminiAdapter {
    bool active_=false;
    int ego_id_=-1;
    std::vector<int> target_ids_;
public:
    EsminiAdapter(const PreparedScenario& scenario,const std::filesystem::path& output,
                  const std::filesystem::path& resource_home,double dt,bool viewer=false,
                  const ACCSIM_SensorConfig& sensor = {},
                  const ScenarioOverrides& overrides = {});
    ~EsminiAdapter();
    EsminiAdapter(const EsminiAdapter&)=delete;
    EsminiAdapter& operator=(const EsminiAdapter&)=delete;
    WorldTruthFrame truth() const;
    void advance_native(double dt);
    bool quit() const;
    bool collision() const;
};
} // namespace accsim
