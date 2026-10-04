// Author: Zhuo Ma
#pragma once
#include "scenario.hpp"
#include <filesystem>
#include <vector>
namespace accsim {
class EsminiAdapter {
    bool active_=false;
    int ego_id_=-1,sensor_id_=-1;
    std::vector<int> target_ids_,sensor_buffer_;
public:
    EsminiAdapter(const PreparedScenario& scenario,const std::filesystem::path& output,
                  const std::filesystem::path& resource_home,const SensorConfig& sensor,double dt);
    ~EsminiAdapter();
    EsminiAdapter(const EsminiAdapter&)=delete;
    EsminiAdapter& operator=(const EsminiAdapter&)=delete;
    WorldTruthFrame truth() const;
    PerceptionFrame sense(std::uint64_t sequence) const;
    void advance(const EgoMotion& ego,const AppliedCommand& command,double dt);
    bool quit() const;
    bool collision() const;
};
} // namespace accsim
