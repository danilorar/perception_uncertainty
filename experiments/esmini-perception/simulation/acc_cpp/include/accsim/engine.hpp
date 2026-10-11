// Author: Zhuo Ma
#pragma once
#include "scenario.hpp"
#include <filesystem>
#include <vector>
namespace accsim {
class EsminiAdapter {
    bool active_=false;
    int ego_id_=-1;
    std::vector<int> target_ids_;
public:
    EsminiAdapter(const PreparedScenario& scenario,const std::filesystem::path& output,
                  const std::filesystem::path& resource_home,double dt,bool viewer=false);
    ~EsminiAdapter();
    EsminiAdapter(const EsminiAdapter&)=delete;
    EsminiAdapter& operator=(const EsminiAdapter&)=delete;
    WorldTruthFrame truth() const;
    void advance_native(double dt);
    bool quit() const;
    bool collision() const;
};
} // namespace accsim
