// Author: Zhuo Ma
#pragma once
#include "types.hpp"
#include <filesystem>
#include <vector>
namespace accsim {
struct ReferenceVertex { double time_s=0, distance_m=0; Pose pose; };
class ReferencePath {
public:
    std::vector<ReferenceVertex> vertices;
    Pose at_distance(double distance_m) const;
    double initial_speed() const;
    // Nearest path station/lateral offset; end segments permit extrapolation.
    std::pair<double,double> project(const Pose& pose) const;
};
struct PreparedScenario {
    std::filesystem::path source, road, generated;
    ReferencePath reference;
    double duration_s=10;
    double max_acceleration=10, max_deceleration=10;
};
PreparedScenario prepare_scenario(const std::filesystem::path& source,
                                  const std::filesystem::path& generated,
                                  const std::string& ego_name="object_1");
} // namespace accsim
