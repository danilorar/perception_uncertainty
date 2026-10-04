// Author: Zhuo Ma
#pragma once
#include "types.hpp"
namespace accsim {
class DetectionDropout {
    double probability_;
    std::uint64_t seed_;
public:
    DetectionDropout(double probability, std::uint64_t seed);
    DropoutResult apply(const PerceptionFrame& ideal) const;
};
} // namespace accsim
