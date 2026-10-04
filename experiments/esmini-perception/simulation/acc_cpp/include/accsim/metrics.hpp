// Author: Zhuo Ma
#pragma once
#include "types.hpp"
namespace accsim {
Geometry geometry(const ObjectState& ego, const ObjectState& target);
StepMetrics evaluate_truth(const WorldTruthFrame& truth);
} // namespace accsim
