// Author: Zhuo Ma
#pragma once
#include "scenario.hpp"
namespace accsim {
class EsminiAcc {
    AccConfig config_;
public:
    explicit EsminiAcc(AccConfig config): config_(config) {}
    ControlRequest update(const ControllerInput& input) const;
};
// The arbitration layer owns actuator bounds, including the upstream <1m stop request.
AppliedCommand arbitrate(const ControlRequest& request, const AccConfig& limits);
void advance_vehicle(EgoMotion& ego, const AppliedCommand& command, double dt, const ReferencePath& reference);
} // namespace accsim
