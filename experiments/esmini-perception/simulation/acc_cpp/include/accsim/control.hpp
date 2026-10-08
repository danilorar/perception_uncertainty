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
// Keeps upstream brake-ramp/latch state for the entire run. Ego remains exact;
// external-object states come only from the cached post-uncertainty frame.
class EsminiAeb {
    void* context_=nullptr;
public:
    explicit EsminiAeb(AebConfig config);
    ~EsminiAeb();
    EsminiAeb(const EsminiAeb&)=delete;
    EsminiAeb& operator=(const EsminiAeb&)=delete;
    ControlRequest update(const ControllerInput& input);
};
// The arbitration layer owns actuator bounds, including the upstream <1m stop request.
AppliedCommand arbitrate(const ControlRequest& request, const AccConfig& limits);
void advance_vehicle(EgoMotion& ego, const AppliedCommand& command, double dt, const ReferencePath& reference);
} // namespace accsim
