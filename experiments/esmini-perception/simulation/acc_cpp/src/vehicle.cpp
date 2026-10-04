// Author: Zhuo Ma
// One longitudinal kinematic plant, with an ideal lateral path-following driver.
// No tyres, actuator delay or collision response are modelled in this version.
#include "accsim/control.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>
namespace accsim {
AppliedCommand arbitrate(const ControlRequest& request, const AccConfig& limits) {
    if (!std::isfinite(request.acceleration_mps2) || limits.max_acceleration <= 0 || limits.max_deceleration <= 0)
        throw std::invalid_argument("Invalid command or actuator limits");
    const auto acceleration = std::clamp(request.acceleration_mps2, -limits.max_deceleration, limits.max_acceleration);
    return {acceleration, std::abs(acceleration-request.acceleration_mps2)>1e-10};
}
void advance_vehicle(EgoMotion& ego, const AppliedCommand& command, double dt, const ReferencePath& reference) {
    if (!(dt>0) || !std::isfinite(dt) || ego.state.speed<0) throw std::invalid_argument("Invalid vehicle timestep/state");
    // Exact constant-acceleration integration, including stopping inside a step.
    const double moving_dt = command.acceleration_mps2<0 ? std::min(dt, ego.state.speed/-command.acceleration_mps2) : dt;
    ego.progress_m += ego.state.speed*moving_dt + .5*command.acceleration_mps2*moving_dt*moving_dt;
    ego.state.speed = std::max(0.0, ego.state.speed+command.acceleration_mps2*moving_dt);
    ego.state.pose = reference.at_distance(ego.progress_m);
}
} // namespace accsim
