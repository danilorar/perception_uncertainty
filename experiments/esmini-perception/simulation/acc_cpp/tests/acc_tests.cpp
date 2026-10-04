// Author: Zhuo Ma
// Golden numerical cases executed by the original esmini v3.8.1 ControllerACC.
// Checks deliberately use exceptions instead of assert(), so Release builds
// retain every check even if NDEBUG is set.
#include "accsim/control.hpp"
#include "accsim/engine.hpp"
#include <filesystem>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>

namespace {
void require(bool condition, const std::string& message) {
    if (!condition) throw std::runtime_error(message);
}
void near(double actual, double expected, const std::string& message, double tolerance=1e-9) {
    require(std::isfinite(actual) && std::abs(actual - expected) <= tolerance,
            message + ": expected " + std::to_string(expected) + ", got " + std::to_string(actual));
}
accsim::ControllerInput input_with_speed(double speed) {
    accsim::ControllerInput input;
    input.dt_s = 0.01;
    input.ego.id = 1;
    input.ego.pose = {-54.76,-10.17,.1,.145748,0,0};
    input.ego.speed = speed;
    input.ego.box.length = 5.0;
    input.ego.box.width = 1.8;
    input.ego.box.height = 1.6;
    return input;
}
accsim::Observation target(double x, double speed, int id=2) {
    accsim::Observation observation;
    observation.track_id = id;
    observation.pose = {-54.76+x*std::cos(.145748),-10.17+x*std::sin(.145748),.1,.145748,0,0};
    observation.speed = speed;
    observation.box.length = 5.0;
    observation.box.width = 1.8;
    observation.box.height = 1.6;
    return observation;
}
} // namespace

int main(int argc,char** argv) {
    try {
        if(argc!=3) throw std::invalid_argument("acc_tests ORIGINAL_SCENARIO TEMP_DIRECTORY");
        const std::filesystem::path output=argv[2];
        std::filesystem::create_directories(output);
        const auto scene=accsim::prepare_scenario(argv[1],output/"generated.xosc");
        accsim::EsminiAdapter engine(scene,output,{},accsim::SensorConfig{},.01);
        accsim::AccConfig config;
        config.set_speed_mps = 20.0;
        accsim::EsminiAcc controller(config);
        auto input = input_with_speed(10.0);

        auto request = controller.update(input);
        near(request.acceleration_mps2, 7.0, "Free-road acceleration uses 0.7*max_acceleration");
        near(request.desired_speed_mps, 10.07, "Free-road next speed");
        require(request.lead_id == -1, "Empty observation list has no lead");

        // Exact upstream weighted relative-distance / relative-speed case.
        config.set_speed_mps = 15.0;
        accsim::EsminiAcc weighted(config);
        input.perception.objects = {target(25.0, 8.0)}; // bumper gap20, desiredgap18
        request = weighted.update(input);
        near(request.observed_gap_m, 20.0, "Bumper gap from two 5m vehicles");
        near(request.acceleration_mps2, -17.0 / 18.0, "Weighted upstream ACC formula");
        require(request.lead_id == 2, "Detected target selected");

        // A zero lower clamp on the distance factor would produce zero here.
        config.set_speed_mps = 10.0;
        accsim::EsminiAcc negative_factor(config);
        input.perception.objects = {target(14.0, 10.0)}; // gap9, desiredgap18, factor-.5
        near(negative_factor.update(input).acceleration_mps2, -1.25,
             "Negative distance factor must remain negative");

        input.perception.objects = {target(5.5, 0.0)};
        request = negative_factor.update(input);
        require(request.close_gap_stop, "Sub-metre gap requests upstream immediate stop");
        near(request.desired_speed_mps, 0.0, "Sub-metre desired speed");
        near(request.acceleration_mps2, -1000.0, "Stop request is exposed for separate actuator limiting");

        // Truth can contain a close obstacle, but it is not a controller input.
        accsim::WorldTruthFrame truth;
        truth.ego = input.ego;
        accsim::ObjectState real_target;
        real_target.id = 2;
        real_target.pose.x = 5.5;
        truth.targets.push_back(real_target);
        input.perception.objects.clear();
        require(!truth.targets.empty(), "Fixture contains an unobserved obstacle");
        request = controller.update(input);
        near(request.acceleration_mps2, 7.0, "Missed target cannot influence control through truth");
        require(request.lead_id == -1, "No truth target is selected through an empty perception");

        input.perception.objects = {target(45.0, 10.0, 3), target(25.0, 10.0, 2)};
        require(controller.update(input).lead_id == 2, "Nearest front target selected");
        input.perception.objects = {target(25.0, 10.0, 2), target(45.0, 10.0, 3)};
        require(controller.update(input).lead_id == 2, "Nearest front target independent of ordinary list order");

        input.ego.box.cx = 1.0;
        input.perception.objects = {target(25.0, 10.0)};
        input.perception.objects.front().box.cx = 0.5;
        near(controller.update(input).observed_gap_m, 19.5, "Bounding-box reference offsets retained");
        input.perception.objects.front().pose.h = .145748+accsim::pi;
        near(controller.update(input).observed_gap_m, 18.5, "Opposite-heading box offset branch retained");
        input = input_with_speed(10.0);

        auto sideways = target(25.0, 0.0);
        sideways.pose.y += 10.0;
        input.perception.objects = {sideways};
        require(controller.update(input).lead_id == -1, "Outside lateral gate excluded");
        input.perception.objects = {target(-25.0, 0.0)};
        require(controller.update(input).lead_id == -1, "Target behind ego excluded");

        // Far lead would accelerate, but the set-speed ceiling takes priority.
        config.set_speed_mps = 10.001;
        accsim::EsminiAcc speed_ceiling(config);
        input.perception.objects = {target(45.0, 10.0)};
        request = speed_ceiling.update(input);
        near(request.desired_speed_mps, 10.001, "Following-mode set-speed ceiling");
        near(request.acceleration_mps2, 0.1, "Acceleration reflects the clipped requested speed");

        // Severe close-speed mismatch, above the special one-metre branch.
        input = input_with_speed(30.0);
        input.perception.objects = {target(8.0, 0.0)};
        config.set_speed_mps = 30.0;
        accsim::EsminiAcc deceleration_bound(config);
        request = deceleration_bound.update(input);
        near(request.acceleration_mps2, -10.0, "Normal ACC deceleration is bounded");
        require(request.desired_speed_mps >= 0.0, "Desired speed is nonnegative");

        input = input_with_speed(0.0);
        config.set_speed_mps = 0.0;
        accsim::EsminiAcc stopped(config);
        near(stopped.update(input).desired_speed_mps, 0.0, "Stopped cruise stays stopped");

        input.dt_s = 0.0;
        bool rejected = false;
        try { stopped.update(input); } catch (const std::invalid_argument&) { rejected = true; }
        require(rejected, "Zero controller timestep rejected");

        std::cout << "ACC golden cases passed\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "ACC test failed: " << error.what() << '\n';
        return 1;
    }
}
