// Author: Zhuo Ma
// All native API calls and truth sampling are confined to this adapter.
#include "accsim/engine.hpp"
#include "esminiLib.hpp"
#include "native_acc_bridge.h"
#include <cmath>
#include <stdexcept>
#include <string>

namespace accsim {
namespace {
void checked(int status, const char* operation) {
    if (status != 0) {
        throw std::runtime_error(operation);
    }
}

// Callback storage lives until SE_InitWithArgs returns; never throw through esmini.
struct ParameterCallbackData {
    const ScenarioOverrides& values;
    bool called = false;
    std::string error;
};

void apply_parameters(void* user_data) {
    auto& data = *static_cast<ParameterCallbackData*>(user_data);
    data.called = true;
    try {
        if (data.values.set_ego_speed) {
            checked(SE_SetParameterDouble("Ego_speed_kph", data.values.ego_speed_kph),
                    "Cannot override Ego_speed_kph: missing or wrong parameter type");
        }
        if (data.values.set_target_speed) {
            checked(SE_SetParameterDouble("GVT_init_speed_kph", data.values.target_speed_kph),
                    "Cannot override GVT_init_speed_kph: missing or wrong parameter type");
        }
        if (data.values.set_target_braking) {
            checked(SE_SetParameterBool("isCCRbraking", data.values.target_braking),
                    "Cannot override isCCRbraking: missing or wrong parameter type");
        }
    } catch (const std::exception& error) {
        data.error = error.what();
    }
}

ObjectState state(int id) {
    SE_ScenarioObjectState s{};
    checked(SE_GetObjectState(id, &s), "SE_GetObjectState failed");
    ObjectState result{s.id,
                       s.objectType,
                       s.objectCategory,
                       {s.x, s.y, s.z, s.h, s.p, s.r},
                       {s.length, s.width, s.height, s.centerOffsetX, s.centerOffsetY, s.centerOffsetZ},
                       s.speed};
    checked(SE_GetObjectVelocityGlobalXYZ(id, &result.velocity.x, &result.velocity.y, &result.velocity.z),
            "Get object velocity failed");
    checked(SE_GetObjectAccelerationGlobalXYZ(id, &result.acceleration.x, &result.acceleration.y,
                                              &result.acceleration.z),
            "Get object acceleration failed");
    return result;
}
} // namespace

EsminiAdapter::EsminiAdapter(const PreparedScenario& scenario, const std::filesystem::path& output,
                             const std::filesystem::path& resources, double dt, bool viewer,
                             const ACCSIM_SensorConfig& sensor, const ScenarioOverrides& overrides,
                             const ACCSIM_MarkovConfig& markov) {
    if (markov.enabled && std::abs(markov.period_s - dt) > 1e-9) {
        throw std::invalid_argument("Markov period must equal simulation dt");
    }
    // Build the arguments for the embedded esmini engine, not for acc_sim parsing.
    std::vector<std::string> options = {
        "acc_sim",
        "--osc", scenario.generated.string(),
        "--path", scenario.source.parent_path().string(),
        "--disable_stdout",
        "--traj_filter", "0",
        "--seed", "0",
        "--collision",
        "--fixed_timestep", std::to_string(dt),
        "--record", (output / "sim.dat").string(),
        "--logfile_path", (output / "run.log").string()
    };
    if (viewer) {
        if (!ACCSIM_HasViewer()) {
            throw std::runtime_error("Viewer requires rebuilding: python3 tools/build_native.py --viewer");
        }
        options.insert(options.end(), {"--window", "60", "60", "1000", "700"});
    } else {
        options.push_back("--headless");
    }
    if (!resources.empty()) {
        options.insert(options.end(), {"--path", (resources / "resources").string()});
    }

    // The native API expects an array of C strings. options keeps their storage alive.
    std::vector<const char*> argv;
    for (const auto& option : options) {
        argv.push_back(option.c_str());
    }

    // Register our controller names before esmini reads them from the scenario.
    checked(ACCSIM_ConfigureSensor(&sensor), "Invalid ideal sensor settings");
    checked(ACCSIM_ConfigureMarkov(&markov), "Invalid Markov probability/duration/period");
    ACCSIM_RegisterPersistentAcc();

    // Load the generated scenario and initialize esmini. Zero means success.
    // esmini manages run.log and sim.dat; main.cpp opens the JSON and CSV outputs.
    const bool has_overrides = overrides.set_ego_speed || overrides.set_target_speed ||
                               overrides.set_target_braking;
    ParameterCallbackData callback{overrides, false, {}};
    SE_RegisterParameterDeclarationCallback(has_overrides ? apply_parameters : nullptr,
                                             has_overrides ? &callback : nullptr);
    int status = -1;
    try {
        status = SE_InitWithArgs(static_cast<int>(argv.size()), argv.data());
    } catch (...) {
        SE_RegisterParameterDeclarationCallback(nullptr, nullptr);
        SE_Close();
        throw;
    }
    // esmini retains callbacks between runs: remove the pointer to local storage.
    SE_RegisterParameterDeclarationCallback(nullptr, nullptr);
    if (!callback.error.empty() || (has_overrides && !callback.called) || status != 0) {
        SE_Close();
        throw std::runtime_error(callback.error.empty()
            ? "Cannot initialize esmini; inspect run.log" : callback.error);
    }
    active_ = true;
    try {
        // Remember object IDs for later truth sampling.
        ego_id_ = SE_GetIdByName(scenario.ego_name.c_str());
        if (ego_id_ < 0) {
            throw std::runtime_error("Missing ego object: " + scenario.ego_name);
        }
        const int count = SE_GetNumberOfObjects();
        if (count < 1) {
            throw std::runtime_error("Scenario has no objects");
        }
        for (int i = 0; i < count; ++i) {
            const int id = SE_GetId(i);
            if (id != ego_id_) {
                target_ids_.push_back(id);
            }
        }

        if (std::abs(SE_GetSimulationTime()) > 1e-9) {
            throw std::runtime_error("Initialization advanced physical time");
        }
    } catch (...) {
        SE_Close();
        active_ = false;
        throw;
    }
}

EsminiAdapter::~EsminiAdapter() {
    if (active_) {
        SE_Close();
    }
}

// Read actual simulation states for logging and safety evaluation.
WorldTruthFrame EsminiAdapter::truth() const {
    WorldTruthFrame frame;
    frame.time_s = SE_GetSimulationTime();
    frame.ego = state(ego_id_);
    for (int id : target_ids_) {
        frame.targets.push_back(state(id));
    }
    return frame;
}

// ACC movement: esmini executes scenario actions and the persistent ACC controller.
// The wrapper calls upstream ControllerACC::Step; this adapter does not move ego itself.
void EsminiAdapter::advance_native(double dt) {
    checked(SE_StepDT(dt), "Advance native ACC failed");
}

bool EsminiAdapter::quit() const {
    return SE_GetQuitFlag() != 0;
}

bool EsminiAdapter::collision() const {
    return SE_GetObjectNumberOfCollisions(ego_id_) > 0;
}
} // namespace accsim
