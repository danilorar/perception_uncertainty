// Author: Zhuo Ma
// Connects independent target observations to esmini's native ACC.
// It does not implement the ACC algorithm. It is compiled into esminiLib.
// ControllerACC::Step is the original esmini v3.8.1 implementation, not a port.
// One native controller stays attached to the real ego and sees independent target copies.
#define ACCSIM_BRIDGE_BUILD
#include "native_acc_bridge.h"
#include "accsim/perception.hpp"
#include "ControllerACC.hpp"
#include "IdealSensor.hpp"
#include "Entities.hpp"
#include "ScenarioReader.hpp"
#include "ScenarioEngine.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>
#include <stdexcept>

namespace {

using namespace scenarioengine;

// =================================================================
// Latest ACC diagnostics, read by the runner after each simulation step
// =================================================================
// Only the target view differs. The real ego and upstream lifecycle stay intact.
ACCSIM_PersistentAccOutput last_output{};
bool output_valid = false;
ACCSIM_SensorConfig sensor_config{};
std::vector<int> detected_ids;
std::vector<int> dropped_ids;
std::vector<int> supplied_ids;
ACCSIM_MarkovConfig markov_config{};
ACCSIM_AccInitialization initialization{};
bool initialization_valid = false;

// =================================================================
// Persistent native ACC: prepare observations, execute ACC, record output
// =================================================================
class PersistentAcc final : public ControllerACC {
    // true: independent target copies; false: live targets for validation.
    bool copies_;
    Entities observations_;
    ACCSIM_SensorConfig sensor_settings_;
    std::unique_ptr<ObjectSensor> sensor_;
    std::size_t sensor_capacity_ = 0;
    double configured_speed_ = -1;
    ACCSIM_MarkovConfig markov_settings_;
    std::unique_ptr<accsim::MarkovDropout> markov_;
    std::vector<int> registered_ids_;
    std::uint64_t measurement_sequence_ = 0;
    double last_measurement_time_ = -std::numeric_limits<double>::infinity();

    // ===== MARKOV: filter IDs only; native positions are copied below =====
    void filter_markov(Entities& truth, double time) {
        if (!markov_) {
            for (auto* target : truth.object_) {
                if (target && target != object_) {
                    registered_ids_.push_back(target->id_);
                }
            }
            markov_ = std::make_unique<accsim::MarkovDropout>(
                markov_settings_.probability, markov_settings_.mean_missing_s,
                markov_settings_.period_s, markov_settings_.seed, registered_ids_);
        }
        for (auto* target : truth.object_) {
            if (target && target != object_ &&
                std::find(registered_ids_.begin(), registered_ids_.end(), target->id_) == registered_ids_.end()) {
                throw std::runtime_error("Markov does not support dynamically added target IDs");
            }
        }
        if (std::isfinite(last_measurement_time_) && std::abs(time - last_measurement_time_) > 1e-8) {
            ++measurement_sequence_;
        }
        accsim::PerceptionFrame frame;
        frame.sequence = measurement_sequence_;
        frame.measurement_time_s = time;
        frame.delivery_time_s = time;
        for (int id : detected_ids) {
            accsim::Observation observation;
            observation.track_id = id;
            frame.objects.push_back(observation);
        }
        // The first sample initializes stationary states, without a transition.
        // Repeated zero-time samples reuse the cache; each later dt advances once.
        const auto filtered = markov_->apply(frame);
        last_measurement_time_ = time;
        dropped_ids = filtered.dropped_ids;
        supplied_ids.clear();
        for (const auto& observation : filtered.frame.objects) {
            supplied_ids.push_back(observation.track_id);
        }
    }

    // =============================================================
    // IDEAL SENSOR: create esmini ObjectSensor and size its hit buffer
    // =============================================================
    void ensure_sensor() {
        const std::size_t required = entities_->object_.size();
        if (!sensor_ || required > sensor_capacity_) {
            // Upstream Update does not enforce maxObj: allocate for every entity.
            sensor_capacity_ = required;
            sensor_ = std::make_unique<ObjectSensor>(
                entities_, object_, sensor_settings_.x_m, sensor_settings_.y_m,
                sensor_settings_.z_m, sensor_settings_.heading_rad,
                sensor_settings_.near_m, sensor_settings_.far_m,
                sensor_settings_.horizontal_fov_rad, static_cast<int>(sensor_capacity_));
        }
    }

public:
    PersistentAcc(InitArgs* args, bool copies)
        : ControllerACC(args), copies_(copies), sensor_settings_(sensor_config),
          markov_settings_(markov_config) {
        if (markov_settings_.enabled && (!copies_ || !sensor_settings_.enabled)) {
            throw std::runtime_error("Markov requires ideal sensing and target-copy mode");
        }
        output_valid = false;
        initialization_valid = false;
        if (args && args->properties && args->properties->ValueExists("setSpeed")) {
            configured_speed_ = std::stod(args->properties->GetValueStr("setSpeed"));
        }
    }

    int Activate(const ControlActivationMode (&mode)[static_cast<unsigned int>(ControlDomains::COUNT)]) override {
        if (!initialization_valid) {
            if (object_->GetType() != Object::Type::VEHICLE) {
                throw std::runtime_error("ACC ego must resolve to a vehicle");
            }
            initialization = {
                object_->GetSpeed(),
                configured_speed_ < 0 ? object_->GetSpeed() : configured_speed_,
                object_->GetMaxAcceleration(),
                object_->GetMaxDeceleration()
            };
            initialization_valid = true;
        }
        return ControllerACC::Activate(mode);
    }

    ~PersistentAcc() override {
        output_valid = false;
        initialization_valid = false;
    }

    void InitPostPlayer() override {
        ControllerACC::InitPostPlayer();
        if (copies_ && sensor_settings_.enabled) {
            ensure_sensor();
        }
    }

    void Step(double dt) override {
        // ===== 1. Clear the previous step's target copies =====
        Entities* truth = entities_;
        for (auto* target : observations_.object_) {
            delete target;
        }
        observations_.object_.clear();

        // ===== 2. Build an independent target view, in scenario order =====
        // Detect at controller time, before swapping the native entity view.
        detected_ids.clear();
        dropped_ids.clear();
        supplied_ids.clear();

        // ===== IDEAL SENSOR: measure visible targets at controller time =====
        if (copies_ && sensor_settings_.enabled) {
            ensure_sensor();
            sensor_->Update(); // Exactly one update per ACC step.
            for (int i = 0; i < sensor_->nObj_; ++i) {
                detected_ids.push_back(sensor_->hitList_[i].obj_->id_);
            }
        }

        if (!copies_ || !sensor_settings_.enabled) {
            for (auto* target : truth->object_) {
                if (target && target != object_) {
                    detected_ids.push_back(target->id_);
                }
            }
        }
        supplied_ids = detected_ids;
        if (markov_settings_.enabled) {
            filter_markov(*truth, scenario_engine_->getSimulationTime());
        }

        // ===== TARGET COPIES: detected targets, or all targets when sensor is off =====
        if (copies_) {
            for (auto* target : truth->object_) {
                if (!target || target == object_) {
                    continue;
                }

                if (std::find(supplied_ids.begin(), supplied_ids.end(), target->id_) == supplied_ids.end()) {
                    continue;
                }
                // Position assignment copies rel_pos_. Do not retain a link into
                // live scenario state, or treat unresolved relative coordinates as absolute.
                const auto type = target->pos_.GetType();
                if (target->pos_.GetRelativePosition() != nullptr ||
                    (type != roadmanager::Position::PositionType::NORMAL &&
                     type != roadmanager::Position::PositionType::ROUTE)) {
                    throw std::runtime_error("Persistent ACC cannot snapshot relative position for target " +
                                             std::to_string(target->id_));
                }

                // Copy native Position, including road/lane mapping; no world reprojection.
                auto* copy = new Object(target->type_);
                copy->id_ = target->id_;
                copy->category_ = target->category_;
                copy->pos_ = target->pos_;
                copy->boundingbox_ = target->boundingbox_;
                copy->SetSpeed(target->GetSpeed());
                observations_.object_.push_back(copy);
            }
            entities_ = &observations_;
        }

        // ===== 3. Execute upstream ACC on the real ego =====
        // The reference path leaves entities_ pointing to the real scenario.
        const double speed = object_->GetSpeed();

        try {
            ControllerACC::Step(dt);
        } catch (...) {
            // Restore the scenario view even if native ACC fails.
            entities_ = truth;
            throw;
        }

        // ===== 4. Record the executed step's diagnostics =====
        last_output = {
            scenario_engine_->getSimulationTime(),
            speed,
            object_->GetSpeed(),
            std::numeric_limits<double>::infinity(),
            -1,
            0
        };

        // Upstream exposes the selected object's position via its lookahead marker.
        for (auto* target : entities_->object_) {
            if (!target || target == object_) {
                continue;
            }

            ++last_output.target_count;
            if (std::abs(object_->lookahead_sensor_pos_[0] - target->pos_.GetX()) < 1e-7 &&
                std::abs(object_->lookahead_sensor_pos_[1] - target->pos_.GetY()) < 1e-7) {
                last_output.lead_id = target->id_;
                double lateral = 0;
                object_->FreeSpaceDistance(target, &lateral, &last_output.observed_gap_m);
            }
        }

        // ===== 5. Restore the real scenario view for the next step =====
        entities_ = truth;
        output_valid = true;
    }
};

// =================================================================
// Factories: the XOSC controller name chooses copies or reference targets
// =================================================================
Controller* persistent_acc(void* args) {
    return new PersistentAcc(static_cast<Controller::InitArgs*>(args), true);
}

Controller* reference_acc(void* args) {
    return new PersistentAcc(static_cast<Controller::InitArgs*>(args), false);
}

} // namespace

// =================================================================
// C interface used by esmini_adapter.cpp
// =================================================================
extern "C" ACCSIM_API void ACCSIM_RegisterPersistentAcc() {
    ScenarioReader::RegisterController("PerceptionACC", persistent_acc);
    ScenarioReader::RegisterController("ReferenceACC", reference_acc);
}

extern "C" ACCSIM_API int ACCSIM_GetPersistentAccOutput(ACCSIM_PersistentAccOutput* output) {
    if (!output || !output_valid) {
        return -1;
    }

    *output = last_output;
    return 0;
}

extern "C" ACCSIM_API int ACCSIM_HasViewer() {
#ifdef _USE_OSG
    return 1;
#else
    return 0;
#endif
}

extern "C" ACCSIM_API int ACCSIM_ConfigureSensor(const ACCSIM_SensorConfig* config) {
    if (!config) {
        return -1;
    }
    if (config->enabled &&
        (!std::isfinite(config->x_m) || !std::isfinite(config->y_m) ||
         !std::isfinite(config->z_m) || !std::isfinite(config->heading_rad) ||
         !std::isfinite(config->near_m) || !std::isfinite(config->far_m) ||
         !std::isfinite(config->horizontal_fov_rad) || config->near_m < 0 ||
         config->far_m <= config->near_m || config->horizontal_fov_rad <= 0 ||
         config->horizontal_fov_rad > 2 * 3.14159265358979323846)) {
        return -1;
    }
    sensor_config = *config;
    detected_ids.clear();
    return 0;
}

namespace {
int copy_ids(const std::vector<int>& source, int* ids, int capacity) {
    if (!output_valid) {
        return -1;
    }
    if (!ids && capacity == 0) {
        return static_cast<int>(source.size());
    }
    if (capacity < static_cast<int>(source.size()) || (!ids && !source.empty())) {
        return -1;
    }
    std::copy(source.begin(), source.end(), ids);
    return static_cast<int>(source.size());
}
}

extern "C" ACCSIM_API int ACCSIM_GetDetectedTargetIds(int* ids, int capacity) {
    return copy_ids(detected_ids, ids, capacity);
}

extern "C" ACCSIM_API int ACCSIM_GetDroppedTargetIds(int* ids, int capacity) {
    return copy_ids(dropped_ids, ids, capacity);
}

extern "C" ACCSIM_API int ACCSIM_GetSuppliedTargetIds(int* ids, int capacity) {
    return copy_ids(supplied_ids, ids, capacity);
}

extern "C" ACCSIM_API int ACCSIM_ConfigureMarkov(const ACCSIM_MarkovConfig* config) {
    if (!config) {
        return -1;
    }
    try {
        if (config->enabled) {
            accsim::markov_parameters(config->probability, config->mean_missing_s, config->period_s);
        }
    } catch (const std::exception&) {
        return -1;
    }
    markov_config = *config;
    return 0;
}

extern "C" ACCSIM_API int ACCSIM_GetAccInitialization(ACCSIM_AccInitialization* output) {
    if (!output || !initialization_valid) {
        return -1;
    }
    *output = initialization;
    return 0;
}
