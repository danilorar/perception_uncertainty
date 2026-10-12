// Author: Zhuo Ma
// Connects independent target observations to esmini's native ACC.
// It does not implement the ACC algorithm. It is compiled into esminiLib.
// ControllerACC::Step is the original esmini v3.8.1 implementation, not a port.
// One native controller stays attached to the real ego and sees independent target copies.
#define ACCSIM_BRIDGE_BUILD
#include "native_acc_bridge.h"
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
        : ControllerACC(args), copies_(copies), sensor_settings_(sensor_config) {
        output_valid = false;
    }

    ~PersistentAcc() override {
        output_valid = false;
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

        // ===== IDEAL SENSOR: measure visible targets at controller time =====
        if (copies_ && sensor_settings_.enabled) {
            ensure_sensor();
            sensor_->Update(); // Exactly one update per ACC step.
            for (int i = 0; i < sensor_->nObj_; ++i) {
                detected_ids.push_back(sensor_->hitList_[i].obj_->id_);
            }
        }

        // ===== TARGET COPIES: detected targets, or all targets when sensor is off =====
        if (copies_) {
            for (auto* target : truth->object_) {
                if (!target || target == object_) {
                    continue;
                }

                if (sensor_settings_.enabled &&
                    std::find(detected_ids.begin(), detected_ids.end(), target->id_) == detected_ids.end()) {
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
                if (!sensor_settings_.enabled) {
                    detected_ids.push_back(target->id_);
                }
            }
            entities_ = &observations_;
        }

        // ===== 3. Execute upstream ACC on the real ego =====
        // The reference path leaves entities_ pointing to the real scenario.
        // ===== REFERENCE MODE: use live targets without sensor filtering =====
        if (!copies_) {
            for (auto* target : truth->object_) {
                if (target && target != object_) {
                    detected_ids.push_back(target->id_);
                }
            }
        }
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

extern "C" ACCSIM_API int ACCSIM_GetDetectedTargetIds(int* ids, int capacity) {
    if (!output_valid || capacity < static_cast<int>(detected_ids.size()) ||
        (!ids && !detected_ids.empty())) {
        return -1;
    }
    std::copy(detected_ids.begin(), detected_ids.end(), ids);
    return static_cast<int>(detected_ids.size());
}
