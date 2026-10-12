// Author: Zhuo Ma
// Coordinate native ACC runs; esmini owns controller execution and ego movement.
#include "accsim/engine.hpp"
#include "accsim/metrics.hpp"
#include "native_acc_bridge.h"
#include <algorithm>
#include <cmath>
#include <chrono>
#include <thread>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <sstream>
#include <stdexcept>

namespace {
using namespace accsim;
namespace fs = std::filesystem;

// =================================================================
// =======================SIMULATION SETTINGS=======================
// =================================================================

struct Options {
    // Paths: scenario input, run output and optional esmini resources.
    fs::path source;
    fs::path output;
    fs::path resources;

    // Simulation: controller selection and the common simulation clock.
    std::string controller = "acc";
    double dt = 0.01;
    double duration = 10;
    bool stop_on_collision = true;

    // ACC: -1 uses the scenario's initial speed.
    double time_gap = 1.5;
    double set_speed = -1;
    bool native_reference = false;

    // Optional esmini ObjectSensor; disabled keeps the all-target baseline.
    ACCSIM_SensorConfig sensor{0, 0, 0, 0.5, 0, 0, 100, pi / 3};

    // Uncertainty is not connected; only zero dropout is accepted.
    double dropout_p = 0;

    // Output: enable the additional per-step audit CSVs.
    bool detailed_logs = false;
    bool viewer = false;
};

// =================================================================
// ======================ARGUMENT VALUE HELPERS=====================
// =================================================================

std::string trim(const std::string& text) {
    const auto first = text.find_first_not_of(" \t\r\n");
    const auto last = text.find_last_not_of(" \t\r\n");
    if (first == std::string::npos) {
        return "";
    }
    return text.substr(first, last - first + 1);
}

double number(const std::string& text) {
    std::size_t used = 0;
    const double value = std::stod(text, &used);
    if (used != text.size() || !std::isfinite(value)) {
        throw std::invalid_argument("Expected finite numeric value: " + text);
    }
    return value;
}

void assign_number(const std::map<std::string, std::string>& values, const std::string& key, double& target) {
    if (values.count(key)) {
        target = number(values.at(key));
    }
}

void assign_boolean(const std::map<std::string, std::string>& values, const std::string& key, bool& target) {
    if (values.count(key)) {
        const auto& value = values.at(key);
        if (value != "true" && value != "false") {
            throw std::invalid_argument(key + " must be true or false");
        }
        target = value == "true";
    }
}

// =================================================================
// =========================READ CONFIG FILE========================
// =================================================================

void read_config(int argc, char** argv, std::map<std::string, std::string>& values) {
    // Locate the config first, so command-line values can override it later.
    fs::path config;
    for (int i = 1; i < argc; ++i) {
        if (std::string(argv[i]) == "--config") {
            if (i + 1 >= argc) {
                throw std::invalid_argument("--config needs a path");
            }
            ++i;
            config = fs::absolute(argv[i]);
        }
    }
    if (config.empty()) {
        return;
    }

    std::ifstream file(config);
    if (!file) {
        throw std::runtime_error("Cannot read config: " + config.string());
    }
    std::string line;
    while (std::getline(file, line)) {
        line = trim(line);
        if (line.empty() || line[0] == '#' || line[0] == ';') {
            continue;
        }
        const auto equals = line.find('=');
        if (equals == std::string::npos) {
            throw std::runtime_error("Config requires key=value");
        }
        const auto key = trim(line.substr(0, equals));
        auto value = trim(line.substr(equals + 1));
        if (key == "scenario" || key == "output-dir" || key == "esmini-home") {
            const fs::path path(value);
            if (path.is_relative()) {
                value = (config.parent_path() / path).lexically_normal().string();
            }
        }
        values[key] = value;
    }
}

// =================================================================
// ========================READ COMMAND LINE========================
// =================================================================

void read_command_line(int argc, char** argv, std::map<std::string, std::string>& values) {
    for (int i = 1; i < argc; ++i) {
        const std::string key = argv[i];
        if (key == "--help") {
            std::cout
                << "acc_sim --scenario PATH --output-dir NEW_DIRECTORY [--config FILE]\n"
                << "  --dt .01 --duration 10 --time-gap 1.5 --set-speed MPS\n"
                << "  --esmini-home RESOURCES --controller acc --dropout-p 0\n"
                << "  --native-reference true|false (default false: independent target copies)\n"
                << "  --stop-on-collision true|false --detailed-logs true|false\n"
                << "  --viewer true|false (default false; requires graphics build)\n"
                << "  --ideal-sensor true|false (default false: all targets)\n"
                << "  --sensor-x-m 0 --sensor-y-m 0 --sensor-z-m .5 --sensor-heading-rad 0\n"
                << "  --sensor-near-m 0 --sensor-far-m 100 --sensor-horizontal-fov-rad 1.0471975512\n"
                << "ACC-only simulation; uncertainty is not connected yet.\n";
            std::exit(0);
        }
        if (key.rfind("--", 0) != 0 || i + 1 >= argc) {
            throw std::invalid_argument("Expected --option value: " + key);
        }
        ++i;
        const std::string value = argv[i];
        if (key != "--config") {
            values[key.substr(2)] = value;
        }
    }
}

// =================================================================
// =========================POPULATE OPTIONS========================
// =================================================================

// Assign values from the config and command line to the Options struct
Options populate_options(const std::map<std::string, std::string>& values) {
    const std::vector<std::string> known = {
        "scenario", "output-dir", "esmini-home", "dt", "duration", "dropout-p",
        "time-gap", "set-speed", "controller", "stop-on-collision", "detailed-logs", "native-reference", "viewer",
        "ideal-sensor", "sensor-x-m", "sensor-y-m", "sensor-z-m", "sensor-heading-rad",
        "sensor-near-m", "sensor-far-m", "sensor-horizontal-fov-rad"};
    for (const auto& entry : values) {
        if (std::find(known.begin(), known.end(), entry.first) == known.end()) {
            throw std::invalid_argument("Unknown option: " + entry.first);
        }
    }
    if (!values.count("scenario") || !values.count("output-dir")) {
        throw std::invalid_argument("--scenario and --output-dir are required");
    }

    Options options;
    options.source = fs::absolute(values.at("scenario")).lexically_normal();
    options.output = fs::absolute(values.at("output-dir")).lexically_normal();
    if (values.count("esmini-home")) {
        options.resources = fs::absolute(values.at("esmini-home"));
    } else if (const char* environment = std::getenv("ESMINI_RESOURCE_HOME")) {
        options.resources = fs::absolute(environment);
    }
    if (values.count("controller")) {
        options.controller = values.at("controller");
    }
    if (options.controller != "acc") {
        throw std::invalid_argument("Only controller acc is supported");
    }

    assign_number(values, "dt", options.dt);
    assign_number(values, "duration", options.duration);
    assign_number(values, "dropout-p", options.dropout_p);
    assign_number(values, "time-gap", options.time_gap);
    assign_number(values, "set-speed", options.set_speed);
    assign_boolean(values, "stop-on-collision", options.stop_on_collision);
    assign_boolean(values, "detailed-logs", options.detailed_logs);
    assign_boolean(values, "native-reference", options.native_reference);
    assign_boolean(values, "viewer", options.viewer);
    bool ideal_sensor = false;
    assign_boolean(values, "ideal-sensor", ideal_sensor);
    options.sensor.enabled = ideal_sensor;
    assign_number(values, "sensor-x-m", options.sensor.x_m);
    assign_number(values, "sensor-y-m", options.sensor.y_m);
    assign_number(values, "sensor-z-m", options.sensor.z_m);
    assign_number(values, "sensor-heading-rad", options.sensor.heading_rad);
    assign_number(values, "sensor-near-m", options.sensor.near_m);
    assign_number(values, "sensor-far-m", options.sensor.far_m);
    assign_number(values, "sensor-horizontal-fov-rad", options.sensor.horizontal_fov_rad);
    return options;
}

// =================================================================
// =========================VALIDATE OPTIONS========================
// =================================================================

// Check that the options are valid for the selected controller and simulation parameters
void validate_options(const Options& options) {
    if (options.sensor.enabled && options.native_reference) {
        throw std::invalid_argument("ideal-sensor and native-reference cannot both be true");
    }
    if (options.sensor.enabled &&
        (options.sensor.near_m < 0 || options.sensor.far_m <= options.sensor.near_m ||
         options.sensor.horizontal_fov_rad <= 0 || options.sensor.horizontal_fov_rad > 2 * pi)) {
        throw std::invalid_argument("Sensor requires 0 <= near < far and 0 < horizontal FOV <= 2*pi");
    }
    if (options.dropout_p != 0) {
        throw std::invalid_argument(
            "Persistent ACC currently requires dropout-p 0; uncertainty is not connected yet");
    }
    if (options.dt <= 0 || options.dt > 0.1 || options.duration <= 0 || options.time_gap <= 0 ||
        options.set_speed < -1) {
        throw std::invalid_argument("Invalid simulation parameters");
    }
    if (std::abs(options.duration / options.dt - std::round(options.duration / options.dt)) > 1e-8) {
        throw std::invalid_argument("duration must be an integer multiple of dt");
    }
    if (!fs::is_regular_file(options.source)) {
        throw std::runtime_error("Original scenario not found");
    }
    if (fs::exists(options.output) && !fs::is_empty(options.output)) {
        throw std::runtime_error("Output directory is not empty; use a NEW run directory");
    }
}

// =================================================================
// =====================ARGUMENT PARSING STAGES=====================
// =================================================================

// Main entry point: parse options/argument and run the selected controller simulation.
Options parse(int argc, char** argv) {
    std::map<std::string, std::string> values;

    // 1. Read config (paths are relative to the config file).
    read_config(argc, argv, values);

    // 2. Read command line (these values override the config).
    read_command_line(argc, argv, values);

    // 3. Populate options, retaining defaults for omitted settings.
    Options options = populate_options(values);

    // 4. Validate settings for the selected controller.
    validate_options(options);
    return options;
}

// =================================================================
// ================OUTPUT FORMATTING AND FILE HELPERS===============
// =================================================================

std::string json(const std::string& text) {
    std::ostringstream s;
    s << '"';
    for (unsigned char c : text) {
        switch (c) {
        case '"':
            s << "\\\"";
            break;
        case '\\':
            s << "\\\\";
            break;
        case '\n':
            s << "\\n";
            break;
        case '\r':
            s << "\\r";
            break;
        case '\t':
            s << "\\t";
            break;
        default:
            if (c < 32) {
                s << "\\u" << std::hex << std::setw(4) << std::setfill('0') << static_cast<int>(c)
                  << std::dec;
            } else {
                s << static_cast<char>(c);
            }
        }
    }
    return s.str() + '"';
}

std::string numeric(double value) {
    if (!std::isfinite(value)) {
        return "null";
    }
    std::ostringstream s;
    s << std::setprecision(17) << value;
    return s.str();
}

std::string event_numeric(double value) {
    return value < 0 ? "null" : numeric(value);
}

std::ofstream open(const fs::path& path) {
    std::ofstream file(path, std::ios::binary);
    if (!file) {
        throw std::runtime_error("Cannot write " + path.string());
    }
    file.exceptions(std::ios::badbit | std::ios::failbit);
    file << std::setprecision(17);
    return file;
}

std::string fingerprint(const fs::path& path) {
    std::ifstream in(path, std::ios::binary);
    if (!in) {
        throw std::runtime_error("Cannot fingerprint input");
    }
    std::uint64_t hash = UINT64_C(14695981039346656037);
    char c;
    while (in.get(c)) {
        hash ^= static_cast<unsigned char>(c);
        hash *= UINT64_C(1099511628211);
    }
    std::ostringstream s;
    s << std::hex << std::setw(16) << std::setfill('0') << hash;
    return s.str();
}

// ===================================================================
// ===================== WRITE RUN CONFIGURATION =====================
// ====================================================================

void write_config(const Options& o, const PreparedScenario& scene, const AccConfig& acc) {
    auto file = open(o.output / "run_config.json");
    file << "{\n  \"schema_version\": 5,\n"
         << "  \"controller\": \"esmini::ControllerACC::Step\",\n"
         << "  \"esmini_tag\": \"v3.8.1\",\n"
         << "  \"esmini_commit\": \"19d26b68f7f473de4e55046a794b5fd2f91c58c8\",\n"
         << "  \"native_reference\": " << (o.native_reference ? "true" : "false") << ",\n"
         << "  \"acc_observation_source\": "
         << json(o.native_reference ? "live native entities" :
                 (o.sensor.enabled ? "ObjectSensor detections copied in scenario order" :
                                     "native-position copies in scenario order")) << ",\n"
         << "  \"ideal_sensor\": " << (o.sensor.enabled ? "true" : "false") << ",\n"
         << "  \"sensor_x_m\": " << o.sensor.x_m << ",\n"
         << "  \"sensor_y_m\": " << o.sensor.y_m << ",\n"
         << "  \"sensor_z_m\": " << o.sensor.z_m << ",\n"
         << "  \"sensor_heading_rad\": " << o.sensor.heading_rad << ",\n"
         << "  \"sensor_near_m\": " << o.sensor.near_m << ",\n"
         << "  \"sensor_far_m\": " << o.sensor.far_m << ",\n"
         << "  \"sensor_horizontal_fov_rad\": " << o.sensor.horizontal_fov_rad << ",\n"
         << "  \"sensor_update_period_s\": " << o.dt << ",\n"
         << "  \"source_scenario\": " << json(scene.source.string()) << ",\n"
         << "  \"generated_scenario\": " << json(scene.generated.string()) << ",\n"
         << "  \"source_fnv1a64\": " << json(fingerprint(scene.source)) << ",\n"
         << "  \"road_fnv1a64\": " << json(fingerprint(scene.road)) << ",\n"
         << "  \"dt_s\": " << o.dt << ",\n"
         << "  \"duration_s\": " << o.duration << ",\n"
         << "  \"time_gap_s\": " << acc.time_gap_s << ",\n"
         << "  \"set_speed_mps\": " << acc.set_speed_mps << ",\n"
         << "  \"initial_ego_speed_mps\": " << scene.reference.initial_speed() << ",\n"
         << "  \"max_acceleration_mps2\": " << acc.max_acceleration << ",\n"
         << "  \"max_deceleration_mps2\": " << acc.max_deceleration << ",\n"
         << "  \"stop_on_collision\": " << (o.stop_on_collision ? "true" : "false") << ",\n"
         << "  \"viewer\": " << (o.viewer ? "true" : "false") << ",\n"
         << "  \"detailed_logs\": " << (o.detailed_logs ? "true" : "false") << "\n}\n";
}

// =================================================================
// ========================== TRUTH LOGGING ========================
// =================================================================

void write_truth(std::ofstream& file, double time, const ObjectState& s, const char* role) {
    file << time << ',' << s.id << ',' << role << ',' << s.pose.x << ',' << s.pose.y << ',' << s.pose.z << ','
         << s.pose.h << ',' << s.speed << ',' << s.box.length << ',' << s.box.width << ',' << s.box.height
         << ',' << s.box.cx << ',' << s.box.cy << ',' << s.box.cz << ',' << s.object_type << ','
         << s.object_category << ',' << s.velocity.x << ',' << s.velocity.y << ',' << s.velocity.z << ','
         << s.acceleration.x << ',' << s.acceleration.y << ',' << s.acceleration.z << '\n';
}

// CSVs contain only active ACC data; native diagnostics are written separately.
struct RunLogs {
    bool detailed = false;
    std::ofstream truth;
    std::ofstream metrics;
    std::ofstream results;
};

RunLogs open_run_logs(const fs::path& output, bool detailed) {
    RunLogs logs;
    logs.detailed = detailed;
    if (detailed) {
        logs.truth = open(output / "truth.csv");
        logs.metrics = open(output / "metrics.csv");
        logs.truth << "time_s,object_id,role,x_m,y_m,z_m,h_rad,speed_mps,length_m,width_m,height_m,center_x_"
                      "m,center_y_m,center_z_m,object_type,object_category,vx_mps,vy_mps,vz_mps,ax_mps2,ay_"
                      "mps2,az_mps2\n";
        logs.metrics << "time_s,collision,engine_collision,minimum_distance_m,minimum_gap_m,ttc_s\n";
    }
    logs.results = open(output / "results.csv");
    logs.results << "step,time_s,ego_x_m,ego_y_m,ego_heading_rad,ego_speed_mps,ego_progress_m,"
                    "truth_min_gap_m,truth_min_gap_target_id,truth_min_ttc_s,truth_min_distance_m,collision\n";
    return logs;
}

void write_metrics(std::ofstream& file, double time, const StepMetrics& metrics, bool engine_collision) {
    file << time << ',' << metrics.collision << ',' << engine_collision << ',' << metrics.minimum_distance_m
         << ',' << metrics.gap_m << ',' << metrics.ttc_s << '\n';
}

// Writes the value, or an empty cell when it is unavailable (non-finite).
void write_finite(std::ofstream& file, double value) {
    if (std::isfinite(value)) {
        file << value;
    }
}

// Boundary truth only; controller-time diagnostics belong to native_acc.csv.
void write_result(std::ofstream& file, std::uint64_t step, const WorldTruthFrame& world,
                  double ego_progress_m, const StepMetrics& metrics) {
    const auto& ego = world.ego;
    file << step << ',' << world.time_s << ',' << ego.pose.x << ',' << ego.pose.y << ',' << ego.pose.h << ','
         << ego.speed << ',' << ego_progress_m << ',';
    write_finite(file, metrics.gap_m);
    file << ',';
    if (metrics.target_id >= 0) {
        file << metrics.target_id;
    }
    file << ',';
    write_finite(file, metrics.ttc_s);
    file << ',';
    write_finite(file, metrics.minimum_distance_m);
    file << ',' << metrics.collision << '\n';
}

// ===================================================================
// ======================= WRITE FINAL SUMMARY =======================
// ===================================================================

void write_summary(const fs::path& output, const RunSummary& summary,
                   const std::string& end_reason, double end_time, double ego_final_speed) {
    auto file = open(output / "summary.json");
    file << "{\n  \"complete\": true,\n  \"collision\": " << (summary.collision ? "true" : "false") << ",\n"
         << "  \"end_reason\": " << json(end_reason) << ",\n  \"duration_s\": " << end_time << ",\n"
         << "  \"first_collision_s\": " << event_numeric(summary.first_collision_s) << ",\n"
         << "  \"collision_ego_speed_mps\": " << event_numeric(summary.collision_ego_speed_mps) << ",\n"
         << "  \"collision_relative_speed_mps\": " << event_numeric(summary.collision_relative_speed_mps)
         << ",\n"
         << "  \"minimum_distance_m\": " << numeric(summary.minimum_distance_m) << ",\n"
         << "  \"minimum_gap_m\": " << numeric(summary.minimum_gap_m) << ",\n"
         << "  \"minimum_ttc_s\": " << numeric(summary.minimum_ttc_s) << ",\n"
         << "  \"first_deceleration_s\": " << event_numeric(summary.first_deceleration_s) << ",\n"
         << "  \"minimum_acceleration_mps2\": " << summary.minimum_acceleration_mps2 << ",\n"
         << "  \"ego_final_speed_mps\": " << ego_final_speed << ",\n"
         << "  \"close_gap_stop_requested\": " << (summary.close_gap_stop_requested ? "true" : "false")
         << "\n}\n";
}

// =================================================================
// ======================== SIMULATION LOOP ========================
// =================================================================

// Coordinate one run: prepare inputs, initialize esmini, step the world and save results.
// Return zero on success; failures throw an exception handled by main().
int run(const Options& options) {

    // ===== 1. Initialization =====================================================
    // Create the directory selected by --output-dir.
    fs::create_directories(options.output);
    auto scenario_id = options.source.stem().string();
    const std::string source_prefix = "C_original_";
    if (scenario_id.rfind(source_prefix, 0) == 0) {
        scenario_id.erase(0, source_prefix.size());
    }

    AccConfig acc_config;
    acc_config.time_gap_s = options.time_gap;
    acc_config.set_speed_mps = options.set_speed; // -1 resolves to trajectory initial speed.

    // Write a generated XOSC with the selected ego controller; preserve the source files.
    const auto scene = prepare_scenario(
        options.source,
        options.output / "generated" / ("C_" + options.controller + "_" + scenario_id + ".xosc"), "object_1",
        acc_config, options.native_reference);
    const auto original_hash = fingerprint(scene.source);
    const auto road_hash = fingerprint(scene.road);
    if (options.duration > scene.duration_s + 1e-8) {
        throw std::invalid_argument("duration exceeds original scenario stop time");
    }

    // Resolve the settings used by native ACC.
    acc_config.set_speed_mps = options.set_speed < 0 ? scene.reference.initial_speed() : options.set_speed;
    acc_config.max_acceleration = scene.max_acceleration;
    acc_config.max_deceleration = scene.max_deceleration;
    // Save the resolved settings before starting the engine.
    write_config(options, scene, acc_config);

    // Constructing engine calls SE_InitWithArgs inside esmini_adapter.cpp.
    // This loads the generated XOSC and sets up esmini recording (sim.dat and run.log).
    EsminiAdapter engine(
        scene,
        options.output,
        options.resources,
        options.dt,
        options.viewer,
        options.sensor);

    // Open ACC diagnostics separately from the boundary-truth results.
    auto native_log = open(options.output / "native_acc.csv");
    native_log << "time_s,input_speed_mps,ego_speed_mps,ego_x_m,ego_y_m,ego_z_m,lead_id,target_count,observed_gap_m\n";

    // One row per executed ACC step, including an empty list when nothing is detected.
    auto detection_log = open(options.output / "detections.csv");
    detection_log << "step,controller_time_s,observation_mode,detected_target_ids\n";

    // Open results.csv and, when requested, the detailed audit CSVs.
    auto logs = open_run_logs(options.output, options.detailed_logs);
    RunSummary summary;
    std::uint64_t step = 0;
    double end_time = 0;
    std::string end_reason = "duration";

    const auto wall_start = std::chrono::steady_clock::now();

    // ===== 2. Simulation loop (one iteration per dt) =============================
    while (true) {
        // --- 2a. World truth: exact ego feedback, truth/metrics logs, collision check ---
        // Snapshot the actual world at the start of this iteration, before advancement.
        const auto world = engine.truth();
        // Measure current progress before any terminal row can be written.
        const double ego_progress_m = scene.reference.project(world.ego.pose).first;
        end_time = world.time_s;
        if (std::abs(world.time_s - static_cast<double>(step) * options.dt) > 1e-7) {
            throw std::runtime_error("Simulation clock lost synchronization");
        }

        // Retain exact ego feedback and optionally record the current truth states.
        if (logs.detailed) {
            write_truth(logs.truth, world.time_s, world.ego, "ego");
            for (const auto& target : world.targets) {
                write_truth(logs.truth, world.time_s, target, "target");
            }
        }

        // Calculate gap, time to collision (TTC) and box collisions from truth.
        // These safety measurements do not become target observations for ACC.
        const auto metrics = evaluate_truth(world);
        const bool engine_collision = engine.collision();
        if (logs.detailed) {
            write_metrics(logs.metrics, world.time_s, metrics, engine_collision);
        }
        if (metrics.collision != engine_collision) {
            throw std::runtime_error("Native collision flag disagrees with truth-box evaluation");
        }

        summary.minimum_distance_m = std::min(summary.minimum_distance_m, metrics.minimum_distance_m);
        summary.minimum_gap_m = std::min(summary.minimum_gap_m, metrics.gap_m);
        summary.minimum_ttc_s = std::min(summary.minimum_ttc_s, metrics.ttc_s);

        // --- 2b. Stop conditions: first collision (only if stop_on_collision), duration, scenario end ---
        // A terminal results row records truth at this time, without a new sensor frame or command.
        const auto write_terminal_result = [&] {
            write_result(logs.results, step, world, ego_progress_m, metrics);
        };
        if (metrics.collision && !summary.collision) {
            summary.collision = true;
            summary.first_collision_s = world.time_s;
            summary.collision_ego_speed_mps = world.ego.speed;
            for (const auto& target : world.targets) {
                // Record the relative speed of the vehicles involved in the collision.
                if (geometry(world.ego, target).collision) {
                    summary.collision_relative_speed_mps =
                        std::abs(geometry(world.ego, target).closing_speed_mps);
                }
            }
            // Record every first collision; stop here only when requested.
            if (options.stop_on_collision) {
                end_reason = "collision";
                write_terminal_result();
                break;
            }
        }
        // Finish at the requested duration, allowing for floating-point clock error.
        if (world.time_s >= options.duration - 1e-8) {
            write_terminal_result();
            break;
        }
        // esmini can also end the run through the scenario's own stopping conditions.
        if (engine.quit()) {
            end_reason = "scenario_stop";
            write_terminal_result();
            break;
        }

        // Advance one native step: scenario actions, target snapshots, ACC and ego motion.
        // No custom vehicle integration is performed on this path.
        engine.advance_native(options.dt);

        // Retrieve the controller diagnostics captured during that native step.
        ACCSIM_PersistentAccOutput native{};
        if (ACCSIM_GetPersistentAccOutput(&native) != 0) {
            throw std::runtime_error("Missing native ACC diagnostics");
        }

        std::vector<int> detected_ids(static_cast<std::size_t>(native.target_count));
        const int detected_count = ACCSIM_GetDetectedTargetIds(
            detected_ids.data(), static_cast<int>(detected_ids.size()));
        if (detected_count < 0) {
            throw std::runtime_error("Missing detected target IDs");
        }
        detection_log << step << ',' << native.time_s << ','
                      << (options.native_reference ? "reference" :
                          (options.sensor.enabled ? "ideal" : "all-targets")) << ",\"[";
        for (int i = 0; i < detected_count; ++i) {
            if (i > 0) {
                detection_log << ',';
            }
            detection_log << detected_ids[i];
        }
        detection_log << "]\"\n";

        // Read post-step truth and save it beside the ACC diagnostics.
        const auto next = engine.truth();
        native_log << native.time_s << ',' << native.input_speed_mps << ',' << next.ego.speed << ','
                   << next.ego.pose.x << ',' << next.ego.pose.y << ',' << next.ego.pose.z << ','
                   << native.lead_id << ',' << native.target_count << ',' << native.observed_gap_m
                   << '\n';

        // Save the PRE-step world snapshot to results.csv. The post-step state and
        // internal controller diagnostics above belong to native_acc.csv.
        write_result(logs.results, step, world, ego_progress_m, metrics);

        // Derive acceleration for reporting only; native ACC has already moved ego.
        const double acceleration = (native.desired_speed_mps - native.input_speed_mps) / options.dt;
        if (acceleration < -.01 && summary.first_deceleration_s < 0) {
            summary.first_deceleration_s = native.time_s;
        }
        summary.minimum_acceleration_mps2 = std::min(summary.minimum_acceleration_mps2, acceleration);
        summary.close_gap_stop_requested =
            summary.close_gap_stop_requested ||
            (native.lead_id >= 0 && native.input_speed_mps > 0 && native.desired_speed_mps == 0);

        // Begin the next native simulation step.
        ++step;
        if (options.viewer) {
            // Pace the viewer in wall time without changing the fixed simulation timestep.
            std::this_thread::sleep_until(wall_start + std::chrono::duration<double>(step * options.dt));
        }
    }

    // ===== 3. Results ============================================================
    if (fingerprint(scene.source) != original_hash || fingerprint(scene.road) != road_hash) {
        throw std::runtime_error("Original data changed during run");
    }
    // After the loop stops, write summary.json using the final actual ego speed.
    write_summary(options.output, summary, end_reason, end_time, engine.truth().ego.speed);
    std::cout << "Completed: " << options.output << "\ncollision=" << summary.collision
              << ", min_gap=" << summary.minimum_gap_m << " m, min_TTC=" << summary.minimum_ttc_s << " s\n";
    return 0;
}
} // namespace

// =================================================================
// =======================PROGRAM ENTRY POINT=======================
// =================================================================

int main(int argc, char** argv) {
    try {
        return run(parse(argc, argv));
    } catch (const std::exception& error) {
        std::cerr << "acc_sim: " << error.what() << '\n';
        return 1;
    }
}

/*
- When launching e.g:
    ./build/runner/acc_sim --controller acc --dt 0.01 ...

- The operating system supplies those words through argc and argv. The call:
    run(parse(argc,argv));

- First turns them into an *Options* object, then starts the sim loop with those settings.
*/

/*
- Inside parse(), there are 4 steps:
    1. Read_config(): reads an optional config file (e.g acc_sim.conf)
    2. Read_command_line(): read command line and override any config values.
    3. Populate_options(): populate an Options object with the final values, using defaults for any missing
settings.
    4. Validate_options(): validate the options to ensure they are consistent and valid for the selected
controller.
*/