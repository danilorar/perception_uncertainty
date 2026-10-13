// Author: Zhuo Ma
// Exercise the production persistent ACC through XOSC initialization and SE_StepDT.
// Exceptions keep every check active in Release builds.
#include "accsim/engine.hpp"
#include "native_acc_bridge.h"
#include "pugixml.hpp"
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {
constexpr double dt = 0.01;
void require(bool condition, const std::string& message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}
void near(double actual, double expected, const std::string& message, double tolerance = 1e-8) {
    require(std::isfinite(actual) && std::abs(actual - expected) <= tolerance,
            message + ": expected " + std::to_string(expected) + ", got " + std::to_string(actual));
}
struct Target {
    double distance;
    double speed;
    double lateral = 0;
    double center = 0;
    double heading = 0;
};

// A straight road makes native longitudinal distance and movement independently
// predictable. Targets use ordinary scenario motion; only ego has a controller.
accsim::PreparedScenario fixture(const std::filesystem::path& directory, double speed,
                                double set_speed, const std::vector<Target>& targets,
                                double ego_center = 0) {
    std::filesystem::create_directories(directory);
    accsim::PreparedScenario scene;
    scene.source = directory / "fixture.xosc";
    scene.generated = scene.source;
    scene.road = directory / "straight.xodr";
    std::ofstream road(scene.road);
    road << R"(<OpenDRIVE><header revMajor="1" revMinor="4" name="ACC test" version="1" date="" north="0" south="0" east="0" west="0"/>
<road name="straight" length="1000" id="1" junction="-1"><planView><geometry s="0" x="0" y="0" hdg="0" length="1000"><line/></geometry></planView>
<lanes><laneSection s="0"><center><lane id="0" type="none" level="false"/></center><right><lane id="-1" type="driving" level="false"><width sOffset="0" a="3.5" b="0" c="0" d="0"/></lane></right></laneSection></lanes></road></OpenDRIVE>)";
    road.close();
    std::ofstream file(scene.generated);
    file << std::setprecision(17);
    file << R"(<OpenSCENARIO><FileHeader revMajor="1" revMinor="1" date="2026-01-01T00:00:00" description="Persistent ACC tests" author="accsim"/>
<ParameterDeclarations/><CatalogLocations/><RoadNetwork><LogicFile filepath="straight.xodr"/></RoadNetwork><Entities>)";
    for (std::size_t i = 0; i <= targets.size(); ++i) {
        const double center = i == 0 ? ego_center : targets[i - 1].center;
        file << "<ScenarioObject name=\"object_" << i + 1 << "\"><Vehicle name=\"car\" vehicleCategory=\"car\">"
             << "<BoundingBox><Center x=\"" << center << "\" y=\"0\" z=\"0\"/>"
             << "<Dimensions width=\"1.8\" length=\"5\" height=\"1.6\"/></BoundingBox>"
             << "<Performance maxSpeed=\"70\" maxAcceleration=\"10\" maxDeceleration=\"10\"/>"
             << "<Axles><FrontAxle maxSteering=\"0.5\" wheelDiameter=\"0.6\" trackWidth=\"1.6\" positionX=\"2\" positionZ=\"0.3\"/>"
             << "<RearAxle maxSteering=\"0\" wheelDiameter=\"0.6\" trackWidth=\"1.6\" positionX=\"0\" positionZ=\"0.3\"/></Axles><Properties/></Vehicle>";
        if (i == 0) {
            file << "<ObjectController><Controller name=\"testACC\"><Properties>"
                 << "<Property name=\"esminiController\" value=\"PerceptionACC\"/>"
                 << "<Property name=\"mode\" value=\"override\"/>"
                 << "<Property name=\"timeGap\" value=\"1.5\"/>"
                 << "<Property name=\"setSpeed\" value=\"" << set_speed << "\"/>"
                 << "</Properties></Controller></ObjectController>";
        }
        file << "</ScenarioObject>";
    }
    file << "</Entities><Storyboard><Init><Actions>";
    for (std::size_t i = 0; i <= targets.size(); ++i) {
        const Target target = i == 0 ? Target{0, speed} : targets[i - 1];
        file << "<Private entityRef=\"object_" << i + 1 << "\">"
             << "<PrivateAction><TeleportAction><Position><WorldPosition x=\"" << 100 + target.distance
             << "\" y=\"" << -1.75 + target.lateral << "\" z=\"0\" h=\"" << target.heading
             << "\"/></Position></TeleportAction></PrivateAction>"
             << "<PrivateAction><LongitudinalAction><SpeedAction><SpeedActionDynamics dynamicsShape=\"step\" value=\"0\" dynamicsDimension=\"time\"/>"
             << "<SpeedActionTarget><AbsoluteTargetSpeed value=\"" << target.speed
             << "\"/></SpeedActionTarget></SpeedAction></LongitudinalAction></PrivateAction>";
        if (i == 0) {
            file << "<PrivateAction><ControllerAction><ActivateControllerAction longitudinal=\"true\" lateral=\"false\"/>"
                 << "</ControllerAction></PrivateAction>";
        }
        file << "</Private>";
    }
    file << "</Actions></Init><StopTrigger/></Storyboard></OpenSCENARIO>";
    return scene;
}

ACCSIM_PersistentAccOutput advance(accsim::EsminiAdapter& engine) {
    const auto before = engine.truth();
    engine.advance_native(dt);
    ACCSIM_PersistentAccOutput output{};
    require(ACCSIM_GetPersistentAccOutput(&output) == 0, "Persistent diagnostics available");
    const auto after = engine.truth();
    near(output.input_speed_mps, before.ego.speed, "Real ego feedback");
    near(after.ego.speed, output.desired_speed_mps, "Native speed applied to real ego");
    near(after.ego.pose.x - before.ego.pose.x, after.ego.speed * dt, "Native road movement");
    near(after.ego.pose.y, before.ego.pose.y, "Native lane position retained");
    near(after.time_s - before.time_s, dt, "Exactly one simulation timestep");
    return output;
}

void check_action_scenario(const std::filesystem::path& source, const std::filesystem::path& root) {
    std::vector<accsim::WorldTruthFrame> baseline;
    std::vector<int> leads;
    for (int mode = 0; mode < 3; ++mode) {
        const auto directory = root / ("actions_" + std::to_string(mode));
        const auto scene = accsim::prepare_scenario(source, directory / "generated.xosc", "Ego",
                                                    accsim::AccConfig{1.5, -1}, mode == 2);
        require(scene.reference.vertices.empty(), "Action scenario must not invent a reference trajectory");
        const ACCSIM_SensorConfig sensor{mode == 1, 0, 0, .5, 0, 0, 100, accsim::pi};
        accsim::EsminiAdapter engine(scene, directory, {}, dt, false, sensor);
        ACCSIM_AccInitialization init{};
        require(ACCSIM_GetAccInitialization(&init) == 0, "Native initialization available");
        near(init.initial_speed_mps, 20.0 / 3.6, "Parameter-derived ego speed", 1e-6);
        near(init.set_speed_mps, init.initial_speed_mps, "Native cruise captures initialized speed");
        near(init.max_acceleration, 5, "Catalog vehicle acceleration limit");
        near(engine.truth().time_s, 0, "No initialization timestep");
        near(engine.truth().targets[0].speed, 20.0 / 3.6, "Target initial speed", 1e-6);
        for (int step = 0; step < 500; ++step) {
            engine.advance_native(dt);
            const auto world = engine.truth();
            ACCSIM_PersistentAccOutput output{};
            require(ACCSIM_GetPersistentAccOutput(&output) == 0, "Action ACC diagnostics");
            if (mode == 0) {
                baseline.push_back(world);
                leads.push_back(output.lead_id);
            } else {
                near(world.ego.speed, baseline[step].ego.speed, "Action mode speed identity", 0);
                near(world.ego.pose.x, baseline[step].ego.pose.x, "Action mode position identity", 0);
                near(world.targets[0].speed, baseline[step].targets[0].speed, "Target action preserved", 0);
                require(output.lead_id == leads[step], "Action target selection identity");
            }
        }
        require(engine.truth().targets[0].speed < 3, "Native delayed target braking still executes");
    }
    // The same base XOSC supports moving, stationary and braking target setups.
    for (const std::string setup : {"CCRm", "CCRs", "CCRb"}) {
        accsim::ScenarioOverrides overrides;
        overrides.set_ego_speed = true;
        overrides.ego_speed_kph = 50;
        overrides.set_target_speed = true;
        overrides.target_speed_kph = setup == "CCRs" ? 0 : (setup == "CCRb" ? 50 : 20);
        overrides.set_target_braking = true;
        overrides.target_braking = setup == "CCRb";
        baseline.clear();
        leads.clear();
        for (int mode = 0; mode < 3; ++mode) {
            const auto directory = root / (setup + "_" + std::to_string(mode));
            const auto scene = accsim::prepare_scenario(source, directory / "generated.xosc", "Ego",
                                                        accsim::AccConfig{1.5, -1}, mode == 2);
            const ACCSIM_SensorConfig sensor{mode == 1, 0, 0, .5, 0, 0, 100, accsim::pi};
            accsim::EsminiAdapter engine(scene, directory, {}, dt, false, sensor, overrides);
            ACCSIM_AccInitialization init{};
            require(ACCSIM_GetAccInitialization(&init) == 0, "Overridden initialization available");
            near(init.initial_speed_mps, 50.0 / 3.6, "Derived ego speed is re-evaluated", 1e-6);
            near(init.set_speed_mps, init.initial_speed_mps, "ACC captures overridden initial speed");
            near(engine.truth().time_s, 0, "Overrides do not advance time");
            near(engine.truth().targets[0].speed, overrides.target_speed_kph / 3.6,
                 "Derived target speed is re-evaluated", 1e-6);
            for (int step = 0; step < 500; ++step) {
                engine.advance_native(dt);
                const auto world = engine.truth();
                ACCSIM_PersistentAccOutput output{};
                require(ACCSIM_GetPersistentAccOutput(&output) == 0, "Overridden ACC diagnostics");
                if (!overrides.target_braking) {
                    near(world.targets[0].speed, overrides.target_speed_kph / 3.6,
                         "Disabled braking preserves constant target speed", 1e-6);
                }
                if (mode == 0) {
                    baseline.push_back(world);
                    leads.push_back(output.lead_id);
                } else {
                    near(world.ego.speed, baseline[step].ego.speed, "Override mode speed identity", 0);
                    near(world.ego.pose.x, baseline[step].ego.pose.x, "Override mode position identity", 0);
                    near(world.targets[0].speed, baseline[step].targets[0].speed, "Override target motion identity", 0);
                    require(output.lead_id == leads[step], "Override target selection identity");
                }
            }
            if (overrides.target_braking) {
                require(engine.truth().targets[0].speed < overrides.target_speed_kph / 3.6 - 3,
                        "Enabled braking still executes");
            }
        }
    }
    // No overrides after an overridden run: no retained callback or values.
    {
        const auto directory = root / "defaults_after_overrides";
        const auto scene = accsim::prepare_scenario(source, directory / "generated.xosc", "Ego");
        accsim::EsminiAdapter engine(scene, directory, {}, dt);
        near(engine.truth().ego.speed, 20.0 / 3.6, "Default ego speed restored", 1e-6);
        near(engine.truth().targets[0].speed, 20.0 / 3.6, "Default target speed restored", 1e-6);
    }
    // Validate external maneuver contents, not just the main scenario XML.
    const auto catalog_directory = root / "catalog_validation";
    std::filesystem::create_directories(catalog_directory);
    pugi::xml_document fixture_xml;
    require(fixture_xml.load_file(source.c_str()), "Load catalog validation source");
    for (auto selected : fixture_xml.select_nodes("//CatalogLocations/*/Directory")) {
        auto directory = selected.node();
        const auto path = std::filesystem::canonical(source.parent_path() / directory.attribute("path").value());
        directory.attribute("path") = path.string().c_str();
    }
    auto maneuver_directory = fixture_xml.select_node("//CatalogLocations/ManeuverCatalog/Directory").node();
    const auto original_catalog = std::filesystem::path(maneuver_directory.attribute("path").value()) /
                                  "ManeuverCatalog.xosc";
    maneuver_directory.attribute("path") = catalog_directory.string().c_str();
    auto road = fixture_xml.select_node("//RoadNetwork/LogicFile").node();
    road.attribute("filepath") = std::filesystem::canonical(source.parent_path() /
                                                           road.attribute("filepath").value()).string().c_str();
    const auto catalog_source = catalog_directory / "source.xosc";
    require(fixture_xml.save_file(catalog_source.c_str()), "Save catalog validation source");
    const auto check_catalog = [&](pugi::xml_document& catalog, const std::string& expected_error) {
        require(catalog.save_file((catalog_directory / "ManeuverCatalog.xosc").c_str()), "Save catalog fixture");
        std::string error;
        try {
            accsim::prepare_scenario(catalog_source, catalog_directory / "generated.xosc", "Ego");
        } catch (const std::exception& exception) {
            error = exception.what();
        }
        require(expected_error.empty() ? error.empty() : error.find(expected_error) != std::string::npos,
                "Expected catalog validation outcome: " + expected_error + "; got: " + error);
    };
    pugi::xml_document catalog;
    require(catalog.load_file(original_catalog.c_str()), "Load original observer catalog");
    check_catalog(catalog, ""); // Existing NCAP variable observers remain valid.
    for (const std::string kind : {"EntityAction", "PrivateAction", "UserDefinedAction", "ParameterAction"}) {
        require(catalog.load_file(original_catalog.c_str()), "Reload observer catalog");
        auto action = catalog.select_node("//Maneuver/Event/Action").node();
        while (action.first_child()) {
            action.remove_child(action.first_child());
        }
        if (kind == "EntityAction" || kind == "ParameterAction") {
            action.append_child("GlobalAction").append_child(kind.c_str());
        } else {
            action.append_child(kind.c_str());
        }
        check_catalog(catalog, "only VariableAction/SetAction observers");
    }
    require(catalog.load_file(original_catalog.c_str()), "Reload nested fixture");
    catalog.select_node("//Maneuver").node().append_child("CatalogReference");
    check_catalog(catalog, "nested maneuver catalog references");
    require(catalog.load_file(original_catalog.c_str()), "Reload missing entry fixture");
    catalog.select_node("//Maneuver").node().attribute("name") = "OtherEntry";
    check_catalog(catalog, "missing maneuver catalog entry");
    require(catalog.load_file(original_catalog.c_str()), "Reload duplicate fixture");
    catalog.select_node("//Catalog").node().append_copy(catalog.select_node("//Maneuver").node());
    check_catalog(catalog, "duplicate maneuver catalog entry");
    // esmini searches across declared catalog directories; reject shadowed names.
    require(catalog.load_file(original_catalog.c_str()), "Reload ambiguous fixture");
    const auto shadow_directory = catalog_directory / "shadow";
    std::filesystem::create_directories(shadow_directory);
    require(catalog.save_file((shadow_directory / "ManeuverCatalog.xosc").c_str()), "Save shadow catalog");
    auto vehicle_directory = fixture_xml.select_node("//CatalogLocations/VehicleCatalog/Directory").node();
    const std::string vehicle_path = vehicle_directory.attribute("path").value();
    vehicle_directory.attribute("path") = shadow_directory.string().c_str();
    require(fixture_xml.save_file(catalog_source.c_str()), "Save ambiguous scenario");
    check_catalog(catalog, "ambiguous maneuver catalog");
    vehicle_directory.attribute("path") = vehicle_path.c_str();
    require(fixture_xml.save_file(catalog_source.c_str()), "Restore scenario directories");
    std::filesystem::remove(catalog_directory / "ManeuverCatalog.xosc");
    bool missing_catalog_rejected = false;
    try {
        accsim::prepare_scenario(catalog_source, catalog_directory / "generated.xosc", "Ego");
    } catch (const std::exception& exception) {
        missing_catalog_rejected = std::string(exception.what()).find("cannot find maneuver catalog") != std::string::npos;
    }
    require(missing_catalog_rejected, "Missing maneuver catalog must not fall back to unchecked resources");

    // Later ego instructions must be rejected instead of silently competing with ACC.
    pugi::xml_document document;
    require(document.load_file(source.c_str()), "Load NCAP fixture");
    document.select_node("//ManeuverGroup[@name='GVT_TeleportAndBrake']/Actors/EntityRef")
            .node().attribute("entityRef") = "Ego";
    const auto invalid = root / "ego-conflict.xosc";
    require(document.save_file(invalid.c_str()), "Save conflicting fixture");
    bool rejected = false;
    try {
        accsim::prepare_scenario(invalid, root / "invalid-generated.xosc", "Ego");
    } catch (const std::exception&) {
        rejected = true;
    }
    require(rejected, "Later ego movement must be rejected");
}
}

int main(int argc, char** argv) {
    try {
        if (argc == 3) {
            check_action_scenario(std::filesystem::absolute(argv[2]), std::filesystem::absolute(argv[1]));
            std::cout << "NCAP catalog initialization, target actions and native identity checks passed\n";
            return 0;
        }
        require(argc == 2, "acc_tests TEMP_DIRECTORY [NCAP_SCENARIO]");
        const auto root = std::filesystem::absolute(argv[1]);
        // Inline vehicles can also use native Init actions without trajectories.
        {
            const auto directory = root / "inline_actions";
            const auto source = fixture(directory, 10, 20, {{25, 8}});
            pugi::xml_document document;
            require(document.load_file(source.source.c_str()), "Load inline action fixture");
            for (auto selected : document.select_nodes("//PrivateAction/ControllerAction")) {
                selected.node().parent().remove_child(selected.node());
            }
            require(document.save_file(source.source.c_str()), "Save inline action fixture");
            const auto scene = accsim::prepare_scenario(source.source, directory / "prepared.xosc");
            // An override on a scenario without that parameter must fail explicitly.
            accsim::ScenarioOverrides invalid;
            invalid.set_ego_speed = true;
            invalid.ego_speed_kph = 50;
            bool rejected = false;
            try {
                accsim::EsminiAdapter failed(scene, directory, {}, dt, false, {}, invalid);
            } catch (const std::exception& error) {
                rejected = std::string(error.what()).find("Ego_speed_kph") != std::string::npos;
            }
            require(rejected, "Missing parameter override must fail clearly");
            // This successful default run also checks callback cleanup after failure.
            accsim::EsminiAdapter engine(scene, directory, {}, dt);
            ACCSIM_AccInitialization init{};
            require(ACCSIM_GetAccInitialization(&init) == 0, "Inline native initialization");
            near(init.initial_speed_mps, 10, "Inline Init speed preserved");
            near(init.set_speed_mps, 10, "Inline native cruise default");
            advance(engine);
        }
        int index = 0;
        const auto check = [&](double speed, double set_speed, const std::vector<Target>& targets,
                               int lead, double expected_speed, double center = 0) {
            const auto directory = root / std::to_string(index++);
            const auto scene = fixture(directory, speed, set_speed, targets, center);
            accsim::EsminiAdapter engine(scene, directory, {}, dt);
            const auto output = advance(engine);
            require(output.target_count == static_cast<int>(targets.size()), "Every non-ego target copied");
            require(output.lead_id == lead, "Expected native target selection");
            near(output.desired_speed_mps, expected_speed, "Native speed calculation");
            return output;
        };
        // IDs follow entity order: ego=0, targets=1,2,... .
        // A rear target is in the copied view but must not affect cruise.
        check(10, 20, {{-25, 0}}, -1, 10.07);
        // Ordinary targets advance before ACC: use the gap at controller time.
        check(10, 15, {{25, 8}}, 1, 10 - dt * .9022222222222222);
        check(10, 10, {{14, 10}}, 1, 10 - dt * 1.2361111111111112);
        check(10, 10, {{5.5, 0}}, 1, 0); // Immediate native stop, with no custom actuator clamp.
        check(10, 20, {{45, 10}, {25, 10}}, 2, 10 + dt * 1.4583333333333333);
        check(10, 20, {{25, 10}, {45, 10}}, 1, 10 + dt * 1.4583333333333333);
        check(10, 20, {{25, 0, 10}}, -1, 10.07);
        check(10, 20, {{200, 0}}, -1, 10.07); // Beyond native lookahead.
        check(10, 10.001, {{45, 10}}, 1, 10.001);
        check(30, 30, {{8, 0}}, 1, 29.9);
        check(0, 0, {{-25, 0}}, -1, 0);
        // Offset/heading cases retain native bounding-box branches.
        check(10, 15, {{25, 8, 0, .5}}, 1, 10 - dt * 1.166111111111111, 1);
        check(10, 15, {{25, 8, 0, .5, accsim::pi}}, 1, 10 - dt * 1.7783333333333333, 1);

        // The native sensor filters targets before they reach ACC. Entity order is retained.
        const auto sensor_check = [&](const std::string& name, const std::vector<Target>& targets,
                                      const std::vector<int>& expected_ids, int lead) {
            const auto directory = root / name;
            const auto scene = fixture(directory, 10, 20, targets);
            const ACCSIM_SensorConfig sensor{1, 0, 0, .5, 0, 2, 60, accsim::pi / 3};
            accsim::EsminiAdapter engine(scene, directory, {}, dt, false, sensor);
            const auto output = advance(engine);
            std::vector<int> ids(targets.size());
            const int count = ACCSIM_GetDetectedTargetIds(ids.data(), static_cast<int>(ids.size()));
            require(count >= 0, "Sensor detection IDs available");
            ids.resize(count);
            require(ids == expected_ids, "Expected visible target IDs in native order");
            require(output.target_count == count, "Only detected snapshots supplied to ACC");
            require(output.lead_id == lead, "Native selection uses sensor-filtered targets");
            if (expected_ids.empty()) {
                near(output.desired_speed_mps, 10.07, "Excluded targets cannot brake ego");
            }
        };
        sensor_check("sensor_visible", {{25, 8}}, {1}, 1);
        sensor_check("sensor_far", {{80, 0}}, {}, -1);
        sensor_check("sensor_rear", {{-25, 0}}, {}, -1);
        sensor_check("sensor_fov", {{25, 0, 20}}, {}, -1);
        sensor_check("sensor_near", {{1, 0}}, {}, -1);
        sensor_check("sensor_order", {{45, 10}, {25, 10}, {-25, 0}, {80, 0}}, {1, 2}, 2);

        // A target crosses into range on step 2: catches stale/player-time sampling.
        {
            const auto directory = root / "sensor_timing";
            const auto scene = fixture(directory, 0, 0, {{60.15, 10, 0, 0, accsim::pi}});
            const ACCSIM_SensorConfig sensor{1, 0, 0, .5, 0, 0, 60, accsim::pi / 3};
            accsim::EsminiAdapter engine(scene, directory, {}, dt, false, sensor);
            require(advance(engine).target_count == 0, "Target outside range at first controller step");
            require(advance(engine).target_count == 1, "Sensor refreshed at second controller step");
        }

        // All visible: sensor-fed copies and all-target copies execute identically.
        std::vector<accsim::WorldTruthFrame> baseline;
        std::vector<int> baseline_leads;
        for (bool sensor_enabled : {false, true}) {
            const auto directory = root / (sensor_enabled ? "sensor_identity" : "copies_identity");
            const auto scene = fixture(directory, 10, 20, {{25, 8}, {45, 10}});
            const ACCSIM_SensorConfig sensor{sensor_enabled, 0, 0, .5, 0, 0, 100, accsim::pi};
            accsim::EsminiAdapter engine(scene, directory, {}, dt, false, sensor);
            for (int step = 0; step < 20; ++step) {
                const auto output = advance(engine);
                const auto world = engine.truth();
                if (!sensor_enabled) {
                    baseline.push_back(world);
                    baseline_leads.push_back(output.lead_id);
                } else {
                    near(world.ego.speed, baseline[step].ego.speed, "All-visible native speed identity", 0);
                    near(world.ego.pose.x, baseline[step].ego.pose.x, "All-visible native position identity", 0);
                    require(output.lead_id == baseline_leads[step], "All-visible target selection identity");
                }
            }
        }

        // Several consecutive steps detect accidental controller reconstruction:
        // cruise must keep its original set speed, not reset it to each ego speed.
        const auto directory = root / "persistent";
        const auto scene = fixture(directory, 10, 20, {{-25, 0}});
        accsim::EsminiAdapter engine(scene, directory, {}, dt);
        for (int step = 1; step <= 20; ++step) {
            near(advance(engine).desired_speed_mps, 10 + step * .07, "Cruise state persists across steps");
        }
        std::cout << "Persistent ACC cruise, following, selection, limits, stopping and movement checks passed\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "ACC test failed: " << error.what() << '\n';
        return 1;
    }
}
