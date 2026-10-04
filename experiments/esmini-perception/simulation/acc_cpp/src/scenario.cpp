// Author: Zhuo Ma
// Scenario preparation for flat, near-parallel two-vehicle rear-end cases.
// The source is read only. Only a generated copy relinquishes ego motion to the
// native runner; the other vehicle's original timed trajectory remains intact.
#include "accsim/scenario.hpp"
#include "pugixml.hpp"
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <map>
#include <stdexcept>
#include <string>

namespace accsim {
namespace {
namespace fs = std::filesystem;
constexpr double epsilon = 1e-10;

[[noreturn]] void unsupported(const std::string& message) {
    throw std::runtime_error("Unsupported scenario for the rear-end adapter: " + message);
}

double number(pugi::xml_node node, const char* attribute, bool required=true) {
    const auto value = node.attribute(attribute);
    if (!value) {
        if (required) unsupported(std::string("missing ") + node.name() + "/@" + attribute);
        return 0;
    }
    char* end = nullptr;
    const double result = std::strtod(value.value(), &end);
    if (end == value.value() || *end != '\0' || !std::isfinite(result))
        unsupported(std::string("expected finite numeric ") + node.name() + "/@" + attribute);
    return result;
}

double angle_delta(double angle) { return std::remainder(angle, 2*pi); }
double separation(const Pose& a, const Pose& b) {
    return std::hypot(std::hypot(b.x-a.x, b.y-a.y), b.z-a.z);
}
Pose read_pose(pugi::xml_node node) {
    if (!node) unsupported("only explicit WorldPosition coordinates are supported");
    return {number(node,"x"), number(node,"y"), number(node,"z",false),
            number(node,"h"), number(node,"p",false), number(node,"r",false)};
}

ReferencePath read_reference(pugi::xml_node group) {
    const auto trajectories = group.select_nodes(".//Trajectory");
    if (trajectories.size() != 1) unsupported("one inline trajectory per vehicle is required");
    const auto trajectory = trajectories[0].node();
    if (trajectory.attribute("closed").as_bool()) unsupported("closed trajectories");
    const auto action = trajectory.parent();
    const auto timing = action.child("TimeReference").child("Timing");
    if (!timing || std::string(timing.attribute("domainAbsoluteRelative").value()) != "absolute" ||
        std::abs(number(timing,"scale")-1) > epsilon || std::abs(number(timing,"offset")) > epsilon)
        unsupported("trajectory timing must be absolute, scale 1 and offset 0");
    if (std::string(action.child("TrajectoryFollowingMode").attribute("followingMode").value()) != "position")
        unsupported("trajectory followingMode must be position");
    ReferencePath result;
    for (const auto vertex : trajectory.child("Shape").child("Polyline").children("Vertex")) {
        ReferenceVertex v;
        v.time_s = number(vertex,"time");
        v.pose = read_pose(vertex.child("Position").child("WorldPosition"));
        if (!result.vertices.empty()) {
            const auto& previous = result.vertices.back();
            if (v.time_s <= previous.time_s) unsupported("trajectory times must strictly increase");
            const double length = separation(previous.pose,v.pose);
            // A spatial reference cannot encode a stationary dwell unambiguously.
            // This scene contains no repeated vertices; reject instead of silently
            // deleting their time information when deriving its initial speed.
            if (length <= epsilon) unsupported("repeated trajectory positions");
            v.distance_m = previous.distance_m + length;
        }
        result.vertices.push_back(v);
    }
    if (result.vertices.size() < 2 || std::abs(result.vertices.front().time_s) > epsilon)
        unsupported("trajectory requires at least two vertices and must start at t=0");
    return result;
}

void validate_flat_parallel(const ReferencePath& path, double common_heading) {
    const auto& origin = path.vertices.front().pose;
    double previous_longitudinal = -infinity;
    for (const auto& vertex : path.vertices) {
        const auto& pose = vertex.pose;
        const double dx=pose.x-origin.x, dy=pose.y-origin.y;
        const double longitudinal=std::cos(common_heading)*dx+std::sin(common_heading)*dy;
        // Accumulated lateral displacement is not a validity criterion. The
        // ideal driver follows the original spatial path at the controlled speed.
        // Retain the flat/small-heading/forward checks used by this rear-end model.
        if (std::abs(pose.z-origin.z)>0.05 ||
            std::abs(angle_delta(pose.h-common_heading))>0.05 ||
            std::abs(pose.p)>0.02 || std::abs(pose.r)>0.02 ||
            longitudinal < previous_longitudinal-epsilon)
            unsupported("only flat, forward-moving trajectories with nearly parallel headings are supported");
        previous_longitudinal=longitudinal;
    }
}

void property(pugi::xml_node properties, const char* name, const char* value) {
    auto node=properties.append_child("Property");
    node.append_attribute("name")=name;
    node.append_attribute("value")=value;
}

void check_reference(const ReferencePath& reference) {
    if (reference.vertices.size()<2) throw std::runtime_error("ReferencePath requires at least two vertices");
}
} // namespace

double ReferencePath::initial_speed() const {
    check_reference(*this);
    const double dt=vertices[1].time_s-vertices[0].time_s;
    if (!(dt>0)) throw std::runtime_error("ReferencePath has invalid initial time interval");
    return (vertices[1].distance_m-vertices[0].distance_m)/dt;
}

Pose ReferencePath::at_distance(double station) const {
    check_reference(*this);
    if (!std::isfinite(station)) throw std::runtime_error("Reference station must be finite");
    auto upper=std::upper_bound(vertices.begin(),vertices.end(),station,
        [](double s,const ReferenceVertex& vertex){return s<vertex.distance_m;});
    std::size_t index=upper==vertices.begin() ? 0 :
        upper==vertices.end() ? vertices.size()-2 : static_cast<std::size_t>(upper-vertices.begin()-1);
    const auto& a=vertices[index];
    const auto& b=vertices[index+1];
    const double length=b.distance_m-a.distance_m;
    if (!(length>epsilon)) throw std::runtime_error("ReferencePath contains zero-length segment");
    const double fraction=(station-a.distance_m)/length;
    // Beyond the original path, continue the endpoint segment's spatial tangent.
    // Orientation is held at that endpoint rather than extrapolated indefinitely.
    const double orientation_fraction=std::clamp(fraction,0.0,1.0);
    Pose pose;
    pose.x=a.pose.x+fraction*(b.pose.x-a.pose.x);
    pose.y=a.pose.y+fraction*(b.pose.y-a.pose.y);
    pose.z=a.pose.z+fraction*(b.pose.z-a.pose.z);
    pose.h=a.pose.h+orientation_fraction*angle_delta(b.pose.h-a.pose.h);
    pose.p=a.pose.p+orientation_fraction*angle_delta(b.pose.p-a.pose.p);
    pose.r=a.pose.r+orientation_fraction*angle_delta(b.pose.r-a.pose.r);
    return pose;
}

std::pair<double,double> ReferencePath::project(const Pose& pose) const {
    check_reference(*this);
    if (!std::isfinite(pose.x)||!std::isfinite(pose.y)) throw std::runtime_error("Projection requires finite position");
    double best_squared=infinity, best_station=0, best_lateral=0;
    for (std::size_t index=0;index+1<vertices.size();++index) {
        const auto& a=vertices[index]; const auto& b=vertices[index+1];
        const double dx=b.pose.x-a.pose.x,dy=b.pose.y-a.pose.y;
        const double squared=dx*dx+dy*dy;
        if (!(squared>epsilon*epsilon)) throw std::runtime_error("ReferencePath has vertical/zero-length planar segment");
        double fraction=((pose.x-a.pose.x)*dx+(pose.y-a.pose.y)*dy)/squared;
        // Endpoint rays continue the first/last segment; interior segments are
        // bounded. Signed lateral offset is positive to the left of travel.
        if (index!=0) fraction=std::max(0.0,fraction);
        if (index+2!=vertices.size()) fraction=std::min(1.0,fraction);
        const double ex=pose.x-(a.pose.x+fraction*dx),ey=pose.y-(a.pose.y+fraction*dy);
        const double squared_distance=ex*ex+ey*ey;
        if (squared_distance<best_squared) {
            best_squared=squared_distance;
            best_station=a.distance_m+fraction*(b.distance_m-a.distance_m);
            best_lateral=(-dy*ex+dx*ey)/std::sqrt(squared);
        }
    }
    return {best_station,best_lateral};
}

PreparedScenario prepare_scenario(const std::filesystem::path& source,
                                  const std::filesystem::path& generated,
                                  const std::string& ego_name) {
    PreparedScenario result;
    result.source=fs::canonical(source);
    result.generated=fs::weakly_canonical(fs::absolute(generated));
    if (result.generated==result.source ||
        (fs::exists(result.generated)&&fs::equivalent(result.generated,result.source)))
        throw std::runtime_error("Generated scenario must not overwrite the original");
    pugi::xml_document document;
    const auto loaded=document.load_file(result.source.c_str());
    if (!loaded) throw std::runtime_error(std::string("Cannot read scenario XML: ")+loaded.description());
    auto root=document.child("OpenSCENARIO");
    if (!root) unsupported("missing OpenSCENARIO root");
    auto entities=root.child("Entities");
    std::map<std::string,pugi::xml_node> objects;
    for (auto object:entities.children("ScenarioObject")) {
        const std::string name=object.attribute("name").value();
        if (name.empty()||!object.child("Vehicle")||!objects.emplace(name,object).second)
            unsupported("unique named inline vehicles are required");
    }
    if (objects.size()!=2||!objects.count(ego_name)) unsupported("exactly two vehicles including the requested ego are required");
    std::map<std::string,ReferencePath> paths;
    pugi::xml_node ego_group;
    for (const auto selected:root.select_nodes("./Storyboard/Story/Act/ManeuverGroup")) {
        auto group=selected.node();
        auto actors=group.child("Actors");
        const auto refs=actors.select_nodes("./EntityRef");
        if (refs.size()!=1||actors.attribute("selectTriggeringEntities").as_bool())
            unsupported("each maneuver group must explicitly control one vehicle");
        const std::string name=refs[0].node().attribute("entityRef").value();
        if (!objects.count(name)||paths.count(name)) unsupported("one maneuver group per declared vehicle is required");
        paths.emplace(name,read_reference(group));
        if (name==ego_name) ego_group=group;
    }
    if (paths.size()!=objects.size()||!ego_group) unsupported("both vehicles need one explicit timed trajectory");
    result.reference=paths.at(ego_name);
    for (const auto& entry:paths) validate_flat_parallel(entry.second,result.reference.vertices.front().pose.h);
    if (!(result.reference.initial_speed()>0)) unsupported("ego must start moving forward");
    const auto limits=objects.at(ego_name).child("Vehicle").child("Performance");
    result.max_acceleration=number(limits,"maxAcceleration");
    result.max_deceleration=number(limits,"maxDeceleration");
    if (!(result.max_acceleration>0&&result.max_deceleration>0)) unsupported("positive vehicle acceleration limits required");
    const auto stop=root.select_nodes("./Storyboard/StopTrigger/ConditionGroup/Condition/ByValueCondition/SimulationTimeCondition");
    if (stop.size()!=1||root.select_nodes("./Storyboard/StopTrigger/ConditionGroup/Condition").size()!=1)
        unsupported("one simulation-time stop condition is required");
    result.duration_s=number(stop[0].node(),"value");
    if (!(result.duration_s>0)) unsupported("positive stop time required");
    const std::string stop_rule=stop[0].node().attribute("rule").value();
    if (stop_rule!="greaterThan"&&stop_rule!="greaterOrEqual") unsupported("unsupported stop-time comparison");
    auto road_node=root.child("RoadNetwork").child("LogicFile");
    const fs::path declared_road=road_node.attribute("filepath").value();
    if (declared_road.empty()) unsupported("missing road LogicFile");
    fs::path road=declared_road.is_absolute()?declared_road:result.source.parent_path()/declared_road;
    // The supplied data bundle places .xosc and .xodr together although the
    // original XML names ../xodr/. Resolve that known bundle layout explicitly.
    if (!fs::is_regular_file(road)) road=result.source.parent_path()/declared_road.filename();
    if (!fs::is_regular_file(road)) throw std::runtime_error("Cannot resolve scenario OpenDRIVE file");
    result.road=fs::canonical(road);
    if (result.generated==result.road ||
        (fs::exists(result.generated)&&fs::equivalent(result.generated,result.road)))
        throw std::runtime_error("Generated scenario must not overwrite original road data");
    // Keep generated inputs self-contained, including Windows cross-drive runs.
    // This is a byte-identical snapshot; the original road remains read-only.
    fs::create_directories(result.generated.parent_path());
    const auto road_snapshot=result.generated.parent_path()/result.road.filename();
    if(!fs::exists(road_snapshot) || !fs::equivalent(road_snapshot,result.road))
        fs::copy_file(result.road,road_snapshot,fs::copy_options::overwrite_existing);
    road_node.attribute("filepath")=result.road.filename().generic_string().c_str();

    auto ego=objects.at(ego_name);
    ego.remove_child("ObjectController");
    auto controller=ego.append_child("ObjectController").append_child("Controller");
    controller.append_attribute("name")="NativeEsminiACC";
    auto controller_properties=controller.append_child("Properties");
    property(controller_properties,"esminiController","ExternalController");
    property(controller_properties,"mode","override");
    // Retain the original data author's FileHeader; ownership of adapter code
    // is declared in this source, not substituted into original data metadata.
    for (const auto& entry:objects) {
        auto properties=entry.second.child("Vehicle").child("Properties");
        if (!properties) properties=entry.second.child("Vehicle").append_child("Properties");
        while (properties.remove_child("File")) {}
        for (auto item=properties.child("Property");item;) {
            const auto next=item.next_sibling("Property");
            const std::string name=item.attribute("name").value();
            if (name=="model_id"||name=="modelId"||name=="modelid"||name=="control") properties.remove_child(item);
            item=next;
        }
        property(properties,"model_id",entry.first==ego_name?"0":"2");
    }
    std::map<std::string,bool> initialized;
    for (auto init:root.child("Storyboard").child("Init").child("Actions").children("Private")) {
        const std::string name=init.attribute("entityRef").value();
        if (!paths.count(name)||initialized[name]) unsupported("one Init/Private per vehicle required");
        initialized[name]=true;
        const auto speed=init.select_nodes("./PrivateAction/LongitudinalAction/SpeedAction/SpeedActionTarget/AbsoluteTargetSpeed");
        const auto teleports=init.select_nodes("./PrivateAction/TeleportAction/Position/WorldPosition");
        if (speed.size()!=1||teleports.size()!=1) unsupported("one initial absolute speed and world teleport per vehicle required");
        if (separation(read_pose(teleports[0].node()),paths.at(name).vertices.front().pose)>1e-6)
            unsupported("initial teleport must equal the first trajectory vertex");
        speed[0].node().attribute("value").set_value(paths.at(name).initial_speed(),17);
        // The input incorrectly nests speed and ControllerAction in the same
        // PrivateAction. Remove its activation and append a separate ego action.
        for (auto action=init.child("PrivateAction");action;) {
            const auto next=action.next_sibling("PrivateAction");
            while(action.remove_child("ControllerAction")) {}
            if (!action.first_child()) init.remove_child(action);
            action=next;
        }
        if (name==ego_name) {
            auto activate=init.append_child("PrivateAction").append_child("ControllerAction").append_child("ActivateControllerAction");
            activate.append_attribute("longitudinal")="true";
            activate.append_attribute("lateral")="true";
        }
    }
    if (initialized.size()!=objects.size()) unsupported("missing vehicle initialization");
    ego_group.parent().remove_child(ego_group);
    fs::create_directories(result.generated.parent_path());
    if (!document.save_file(result.generated.c_str(),"  ",pugi::format_default,pugi::encoding_utf8))
        throw std::runtime_error("Cannot write generated scenario: "+result.generated.string());
    return result;
}
} // namespace accsim
