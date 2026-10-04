// Author: Zhuo Ma
// Standalone fixture checks: argv[1]=original .xosc, argv[2]=temporary output.
#include "accsim/scenario.hpp"
#include "pugixml.hpp"
#include <cmath>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>

namespace {
void require(bool condition,const char* message) {
    if (!condition) throw std::runtime_error(message);
}
void near(double a,double b,double tolerance,const char* message) {
    require(std::isfinite(a)&&std::isfinite(b)&&std::abs(a-b)<=tolerance,message);
}
std::string read(const std::filesystem::path& path) {
    std::ifstream input(path,std::ios::binary);
    require(bool(input),"cannot read fixture");
    return {std::istreambuf_iterator<char>(input),std::istreambuf_iterator<char>()};
}
std::string serialize(pugi::xml_node node) {
    std::ostringstream output; node.print(output); return output.str();
}
double source_initial_speed(pugi::xml_document& document,const char* name) {
    const std::string query=std::string("//ManeuverGroup[Actors/EntityRef/@entityRef='")+name+"']//Trajectory/Shape/Polyline/Vertex";
    const auto vertices=document.select_nodes(query.c_str());
    require(vertices.size()>=2,"source trajectory needs two vertices");
    const auto a=vertices[0].node(),b=vertices[1].node();
    const auto p=a.child("Position").child("WorldPosition"),q=b.child("Position").child("WorldPosition");
    const double dx=q.attribute("x").as_double()-p.attribute("x").as_double();
    const double dy=q.attribute("y").as_double()-p.attribute("y").as_double();
    const double dz=q.attribute("z").as_double()-p.attribute("z").as_double();
    return std::hypot(std::hypot(dx,dy),dz)/(b.attribute("time").as_double()-a.attribute("time").as_double());
}
}
int main(int argc,char** argv) {
    try {
        require(argc==3,"usage: scenario_tests ORIGINAL GENERATED");
        const std::filesystem::path source=argv[1],output=argv[2];
        const auto original_bytes=read(source);
        pugi::xml_document original;
        require(bool(original.load_string(original_bytes.c_str())),"invalid fixture XML");
        const auto prepared=accsim::prepare_scenario(source,output);
        require(read(source)==original_bytes,"preparation changed source bytes");
        require(prepared.reference.vertices.size()==999,"unexpected reference vertex count");
        near(prepared.reference.initial_speed(),source_initial_speed(original,"object_1"),1e-9,"incorrect first-segment ego speed");
        near(prepared.duration_s,10,1e-12,"incorrect duration");
        near(prepared.max_acceleration,10,1e-12,"incorrect acceleration limit");
        near(prepared.max_deceleration,10,1e-12,"incorrect deceleration limit");
        pugi::xml_document generated;
        require(bool(generated.load_file(output.c_str())),"cannot load generated XML");
        const auto root=generated.child("OpenSCENARIO");
        require(std::string(root.child("FileHeader").attribute("author").value())==
                original.child("OpenSCENARIO").child("FileHeader").attribute("author").value(),"data author changed");
        const auto road=output.parent_path()/root.child("RoadNetwork").child("LogicFile").attribute("filepath").value();
        require(read(road)==read(prepared.road),"generated road snapshot differs from original");
        require(root.select_nodes(".//ManeuverGroup/Actors/EntityRef[@entityRef='object_1']").empty(),"ego trajectory still active");
        const auto target=generated.select_node("//ManeuverGroup[Actors/EntityRef/@entityRef='object_2']").node();
        require(bool(target),"missing target maneuver group");
        const auto source_target=original.select_node("//ManeuverGroup[Actors/EntityRef/@entityRef='object_2']").node();
        require(serialize(target)==serialize(source_target),"target maneuver/trajectory changed");
        const auto controller=generated.select_node("//ScenarioObject[@name='object_1']/ObjectController/Controller").node();
        require(std::string(controller.attribute("name").value())=="NativeEsminiACC","wrong controller name");
        require(std::string(controller.select_node("./Properties/Property[@name='esminiController']").node().attribute("value").value())=="ExternalController","wrong bridge controller type");
        require(std::string(controller.select_node("./Properties/Property[@name='mode']").node().attribute("value").value())=="override","bridge is not in override mode");
        require(generated.select_nodes("//PrivateAction[LongitudinalAction and ControllerAction]").empty(),"malformed combined private action remains");
        const auto activation=generated.select_node("//Private[@entityRef='object_1']/PrivateAction/ControllerAction/ActivateControllerAction").node();
        require(activation.attribute("longitudinal").as_bool()&&activation.attribute("lateral").as_bool(),"ego control domains are not active");
        require(generated.select_nodes("//Vehicle/Properties/File").empty(),"unresolved model file remains");
        near(generated.select_node("//Private[@entityRef='object_1']//AbsoluteTargetSpeed").node().attribute("value").as_double(),prepared.reference.initial_speed(),1e-12,"initial ego speed not preserved");
        near(generated.select_node("//Private[@entityRef='object_2']//AbsoluteTargetSpeed").node().attribute("value").as_double(),source_initial_speed(original,"object_2"),1e-9,"initial target speed incorrect");
        // Every ego vertex must survive the conversion into a spatial reference,
        // including 1554431's accumulated lateral displacement above 0.25 m.
        const auto source_vertices=original.select_nodes("//ManeuverGroup[Actors/EntityRef/@entityRef='object_1']//Trajectory/Shape/Polyline/Vertex");
        require(source_vertices.size()==prepared.reference.vertices.size(),"ego path vertex count changed");
        for(std::size_t i=0;i<source_vertices.size();++i) {
            const auto point=source_vertices[i].node().child("Position").child("WorldPosition");
            near(prepared.reference.vertices[i].pose.x,point.attribute("x").as_double(),1e-12,"ego reference x changed");
            near(prepared.reference.vertices[i].pose.y,point.attribute("y").as_double(),1e-12,"ego reference y changed");
            near(prepared.reference.vertices[i].pose.h,point.attribute("h").as_double(),1e-12,"ego reference heading changed");
        }
        const auto& first=prepared.reference.vertices.front();
        const auto& second=prepared.reference.vertices[1];
        const auto midpoint=prepared.reference.at_distance(second.distance_m/2);
        near(midpoint.x,(first.pose.x+second.pose.x)/2,1e-12,"reference interpolation x");
        near(midpoint.y,(first.pose.y+second.pose.y)/2,1e-12,"reference interpolation y");
        const auto& last=prepared.reference.vertices.back();
        const auto& before=prepared.reference.vertices[prepared.reference.vertices.size()-2];
        const auto extended=prepared.reference.at_distance(last.distance_m+5);
        const double length=last.distance_m-before.distance_m;
        near(extended.x,last.pose.x+5*(last.pose.x-before.pose.x)/length,1e-10,"endpoint tangent extrapolation x");
        near(extended.y,last.pose.y+5*(last.pose.y-before.pose.y)/length,1e-10,"endpoint tangent extrapolation y");
        near(extended.h,last.pose.h,1e-12,"endpoint heading was extrapolated");
        auto projection=prepared.reference.project(extended);
        near(projection.first,last.distance_m+5,1e-9,"endpoint projection station");
        near(projection.second,0,1e-9,"endpoint projection lateral");
        // Synthetic path checks signed lateral offset and wrapped heading interpolation.
        accsim::ReferencePath straight;
        straight.vertices={{0,0,{0,0,0,accsim::pi-0.1,0,0}}, {1,10,{10,0,0,-accsim::pi+0.1,0,0}}};
        projection=straight.project({4,2,0,0,0,0});
        near(projection.first,4,1e-12,"interior projection station");
        near(projection.second,2,1e-12,"signed lateral projection");
        projection=straight.project({-3,-2,0,0,0,0});
        near(projection.first,-3,1e-12,"start extrapolation projection");
        near(projection.second,-2,1e-12,"negative lateral projection");
        near(straight.at_distance(5).h,accsim::pi,1e-12,"heading interpolation did not cross shortest arc");
        bool rejected=false;
        try {accsim::prepare_scenario(source,source);} catch(const std::exception&) {rejected=true;}
        require(rejected,"source overwrite was accepted");
        require(read(source)==original_bytes,"overwrite rejection changed source bytes");
        // Reject a mixed-actor group instead of removing a target together with ego.
        const auto invalid_path=output.parent_path()/"ambiguous-fixture.xosc";
        auto actors=original.select_node("//ManeuverGroup[Actors/EntityRef/@entityRef='object_1']/Actors").node();
        actors.append_child("EntityRef").append_attribute("entityRef")="object_2";
        require(original.save_file(invalid_path.c_str()),"cannot write invalid test fixture");
        rejected=false;
        try {accsim::prepare_scenario(invalid_path,output.parent_path()/"invalid-generated.xosc");}
        catch(const std::exception&) {rejected=true;}
        require(rejected,"mixed-actor group was accepted");
        std::filesystem::remove(invalid_path);
        std::cout<<"scenario preparation and reference checks passed\n";
        return 0;
    } catch(const std::exception& error) {
        std::cerr<<error.what()<<'\n'; return 1;
    }
}
