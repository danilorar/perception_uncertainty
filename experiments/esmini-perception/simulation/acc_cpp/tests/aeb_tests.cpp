// Author: Zhuo Ma
// Integration tests execute the upstream AEB against private observation copies.
// They cover trigger isolation, measured velocity, ramp/latch persistence and
// reset; checks remain active in Release builds.
#include "accsim/control.hpp"
#include "accsim/engine.hpp"
#include <cmath>
#include <filesystem>
#include <iostream>
#include <stdexcept>
namespace {
void require(bool condition,const char* text) { if(!condition) throw std::runtime_error(text); }
void near(double actual,double expected,const char* text,double tol=1e-7) {
    require(std::isfinite(actual) && std::abs(actual-expected)<tol,text);
}
accsim::ControllerInput input() {
    accsim::ControllerInput i;
    i.ego.id=1; i.ego.pose={-54.76,-10.17,.1,.145748,0,0};
    i.ego.speed=20; i.ego.box={5,1.8,1.6,0,0,0};
    i.ego.velocity={20*std::cos(i.ego.pose.h),20*std::sin(i.ego.pose.h),0};
    return i;
}
accsim::Observation target(const accsim::ControllerInput& i,double distance,double velocity) {
    accsim::Observation t; t.track_id=2; t.pose=i.ego.pose;
    t.pose.x+=distance*std::cos(t.pose.h); t.pose.y+=distance*std::sin(t.pose.h);
    t.box=i.ego.box; t.speed=velocity;
    t.velocity={velocity*std::cos(t.pose.h),velocity*std::sin(t.pose.h),0};
    return t;
}
}
int main(int argc,char** argv) {
    try {
        require(argc==3,"aeb_tests ORIGINAL TEMP");
        const std::filesystem::path out=argv[2]; std::filesystem::create_directories(out);
        const auto scene=accsim::prepare_scenario(argv[1],out/"generated.xosc");
        accsim::EsminiAdapter engine(scene,out,{},accsim::SensorConfig{},.01);
        const auto initial=engine.truth();
        for(const auto& t:initial.targets)
            near(std::hypot(t.velocity.x,t.velocity.y),t.speed,"Target initial velocity agrees with initial speed",1e-6);
        near(std::hypot(initial.ego.velocity.x,initial.ego.velocity.y),initial.ego.speed,"Exact initial ego velocity",1e-6);

        auto i=input();
        accsim::EsminiAeb aeb({});
        auto r=aeb.update(i);
        require(!r.aeb_active && r.lead_id==-1,"An unseen truth target cannot trigger AEB");
        near(r.acceleration_mps2,0,"Nominal driver holds speed");
        i.perception.objects={target(i,35,10)};
        r=aeb.update(i);
        require(!r.aeb_active && r.lead_id==2,"Safe detected lead is logged without braking");
        near(r.observed_gap_m,30,"Upstream bumper distance");
        near(r.observed_ttc_s,3,"Upstream road-relative TTC");

        // This object is close, but its measured velocity equals ego velocity.
        // Setting just speed=0 must not replace its observed velocity vector.
        auto fast=target(i,15,20); fast.speed=0; i.perception.objects={fast};
        r=aeb.update(i);
        require(!r.aeb_active,"AEB uses measured velocity, not stale speed/zero proxy velocity");

        auto off=target(i,15,10); off.pose.y+=3; i.perception.objects={off};
        require(!aeb.update(i).aeb_active,"Non-overlapping lead cannot trigger frontal AEB");
        i.perception.objects={target(i,-15,0)};
        require(!aeb.update(i).aeb_active,"Rear object cannot trigger frontal AEB");

        i.perception.objects={target(i,15,10)};
        r=aeb.update(i);
        require(r.aeb_active && r.lead_id==2,"Observed TTC below 1.5 triggers original AEB");
        near(r.observed_ttc_s,1,"Original trigger TTC");
        near(r.acceleration_mps2,-.85*9.81/.6*.01,"First native jerk-ramp step");
        i.perception.objects.clear();
        r=aeb.update(i);
        require(r.aeb_active,"Dropout after triggering must not destroy native AEB state");
        near(r.acceleration_mps2,-2*.85*9.81/.6*.01,"Brake ramp persists across missing frame");
        accsim::EsminiAeb independent({});
        require(!independent.update(i).aeb_active,"AEB state cannot leak between runs/contexts");
        for(int k=2;k<70;++k) r=aeb.update(i);
        near(r.acceleration_mps2,-.85*9.81,"Native maximum braking reached and bounded");
        i.ego.speed=0; i.ego.velocity={};
        r=aeb.update(i); require(!r.aeb_active,"Native AEB releases latch after stop");
        near(aeb.update(i).acceleration_mps2,0,"Stopped nominal driver remains stopped");
        bool rejected=false; try { accsim::EsminiAeb invalid({0,8}); } catch(const std::runtime_error&) { rejected=true; }
        require(rejected,"Invalid AEB config rejected");
        i.dt_s=0; rejected=false;
        try { aeb.update(i); } catch(const std::invalid_argument&) { rejected=true; }
        require(rejected,"Zero step rejected");
        std::cout<<"Native AEB component integration checks passed\n";
        return 0;
    } catch(const std::exception& e) { std::cerr<<"AEB test failed: "<<e.what()<<'\n'; return 1; }
}
