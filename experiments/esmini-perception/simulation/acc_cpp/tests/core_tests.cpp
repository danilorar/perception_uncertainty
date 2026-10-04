// Author: Zhuo Ma
#include "accsim/control.hpp"
#include "accsim/metrics.hpp"
#include "accsim/perception.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <stdexcept>
namespace {
void require(bool ok,const char* message) { if(!ok) throw std::runtime_error(message); }
void near(double a,double b,const char* message) { require(std::abs(a-b)<1e-9,message); }
}
int main() {
    try {
        accsim::PerceptionFrame raw; raw.sequence=7; raw.measurement_time_s=.35; raw.delivery_time_s=.35;
        accsim::Observation object; object.track_id=1; object.pose.x=25; object.speed=8; raw.objects.push_back(object);
        auto identity=accsim::DetectionDropout(0,42).apply(raw);
        require(identity.dropped_ids.empty() && identity.frame.objects.size()==1,"p=0 must be identity");
        near(identity.frame.objects[0].pose.x,25,"Identity preserves coordinates");
        near(identity.frame.measurement_time_s,.35,"Identity preserves measurement time");
        const auto missing=accsim::DetectionDropout(1,42).apply(raw);
        require(missing.frame.objects.empty() && missing.dropped_ids==std::vector<int>{1},"p=1 removes object");
        require(raw.objects.size()==1 && raw.objects[0].pose.x==25,"Dropout cannot mutate original snapshot");
        accsim::DetectionDropout dropout(.1,42);
        auto first=dropout.apply(raw),second=dropout.apply(raw);
        require(first.dropped_ids==second.dropped_ids,"Reads must not advance random state");
        object.track_id=2; raw.objects.push_back(object);
        auto ordered=dropout.apply(raw); std::reverse(raw.objects.begin(),raw.objects.end()); auto reversed=dropout.apply(raw);
        std::sort(ordered.dropped_ids.begin(),ordered.dropped_ids.end()); std::sort(reversed.dropped_ids.begin(),reversed.dropped_ids.end());
        require(ordered.dropped_ids==reversed.dropped_ids,"Object iteration order cannot change dropout");
        bool rejected=false; try { accsim::DetectionDropout bad(-.1,0); } catch(const std::invalid_argument&) { rejected=true; }
        require(rejected,"Invalid dropout probability rejected");
        accsim::ReferencePath path; path.vertices={{0,0,{}},{10,100,{100,0,0,0,0,0}}};
        accsim::EgoMotion ego; ego.state.speed=1;
        accsim::advance_vehicle(ego,{-10,false},.2,path);
        near(ego.state.speed,0,"Plant does not reverse after stop"); near(ego.progress_m,.05,"Stop inside timestep integrated exactly");
        accsim::ControlRequest stop; stop.acceleration_mps2=-1000;
        const auto bounded=accsim::arbitrate(stop,accsim::AccConfig{});
        near(bounded.acceleration_mps2,-10,"Instant stop request limited by shared plant"); require(bounded.limited,"Saturation logged");
        accsim::ObjectState host,target; host.box=target.box={5,1.8,1.6,0,0,.7};
        host.speed=10; target.speed=8; target.id=1; target.pose.x=25;
        auto geometry=accsim::geometry(host,target); near(geometry.gap_m,20,"True bumper gap");
        near(geometry.distance_m,20,"True surface distance"); require(!geometry.collision,"Separated boxes do not collide");
        accsim::WorldTruthFrame truth; truth.ego=host; truth.targets={target};
        const auto metrics=accsim::evaluate_truth(truth); near(metrics.ttc_s,10,"True longitudinal TTC");
        target.pose.x=4.9; geometry=accsim::geometry(host,target); require(geometry.collision,"Overlapping truth boxes collide");
        target.pose.z=5; require(!accsim::geometry(host,target).collision,"Height-separated boxes do not collide");
        target.pose.z=0;target.pose.y=10; require(!accsim::geometry(host,target).in_path,"Lateral target excluded from rear-end TTC");
        std::cout<<"Perception, plant and truth metrics checks passed\n";
        return 0;
    } catch(const std::exception& e) { std::cerr<<e.what()<<'\n';return 1; }
}
