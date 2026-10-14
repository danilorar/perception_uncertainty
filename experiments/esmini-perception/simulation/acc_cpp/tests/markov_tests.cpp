// Author: Zhuo Ma
// Temporal state, identity isolation, sensor clock and stationary-law checks.
#include "accsim/perception.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <stdexcept>
namespace {
void require(bool ok,const char* message) { if(!ok) throw std::runtime_error(message); }
void near(double x,double y,double tolerance,const char* message) { require(std::abs(x-y)<=tolerance,message); }
accsim::PerceptionFrame frame(std::uint64_t sequence,bool visible=true) {
    accsim::PerceptionFrame value;
    value.sequence=sequence; value.measurement_time_s=value.delivery_time_s=sequence*.05;
    if(visible) {
        accsim::Observation object; object.track_id=1; object.pose.x=25; object.speed=8;
        value.objects.push_back(object);
    }
    return value;
}
template<class F> void rejects(F fn,const char* message) {
    bool rejected=false; try {fn();} catch(const std::invalid_argument&) {rejected=true;}
    require(rejected,message);
}
}
int main() {
    try {
        using namespace accsim;
        const auto rates=markov_parameters(.1,.5,.05);
        near(rates.enter_missing,1.0/90,1e-12,"alpha from rate and duration");
        near(rates.recover,.1,1e-12,"beta from duration");
        near(rates.enter_missing/(rates.enter_missing+rates.recover),.1,1e-12,"stationary miss rate");
        rejects([]{markov_parameters(.1,.01,.05);},"sub-period missing duration rejected");
        rejects([]{markov_parameters(.9,.05,.05);},"alpha > 1 rejected");
        rejects([]{MarkovDropout(.1,.5,.05,0,{1,1});},"duplicate identities rejected");

        MarkovDropout identity(0,.5,.05,42,{1}), total(1,.5,.05,42,{1});
        for(std::uint64_t i=0;i<100;++i) {
            const auto raw=frame(i);
            const auto kept=identity.apply(raw),missing=total.apply(raw);
            require(kept.frame.objects.size()==1&&kept.dropped_ids.empty(),"Markov p=0 identity");
            near(kept.frame.objects[0].pose.x,25,0,"identity preserves target pose");
            require(missing.frame.objects.empty()&&missing.dropped_ids==std::vector<int>{1},"Markov p=1 total loss");
            require(raw.objects.size()==1&&raw.objects[0].pose.x==25,"raw snapshot unchanged");
        }

        MarkovDropout visible(.1,.5,.05,7,{1}), outside(.1,.5,.05,7,{1});
        for(std::uint64_t i=0;i<1000;++i) {
            const auto result=visible.apply(frame(i));
            const auto empty=outside.apply(frame(i,false));
            require(empty.frame.objects.empty(),"normal state cannot invent a detection outside FOV");
            require(visible.states().at(1).missing==outside.states().at(1).missing,
                    "FOV absence must not pause the chain");
            const auto length=visible.states().at(1).run_length;
            const auto cached=visible.apply(frame(i));
            require(cached.dropped_ids==result.dropped_ids&&visible.states().at(1).run_length==length,
                    "repeated read cannot advance state");
        }
        rejects([&]{visible.apply(frame(1001));},"skipped perception sequence rejected");
        rejects([&]{visible.apply(frame(998));},"out-of-order sequence rejected");
        auto wrong_clock=frame(1000);wrong_clock.measurement_time_s+=.01;
        rejects([&]{visible.apply(wrong_clock);},"control clock cannot advance the perception chain");

        MarkovDropout a(.1,.5,.05,19,{1,2}),b(.1,.5,.05,19,{2,1});
        for(std::uint64_t i=0;i<1000;++i) {
            auto raw=frame(i);auto object=raw.objects[0];object.track_id=2;raw.objects.push_back(object);
            auto ar=a.apply(raw);std::reverse(raw.objects.begin(),raw.objects.end());auto br=b.apply(raw);
            std::sort(ar.dropped_ids.begin(),ar.dropped_ids.end());std::sort(br.dropped_ids.begin(),br.dropped_ids.end());
            require(ar.dropped_ids==br.dropped_ids,"same seed independent of target/list order");
        }

        std::uint64_t initial_missing=0;
        for(std::uint64_t seed=0;seed<10000;++seed) {
            MarkovDropout m(.1,.5,.05,seed,{1});m.apply(frame(0));
            initial_missing+=m.states().at(1).missing;
            MarkovDropout normal(.1,.5,.05,seed,{1},false);normal.apply(frame(0));
            require(!normal.states().at(1).missing,"normal initialization is explicit");
        }
        near(initial_missing/10000.0,.1,.015,"stationary initialization across seeds");

        MarkovDropout long_run(.1,.5,.05,42,{1});
        std::uint64_t misses=0,missing_steps=0,normal_steps=0,entries=0,recoveries=0,finished=0,total_length=0;
        bool previous=false;std::uint64_t length=0;
        constexpr std::uint64_t samples=500000;
        for(std::uint64_t i=0;i<samples;++i) {
            long_run.apply(frame(i));const bool state=long_run.states().at(1).missing;
            misses+=state;
            if(i) {
                if(previous) {++missing_steps;if(!state)++recoveries;}
                else {++normal_steps;if(state)++entries;}
            }
            if(state) ++length;
            else if(length) {++finished;total_length+=length;length=0;}
            previous=state;
        }
        near(misses/double(samples),.1,.01,"long-run missing fraction");
        near(entries/double(normal_steps),rates.enter_missing,.002,"measured G->M rate");
        near(recoveries/double(missing_steps),rates.recover,.01,"measured M->G rate");
        near(.05*total_length/double(finished),.5,.05,"mean completed missing-burst duration");
        std::cout<<"Markov checks passed; fraction="<<misses/double(samples)
                 <<", mean missing="<<.05*total_length/double(finished)<<" s\n";
        return 0;
    } catch(const std::exception& error) {std::cerr<<error.what()<<'\n';return 1;}
}
