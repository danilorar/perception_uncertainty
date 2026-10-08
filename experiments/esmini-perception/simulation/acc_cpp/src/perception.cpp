// Author: Zhuo Ma
#include "accsim/perception.hpp"
#include <cmath>
#include <stdexcept>
namespace accsim {
namespace {
// SplitMix64's integer mixing gives reproducible draws across compilers/OSes.
// Unlike std::hash or distribution engines this mapping is fully specified.
std::uint64_t mix(std::uint64_t x) {
    x += UINT64_C(0x9e3779b97f4a7c15);
    x = (x ^ (x >> 30)) * UINT64_C(0xbf58476d1ce4e5b9);
    x = (x ^ (x >> 27)) * UINT64_C(0x94d049bb133111eb);
    return x ^ (x >> 31);
}
double markov_draw(std::uint64_t seed,std::uint64_t sequence,int id,bool initial) {
    // Distinct initial/transition domains; paired durations share uniform draws.
    const auto domain=initial?UINT64_C(0x243f6a8885a308d3):UINT64_C(0x13198a2e03707344);
    const auto key=mix(seed)^mix(sequence)^mix(static_cast<std::uint64_t>(id)+UINT64_C(0x100000000))^mix(domain);
    return static_cast<double>(mix(key)>>11)*(1.0/9007199254740992.0);
}
}
DetectionDropout::DetectionDropout(double p, std::uint64_t seed): probability_(p), seed_(seed) {
    if (!std::isfinite(p) || p < 0 || p > 1) throw std::invalid_argument("dropout-p must be in [0,1]");
}
DropoutResult DetectionDropout::apply(const PerceptionFrame& ideal) const {
    DropoutResult result;
    result.frame = ideal;
    result.frame.objects.clear();
    // One sensor in version 1. The key uses seed, measurement sequence and ID.
    // Reading this cached output for logs/control cannot consume random state.
    for (const auto& object : ideal.objects) {
        const auto key = mix(seed_) ^ mix(ideal.sequence) ^ mix(static_cast<std::uint64_t>(object.track_id) + UINT64_C(0x100000000));
        const double draw = static_cast<double>(mix(key) >> 11) * (1.0 / 9007199254740992.0);
        if (draw < probability_) result.dropped_ids.push_back(object.track_id);
        else result.frame.objects.push_back(object);
    }
    return result;
}
MarkovParameters markov_parameters(double p,double mean_missing_s,double period_s) {
    if(!std::isfinite(p)||p<0||p>1||!std::isfinite(mean_missing_s)||
       !std::isfinite(period_s)||period_s<=0||mean_missing_s<period_s)
        throw std::invalid_argument("Markov requires p in [0,1] and mean-missing >= sensor-period > 0");
    // Identity and total dropout are explicit absorbing-state validation cases.
    if(p==0) return {0,1};
    if(p==1) return {1,0};
    const double beta=period_s/mean_missing_s;
    const double alpha=p*beta/(1-p);
    if(alpha>1) throw std::invalid_argument("Requested Markov miss rate/duration gives alpha > 1");
    return {alpha,beta};
}
MarkovDropout::MarkovDropout(double p,double mean_missing_s,double period_s,
                           std::uint64_t seed,const std::vector<int>& ids,bool stationary)
    :probability_(p),period_(period_s),seed_(seed),stationary_(stationary),
     parameters_(markov_parameters(p,mean_missing_s,period_s)) {
    for(int id:ids) if(id<0||!states_.emplace(id,MarkovState{}).second)
        throw std::invalid_argument("Markov target IDs must be unique nonnegative integers");
}
DropoutResult MarkovDropout::apply(const PerceptionFrame& ideal) {
    if(!std::isfinite(ideal.measurement_time_s)) throw std::invalid_argument("Invalid Markov measurement time");
    if(initialized_ && ideal.sequence==last_sequence_) {
        if(std::abs(ideal.measurement_time_s-last_time_)>1e-8)
            throw std::invalid_argument("Repeated sequence has another measurement time");
        return cached_; // Reading an already processed frame must not advance state.
    }
    if((!initialized_ && ideal.sequence!=0)||
       (initialized_ && (ideal.sequence!=last_sequence_+1||
        std::abs(ideal.measurement_time_s-last_time_-period_)>1e-7)))
        throw std::invalid_argument("Markov requires consecutive perception frames at sensor-period");
    for(const auto& object:ideal.objects) if(!states_.count(object.track_id))
        throw std::invalid_argument("Ideal detection has an unregistered Markov target ID");
    for(auto& entry:states_) {
        auto& state=entry.second;
        if(!initialized_) {
            state.missing=probability_==1||(stationary_&&markov_draw(seed_,0,entry.first,true)<probability_);
            state.changed=false; state.run_length=1;
        } else {
            const bool previous=state.missing;
            const double draw=markov_draw(seed_,ideal.sequence,entry.first,false);
            state.missing=previous ? !(draw<parameters_.recover) : draw<parameters_.enter_missing;
            state.changed=state.missing!=previous;
            state.run_length=state.changed?1:state.run_length+1;
        }
    }
    cached_.frame=ideal; cached_.frame.objects.clear(); cached_.dropped_ids.clear();
    for(const auto& object:ideal.objects) {
        if(states_.at(object.track_id).missing) cached_.dropped_ids.push_back(object.track_id);
        else cached_.frame.objects.push_back(object);
    }
    initialized_=true; last_sequence_=ideal.sequence; last_time_=ideal.measurement_time_s;
    return cached_;
}
} // namespace accsim
