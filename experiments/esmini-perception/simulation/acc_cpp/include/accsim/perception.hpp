// Author: Zhuo Ma
#pragma once
#include "types.hpp"
#include <map>
namespace accsim {
class DetectionDropout {
    double probability_;
    std::uint64_t seed_;
public:
    DetectionDropout(double probability, std::uint64_t seed);
    DropoutResult apply(const PerceptionFrame& ideal) const;
};
// Two-state chain: G -> M with alpha, M -> G with beta. SI time units.
struct MarkovParameters { double enter_missing=0, recover=1; };
MarkovParameters markov_parameters(double miss_probability,double mean_missing_s,double sensor_period_s);
struct MarkovState {
    bool missing=false, changed=false;
    std::uint64_t run_length=0;
};
class MarkovDropout {
    double probability_, period_;
    std::uint64_t seed_;
    bool stationary_, initialized_=false;
    MarkovParameters parameters_;
    std::map<int,MarkovState> states_;
    std::uint64_t last_sequence_=0;
    double last_time_=0;
    DropoutResult cached_;
public:
    // IDs register independent chains, not target truth. All chains advance even
    // when the ideal sensor has no observation for their target in this frame.
    MarkovDropout(double probability,double mean_missing_s,double period_s,
                  std::uint64_t seed,const std::vector<int>& target_ids,bool stationary=true);
    DropoutResult apply(const PerceptionFrame& ideal);
    const std::map<int,MarkovState>& states() const { return states_; }
};
} // namespace accsim
