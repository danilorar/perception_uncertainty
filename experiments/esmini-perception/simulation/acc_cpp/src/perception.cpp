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
} // namespace accsim
