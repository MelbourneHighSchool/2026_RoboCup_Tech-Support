#pragma once

#include <cmath>
#include <cstddef>
#include <cstdint>

// The Linux SDK's timestamp is CLOCK_MONOTONIC microseconds, like the
// steady-clock odometry history. Count includes misses, before filtering.
inline double scan_midpoint_s(std::uint64_t first_sample_us, std::size_t count,
                              double us_per_sample, double retrieval_time_s) {
    if (!first_sample_us || count == 0 || !std::isfinite(us_per_sample)
        || us_per_sample <= 0.0 || !std::isfinite(retrieval_time_s)) {
        return -1.0;
    }
    const double midpoint =
        (static_cast<double>(first_sample_us) + (count - 1) * us_per_sample * 0.5)
        * 1e-6;
    const double last=(static_cast<double>(first_sample_us)+(count-1)*us_per_sample)*1e-6;
    return std::isfinite(midpoint) && last <= retrieval_time_s
        ? midpoint : -1.0;
}
