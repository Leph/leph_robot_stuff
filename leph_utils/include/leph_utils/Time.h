#ifndef LEPH_UTILS_TIME_H
#define LEPH_UTILS_TIME_H

namespace leph {

/**
 * Convert a time from double in seconds to
 * standard chrono.
 *
 * @param Time in double seconds.
 * @return the chrono time point using
 * the steady clock.
 */
inline std::chrono::steady_clock::time_point
    TimeDoubleToTimePoint(double timeInSec)
{
    std::chrono::duration<double> dur1(timeInSec);
    auto dur2 = std::chrono::duration_cast<
        std::chrono::steady_clock::duration>(dur1);
    return std::chrono::steady_clock::time_point(dur2);
}

}

#endif

