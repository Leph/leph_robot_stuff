#include <iostream>
#include <leph_maths/TimeSeries.hpp>

template <typename T, typename U>
inline void checkEqual(T v1, U v2, const std::string& msg = "")
{
    if (v1 != v2) {
        std::cout 
            << "Test [" << msg << "] failed: " 
            << v1 << " != " << v2 
            << std::endl;
    }
}

int main()
{
    leph::TimeSeries<double> series;

    checkEqual(series.size(), (size_t)0);
    series.append(1.0, 1.5);
    series.append(2.0, 2.5);
    checkEqual(series.size(), (size_t)2);
    checkEqual(series.timeMin(), 1.0);
    checkEqual(series.timeMax(), 2.0);
    checkEqual(series.lastValue(), 2.5);
    series.append(3.0, 3.5);
    series.append(4.0, 4.5);
    checkEqual(series.size(), (size_t)4);
    checkEqual(series.at(3).time, 4.0);
    checkEqual(series.at(3).value, 4.5);
    checkEqual(series.lowerIndex(2.5), (size_t)1);
    checkEqual(series.upperIndex(2.5), (size_t)2);
    checkEqual(series.lowerValue(2.5), 2.5);
    checkEqual(series.upperValue(2.5), 3.5);
    checkEqual(series.lowerIndex(2.0), (size_t)1);
    checkEqual(series.upperIndex(2.0), (size_t)2);
    checkEqual(series.interpolate(2.0), 2.5);
    checkEqual(series.interpolate(2.25), 2.75);
    checkEqual(series.interpolate(2.5), 3.0);
    checkEqual(series.interpolate(3.0), 3.5);

    return 0;
}

