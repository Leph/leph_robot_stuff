#include <iostream>
#include <cassert>
#include <chrono>
#include <thread>
#include <cmath>
#include <leph_utils/TimeSeriesBuffer.hpp>

static double angleWeightedAverage(
    double weight1, double angle1, double weight2, double angle2)
{
    double x1 = std::cos(angle1);
    double y1 = std::sin(angle1);
    double x2 = std::cos(angle2);
    double y2 = std::sin(angle2);

    double meanX = weight1*x1 + weight2*x2;
    double meanY = weight1*y1 + weight2*y2;

    return std::atan2(meanY, meanX);
}

struct ExampleData {
    double pos;
    double angle;
};

ExampleData exampleAverage(
    double w1, const ExampleData& d1, 
    double w2, const ExampleData& d2)
{
    ExampleData result;
    result.pos = w1*d1.pos + w2*d2.pos;
    result.angle = angleWeightedAverage(w1, d1.angle, w2, d2.angle);

    return result;
}

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
    //Check that atomic operations are supported
    std::atomic<bool> testAtomic1;
    checkEqual(
        testAtomic1.is_lock_free(), true,
        "Atomic bool");
    std::atomic<int> testAtomic2;
    checkEqual(
        testAtomic2.is_lock_free(), true,
        "Atomic int");
    std::atomic<size_t> testAtomic3;
    checkEqual(
        testAtomic3.is_lock_free(), true,
        "Atomic size_t");
    std::atomic<float> testAtomic4;
    checkEqual(
        testAtomic4.is_lock_free(), true,
        "Atomic float");
    std::atomic<double> testAtomic5;
    checkEqual(
        testAtomic5.is_lock_free(), true,
        "Atomic double");

    //TimeSeriesBuffer
    leph::TimeSeriesBuffer<ExampleData, exampleAverage> buffer(10);
    checkEqual(buffer.length(), (size_t)10, "length()");
    checkEqual(buffer.size(), (size_t)0, "size()");
    buffer.nextPoint().time = 1.0;
    buffer.nextPoint().value = {1.0, 1.0};
    checkEqual(buffer.push(), true);
    checkEqual(buffer.size(), (size_t)1, "size()");
    checkEqual(buffer.lastPoint().time, 1.0, "lastPoint() time");
    checkEqual(buffer.lastPoint().value.pos, 1.0, "lastPoint() pos");
    checkEqual(buffer.getPoint(0).time, 1.0, "getPoint() time");
    checkEqual(buffer.getPoint(0).value.pos, 1.0, "getPoint() pos");
    buffer.nextPoint().time = 2.0;
    buffer.nextPoint().value = {2.0, 2.0};
    checkEqual(buffer.push(), true);
    buffer.nextPoint().time = 3.0;
    buffer.nextPoint().value = {3.0, 3.0};
    checkEqual(buffer.push(), true);
    checkEqual(buffer.size(), (size_t)3, "size()");
    checkEqual(buffer.getPoint(1).time, 2.0, "getPoint() time");
    checkEqual(buffer.getPoint(1).value.pos, 2.0, "getPoint() pos");
    checkEqual(buffer.getInterpolation(1.5).value.pos, 1.5, "getInterpolation()");
    checkEqual(buffer.getInterpolation(1.25).value.pos, 1.25, "getInterpolation()");
    checkEqual(buffer.getInterpolation(2.0).value.pos, 2.0, "getInterpolation()");

    return 0;
}

